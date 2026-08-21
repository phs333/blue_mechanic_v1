#include "hardware.h"

#include <ctype.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "stepper_motor_encoder.h"

#define RMT_MIN_STEP_FREQ_HZ 16U
#define RMT_MAX_STEP_FREQ_HZ 60000U
#define RMT_MAX_RAMP_SAMPLES 1024U

static rmt_channel_handle_t s_rmt_chan[AXIS_COUNT] = {NULL, NULL, NULL};

static esp_err_t init_rmt_channels(void)
{
    const gpio_num_t step_pins[AXIS_COUNT] = {STEP_C, STEP_A, STEP_Z};

    for (size_t i = 0; i < AXIS_COUNT; i++) {
        if (s_rmt_chan[i] != NULL) {
            continue;
        }
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = step_pins[i],
            .mem_block_symbols = 48,
            .resolution_hz = 1000000,
            .trans_queue_depth = 10,
        };
        ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_rmt_chan[i]), APP_TAG, "Falha ao criar canal RMT");
        ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_chan[i]), APP_TAG, "Falha ao habilitar canal RMT");
    }
    return ESP_OK;
}

static const gpio_num_t k_laser_pins[2] = {LASER_1_PIN, LASER_2_PIN};
static const ledc_channel_t k_laser_channels[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static i2c_master_bus_handle_t k_encoder_buses[2];
static i2c_master_dev_handle_t k_encoder_devices[2];
static SemaphoreHandle_t s_i2c_mutex = NULL;

static esp_err_t init_gpio_matrix(void);
static esp_err_t init_i2c_buses(void);
static esp_err_t init_led_pwm(app_context_t *ctx);
static esp_err_t read_encoder_deg(size_t encoder_index, float *angle_deg);
static uint32_t laser_level_to_duty(uint16_t level);
static void normalize_rmt_frequencies(uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                      uint32_t ramp_threshold_hz, uint32_t start_divisor);

static inline void one_wire_drive_low(void)
{
    gpio_set_level(TEMP_PIN, 0);
}

static inline void one_wire_release(void)
{
    gpio_set_level(TEMP_PIN, 1);
}

static portMUX_TYPE s_onewire_mux = portMUX_INITIALIZER_UNLOCKED;

static bool one_wire_reset(void)
{
    portENTER_CRITICAL(&s_onewire_mux);
    one_wire_drive_low();
    esp_rom_delay_us(480);
    one_wire_release();
    esp_rom_delay_us(70);
    bool present = (gpio_get_level(TEMP_PIN) == 0);
    esp_rom_delay_us(410);
    portEXIT_CRITICAL(&s_onewire_mux);
    return present;
}

static void one_wire_write_bit(int bit_value)
{
    portENTER_CRITICAL(&s_onewire_mux);
    one_wire_drive_low();
    if (bit_value) {
        esp_rom_delay_us(6);
        one_wire_release();
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        one_wire_release();
        esp_rom_delay_us(10);
    }
    portEXIT_CRITICAL(&s_onewire_mux);
}

static int one_wire_read_bit(void)
{
    portENTER_CRITICAL(&s_onewire_mux);
    one_wire_drive_low();
    esp_rom_delay_us(6);
    one_wire_release();
    esp_rom_delay_us(9);
    int bit_value = gpio_get_level(TEMP_PIN);
    esp_rom_delay_us(55);
    portEXIT_CRITICAL(&s_onewire_mux);
    return bit_value;
}

static void one_wire_write_byte(uint8_t value)
{
    for (int i = 0; i < 8; ++i) {
        one_wire_write_bit((value >> i) & 0x01);
    }
}

static uint8_t one_wire_read_byte(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= (uint8_t)(one_wire_read_bit() << i);
    }
    return value;
}

static uint8_t ds18b20_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0U;
    for (size_t i = 0; i < len; ++i) {
        uint8_t value = data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            uint8_t mix = (uint8_t)((crc ^ value) & 0x01U);
            crc >>= 1;
            if (mix != 0U) {
                crc ^= 0x8CU;
            }
            value >>= 1;
        }
    }
    return crc;
}

static void normalize_rmt_frequencies(uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                      uint32_t ramp_threshold_hz, uint32_t start_divisor)
{
    uint32_t target = *target_freq_hz;
    if (target < RMT_MIN_STEP_FREQ_HZ) {
        target = RMT_MIN_STEP_FREQ_HZ;
    } else if (target > RMT_MAX_STEP_FREQ_HZ) {
        target = RMT_MAX_STEP_FREQ_HZ;
    }

    uint32_t start = *start_freq_hz;
    if (target <= ramp_threshold_hz) {
        start = target;
    } else if (start < RMT_MIN_STEP_FREQ_HZ || start >= target) {
        start = target / start_divisor;
        if (start < RMT_MIN_STEP_FREQ_HZ) {
            start = RMT_MIN_STEP_FREQ_HZ;
        }
    }

    *start_freq_hz = start;
    *target_freq_hz = target;
}

esp_err_t hardware_init(app_context_t *ctx)
{
    ESP_RETURN_ON_ERROR(init_gpio_matrix(), APP_TAG, "Falha ao configurar GPIOs");
    ESP_RETURN_ON_ERROR(init_rmt_channels(), APP_TAG, "Falha ao iniciar RMT");

    ESP_LOGI(APP_TAG, "SISTEMA ENERGIZADO: aguardando estabilizacao da fonte...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_RETURN_ON_ERROR(init_i2c_buses(), APP_TAG, "Falha ao iniciar I2C");
    ESP_RETURN_ON_ERROR(init_led_pwm(ctx), APP_TAG, "Falha ao iniciar PWM");

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    hardware_set_driver_enable(ctx, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    return ESP_OK;
}


/**
 * Temporarily release a STEP pin from RMT control so legacy bit-bang
 * code (homing, Z per-step loop) can use gpio_set_level().
 * Must be paired with hardware_rmt_reacquire_pin() when done.
 */
void hardware_rmt_release_pin(gpio_num_t step_pin)
{
    size_t idx = 0;
    if (step_pin == STEP_C) idx = 0;
    else if (step_pin == STEP_A) idx = 1;
    else if (step_pin == STEP_Z) idx = 2;
    else return;

    if (s_rmt_chan[idx] != NULL) {
        rmt_disable(s_rmt_chan[idx]);
        rmt_del_channel(s_rmt_chan[idx]);
        s_rmt_chan[idx] = NULL;
    }

    // Reclaim the GPIO for direct control
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << step_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(step_pin, 0);
}

/**
 * Re-create the RMT TX channel for a STEP pin after bit-bang is done.
 */
void hardware_rmt_reacquire_pin(gpio_num_t step_pin)
{
    size_t idx = 0;
    if (step_pin == STEP_C) idx = 0;
    else if (step_pin == STEP_A) idx = 1;
    else if (step_pin == STEP_Z) idx = 2;
    else return;

    if (s_rmt_chan[idx] != NULL) {
        return; // already acquired
    }

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = step_pin,
        .mem_block_symbols = 48,
        .resolution_hz = 1000000,
        .trans_queue_depth = 10,
    };
    if (rmt_new_tx_channel(&tx_chan_config, &s_rmt_chan[idx]) == ESP_OK) {
        rmt_enable(s_rmt_chan[idx]);
    }
}

void hardware_step_pulse(gpio_num_t step_pin, uint32_t delay_us)
{
    gpio_set_level(step_pin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(step_pin, 0);
    uint32_t low_delay = (delay_us * 2U > 5U) ? (delay_us * 2U - 5U) : 5U;
    esp_rom_delay_us(low_delay);
}

esp_err_t hardware_step_pulse_profiled(char axis, const uint32_t *delay_us, uint32_t steps)
{
    if (steps == 0 || delay_us == NULL) {
        return ESP_OK;
    }

    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t step_pin;
    if (axis_upper == 'C' || axis_upper == 'X') {
        step_pin = STEP_C;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        step_pin = STEP_A;
    } else if (axis_upper == 'Z') {
        step_pin = STEP_Z;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t last_yield = esp_timer_get_time();

    for (uint32_t i = 0; i < steps; i++) {
        uint32_t half_delay = delay_us[i];
        if (half_delay < 5U) {
            half_delay = 5U;
        }
        if (half_delay > 20000U) {
            half_delay = 20000U;
        }

        gpio_set_level(step_pin, 1);
        esp_rom_delay_us(5);
        gpio_set_level(step_pin, 0);

        uint32_t low_delay = (half_delay * 2U > 5U) ? (half_delay * 2U - 5U) : 5U;
        esp_rom_delay_us(low_delay);

        if ((i & 0x7FU) == 0U && i > 0) {
            int64_t now = esp_timer_get_time();
            if ((now - last_yield) > 300000LL) {
                vTaskDelay(pdMS_TO_TICKS(1));
                last_yield = esp_timer_get_time();
            }
        }
    }

    return ESP_OK;
}

uint32_t hardware_compute_step_delay(uint32_t start_delay, uint32_t end_delay, uint32_t step_num, uint32_t total_steps, uint32_t ramp_steps)
{
    if (total_steps == 0 || ramp_steps == 0) {
        return end_delay;
    }

    float start_speed = 1000000.0f / (2.0f * (float)start_delay);
    float end_speed   = 1000000.0f / (2.0f * (float)end_delay);

    if (step_num < ramp_steps) {
        float t = (float)step_num / (float)ramp_steps;
        float speed = start_speed + (end_speed - start_speed) * t;
        float delay_f = 1000000.0f / (2.0f * speed);
        if (delay_f < 10.0f) {
            delay_f = 10.0f;
        }
        if (delay_f > 32767.0f) {
            delay_f = 32767.0f;
        }
        return (uint32_t)delay_f;
    }

    if (step_num >= total_steps - ramp_steps) {
        float t = (float)(step_num - (total_steps - ramp_steps)) / (float)ramp_steps;
        if (t > 1.0f) {
            t = 1.0f;
        }
        float speed = end_speed + (start_speed - end_speed) * t;
        float delay_f = 1000000.0f / (2.0f * speed);
        if (delay_f < 10.0f) {
            delay_f = 10.0f;
        }
        if (delay_f > 32767.0f) {
            delay_f = 32767.0f;
        }
        return (uint32_t)delay_f;
    }

    return end_delay;
}

esp_err_t hardware_step_pulse_rmt_move(char axis, uint32_t total_steps, uint32_t start_freq_hz, uint32_t target_freq_hz, uint32_t ramp_steps)
{
    if (total_steps == 0) {
        return ESP_OK;
    }

    char axis_upper = (char)toupper((unsigned char)axis);
    size_t axis_idx;
    if (axis_upper == 'C' || axis_upper == 'X') {
        axis_idx = 0;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        axis_idx = 1;
    } else if (axis_upper == 'Z') {
        axis_idx = 2;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    rmt_channel_handle_t chan = s_rmt_chan[axis_idx];
    if (chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    normalize_rmt_frequencies(&start_freq_hz, &target_freq_hz, 600U, 3U);

    uint32_t accel_steps = ramp_steps;
    uint32_t decel_steps = ramp_steps;
    if (accel_steps > RMT_MAX_RAMP_SAMPLES) {
        accel_steps = RMT_MAX_RAMP_SAMPLES;
        decel_steps = RMT_MAX_RAMP_SAMPLES;
    }

    // For short moves (like JOGs <= 24 steps), run directly with uniform speed
    bool use_ramp = (total_steps > 24U) && (accel_steps >= 2U) && (target_freq_hz > (start_freq_hz + 30U));

    if (use_ramp) {
        if (accel_steps * 2U > total_steps) {
            accel_steps = total_steps / 4U;
            if (accel_steps < 2U) accel_steps = 2U;
            decel_steps = accel_steps;
        }
        if (accel_steps * 2U > total_steps) {
            accel_steps = total_steps / 2U;
            decel_steps = total_steps - accel_steps;
        }
        if ((target_freq_hz - start_freq_hz) < accel_steps) {
            accel_steps = target_freq_hz - start_freq_hz;
            if (accel_steps < 2U) {
                use_ramp = false;
            } else {
                decel_steps = accel_steps;
            }
        }
    }

    uint32_t cruise_steps = use_ramp ? (total_steps - accel_steps - decel_steps) : total_steps;

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    if (use_ramp) {
        stepper_motor_curve_encoder_config_t accel_cfg = {
            .resolution = 1000000,
            .sample_points = accel_steps,
            .start_freq_hz = start_freq_hz,
            .end_freq_hz = target_freq_hz,
        };
        rmt_encoder_handle_t accel_encoder = NULL;
        ESP_RETURN_ON_ERROR(rmt_new_stepper_motor_curve_encoder(&accel_cfg, &accel_encoder), APP_TAG, "Falha ao criar encoder aceleracao");

        stepper_motor_uniform_encoder_config_t uniform_cfg = {
            .resolution = 1000000,
        };
        rmt_encoder_handle_t uniform_encoder = NULL;
        esp_err_t u_err = rmt_new_stepper_motor_uniform_encoder(&uniform_cfg, &uniform_encoder);
        if (u_err != ESP_OK) {
            rmt_del_encoder(accel_encoder);
            return u_err;
        }

        stepper_motor_curve_encoder_config_t decel_cfg = {
            .resolution = 1000000,
            .sample_points = decel_steps,
            .start_freq_hz = target_freq_hz,
            .end_freq_hz = start_freq_hz,
        };
        rmt_encoder_handle_t decel_encoder = NULL;
        esp_err_t d_err = rmt_new_stepper_motor_curve_encoder(&decel_cfg, &decel_encoder);
        if (d_err != ESP_OK) {
            rmt_del_encoder(accel_encoder);
            rmt_del_encoder(uniform_encoder);
            return d_err;
        }

        // Transmit Acceleration phase
        tx_config.loop_count = 0;
        esp_err_t err = rmt_transmit(chan, accel_encoder, &accel_steps, sizeof(accel_steps), &tx_config);

        // Transmit Uniform Cruise phase
        if (err == ESP_OK && cruise_steps > 0) {
            tx_config.loop_count = (cruise_steps > 0) ? (cruise_steps - 1) : 0;
            err = rmt_transmit(chan, uniform_encoder, &target_freq_hz, sizeof(target_freq_hz), &tx_config);
        }

        // Transmit Deceleration phase
        if (err == ESP_OK && decel_steps > 0) {
            tx_config.loop_count = 0;
            err = rmt_transmit(chan, decel_encoder, &decel_steps, sizeof(decel_steps), &tx_config);
        }

        // Wait for all hardware pulses to complete
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(chan, -1);
        }

        rmt_del_encoder(accel_encoder);
        rmt_del_encoder(uniform_encoder);
        rmt_del_encoder(decel_encoder);
        return err;
    } else {
        stepper_motor_uniform_encoder_config_t uniform_cfg = {
            .resolution = 1000000,
        };
        rmt_encoder_handle_t uniform_encoder = NULL;
        ESP_RETURN_ON_ERROR(rmt_new_stepper_motor_uniform_encoder(&uniform_cfg, &uniform_encoder), APP_TAG, "Falha ao criar encoder uniforme");

        tx_config.loop_count = (total_steps > 0) ? (total_steps - 1) : 0;
        esp_err_t err = rmt_transmit(chan, uniform_encoder, &target_freq_hz, sizeof(target_freq_hz), &tx_config);
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(chan, -1);
        }
        rmt_del_encoder(uniform_encoder);
        return err;
    }
}

esp_err_t hardware_step_pulse_rmt_move_sync3(uint32_t steps_c, uint32_t start_freq_c, uint32_t target_freq_c, uint32_t ramp_c,
                                             uint32_t steps_a, uint32_t start_freq_a, uint32_t target_freq_a, uint32_t ramp_a,
                                             uint32_t steps_z, uint32_t start_freq_z, uint32_t target_freq_z, uint32_t ramp_z)
{
    if (steps_c == 0 && steps_a == 0 && steps_z == 0) {
        return ESP_OK;
    }

    rmt_channel_handle_t chan_c = s_rmt_chan[0];
    rmt_channel_handle_t chan_a = s_rmt_chan[1];
    rmt_channel_handle_t chan_z = s_rmt_chan[2];

    if ((steps_c > 0 && chan_c == NULL) ||
        (steps_a > 0 && chan_a == NULL) ||
        (steps_z > 0 && chan_z == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_encoder_handle_t accel_enc_c = NULL, unif_enc_c = NULL, decel_enc_c = NULL;
    rmt_encoder_handle_t accel_enc_a = NULL, unif_enc_a = NULL, decel_enc_a = NULL;
    rmt_encoder_handle_t accel_enc_z = NULL, unif_enc_z = NULL, decel_enc_z = NULL;
    stepper_motor_uniform_encoder_config_t uniform_cfg = { .resolution = 1000000 };

    // Axis C params
    uint32_t accel_c = ramp_c, decel_c = ramp_c, cruise_c = steps_c;
    if (accel_c > RMT_MAX_RAMP_SAMPLES) accel_c = decel_c = RMT_MAX_RAMP_SAMPLES;
    bool use_ramp_c = false;
    if (steps_c > 0) {
        normalize_rmt_frequencies(&start_freq_c, &target_freq_c, 600U, 3U);

        use_ramp_c = (steps_c > 24U) && (accel_c >= 2U) && (target_freq_c > (start_freq_c + 30U));
        if (use_ramp_c) {
            if (accel_c * 2U > steps_c) {
                accel_c = steps_c / 4U;
                if (accel_c < 2U) accel_c = 2U;
                decel_c = accel_c;
            }
            if (accel_c * 2U > steps_c) {
                accel_c = steps_c / 2U;
                decel_c = steps_c - accel_c;
            }
            if ((target_freq_c - start_freq_c) < accel_c) {
                accel_c = target_freq_c - start_freq_c;
                if (accel_c < 2U) use_ramp_c = false;
                else decel_c = accel_c;
            }
        }
        cruise_c = use_ramp_c ? (steps_c - accel_c - decel_c) : steps_c;
    }

    // Axis A params
    uint32_t accel_a = ramp_a, decel_a = ramp_a, cruise_a = steps_a;
    if (accel_a > RMT_MAX_RAMP_SAMPLES) accel_a = decel_a = RMT_MAX_RAMP_SAMPLES;
    bool use_ramp_a = false;
    if (steps_a > 0) {
        normalize_rmt_frequencies(&start_freq_a, &target_freq_a, 600U, 3U);

        use_ramp_a = (steps_a > 24U) && (accel_a >= 2U) && (target_freq_a > (start_freq_a + 30U));
        if (use_ramp_a) {
            if (accel_a * 2U > steps_a) {
                accel_a = steps_a / 4U;
                if (accel_a < 2U) accel_a = 2U;
                decel_a = accel_a;
            }
            if (accel_a * 2U > steps_a) {
                accel_a = steps_a / 2U;
                decel_a = steps_a - accel_a;
            }
            if ((target_freq_a - start_freq_a) < accel_a) {
                accel_a = target_freq_a - start_freq_a;
                if (accel_a < 2U) use_ramp_a = false;
                else decel_a = accel_a;
            }
        }
        cruise_a = use_ramp_a ? (steps_a - accel_a - decel_a) : steps_a;
    }

    // Axis Z params
    uint32_t accel_z = ramp_z, decel_z = ramp_z, cruise_z = steps_z;
    if (accel_z > RMT_MAX_RAMP_SAMPLES) accel_z = decel_z = RMT_MAX_RAMP_SAMPLES;
    bool use_ramp_z = false;
    if (steps_z > 0) {
        normalize_rmt_frequencies(&start_freq_z, &target_freq_z, 400U, 2U);

        use_ramp_z = (steps_z > 24U) && (accel_z >= 2U) && (target_freq_z > (start_freq_z + 30U));
        if (use_ramp_z) {
            if (accel_z * 2U > steps_z) {
                accel_z = steps_z / 4U;
                if (accel_z < 2U) accel_z = 2U;
                decel_z = accel_z;
            }
            if (accel_z * 2U > steps_z) {
                accel_z = steps_z / 2U;
                decel_z = steps_z - accel_z;
            }
            if ((target_freq_z - start_freq_z) < accel_z) {
                accel_z = target_freq_z - start_freq_z;
                if (accel_z < 2U) use_ramp_z = false;
                else decel_z = accel_z;
            }
        }
        cruise_z = use_ramp_z ? (steps_z - accel_z - decel_z) : steps_z;
    }

    // Build encoders for Axis C
    if (steps_c > 0) {
        if (use_ramp_c) {
            stepper_motor_curve_encoder_config_t ac_cfg = {
                .resolution = 1000000, .sample_points = accel_c,
                .start_freq_hz = start_freq_c, .end_freq_hz = target_freq_c
            };
            rmt_new_stepper_motor_curve_encoder(&ac_cfg, &accel_enc_c);
            stepper_motor_curve_encoder_config_t dc_cfg = {
                .resolution = 1000000, .sample_points = decel_c,
                .start_freq_hz = target_freq_c, .end_freq_hz = start_freq_c
            };
            rmt_new_stepper_motor_curve_encoder(&dc_cfg, &decel_enc_c);
        }
        rmt_new_stepper_motor_uniform_encoder(&uniform_cfg, &unif_enc_c);
    }

    // Build encoders for Axis A
    if (steps_a > 0) {
        if (use_ramp_a) {
            stepper_motor_curve_encoder_config_t aa_cfg = {
                .resolution = 1000000, .sample_points = accel_a,
                .start_freq_hz = start_freq_a, .end_freq_hz = target_freq_a
            };
            rmt_new_stepper_motor_curve_encoder(&aa_cfg, &accel_enc_a);
            stepper_motor_curve_encoder_config_t da_cfg = {
                .resolution = 1000000, .sample_points = decel_a,
                .start_freq_hz = target_freq_a, .end_freq_hz = start_freq_a
            };
            rmt_new_stepper_motor_curve_encoder(&da_cfg, &decel_enc_a);
        }
        rmt_new_stepper_motor_uniform_encoder(&uniform_cfg, &unif_enc_a);
    }

    // Build encoders for Axis Z
    if (steps_z > 0) {
        if (use_ramp_z) {
            stepper_motor_curve_encoder_config_t az_cfg = {
                .resolution = 1000000, .sample_points = accel_z,
                .start_freq_hz = start_freq_z, .end_freq_hz = target_freq_z
            };
            rmt_new_stepper_motor_curve_encoder(&az_cfg, &accel_enc_z);
            stepper_motor_curve_encoder_config_t dz_cfg = {
                .resolution = 1000000, .sample_points = decel_z,
                .start_freq_hz = target_freq_z, .end_freq_hz = start_freq_z
            };
            rmt_new_stepper_motor_curve_encoder(&dz_cfg, &decel_enc_z);
        }
        rmt_new_stepper_motor_uniform_encoder(&uniform_cfg, &unif_enc_z);
    }

    // Transmit Axis C queue in parallel
    rmt_transmit_config_t tx_c = { .loop_count = 0 };
    if (steps_c > 0) {
        if (use_ramp_c && accel_enc_c) {
            rmt_transmit(chan_c, accel_enc_c, &accel_c, sizeof(accel_c), &tx_c);
        }
        if (cruise_c > 0 && unif_enc_c) {
            tx_c.loop_count = (cruise_c > 0) ? (cruise_c - 1) : 0;
            rmt_transmit(chan_c, unif_enc_c, &target_freq_c, sizeof(target_freq_c), &tx_c);
        }
        if (use_ramp_c && decel_enc_c) {
            tx_c.loop_count = 0;
            rmt_transmit(chan_c, decel_enc_c, &decel_c, sizeof(decel_c), &tx_c);
        }
    }

    // Transmit Axis A queue in parallel
    rmt_transmit_config_t tx_a = { .loop_count = 0 };
    if (steps_a > 0) {
        if (use_ramp_a && accel_enc_a) {
            rmt_transmit(chan_a, accel_enc_a, &accel_a, sizeof(accel_a), &tx_a);
        }
        if (cruise_a > 0 && unif_enc_a) {
            tx_a.loop_count = (cruise_a > 0) ? (cruise_a - 1) : 0;
            rmt_transmit(chan_a, unif_enc_a, &target_freq_a, sizeof(target_freq_a), &tx_a);
        }
        if (use_ramp_a && decel_enc_a) {
            tx_a.loop_count = 0;
            rmt_transmit(chan_a, decel_enc_a, &decel_a, sizeof(decel_a), &tx_a);
        }
    }

    // Transmit Axis Z queue in parallel
    rmt_transmit_config_t tx_z = { .loop_count = 0 };
    if (steps_z > 0) {
        if (use_ramp_z && accel_enc_z) {
            rmt_transmit(chan_z, accel_enc_z, &accel_z, sizeof(accel_z), &tx_z);
        }
        if (cruise_z > 0 && unif_enc_z) {
            tx_z.loop_count = (cruise_z > 0) ? (cruise_z - 1) : 0;
            rmt_transmit(chan_z, unif_enc_z, &target_freq_z, sizeof(target_freq_z), &tx_z);
        }
        if (use_ramp_z && decel_enc_z) {
            tx_z.loop_count = 0;
            rmt_transmit(chan_z, decel_enc_z, &decel_z, sizeof(decel_z), &tx_z);
        }
    }

    // Wait for all active axes to complete
    esp_err_t err_c = (steps_c > 0) ? rmt_tx_wait_all_done(chan_c, -1) : ESP_OK;
    esp_err_t err_a = (steps_a > 0) ? rmt_tx_wait_all_done(chan_a, -1) : ESP_OK;
    esp_err_t err_z = (steps_z > 0) ? rmt_tx_wait_all_done(chan_z, -1) : ESP_OK;

    // Clean up all allocated encoders
    if (accel_enc_c) rmt_del_encoder(accel_enc_c);
    if (unif_enc_c)  rmt_del_encoder(unif_enc_c);
    if (decel_enc_c) rmt_del_encoder(decel_enc_c);
    if (accel_enc_a) rmt_del_encoder(accel_enc_a);
    if (unif_enc_a)  rmt_del_encoder(unif_enc_a);
    if (decel_enc_a) rmt_del_encoder(decel_enc_a);
    if (accel_enc_z) rmt_del_encoder(accel_enc_z);
    if (unif_enc_z)  rmt_del_encoder(unif_enc_z);
    if (decel_enc_z) rmt_del_encoder(decel_enc_z);

    if (err_c != ESP_OK) return err_c;
    if (err_a != ESP_OK) return err_a;
    return err_z;
}

esp_err_t hardware_step_pulse_rmt_move_dual(uint32_t steps_c, uint32_t start_freq_c, uint32_t target_freq_c, uint32_t ramp_c,
                                            uint32_t steps_a, uint32_t start_freq_a, uint32_t target_freq_a, uint32_t ramp_a)
{
    return hardware_step_pulse_rmt_move_sync3(steps_c, start_freq_c, target_freq_c, ramp_c,
                                              steps_a, start_freq_a, target_freq_a, ramp_a,
                                              0, 0, 0, 0);
}

void hardware_set_driver_enable(app_context_t *ctx, bool enable)
{
    gpio_set_level(EN_PIN, enable ? 0 : 1);
    if (ctx != NULL) {
        ctx->state.drivers_enabled = enable;
    }
}

void hardware_set_fan_output(app_context_t *ctx, bool on)
{
    gpio_set_level(FAN_PIN, on ? 1 : 0);
    if (ctx != NULL) {
        ctx->state.fan_output_on = on;
    }
}

esp_err_t hardware_set_laser_level(app_context_t *ctx, size_t laser_index, uint16_t level)
{
    ESP_RETURN_ON_FALSE(laser_index < 2, ESP_ERR_INVALID_ARG, APP_TAG, "Laser invalido");

    uint32_t duty = laser_level_to_duty(level);
    ESP_RETURN_ON_ERROR(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, k_laser_channels[laser_index], duty),
        APP_TAG,
        "Falha ao ajustar duty do laser");
    ESP_RETURN_ON_ERROR(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, k_laser_channels[laser_index]),
        APP_TAG,
        "Falha ao atualizar PWM do laser");

    if (ctx != NULL) {
        ctx->state.laser_level[laser_index] = level;
    }
    return ESP_OK;
}

static uint32_t laser_level_to_duty(uint16_t level)
{
    if (level == 0U) {
        return 0U;
    }
    if (level > LASER_PWM_MAX_LEVEL) {
        level = LASER_PWM_MAX_LEVEL;
    }
    return (uint32_t)level;
}

bool hardware_is_z_switch_pressed(void)
{
    if (gpio_get_level(SWITCH_Z) == 1) {
        esp_rom_delay_us(5000);
        return gpio_get_level(SWITCH_Z) == 1;
    }
    return false;
}

esp_err_t hardware_read_axis_encoder(char axis, float *angle_deg)
{
    ESP_RETURN_ON_FALSE(angle_deg != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "angle_deg nulo");

    switch (toupper((unsigned char)axis)) {
    case 'C':
    case 'X':
        return read_encoder_deg(0, angle_deg);
    case 'A':
    case 'Y':
        return read_encoder_deg(1, angle_deg);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t hardware_read_temperature_c(float *temp_c)
{
    ESP_RETURN_ON_FALSE(temp_c != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "temp_c nulo");

    for (int retry = 0; retry < 2; ++retry) {
        if (!one_wire_reset()) {
            if (retry == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            return ESP_ERR_NOT_FOUND;
        }

        one_wire_write_byte(0xCC);
        one_wire_write_byte(0x44);
        one_wire_release();

        // 12-bit DS18B20 conversion takes up to 750 ms.
        // Wait 750 ms without bus polling to avoid draining power/disturbing conversion.
        vTaskDelay(pdMS_TO_TICKS(750));

        if (!one_wire_reset()) {
            if (retry == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            return ESP_ERR_NOT_FOUND;
        }

        one_wire_write_byte(0xCC);
        one_wire_write_byte(0xBE);

        uint8_t scratchpad[9] = {0};
        for (size_t i = 0; i < sizeof(scratchpad); ++i) {
            scratchpad[i] = one_wire_read_byte();
        }

        if (ds18b20_crc8(scratchpad, sizeof(scratchpad) - 1U) != scratchpad[8]) {
            if (retry == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            return ESP_ERR_INVALID_CRC;
        }

        int16_t raw = (int16_t)(((uint16_t)scratchpad[1] << 8) | scratchpad[0]);
        float val = (float)raw / 16.0f;
        if (val < -55.0f || val > 125.0f) {
            if (retry == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            return ESP_ERR_INVALID_CRC;
        }

        *temp_c = val;
        return ESP_OK;
    }

    return ESP_FAIL;
}

static esp_err_t init_gpio_matrix(void)
{
    // STEP pins are NOT included here — they are owned exclusively by the RMT
    // peripheral.  Configuring them as GPIO_MODE_OUTPUT would override the RMT
    // connection in the IO MUX and the motor would never receive pulses.
    const uint64_t outputs =
        (1ULL << DIR_X) |
        (1ULL << DIR_Y) |
        (1ULL << DIR_Z) |
        (1ULL << EN_PIN) | (1ULL << FAN_PIN);

    gpio_config_t output_conf = {
        .pin_bit_mask = outputs,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_conf), APP_TAG, "Falha ao configurar saidas");

    gpio_config_t switch_conf = {
        .pin_bit_mask = (1ULL << SWITCH_Z),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&switch_conf), APP_TAG, "Falha ao configurar switch Z");

    gpio_config_t temp_conf = {
        .pin_bit_mask = (1ULL << TEMP_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&temp_conf), APP_TAG, "Falha ao configurar DS18B20");

    // STEP pins are controlled by RMT — do NOT set their level here.
    gpio_set_level(DIR_X, 0);
    gpio_set_level(DIR_Y, 0);
    gpio_set_level(DIR_Z, Z_DIR_UP);
    gpio_set_level(TEMP_PIN, 1);
    gpio_set_level(FAN_PIN, 0);
    gpio_set_level(EN_PIN, 0); // 0 = DRIVERS ENERGIZADOS (Active Low)

    return ESP_OK;
}

static esp_err_t init_i2c_buses(void)
{
    const struct {
        i2c_port_num_t port;
        gpio_num_t sda;
        gpio_num_t scl;
    } buses[2] = {
        {I2C_NUM_0, SDA_0, SCL_0},
        {I2C_NUM_1, SDA_1, SCL_1},
    };

    for (size_t i = 0; i < 2; ++i) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = buses[i].port,
            .sda_io_num = buses[i].sda,
            .scl_io_num = buses[i].scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
                .allow_pd = 0,
            },
        };
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ENCODER_ADDR,
            .scl_speed_hz = 100000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = 0,
            },
        };

        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &k_encoder_buses[i]), APP_TAG, "Falha ao criar barramento I2C");
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(k_encoder_buses[i], &device_config, &k_encoder_devices[i]), APP_TAG, "Falha ao adicionar encoder no I2C");
    }

    return ESP_OK;
}

static esp_err_t init_led_pwm(app_context_t *ctx)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LASER_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), APP_TAG, "Falha ao configurar timer LEDC");

    for (size_t i = 0; i < 2; ++i) {
        ledc_channel_config_t channel_config = {
            .gpio_num = k_laser_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = k_laser_channels[i],
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = {.output_invert = 0},
            .deconfigure = false,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), APP_TAG, "Falha ao configurar canal do laser");
    }

    ESP_RETURN_ON_ERROR(hardware_set_laser_level(ctx, 0, 0), APP_TAG, "Falha ao zerar laser 1");
    ESP_RETURN_ON_ERROR(hardware_set_laser_level(ctx, 1, 0), APP_TAG, "Falha ao zerar laser 2");
    return ESP_OK;
}

static esp_err_t read_encoder_deg(size_t encoder_index, float *angle_deg)
{
    ESP_RETURN_ON_FALSE(encoder_index < 2, ESP_ERR_INVALID_ARG, APP_TAG, "Encoder invalido");

    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = ENCODER_REG_RAW_ANGLE;
    uint8_t raw_data[2] = {0};
    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 0; attempt < 3; ++attempt) {
        if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(ENCODER_I2C_TIMEOUT_MS)) == pdTRUE) {
            last_err = i2c_master_transmit_receive(
                k_encoder_devices[encoder_index], &reg, sizeof(reg),
                raw_data, sizeof(raw_data), ENCODER_I2C_TIMEOUT_MS);
            xSemaphoreGive(s_i2c_mutex);

            if (last_err == ESP_OK) {
                uint16_t raw = ((uint16_t)raw_data[0] << 8) | raw_data[1];
                raw &= 0x0FFF;
                *angle_deg = ((float)raw * 360.0f) / 4096.0f;
                return ESP_OK;
            }
        }
        esp_rom_delay_us(200);
    }
    return last_err;
}

void hardware_deinit(void)
{
    if (s_i2c_mutex != NULL) {
        vSemaphoreDelete(s_i2c_mutex);
        s_i2c_mutex = NULL;
    }

    for (size_t i = 0; i < 2; ++i) {
        if (k_encoder_devices[i] != NULL) {
            i2c_master_bus_rm_device(k_encoder_devices[i]);
            k_encoder_devices[i] = NULL;
        }
        if (k_encoder_buses[i] != NULL) {
            i2c_del_master_bus(k_encoder_buses[i]);
            k_encoder_buses[i] = NULL;
        }
    }

    for (size_t i = 0; i < AXIS_COUNT; i++) {
        if (s_rmt_chan[i] != NULL) {
            rmt_disable(s_rmt_chan[i]);
            rmt_del_channel(s_rmt_chan[i]);
            s_rmt_chan[i] = NULL;
        }
    }
}
