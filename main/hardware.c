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
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

static rmt_channel_handle_t s_rmt_x_chan = NULL;
static rmt_channel_handle_t s_rmt_y_chan = NULL;
static rmt_encoder_handle_t s_rmt_copy_encoder = NULL;

static const gpio_num_t k_laser_pins[2] = {LASER_1_PIN, LASER_2_PIN};
static const ledc_channel_t k_laser_channels[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static i2c_master_bus_handle_t k_encoder_buses[2];
static i2c_master_dev_handle_t k_encoder_devices[2];
static SemaphoreHandle_t s_i2c_mutex = NULL;

static esp_err_t init_gpio_matrix(void);
static esp_err_t init_i2c_buses(void);
static esp_err_t init_led_pwm(app_context_t *ctx);
static esp_err_t read_encoder_deg(size_t encoder_index, float *angle_deg);

static inline void one_wire_drive_low(void)
{
    gpio_set_level(TEMP_PIN, 0);
}

static inline void one_wire_release(void)
{
    gpio_set_level(TEMP_PIN, 1);
}

static bool one_wire_reset(void)
{
    one_wire_drive_low();
    esp_rom_delay_us(480);
    one_wire_release();
    esp_rom_delay_us(70);
    bool present = (gpio_get_level(TEMP_PIN) == 0);
    esp_rom_delay_us(410);
    return present;
}

static void one_wire_write_bit(int bit_value)
{
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
}

static int one_wire_read_bit(void)
{
    one_wire_drive_low();
    esp_rom_delay_us(6);
    one_wire_release();
    esp_rom_delay_us(9);
    int bit_value = gpio_get_level(TEMP_PIN);
    esp_rom_delay_us(55);
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

esp_err_t hardware_init(app_context_t *ctx)
{
    ESP_RETURN_ON_ERROR(init_gpio_matrix(), APP_TAG, "Falha ao configurar GPIOs");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = 1000000,
        .trans_queue_depth = 4,
    };

    tx_chan_config.gpio_num = STEP_X;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_rmt_x_chan), APP_TAG, "RMT X failed");
    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_x_chan), APP_TAG, "RMT enable X failed");

    tx_chan_config.gpio_num = STEP_Y;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_rmt_y_chan), APP_TAG, "RMT Y failed");
    ESP_RETURN_ON_ERROR(rmt_enable(s_rmt_y_chan), APP_TAG, "RMT enable Y failed");

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &s_rmt_copy_encoder), APP_TAG, "RMT copy encoder failed");

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

void hardware_step_pulse(gpio_num_t step_pin, uint32_t delay_us)
{
    gpio_set_level(step_pin, 1);
    esp_rom_delay_us(delay_us);
    gpio_set_level(step_pin, 0);
    esp_rom_delay_us(delay_us);
}

esp_err_t hardware_step_pulse_rmt(char axis, uint32_t steps, uint32_t delay_us)
{
    if (steps == 0) {
        return ESP_OK;
    }

    rmt_channel_handle_t chan = NULL;
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'X') {
        chan = s_rmt_x_chan;
    } else if (axis_upper == 'Y') {
        chan = s_rmt_y_chan;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (chan == NULL || s_rmt_copy_encoder == NULL) {
        ESP_LOGE(APP_TAG, "RMT %c: canal=%p encoder=%p", axis_upper, (void *)chan, (void *)s_rmt_copy_encoder);
        return ESP_ERR_INVALID_STATE;
    }

    if (delay_us > 32767U) {
        delay_us = 32767U;
    }

    rmt_symbol_word_t pulse = {
        .duration0 = delay_us,
        .level0 = 1,
        .duration1 = delay_us,
        .level1 = 0,
    };

    rmt_transmit_config_t tx_config = {
        .loop_count = steps - 1,
        .flags = {
            .eot_level = 0,
        }
    };

    ESP_LOGI(APP_TAG, "RMT %c: steps=%u delay=%u loop=%u", axis_upper, (unsigned)steps, (unsigned)delay_us, (unsigned)tx_config.loop_count);
    esp_err_t err = rmt_transmit(chan, s_rmt_copy_encoder, &pulse, sizeof(pulse), &tx_config);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "RMT %c transmit falhou: %s", axis_upper, esp_err_to_name(err));
        return err;
    }

    err = rmt_tx_wait_all_done(chan, -1);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "RMT %c wait falhou: %s", axis_upper, esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
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

esp_err_t hardware_set_laser_level(app_context_t *ctx, size_t laser_index, uint8_t level)
{
    ESP_RETURN_ON_FALSE(laser_index < 2, ESP_ERR_INVALID_ARG, APP_TAG, "Laser invalido");

    uint32_t duty = (LASER_PWM_MAX_DUTY * level) / 255U;
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
    case 'X':
        return read_encoder_deg(0, angle_deg);
    case 'Y':
        return read_encoder_deg(1, angle_deg);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t hardware_read_temperature_c(float *temp_c)
{
    ESP_RETURN_ON_FALSE(temp_c != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "temp_c nulo");

    if (!one_wire_reset()) {
        return ESP_ERR_NOT_FOUND;
    }

    one_wire_write_byte(0xCC);
    one_wire_write_byte(0x44);

    int waited_ms = 0;
    while (waited_ms < DS18B20_CONVERSION_TIMEOUT_MS) {
        if (one_wire_read_bit()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }

    if (waited_ms >= DS18B20_CONVERSION_TIMEOUT_MS) {
        return ESP_ERR_TIMEOUT;
    }

    if (!one_wire_reset()) {
        return ESP_ERR_NOT_FOUND;
    }

    one_wire_write_byte(0xCC);
    one_wire_write_byte(0xBE);

    uint8_t scratchpad[9];
    for (size_t i = 0; i < sizeof(scratchpad); ++i) {
        scratchpad[i] = one_wire_read_byte();
    }

    int16_t raw = (int16_t)(((uint16_t)scratchpad[1] << 8) | scratchpad[0]);
    *temp_c = (float)raw / 16.0f;
    if (*temp_c < -55.0f || *temp_c > 125.0f) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

static esp_err_t init_gpio_matrix(void)
{
    const uint64_t outputs =
        (1ULL << DIR_X) |
        (1ULL << DIR_Y) |
        (1ULL << DIR_Z) | (1ULL << STEP_Z) |
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

    gpio_set_level(STEP_Z, 0);
    gpio_set_level(DIR_X, 0);
    gpio_set_level(DIR_Y, 0);
    gpio_set_level(DIR_Z, Z_DIR_UP);
    gpio_set_level(TEMP_PIN, 1);
    gpio_set_level(FAN_PIN, 0);
    gpio_set_level(EN_PIN, 1);

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
            .scl_speed_hz = 400000,
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
        .duty_resolution = LEDC_TIMER_13_BIT,
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

    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(ENCODER_I2C_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t reg = ENCODER_REG_ANGLE;
    uint8_t raw_data[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(k_encoder_devices[encoder_index], &reg, sizeof(reg), raw_data, sizeof(raw_data), ENCODER_I2C_TIMEOUT_MS);

    xSemaphoreGive(s_i2c_mutex);

    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw = ((uint16_t)raw_data[0] << 8) | raw_data[1];
    raw &= 0x0FFF;
    *angle_deg = ((float)raw * 360.0f) / 4096.0f;
    return ESP_OK;
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

    if (s_rmt_copy_encoder != NULL) {
        rmt_del_encoder(s_rmt_copy_encoder);
        s_rmt_copy_encoder = NULL;
    }
    if (s_rmt_y_chan != NULL) {
        rmt_disable(s_rmt_y_chan);
        rmt_del_channel(s_rmt_y_chan);
        s_rmt_y_chan = NULL;
    }
    if (s_rmt_x_chan != NULL) {
        rmt_disable(s_rmt_x_chan);
        rmt_del_channel(s_rmt_x_chan);
        s_rmt_x_chan = NULL;
    }
}
