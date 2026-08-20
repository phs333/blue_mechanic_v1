#include "motion.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "hardware.h"
#include "storage.h"
#include "can_bus.h"

static float get_deg_per_step(app_context_t *ctx, char axis);
static float get_step_size(app_context_t *ctx, char axis);
static size_t axis_to_index(char axis);
static float normalize_angle_deg(float angle);
static esp_err_t compute_axis_deviation(app_context_t *ctx, char axis, float *deviation_deg);
static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps);
static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps);

static float get_deg_per_step(app_context_t *ctx, char axis)
{
    size_t axis_index = axis_to_index(axis);
    uint16_t msteps = ctx->settings.tmc_microsteps[axis_index];
    if (msteps == 0) {
        msteps = 16;
    }
    float spr = (float)ctx->settings.steps_per_rev[axis_index];
    if (spr < 1.0f) {
        spr = 200.0f;
    }
    return 360.0f / (spr * (float)msteps);
}

static size_t axis_to_index(char axis)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'C' || axis_upper == 'X') {
        return AXIS_C_ID;
    }
    if (axis_upper == 'A' || axis_upper == 'Y') {
        return AXIS_A_ID;
    }
    return AXIS_Z_ID;
}

static float get_step_size(app_context_t *ctx, char axis)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    size_t axis_index = axis_to_index(axis_upper);
    uint16_t msteps = ctx->settings.tmc_microsteps[axis_index];
    if (msteps == 0) {
        msteps = 16;
    }
    float spr = (float)ctx->settings.steps_per_rev[axis_index];
    if (spr < 1.0f) {
        spr = 200.0f;
    }
    if (axis_upper == 'C' || axis_upper == 'X' || axis_upper == 'A' || axis_upper == 'Y') {
        return 360.0f / (spr * (float)msteps);
    }
    uint16_t teeth = ctx->settings.z_pulley_teeth;
    if (teeth == 0) {
        teeth = DEFAULT_Z_PULLEY_TEETH;
    }
    return (float)(teeth * Z_BELT_PITCH_MM) / (spr * (float)msteps);
}

static uint32_t compute_ramp_steps(app_context_t *ctx, char axis, uint32_t start_delay, uint32_t end_delay, uint32_t total_steps)
{
    if (total_steps == 0) {
        return 0;
    }

    size_t axis_index = axis_to_index(axis);
    float accel = ctx->settings.accel[axis_index];
    if (accel <= 0.0f) {
        return 0;
    }

    float step_size = get_step_size(ctx, axis);
    float accel_steps_per_s2 = accel / step_size;

    float start_speed = 1000000.0f / (2.0f * (float)start_delay);
    float end_speed   = 1000000.0f / (2.0f * (float)end_delay);
    float ramp_steps_f = (end_speed * end_speed - start_speed * start_speed) / (2.0f * accel_steps_per_s2);

    uint32_t ramp_steps = (uint32_t)(ramp_steps_f + 0.5f);
    if (ramp_steps < 1) {
        ramp_steps = 1;
    }
    if (ramp_steps > total_steps / 2) {
        ramp_steps = total_steps / 2;
    }
    return ramp_steps;
}

//
// Pre-computes the delay for every step in a trapezoidal move profile.
// Uses the standard kinematic equations from FluidNC/GRBL:
//   Acceleration:  v(t) = v_start + a*t
//   Cruise:        v(t) = v_max
//   Deceleration:  v(t) = v_max - a*t
// Each step consists of a high pulse and a low pulse, so:
//   delay_us = 1e6 / (2 * speed_in_steps_per_sec)
// When the ramp exceeds half the move, the profile becomes triangular
// and the peak speed is derived from: v_peak = sqrt(v_start² + 2*a*ramp_steps)
//
void compute_trapezoidal_profile(app_context_t *ctx, char axis,
                                  uint32_t start_delay, uint32_t end_delay,
                                  uint32_t total_steps, uint32_t ramp_steps,
                                  uint32_t *delay_us)
{
    if (total_steps == 0) {
        return;
    }

    float start_speed = 1000000.0f / (2.0f * (float)start_delay);
    float max_speed   = 1000000.0f / (2.0f * (float)end_delay);

    size_t axis_index = axis_to_index(axis);
    float step_size = get_step_size(ctx, axis);
    float accel_steps_per_s2 = ctx->settings.accel[axis_index] / step_size;

    if (ramp_steps == 0) {
        for (uint32_t i = 0; i < total_steps; i++) {
            delay_us[i] = end_delay;
        }
        return;
    }

    bool triangular = (ramp_steps * 2 > total_steps);
    float effective_max_speed = max_speed;
    uint32_t accel_steps = ramp_steps;
    uint32_t decel_steps = ramp_steps;

    if (triangular) {
        accel_steps = total_steps / 2;
        decel_steps = total_steps - accel_steps;
        // v_peak² = v_start² + 2*a*s
        float peak_speed_sq = start_speed * start_speed + 2.0f * accel_steps_per_s2 * (float)accel_steps;
        effective_max_speed = sqrtf(peak_speed_sq);
    }

    for (uint32_t i = 0; i < total_steps; i++) {
        float speed;

        if (i < accel_steps) {
            // Acceleration phase
            float t = (float)i / (float)accel_steps;
            speed = start_speed + (effective_max_speed - start_speed) * t;
        } else if (!triangular && i < total_steps - decel_steps) {
            // Cruise phase
            speed = effective_max_speed;
        } else {
            // Deceleration phase
            uint32_t decel_start = triangular ? accel_steps : (total_steps - decel_steps);
            uint32_t decel_step = i - decel_start;
            if (decel_steps == 0) {
                speed = start_speed;
            } else {
                float t = (float)decel_step / (float)decel_steps;
                if (t > 1.0f) {
                    t = 1.0f;
                }
                speed = effective_max_speed - (effective_max_speed - start_speed) * t;
            }
        }

        if (speed < start_speed * 0.5f) {
            speed = start_speed * 0.5f;
        }
        if (speed < 1.0f) {
            speed = 1.0f;
        }

        float delay_f = 1000000.0f / (2.0f * speed);
        if (delay_f < 10.0f) {
            delay_f = 10.0f;
        }
        if (delay_f > 32767.0f) {
            delay_f = 32767.0f;
        }

        delay_us[i] = (uint32_t)delay_f;
    }
}

uint32_t motion_speed_to_delay_us(app_context_t *ctx, char axis, float speed)
{
    if (speed <= 0.0f) {
        return 32767U;
    }

    size_t axis_index = axis_to_index(axis);
    uint16_t msteps = ctx->settings.tmc_microsteps[axis_index];
    if (msteps == 0) {
        msteps = 16;
    }
    float spr = (float)ctx->settings.steps_per_rev[axis_index];
    if (spr < 1.0f) {
        spr = 200.0f;
    }

    char axis_upper = (char)toupper((unsigned char)axis);
    float step_size = get_step_size(ctx, axis_upper);
    float delay_us = (1000000.0f * step_size) / (2.0f * speed);

    if (delay_us < 10.0f) {
        delay_us = 10.0f;
    }
    if (delay_us > 32767.0f) {
        delay_us = 32767.0f;
    }

    return (uint32_t)delay_us;
}


static esp_err_t do_motion_adjust_axis_to_home(app_context_t *ctx, char axis)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float target_deg;

    if (axis_upper == 'C' || axis_upper == 'X') {
        dir_pin = DIR_C;
        invert = ctx->state.inverter[AXIS_C_ID];
        target_deg = ctx->settings.home_c_deg;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        dir_pin = DIR_A;
        invert = ctx->state.inverter[AXIS_A_ID];
        target_deg = ctx->settings.home_a_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        float actual_deg = 0.0f;
        esp_err_t err = hardware_read_axis_encoder(axis_upper, &actual_deg);
        if (err != ESP_OK) {
            xSemaphoreGive(ctx->motion_mutex);
            return err;
        }

        float error_deg = normalize_angle_deg(target_deg - actual_deg);
        if (fabsf(error_deg) <= 0.3f) {
            break;
        }

        float deg_per_step = get_deg_per_step(ctx, axis_upper);
        int32_t steps_to_move = (int32_t)floorf(fabsf(error_deg) / deg_per_step);
        if (steps_to_move <= 0) {
            break;
        }

        gpio_set_level(dir_pin, ((error_deg > 0.0f) ^ invert) ? 1 : 0);
        esp_rom_delay_us(5);
        uint32_t ramp_steps = compute_ramp_steps(ctx, axis_upper, 800, 800, (uint32_t)steps_to_move);
        if (ramp_steps > steps_to_move / 2) {
            ramp_steps = steps_to_move / 2;
        }
        esp_err_t rmt_err = hardware_step_pulse_rmt_with_accel(axis_upper, steps_to_move, 800, 800, ramp_steps);
        (void)rmt_err;
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}


static esp_err_t do_motion_home_z(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    ctx->state.z_bloqueado = false;
    ctx->state.em_homing_z = true;

    gpio_set_level(DIR_Z, Z_DIR_DOWN);
    int32_t search_steps = 0;
    while (!hardware_is_z_switch_pressed() && search_steps < Z_HOME_SEARCH_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 400);
        ++search_steps;
        if ((search_steps & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (search_steps >= Z_HOME_SEARCH_LIMIT_STEPS && !hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(DIR_Z, Z_DIR_UP);
    int32_t release_steps = 0;
    while (hardware_is_z_switch_pressed() && release_steps < Z_HOME_RELEASE_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 800);
        ++release_steps;
        if ((release_steps & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (release_steps >= Z_HOME_RELEASE_LIMIT_STEPS && hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    for (int32_t i = 0; i < PASSOS_ALIVIO_EXTRA_Z; ++i) {
        hardware_step_pulse(STEP_Z, 800);
    }

    ctx->state.atual_z = 0;
    ctx->state.em_homing_z = false;
    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}



static esp_err_t do_motion_move_axis_force(app_context_t *ctx, char axis, int32_t requested_steps, float speed_override, float accel_override)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    ESP_LOGI(APP_TAG, "MOVE_F %c steps=%d", axis_upper, (int)requested_steps);

    if (axis_upper == 'C' || axis_upper == 'X' || axis_upper == 'A' || axis_upper == 'Y') {
        gpio_num_t dir_pin;
        bool invert;
        size_t axis_idx = axis_to_index(axis_upper);

        if (axis_upper == 'C' || axis_upper == 'X') {
            dir_pin = DIR_C;
            invert = ctx->state.inverter[AXIS_C_ID];
        } else {
            dir_pin = DIR_A;
            invert = ctx->state.inverter[AXIS_A_ID];
        }

        bool positive_motion = requested_steps > 0;
        if (invert) {
            positive_motion = !positive_motion;
        }

        if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGE(APP_TAG, "MOVE_F %c timeout no motion_mutex", axis_upper);
            return ESP_ERR_TIMEOUT;
        }

         gpio_set_level(dir_pin, positive_motion ? 1 : 0);
         esp_rom_delay_us(5);
         ESP_LOGI(APP_TAG, "MOVE_F %c steps=%d dir_pin=%d invert=%d",
                  axis_upper, (int)requested_steps,
                  (int)(positive_motion ? 1 : 0), (int)invert);
         uint32_t target_delay = ctx->state.speed_delay_us[axis_idx];
         if (speed_override > 0.0f) {
             uint32_t override_delay = motion_speed_to_delay_us(ctx, axis_upper, speed_override);
             target_delay = override_delay;
         }
         uint32_t start_delay = (target_delay > 2000) ? target_delay : 2000;

         // Use per-move accel override if provided, else use default
         float save_accel = ctx->settings.accel[axis_idx];
         if (accel_override > 0.0f) {
             ctx->settings.accel[axis_idx] = accel_override;
         }

         uint32_t abs_steps = (uint32_t)labs(requested_steps);
         uint32_t ramp_steps = compute_ramp_steps(ctx, axis_upper, start_delay, target_delay, abs_steps);

         uint32_t *delay_buffer = heap_caps_malloc(abs_steps * sizeof(uint32_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
         if (delay_buffer == NULL) {
             ESP_LOGE(APP_TAG, "MOVE_F %c: malloc do buffer de delays falhou", axis_upper);
             if (accel_override > 0.0f) {
                 ctx->settings.accel[axis_idx] = save_accel;
             }
             xSemaphoreGive(ctx->motion_mutex);
             return ESP_ERR_NO_MEM;
         }
         compute_trapezoidal_profile(ctx, axis_upper, start_delay, target_delay, abs_steps, ramp_steps, delay_buffer);
         esp_err_t rmt_err = hardware_step_pulse_rmt_profiled(axis_upper, delay_buffer, abs_steps);
         heap_caps_free(delay_buffer);
         if (accel_override > 0.0f) {
             ctx->settings.accel[axis_idx] = save_accel;
         }
         ESP_LOGI(APP_TAG, "MOVE_F %c RMT retornou %s", axis_upper, esp_err_to_name(rmt_err));

         xSemaphoreGive(ctx->motion_mutex);
         return rmt_err;
    }

    if (axis_upper == 'Z') {
        if (ctx->state.z_bloqueado) {
            return ESP_ERR_INVALID_STATE;
        }

        if (requested_steps == 0) {
            return ESP_OK;
        }

        if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        bool move_up = requested_steps > 0;
        if (ctx->state.inverter[AXIS_Z_ID]) {
            move_up = !move_up;
        }
        int32_t steps = labs(requested_steps);
        gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);
        esp_rom_delay_us(5);
        ESP_LOGI(APP_TAG, "MOVE_F Z steps=%d dir_pin=%d (Z_DIR_UP=%d Z_DIR_DOWN=%d)",
                 (int)requested_steps,
                 (int)(move_up ? Z_DIR_UP : Z_DIR_DOWN),
                 (int)Z_DIR_UP, (int)Z_DIR_DOWN);

        uint32_t target_delay = ctx->state.speed_delay_us[AXIS_Z_ID];
        if (speed_override > 0.0f) {
            target_delay = motion_speed_to_delay_us(ctx, 'Z', speed_override);
        }
        uint32_t start_delay = (target_delay > 2000) ? target_delay : 2000;
        float save_accel = ctx->settings.accel[AXIS_Z_ID];
        if (accel_override > 0.0f) {
            ctx->settings.accel[AXIS_Z_ID] = accel_override;
        }
        uint32_t z_ramp = compute_ramp_steps(ctx, 'Z', start_delay, target_delay, (uint32_t)steps);

        for (int32_t i = 0; i < steps; ++i) {
            if (!move_up) {
                if (ctx->state.atual_z <= 0 || hardware_is_z_switch_pressed()) {
                    ctx->state.z_bloqueado = hardware_is_z_switch_pressed();
                    break;
                }
                ctx->state.atual_z--;
            } else {
                if (ctx->state.atual_z >= ctx->settings.max_passos_z) {
                    break;
                }
                ctx->state.atual_z++;
            }
            uint32_t delay = hardware_compute_step_delay(start_delay, target_delay, (uint32_t)i, (uint32_t)steps, z_ramp);
            hardware_step_pulse(STEP_Z, delay);
        }

        if (accel_override > 0.0f) {
            ctx->settings.accel[AXIS_Z_ID] = save_accel;
        }
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

static esp_err_t do_motion_move_axis(app_context_t *ctx, char axis, int32_t requested_steps)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'C' || axis_upper == 'X' || axis_upper == 'A' || axis_upper == 'Y') {
        return do_motion_move_axis_relative(ctx, axis_upper, requested_steps);
    }
    if (axis_upper == 'Z') {
        return do_motion_move_z_relative(ctx, requested_steps);
    }
    return ESP_ERR_INVALID_ARG;
}

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static esp_err_t compute_axis_deviation(app_context_t *ctx, char axis, float *deviation_deg)
{
    float actual_deg = 0.0f;
    ESP_RETURN_ON_FALSE(deviation_deg != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "deviation_deg nulo");
    ESP_RETURN_ON_ERROR(hardware_read_axis_encoder(axis, &actual_deg), APP_TAG, "Falha ao ler encoder");

    float home_deg = 0.0f;
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'C' || axis_upper == 'X') {
        home_deg = ctx->settings.home_c_deg;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        home_deg = ctx->settings.home_a_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    *deviation_deg = normalize_angle_deg(actual_deg - home_deg);
    return ESP_OK;
}

static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float reduction;

    if (axis_upper == 'C' || axis_upper == 'X') {
        dir_pin = DIR_C;
        invert = ctx->state.inverter[AXIS_C_ID];
        reduction = REDUCAO_C;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        dir_pin = DIR_A;
        invert = ctx->state.inverter[AXIS_A_ID];
        reduction = REDUCAO_A;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    float current_deviation_deg = 0.0f;
    ESP_RETURN_ON_ERROR(compute_axis_deviation(ctx, axis_upper, &current_deviation_deg), APP_TAG, "Falha ao calcular desvio");

    bool positive_motion = requested_steps > 0;
    if (invert) {
        positive_motion = !positive_motion;
    }

    float deg_per_step = get_deg_per_step(ctx, axis_upper);
    float requested_move_deg = (positive_motion ? 1.0f : -1.0f) * fabsf((float)requested_steps) * deg_per_step;
    float target_deviation_deg = current_deviation_deg + requested_move_deg;
    float limit_deg = LIMITE_GRAUS_CA * reduction;

    if (target_deviation_deg > limit_deg) {
        target_deviation_deg = limit_deg;
    } else if (target_deviation_deg < -limit_deg) {
        target_deviation_deg = -limit_deg;
    }

    float permitted_move_deg = target_deviation_deg - current_deviation_deg;
    if (fabsf(permitted_move_deg) < deg_per_step) {
        return ESP_OK;
    }

    int32_t steps_to_execute = (int32_t)floorf(fabsf(permitted_move_deg) / deg_per_step);
    if (steps_to_execute <= 0) {
        return ESP_OK;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(dir_pin, ((permitted_move_deg > 0.0f) ^ invert) ? 1 : 0);
    esp_rom_delay_us(2);
    ESP_LOGI(APP_TAG, "MOVE %c steps=%d dir_pin=%d invert=%d permitted_deg=%.2f",
             axis_upper, (int)requested_steps,
             (int)(((permitted_move_deg > 0.0f) ^ invert) ? 1 : 0),
             (int)invert, permitted_move_deg);
    size_t axis_idx = axis_to_index(axis_upper);
    uint32_t target_delay = ctx->state.speed_delay_us[axis_idx];
    uint32_t start_delay = (target_delay > 2000) ? target_delay : 2000;
    uint32_t abs_steps = (uint32_t)steps_to_execute;
    uint32_t ramp_steps = compute_ramp_steps(ctx, axis, start_delay, target_delay, abs_steps);

    uint32_t *delay_buffer = heap_caps_malloc(abs_steps * sizeof(uint32_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (delay_buffer == NULL) {
        ESP_LOGE(APP_TAG, "MOVE %c: malloc do buffer de delays falhou", axis);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_NO_MEM;
    }
    compute_trapezoidal_profile(ctx, axis, start_delay, target_delay, abs_steps, ramp_steps, delay_buffer);
    esp_err_t rmt_err = hardware_step_pulse_rmt_profiled(axis, delay_buffer, abs_steps);
    heap_caps_free(delay_buffer);

    xSemaphoreGive(ctx->motion_mutex);
    return rmt_err;
}

static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps)
{
    if (ctx->state.z_bloqueado) {
        return ESP_ERR_INVALID_STATE;
    }

    if (requested_steps == 0) {
        return ESP_OK;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool move_up = requested_steps > 0;
    if (ctx->state.inverter[AXIS_Z_ID]) {
        move_up = !move_up;
    }
    int32_t steps = labs(requested_steps);
    gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);
    esp_rom_delay_us(2);
    ESP_LOGI(APP_TAG, "MOVE Z steps=%d dir_pin=%d (Z_DIR_UP=%d Z_DIR_DOWN=%d)",
             (int)requested_steps,
             (int)(move_up ? Z_DIR_UP : Z_DIR_DOWN),
             (int)Z_DIR_UP, (int)Z_DIR_DOWN);

    for (int32_t i = 0; i < steps; ++i) {
        if (!move_up) {
            if (ctx->state.atual_z <= 0 || hardware_is_z_switch_pressed()) {
                ctx->state.z_bloqueado = hardware_is_z_switch_pressed();
                break;
            }
            ctx->state.atual_z--;
        } else {
            if (ctx->state.atual_z >= ctx->settings.max_passos_z) {
                break;
            }
            ctx->state.atual_z++;
        }
        uint32_t target_delay = ctx->state.speed_delay_us[AXIS_Z_ID];
        uint32_t start_delay = (target_delay > 2000) ? target_delay : 2000;
        uint32_t z_ramp = compute_ramp_steps(ctx, 'Z', start_delay, target_delay, (uint32_t)steps);
        uint32_t delay = hardware_compute_step_delay(start_delay, target_delay, (uint32_t)i, (uint32_t)steps, z_ramp);
        hardware_step_pulse(STEP_Z, delay);
        if ((i & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}

static void motion_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    motion_cmd_t cmd;

    while (true) {
        if (xQueueReceive(ctx->motion_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            esp_err_t err = ESP_OK;
            ESP_LOGI(APP_TAG, "motion_task recebeu cmd type=%d axis=%c steps=%d opcode=0x%02X", (int)cmd.type, cmd.axis, (int)cmd.steps, cmd.opcode);
            switch (cmd.type) {
            case MOTION_CMD_MOVE_REL:
                err = do_motion_move_axis(ctx, cmd.axis, cmd.steps);
                break;
            case MOTION_CMD_MOVE_FORCE:
                err = do_motion_move_axis_force(ctx, cmd.axis, cmd.steps, cmd.speed_override, cmd.accel_override);
                break;
            case MOTION_CMD_HOME:
                if (cmd.axis == 'Z') {
                    err = do_motion_home_z(ctx);
                } else {
                    err = do_motion_adjust_axis_to_home(ctx, cmd.axis);
                }
                break;
            }

            ESP_LOGI(APP_TAG, "motion_task cmd type=%d axis=%c finalizado err=%s", (int)cmd.type, cmd.axis, esp_err_to_name(err));

            bool can_online = false;
            if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                can_online = ctx->state.can_online;
                xSemaphoreGive(ctx->state_mutex);
            }

            if (can_online && cmd.opcode != 0) {
                (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_DONE : CAN_EVT_ERROR, cmd.opcode, (uint8_t)err);
            }
        }
    }
}

static esp_err_t validate_motion_enqueue_request(app_context_t *ctx, char axis, int32_t steps)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper != 'C' && axis_upper != 'X' && axis_upper != 'A' && axis_upper != 'Y' && axis_upper != 'Z') {
        return ESP_ERR_INVALID_ARG;
    }

    if (axis_upper == 'Z' && steps != 0 && ctx->state.z_bloqueado) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t motion_init(app_context_t *ctx)
{
    ctx->motion_queue = xQueueCreate(10, sizeof(motion_cmd_t));
    if (ctx->motion_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t created = xTaskCreatePinnedToCore(motion_task, "motion_task", 4096, ctx, 8, NULL, 1);
    if (created != pdPASS) {
        vQueueDelete(ctx->motion_queue);
        ctx->motion_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t enqueue_motion_cmd(app_context_t *ctx, motion_cmd_t *cmd)
{
    if (ctx->motion_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(ctx->motion_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t motion_post_move_axis(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode)
{
    ESP_RETURN_ON_ERROR(validate_motion_enqueue_request(ctx, axis, steps), APP_TAG, "movimento rejeitado");

    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_REL,
        .axis = (char)toupper((unsigned char)axis),
        .steps = steps,
        .sender_node_id = sender_id,
        .opcode = opcode,
        .speed_override = -1.0f,
        .accel_override = -1.0f
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

esp_err_t motion_post_move_axis_force(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode)
{
    ESP_RETURN_ON_ERROR(validate_motion_enqueue_request(ctx, axis, steps), APP_TAG, "movimento forçado rejeitado");

    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_FORCE,
        .axis = (char)toupper((unsigned char)axis),
        .steps = steps,
        .sender_node_id = sender_id,
        .opcode = opcode,
        .speed_override = -1.0f,
        .accel_override = -1.0f
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

esp_err_t motion_post_move_axis_with_params(app_context_t *ctx, char axis, int32_t steps, float speed, float accel, uint8_t sender_id, uint8_t opcode)
{
    ESP_RETURN_ON_ERROR(validate_motion_enqueue_request(ctx, axis, steps), APP_TAG, "movimento parametrizado rejeitado");

    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_FORCE,
        .axis = (char)toupper((unsigned char)axis),
        .steps = steps,
        .sender_node_id = sender_id,
        .opcode = opcode,
        .speed_override = speed,
        .accel_override = accel
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

esp_err_t motion_post_home_axis(app_context_t *ctx, char axis, uint8_t sender_id, uint8_t opcode)
{
    motion_cmd_t cmd = {
        .type = MOTION_CMD_HOME,
        .axis = (char)toupper((unsigned char)axis),
        .steps = 0,
        .sender_node_id = sender_id,
        .opcode = opcode,
        .speed_override = -1.0f,
        .accel_override = -1.0f
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

