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
static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps,
                                              float speed_override, float accel_override);
static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps,
                                           float speed_override, float accel_override);
static void compute_axis_rmt_profile(float step_size, float speed, float accel,
                                     uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                     uint32_t *ramp_steps);

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

static uint32_t compute_ramp_steps(app_context_t *ctx, char axis, uint32_t start_delay,
                                   uint32_t end_delay, uint32_t total_steps,
                                   float accel_override)
{
    if (total_steps == 0) {
        return 0;
    }

    size_t axis_index = axis_to_index(axis);
    float accel = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[axis_index];
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

    float v0_sq = start_speed * start_speed;
    float vmax_sq = max_speed * max_speed;

    for (uint32_t i = 0; i < total_steps; i++) {
        float v_accel_sq = v0_sq + 2.0f * accel_steps_per_s2 * (float)i;
        float v_decel_sq = v0_sq + 2.0f * accel_steps_per_s2 * (float)(total_steps - 1 - i);

        float v_sq = v_accel_sq;
        if (v_sq > vmax_sq) {
            v_sq = vmax_sq;
        }
        if (v_sq > v_decel_sq) {
            v_sq = v_decel_sq;
        }
        if (v_sq < v0_sq) {
            v_sq = v0_sq;
        }

        float speed = sqrtf(v_sq);
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

float motion_delay_us_to_speed(app_context_t *ctx, char axis, uint32_t delay_us)
{
    if (ctx == NULL || delay_us == 0U) {
        return 0.0f;
    }
    return (1000000.0f * get_step_size(ctx, axis)) / (2.0f * (float)delay_us);
}

static void compute_axis_rmt_profile(float step_size, float speed, float accel,
                                     uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                     uint32_t *ramp_steps)
{
    float safe_step_size = (isfinite(step_size) && step_size > 0.0f) ? step_size : 1.0f;
    float safe_speed = (isfinite(speed) && speed > 0.0f) ? speed : safe_step_size;
    float safe_accel = (isfinite(accel) && accel > 0.0f) ? accel : safe_step_size;

    uint32_t target = (uint32_t)fmaxf(1.0f, floorf((safe_speed / safe_step_size) + 0.5f));
    uint32_t start = target;
    if (target > 1U) {
        start = target / 3U;
        if (start == 0U) {
            start = 1U;
        }
    }

    uint32_t ramp = 0U;
    if (target > start) {
        float accel_steps_per_s2 = safe_accel / safe_step_size;
        float ramp_f = ((float)target * (float)target - (float)start * (float)start) /
                       (2.0f * accel_steps_per_s2);
        if (isfinite(ramp_f) && ramp_f > 0.0f) {
            ramp = (uint32_t)fmaxf(1.0f, floorf(ramp_f + 0.5f));
        }
    }

    *start_freq_hz = start;
    *target_freq_hz = target;
    *ramp_steps = ramp;
}


static esp_err_t do_motion_adjust_axis_to_home(app_context_t *ctx, char axis)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float target_deg = 0.0f;
    float min_limit_deg = 0.0f;
    float max_limit_deg = 360.0f;

    if (axis_upper == 'C' || axis_upper == 'X') {
        dir_pin = DIR_C;
        invert = ctx->state.inverter[AXIS_C_ID];
        target_deg = ctx->settings.home_c_deg;
        min_limit_deg = ctx->settings.limit_min_c_deg;
        max_limit_deg = ctx->settings.limit_max_c_deg;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        dir_pin = DIR_A;
        invert = ctx->state.inverter[AXIS_A_ID];
        target_deg = ctx->settings.home_a_deg;
        min_limit_deg = ctx->settings.limit_min_a_deg;
        max_limit_deg = ctx->settings.limit_max_a_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    // Ensure target_deg is strictly clamped within configured NVS limits
    if (target_deg < min_limit_deg) {
        target_deg = min_limit_deg;
    }
    if (target_deg > max_limit_deg) {
        target_deg = max_limit_deg;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    gpio_num_t step_pin = (axis_upper == 'C' || axis_upper == 'X') ? STEP_C : STEP_A;
    hardware_rmt_release_pin(step_pin);

    for (int attempt = 0; attempt < 5; ++attempt) {
        float actual_deg = 0.0f;
        esp_err_t err = hardware_read_axis_encoder(axis_upper, &actual_deg);
        if (err != ESP_OK) {
            hardware_rmt_reacquire_pin(step_pin);
            xSemaphoreGive(ctx->motion_mutex);
            return err;
        }

        // Direct difference without wrapping into unpermitted ranges
        float error_deg = target_deg - actual_deg;
        if (fabsf(error_deg) <= 0.3f) {
            break;
        }

        float deg_per_step = get_deg_per_step(ctx, axis_upper);
        int32_t steps_to_move = (int32_t)floorf(fabsf(error_deg) / deg_per_step);
        if (steps_to_move <= 0) {
            break;
        }

        // Directional bounds check
        bool moving_positive = (error_deg > 0.0f);
        if (moving_positive && actual_deg >= max_limit_deg) {
            break;
        }
        if (!moving_positive && actual_deg <= min_limit_deg) {
            break;
        }

        bool dir_level = moving_positive;
        if (invert) {
            dir_level = !dir_level;
        }
        gpio_set_level(dir_pin, dir_level ? 1 : 0);
        esp_rom_delay_us(5);
        for (int32_t s = 0; s < steps_to_move; s++) {
            hardware_step_pulse(step_pin, 800);
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    hardware_rmt_reacquire_pin(step_pin);
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

    hardware_rmt_release_pin(STEP_Z);

    gpio_set_level(DIR_Z, Z_DIR_DOWN);
    esp_rom_delay_us(5);
    int32_t search_steps = 0;
    while (!hardware_is_z_switch_pressed() && search_steps < Z_HOME_SEARCH_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 400);
        ++search_steps;
        if ((search_steps & 0x3FU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (search_steps >= Z_HOME_SEARCH_LIMIT_STEPS && !hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        hardware_rmt_reacquire_pin(STEP_Z);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(DIR_Z, Z_DIR_UP);
    esp_rom_delay_us(5);
    int32_t release_steps = 0;
    while (hardware_is_z_switch_pressed() && release_steps < Z_HOME_RELEASE_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 800);
        ++release_steps;
        if ((release_steps & 0x3FU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (release_steps >= Z_HOME_RELEASE_LIMIT_STEPS && hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        hardware_rmt_reacquire_pin(STEP_Z);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    for (int32_t i = 0; i < PASSOS_ALIVIO_EXTRA_Z; ++i) {
        hardware_step_pulse(STEP_Z, 800);
    }

    ctx->state.atual_z = 0;
    ctx->state.em_homing_z = false;
    hardware_rmt_reacquire_pin(STEP_Z);
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
         float step_size = get_step_size(ctx, axis_upper);
         float accel_val = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[axis_idx];
         if (accel_val < 10.0f) {
             accel_val = 10.0f;
         }
         float speed_val = (speed_override > 0.0f) ? speed_override : (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[axis_idx]) * step_size);
         if (speed_val < 1.0f) {
             speed_val = 1.0f;
         }

         uint32_t start_freq_hz = 0U;
         uint32_t target_freq_hz = 0U;
         uint32_t ramp_steps = 0U;
         compute_axis_rmt_profile(step_size, speed_val, accel_val,
                                  &start_freq_hz, &target_freq_hz, &ramp_steps);

         uint32_t abs_steps = (uint32_t)labs(requested_steps);
         esp_err_t rmt_err = hardware_step_pulse_rmt_move(axis_upper, abs_steps, start_freq_hz, target_freq_hz, ramp_steps);
         ESP_LOGI(APP_TAG, "MOVE_F %c concluido: %s", axis_upper, esp_err_to_name(rmt_err));

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

        hardware_rmt_release_pin(STEP_Z);

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
        uint32_t z_ramp = compute_ramp_steps(ctx, 'Z', start_delay, target_delay,
                                             (uint32_t)steps, accel_override);

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

        hardware_rmt_reacquire_pin(STEP_Z);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

static esp_err_t do_motion_move_axis(app_context_t *ctx, char axis, int32_t requested_steps,
                                     float speed_override, float accel_override)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'C' || axis_upper == 'X' || axis_upper == 'A' || axis_upper == 'Y') {
        return do_motion_move_axis_relative(ctx, axis_upper, requested_steps,
                                            speed_override, accel_override);
    }
    if (axis_upper == 'Z') {
        return do_motion_move_z_relative(ctx, requested_steps, speed_override, accel_override);
    }
    return ESP_ERR_INVALID_ARG;
}





int32_t motion_plan_limited_steps(float actual_deg, float home_deg,
                                  float min_limit_deg, float max_limit_deg,
                                  float deg_per_step, int32_t requested_steps)
{
    (void)home_deg;
    if (requested_steps == 0 || !isfinite(actual_deg) ||
        !isfinite(min_limit_deg) || !isfinite(max_limit_deg) || !isfinite(deg_per_step) ||
        deg_per_step <= 0.0f || max_limit_deg <= min_limit_deg) {
        return 0;
    }

    // Direct comparison against exact configured limits
    if (requested_steps > 0) {
        // Moving in positive direction
        if (actual_deg >= max_limit_deg) {
            return 0; // Already at or beyond max limit
        }
        float requested_delta_deg = (float)requested_steps * deg_per_step;
        float target_deg = actual_deg + requested_delta_deg;
        if (target_deg > max_limit_deg) {
            target_deg = max_limit_deg;
        }
        float permitted_delta_deg = target_deg - actual_deg;
        if (permitted_delta_deg <= 0.0f) {
            return 0;
        }
        int32_t permitted_steps = (int32_t)floorf(permitted_delta_deg / deg_per_step);
        if (permitted_steps > requested_steps) {
            permitted_steps = requested_steps;
        }
        return permitted_steps;
    } else {
        // Moving in negative direction
        if (actual_deg <= min_limit_deg) {
            return 0; // Already at or below min limit
        }
        float requested_delta_deg = (float)requested_steps * deg_per_step; // negative
        float target_deg = actual_deg + requested_delta_deg;
        if (target_deg < min_limit_deg) {
            target_deg = min_limit_deg;
        }
        float permitted_delta_deg = target_deg - actual_deg; // negative
        if (permitted_delta_deg >= 0.0f) {
            return 0;
        }
        int32_t permitted_steps = (int32_t)floorf(fabsf(permitted_delta_deg) / deg_per_step);
        int32_t requested_abs = -requested_steps;
        if (permitted_steps > requested_abs) {
            permitted_steps = requested_abs;
        }
        return -permitted_steps;
    }
}

static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps,
                                              float speed_override, float accel_override)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float home_deg = 0.0f;
    float min_limit_deg = 0.0f;
    float max_limit_deg = 360.0f;

    if (axis_upper == 'C' || axis_upper == 'X') {
        dir_pin = DIR_C;
        invert = ctx->state.inverter[AXIS_C_ID];
        home_deg = ctx->settings.home_c_deg;
        min_limit_deg = ctx->settings.limit_min_c_deg;
        max_limit_deg = ctx->settings.limit_max_c_deg;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        dir_pin = DIR_A;
        invert = ctx->state.inverter[AXIS_A_ID];
        home_deg = ctx->settings.home_a_deg;
        min_limit_deg = ctx->settings.limit_min_a_deg;
        max_limit_deg = ctx->settings.limit_max_a_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    float actual_deg = 0.0f;
    esp_err_t enc_err = hardware_read_axis_encoder(axis_upper, &actual_deg);
    if (enc_err != ESP_OK) {
        ESP_LOGE(APP_TAG, "MOVE %c rejeitado: encoder indisponivel (%s). Use MOVE_F somente se a malha aberta for intencional.",
                 axis_upper, esp_err_to_name(enc_err));
        return enc_err;
    }

    float deg_per_step = get_deg_per_step(ctx, axis_upper);
    int32_t planned_steps = motion_plan_limited_steps(actual_deg, home_deg, min_limit_deg, max_limit_deg,
                                                      deg_per_step, requested_steps);
    if (planned_steps == 0) {
        ESP_LOGW(APP_TAG,
                 "MOVE %c bloqueado pelo limite: pedido=%ld pos=%.2f limites=[%.2f, %.2f]",
                 axis_upper, (long)requested_steps, actual_deg, min_limit_deg, max_limit_deg);
        return ESP_OK;
    }
    bool moving_positive = planned_steps > 0;
    uint32_t steps_to_execute = (uint32_t)labs(planned_steps);
    float permitted_move_deg = (float)planned_steps * deg_per_step;

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool dir_level = moving_positive;
    if (invert) {
        dir_level = !dir_level;
    }
    gpio_set_level(dir_pin, dir_level ? 1 : 0);
    esp_rom_delay_us(5);
    ESP_LOGI(APP_TAG, "MOVE %c pedido=%ld exec=%ld dir_pin=%d invert=%d delta=%.2f pos=%.2f limites=[%.2f, %.2f]",
             axis_upper, (long)requested_steps, (long)planned_steps,
             (int)(dir_level ? 1 : 0),
             (int)invert, permitted_move_deg, actual_deg, min_limit_deg, max_limit_deg);
    size_t axis_idx = axis_to_index(axis_upper);
    float step_size = get_step_size(ctx, axis_upper);
    float accel_val = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[axis_idx];
    if (accel_val < 10.0f) {
        accel_val = 10.0f;
    }
    float speed_val = (speed_override > 0.0f) ? speed_override :
                      (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[axis_idx]) * step_size);
    if (speed_val < 1.0f) {
        speed_val = 1.0f;
    }

    uint32_t start_freq_hz = 0U;
    uint32_t target_freq_hz = 0U;
    uint32_t ramp_steps = 0U;
    compute_axis_rmt_profile(step_size, speed_val, accel_val,
                             &start_freq_hz, &target_freq_hz, &ramp_steps);

    esp_err_t rmt_err = hardware_step_pulse_rmt_move(axis_upper, steps_to_execute,
                                                     start_freq_hz, target_freq_hz, ramp_steps);

    xSemaphoreGive(ctx->motion_mutex);
    return rmt_err;
}

static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps,
                                           float speed_override, float accel_override)
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

    hardware_rmt_release_pin(STEP_Z);

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

    uint32_t target_delay = (speed_override > 0.0f) ?
                            motion_speed_to_delay_us(ctx, 'Z', speed_override) :
                            ctx->state.speed_delay_us[AXIS_Z_ID];
    uint32_t start_delay = (target_delay > 2000U) ? target_delay : 2000U;
    uint32_t z_ramp = compute_ramp_steps(ctx, 'Z', start_delay, target_delay,
                                         (uint32_t)steps, accel_override);

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
        if ((i & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    hardware_rmt_reacquire_pin(STEP_Z);
    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}

static esp_err_t do_motion_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z, float speed_override, float accel_override)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    if (steps_c == 0 && steps_a == 0 && steps_z == 0) {
        return ESP_OK;
    }

    if (steps_z != 0 && ctx->state.z_bloqueado) {
        ESP_LOGW(APP_TAG, "MOVE_SYNC Z bloqueado por seguranca");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(APP_TAG, "MOVE_SYNC timeout no motion_mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Set Axis C DIR
    bool dir_c = (steps_c > 0);
    if (ctx->state.inverter[AXIS_C_ID]) dir_c = !dir_c;
    gpio_set_level(DIR_C, dir_c ? 1 : 0);

    // Set Axis A DIR
    bool dir_a = (steps_a > 0);
    if (ctx->state.inverter[AXIS_A_ID]) dir_a = !dir_a;
    gpio_set_level(DIR_A, dir_a ? 1 : 0);

    // Set Axis Z DIR
    bool move_up = (steps_z > 0);
    if (ctx->state.inverter[AXIS_Z_ID]) move_up = !move_up;
    gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);

    esp_rom_delay_us(5);

    uint32_t abs_c = (uint32_t)labs(steps_c);
    uint32_t abs_a = (uint32_t)labs(steps_a);
    uint32_t abs_z = (uint32_t)labs(steps_z);

    float step_size_c = get_step_size(ctx, 'C');
    float step_size_a = get_step_size(ctx, 'A');
    float step_size_z = get_step_size(ctx, 'Z');

    // Accel
    float accel_c = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[AXIS_C_ID];
    float accel_a = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[AXIS_A_ID];
    float accel_z = (accel_override > 0.0f) ? accel_override : ctx->settings.accel[AXIS_Z_ID];
    if (accel_c < 10.0f) accel_c = 10.0f;
    if (accel_a < 10.0f) accel_a = 10.0f;
    if (accel_z < 10.0f) accel_z = 10.0f;

    // Speed
    float speed_c = (speed_override > 0.0f) ? speed_override : (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_C_ID]) * step_size_c);
    float speed_a = (speed_override > 0.0f) ? speed_override : (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_A_ID]) * step_size_a);
    float speed_z = (speed_override > 0.0f) ? speed_override : (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_Z_ID]) * step_size_z);
    if (speed_c < 1.0f) speed_c = 1.0f;
    if (speed_a < 1.0f) speed_a = 1.0f;
    if (speed_z < 1.0f) speed_z = 1.0f;

    // Freq & ramps
    uint32_t target_freq_c = (uint32_t)(speed_c / step_size_c + 0.5f);
    uint32_t target_freq_a = (uint32_t)(speed_a / step_size_a + 0.5f);
    uint32_t target_freq_z = (uint32_t)(speed_z / step_size_z + 0.5f);

    uint32_t start_freq_c = (target_freq_c > 600U) ? (target_freq_c / 3U) : 300U;
    uint32_t start_freq_a = (target_freq_a > 600U) ? (target_freq_a / 3U) : 300U;
    uint32_t start_freq_z = (target_freq_z > 400U) ? (target_freq_z / 2U) : 200U;
    if (start_freq_c >= target_freq_c) start_freq_c = target_freq_c / 2U;
    if (start_freq_a >= target_freq_a) start_freq_a = target_freq_a / 2U;
    if (start_freq_z >= target_freq_z) start_freq_z = target_freq_z / 2U;

    float ramp_c_f = ((float)target_freq_c * (float)target_freq_c - (float)start_freq_c * (float)start_freq_c) / (2.0f * (accel_c / step_size_c));
    uint32_t ramp_c = (uint32_t)(ramp_c_f + 0.5f);
    if (ramp_c < 1U) ramp_c = 1U;

    float ramp_a_f = ((float)target_freq_a * (float)target_freq_a - (float)start_freq_a * (float)start_freq_a) / (2.0f * (accel_a / step_size_a));
    uint32_t ramp_a = (uint32_t)(ramp_a_f + 0.5f);
    if (ramp_a < 1U) ramp_a = 1U;

    float ramp_z_f = ((float)target_freq_z * (float)target_freq_z - (float)start_freq_z * (float)start_freq_z) / (2.0f * (accel_z / step_size_z));
    uint32_t ramp_z = (uint32_t)(ramp_z_f + 0.5f);
    if (ramp_z < 1U) ramp_z = 1U;

    ESP_LOGI(APP_TAG, "MOVE_SYNC C=%lu A=%lu Z=%lu", (unsigned long)abs_c, (unsigned long)abs_a, (unsigned long)abs_z);
    esp_err_t err = hardware_step_pulse_rmt_move_sync3(abs_c, start_freq_c, target_freq_c, ramp_c,
                                                       abs_a, start_freq_a, target_freq_a, ramp_a,
                                                       abs_z, start_freq_z, target_freq_z, ramp_z);
    if (err == ESP_OK && steps_z != 0) {
        ctx->state.atual_z += steps_z;
    }

    xSemaphoreGive(ctx->motion_mutex);
    return err;
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
                err = do_motion_move_axis(ctx, cmd.axis, cmd.steps,
                                          cmd.speed_override, cmd.accel_override);
                break;
            case MOTION_CMD_MOVE_FORCE:
                err = do_motion_move_axis_force(ctx, cmd.axis, cmd.steps, cmd.speed_override, cmd.accel_override);
                break;
            case MOTION_CMD_MOVE_SYNC:
                err = do_motion_move_sync(ctx, cmd.steps_c, cmd.steps_a, cmd.steps_z, cmd.speed_override, cmd.accel_override);
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

            // Instant position broadcast for connected serial/UI client
            float live_c_deg = 0.0f, live_a_deg = 0.0f;
            if (hardware_read_axis_encoder('C', &live_c_deg) == ESP_OK) {
                printf("Eixo C (Base): %.2f deg\n", live_c_deg);
            }
            if (hardware_read_axis_encoder('A', &live_a_deg) == ESP_OK) {
                printf("Eixo A (Pivot): %.2f deg\n", live_a_deg);
            }
            printf("Eixo Z: %ld / %ld passos\n", (long)ctx->state.atual_z, (long)ctx->settings.max_passos_z);

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

    if (steps == INT32_MIN) {
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
    return motion_post_move_axis_profile(ctx, axis, steps, speed, accel, true,
                                         sender_id, opcode);
}

esp_err_t motion_post_move_axis_profile(app_context_t *ctx, char axis, int32_t steps,
                                        float speed, float accel, bool force_no_encoder,
                                        uint8_t sender_id, uint8_t opcode)
{
    ESP_RETURN_ON_ERROR(validate_motion_enqueue_request(ctx, axis, steps), APP_TAG,
                        "movimento parametrizado rejeitado");
    ESP_RETURN_ON_FALSE((speed < 0.0f || isfinite(speed)) && (accel < 0.0f || isfinite(accel)),
                        ESP_ERR_INVALID_ARG, APP_TAG, "perfil de movimento invalido");

    char axis_upper = (char)toupper((unsigned char)axis);
    if (speed > 0.0f) {
        bool speed_valid = (axis_upper == 'Z') ?
                           (speed >= SPEED_MIN_MM_S_Z && speed <= SPEED_MAX_MM_S_Z) :
                           (speed >= SPEED_MIN_DEG_S_CA && speed <= SPEED_MAX_DEG_S_CA);
        ESP_RETURN_ON_FALSE(speed_valid, ESP_ERR_INVALID_ARG, APP_TAG, "velocidade fora da faixa");
    }
    if (accel > 0.0f) {
        bool accel_valid = (axis_upper == 'Z') ?
                           (accel >= ACCEL_MIN_MM_S2_Z && accel <= ACCEL_MAX_MM_S2_Z) :
                           (accel >= ACCEL_MIN_DEG_S2_CA && accel <= ACCEL_MAX_DEG_S2_CA);
        ESP_RETURN_ON_FALSE(accel_valid, ESP_ERR_INVALID_ARG, APP_TAG, "aceleracao fora da faixa");
    }

    motion_cmd_t cmd = {
        .type = force_no_encoder ? MOTION_CMD_MOVE_FORCE : MOTION_CMD_MOVE_REL,
        .axis = axis_upper,
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

esp_err_t motion_post_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z, float speed, float accel, uint8_t sender_id, uint8_t opcode)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_SYNC,
        .axis = 'S',
        .steps = 0,
        .steps_c = steps_c,
        .steps_a = steps_a,
        .steps_z = steps_z,
        .sender_node_id = sender_id,
        .opcode = opcode,
        .speed_override = speed,
        .accel_override = accel
    };
    return enqueue_motion_cmd(ctx, &cmd);
}
