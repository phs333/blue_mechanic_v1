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


static void compute_z_motion_profile(app_context_t *ctx, int32_t steps, float speed_override, float accel_override,
                                     uint32_t *out_start_delay, uint32_t *out_target_delay, uint32_t *out_ramp_steps)
{
    uint32_t target_delay = (speed_override > 0.0f) ?
                            motion_speed_to_delay_us(ctx, 'Z', speed_override) :
                            ctx->state.speed_delay_us[AXIS_Z_ID];

    // For short displacements (<= 400 steps / 4 mm), cap maximum speed and use gentle start delay
    // to avoid abrupt solavanco / mechanical jerk:
    uint32_t start_delay;
    if (steps <= 20) {
        // <= 0.2 mm: micro-jog, ultra soft
        start_delay = 6000U; // ~83 steps/s = 0.83 mm/s
        if (target_delay < 3333U) target_delay = 3333U; // max 1.5 mm/s
    } else if (steps <= 50) {
        // <= 0.5 mm: soft jog
        start_delay = 5500U; // ~90 steps/s = 0.9 mm/s
        if (target_delay < 2000U) target_delay = 2000U; // max 2.5 mm/s
    } else if (steps <= 100) {
        // <= 1.0 mm
        start_delay = 5000U; // 100 steps/s = 1.0 mm/s
        if (target_delay < 1250U) target_delay = 1250U; // max 4.0 mm/s
    } else if (steps <= 200) {
        // <= 2.0 mm
        start_delay = 4000U; // 125 steps/s = 1.25 mm/s
        if (target_delay < 833U)  target_delay = 833U;  // max 6.0 mm/s
    } else if (steps <= 400) {
        // <= 4.0 mm
        start_delay = 3000U; // 166 steps/s = 1.66 mm/s
        if (target_delay < 625U)  target_delay = 625U;  // max 8.0 mm/s
    } else {
        // Long displacements (> 4 mm)
        start_delay = (target_delay > 2000U) ? target_delay : 2000U;
    }

    if (start_delay < target_delay) {
        start_delay = target_delay;
    }

    // Accel scaling for short moves to prevent jerky transitions
    float effective_accel = accel_override;
    if (effective_accel <= 0.0f) {
        float base_accel = ctx->settings.accel[AXIS_Z_ID];
        if (steps <= 50) {
            effective_accel = (base_accel < 80.0f) ? base_accel : 80.0f; // 80 mm/s^2
        } else if (steps <= 200) {
            effective_accel = (base_accel < 120.0f) ? base_accel : 120.0f; // 120 mm/s^2
        } else {
            effective_accel = base_accel;
        }
    }

    uint32_t z_ramp = compute_ramp_steps(ctx, 'Z', start_delay, target_delay,
                                         (uint32_t)steps, effective_accel);

    if (steps <= 50) {
        z_ramp = (uint32_t)(steps / 2);
    }
    if (z_ramp < 1 && steps > 1) {
        z_ramp = 1;
    }
    if (z_ramp > (uint32_t)(steps / 2)) {
        z_ramp = (uint32_t)(steps / 2);
    }

    *out_start_delay = start_delay;
    *out_target_delay = target_delay;
    *out_ramp_steps = z_ramp;
}

static esp_err_t do_motion_adjust_axis_to_home(app_context_t *ctx, char axis)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float target_deg = 0.0f;
    float min_limit_deg = -540.0f;
    float max_limit_deg = 540.0f;

    if (axis_upper == 'C' || axis_upper == 'X') {
        dir_pin = DIR_C;
        invert = ctx->state.inverter[AXIS_C_ID];
        target_deg = 0.0f; // Home zero relativo
        min_limit_deg = ctx->settings.limit_min_c_deg;
        max_limit_deg = ctx->settings.limit_max_c_deg;
    } else if (axis_upper == 'A' || axis_upper == 'Y') {
        dir_pin = DIR_A;
        invert = ctx->state.inverter[AXIS_A_ID];
        target_deg = 0.0f; // Home zero relativo
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

    float actual_deg = 0.0f;
    esp_err_t err = hardware_read_axis_encoder(axis_upper, &actual_deg);
    if (err != ESP_OK) {
        hardware_rmt_reacquire_pin(step_pin);
        xSemaphoreGive(ctx->motion_mutex);
        return err;
    }

    float deg_per_step = get_deg_per_step(ctx, axis_upper);
    if (deg_per_step <= 0.0f) {
        deg_per_step = 0.1125f;
    }

    const float TOLERANCE_DEG = 0.25f;
    float error_deg = target_deg - actual_deg;

    ESP_LOGI(APP_TAG, "Home %c em malha fechada iniciado. Pos atual: %.2f deg (alvo: %.2f deg)",
             axis_upper, actual_deg, target_deg);

    if (fabsf(error_deg) <= TOLERANCE_DEG) {
        ESP_LOGI(APP_TAG, "Home %c: Eixo ja esta alinhado no zero (%.2f deg).", axis_upper, actual_deg);
        if (axis_upper == 'C' || axis_upper == 'X') {
            printf("Eixo C (Base): %.2f deg\n", actual_deg);
        } else {
            printf("Eixo A (Pivot): %.2f deg\n", actual_deg);
        }
        hardware_rmt_reacquire_pin(step_pin);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_OK;
    }

    // Dynamic safety limit in steps proportional to distance to home (max 1 full revolution = 3200 steps)
    uint32_t expected_steps = (uint32_t)ceilf(fabsf(error_deg) / deg_per_step);
    uint32_t max_allowed_steps = expected_steps * 2 + 300;
    if (max_allowed_steps > 3200) {
        max_allowed_steps = 3200;
    }
    if (max_allowed_steps < 300) {
        max_allowed_steps = 300;
    }
    uint32_t total_steps_executed = 0;
    float initial_abs_error = fabsf(error_deg);

    while (fabsf(error_deg) > TOLERANCE_DEG && total_steps_executed < max_allowed_steps) {
        float abs_err = fabsf(error_deg);
        int32_t burst_steps = 0;
        uint32_t step_delay = 1000; // us

        // Adaptive burst step count and delay for whisper-smooth deceleration and precision zeroing:
        if (abs_err > 60.0f) {
            burst_steps = 40;     // ~4.5 deg
            step_delay = 600;     // ~833 steps/s (~94 deg/s)
        } else if (abs_err > 25.0f) {
            burst_steps = 25;     // ~2.8 deg
            step_delay = 750;     // ~667 steps/s (~75 deg/s)
        } else if (abs_err > 10.0f) {
            burst_steps = 15;     // ~1.7 deg
            step_delay = 900;     // ~555 steps/s (~62 deg/s)
        } else if (abs_err > 4.0f) {
            burst_steps = 8;      // ~0.9 deg
            step_delay = 1200;    // ~416 steps/s (~47 deg/s)
        } else if (abs_err > 1.5f) {
            burst_steps = 4;      // ~0.45 deg
            step_delay = 1600;    // ~312 steps/s (~35 deg/s)
        } else if (abs_err > 0.6f) {
            burst_steps = 2;      // ~0.22 deg
            step_delay = 2000;    // ~250 steps/s (~28 deg/s)
        } else {
            // Fine-creeping mode: 1 single step (0.1125 deg) at a time!
            // Cannot overshoot because 0.1125 deg < TOLERANCE_DEG (0.25 deg)
            burst_steps = 1;
            step_delay = 2500;    // ~200 steps/s (~22 deg/s)
        }

        bool moving_positive = (error_deg > 0.0f);

        // Limit verification
        if (moving_positive && actual_deg >= max_limit_deg) {
            ESP_LOGW(APP_TAG, "Home %c interrompido: limite maximo atingido (%.2f deg)", axis_upper, actual_deg);
            break;
        }
        if (!moving_positive && actual_deg <= min_limit_deg) {
            ESP_LOGW(APP_TAG, "Home %c interrompido: limite minimo atingido (%.2f deg)", axis_upper, actual_deg);
            break;
        }

        bool dir_level = moving_positive;
        if (invert) {
            dir_level = !dir_level;
        }
        gpio_set_level(dir_pin, dir_level ? 1 : 0);
        esp_rom_delay_us(5);

        for (int32_t s = 0; s < burst_steps; s++) {
            hardware_step_pulse(step_pin, step_delay);
        }
        total_steps_executed += (uint32_t)burst_steps;

        // Yield for safety_task, CAN and sensor settling
        uint32_t yield_ms = (burst_steps <= 2) ? 6 : 10;
        vTaskDelay(pdMS_TO_TICKS(yield_ms));

        err = hardware_read_axis_encoder(axis_upper, &actual_deg);
        if (err != ESP_OK) {
            ESP_LOGE(APP_TAG, "Home %c: erro ao ler encoder durante movimento: %s", axis_upper, esp_err_to_name(err));
            break;
        }

        // Sanity check: if initial error was large and after 60+ steps the error grew significantly,
        // the axis inversion setting in NVS is opposite to physical motor wiring.
        if (initial_abs_error > 5.0f && total_steps_executed >= 60) {
            float current_abs_err = fabsf(target_deg - actual_deg);
            if (current_abs_err > initial_abs_error + 3.0f) {
                ESP_LOGE(APP_TAG, "Home %c: erro aumentou de %.2f para %.2f deg! Sentido invertido no motor. Use 'INVERT %c'.",
                         axis_upper, initial_abs_error, current_abs_err, axis_upper);
                break;
            }
        }

        error_deg = target_deg - actual_deg;
    }

    hardware_rmt_reacquire_pin(step_pin);
    xSemaphoreGive(ctx->motion_mutex);

    // Emite a posicao atualizada no formato padrao do parser serial
    if (axis_upper == 'C' || axis_upper == 'X') {
        printf("Eixo C (Base): %.2f deg\n", actual_deg);
    } else {
        printf("Eixo A (Pivot): %.2f deg\n", actual_deg);
    }

    if (fabsf(error_deg) <= TOLERANCE_DEG) {
        ESP_LOGI(APP_TAG, "Home %c concluido com precisao (pos=%.2f deg, erro=%.2f deg, passos=%lu)",
                 axis_upper, actual_deg, error_deg, (unsigned long)total_steps_executed);
        return ESP_OK;
    } else {
        ESP_LOGW(APP_TAG, "Home %c finalizado com desvio (pos=%.2f deg, erro=%.2f deg, passos=%lu)",
                 axis_upper, actual_deg, error_deg, (unsigned long)total_steps_executed);
        return ESP_OK;
    }
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

    bool dir_down = Z_DIR_DOWN;
    bool dir_up = Z_DIR_UP;
    if (ctx->state.inverter[AXIS_Z_ID]) {
        dir_down = !dir_down;
        dir_up = !dir_up;
    }

    // Se o switch já estiver pressionado no início, afasta suavemente primeiro
    if (hardware_is_z_switch_pressed()) {
        gpio_set_level(DIR_Z, dir_up);
        esp_rom_delay_us(5);
        int32_t clear_steps = 0;
        while (hardware_is_z_switch_pressed() && clear_steps < Z_HOME_RELEASE_LIMIT_STEPS) {
            hardware_step_pulse(STEP_Z, 600);
            ++clear_steps;
            if ((clear_steps & 0x3FU) == 0U) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        for (int32_t i = 0; i < 400; ++i) {
            hardware_step_pulse(STEP_Z, 600);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // Cálculo dinâmico dos passos correspondentes a 5.0 mm de elevação de segurança do eixo Z:
    uint16_t teeth = ctx->settings.z_pulley_teeth ? ctx->settings.z_pulley_teeth : DEFAULT_Z_PULLEY_TEETH;
    float mm_per_rev = (float)teeth * Z_BELT_PITCH_MM;
    if (mm_per_rev <= 0.0f) {
        mm_per_rev = 32.0f;
    }
    uint32_t spr_z = ctx->settings.steps_per_rev[AXIS_Z_ID] ? ctx->settings.steps_per_rev[AXIS_Z_ID] : 200U;
    uint32_t usteps_z = ctx->settings.tmc_microsteps[AXIS_Z_ID] ? ctx->settings.tmc_microsteps[AXIS_Z_ID] : 16U;
    int32_t lift_5mm_steps = (int32_t)lroundf((5.0f * (float)(spr_z * usteps_z)) / mm_per_rev);
    if (lift_5mm_steps < 200) {
        lift_5mm_steps = 500;
    }

    // 3 ciclos de testagem do fim de curso Z com velocidades decrescentes para precisão máxima:
    // Ciclo 1: Busca rápida (450 us) e recuo de 600 passos (~6 mm)
    // Ciclo 2: Busca intermediária (650 us) e recuo de 400 passos (~4 mm)
    // Ciclo 3: Busca lenta de precisão (900 us) e elevação final de 5.0 mm de segurança para evitar colisão ao descer
    const uint32_t approach_delays[3] = { 450, 650, 900 };
    const int32_t search_limits[3] = { Z_HOME_SEARCH_LIMIT_STEPS, 4000, 2500 };
    const int32_t extra_backoffs[3] = { 600, 400, lift_5mm_steps };

    for (int cycle = 0; cycle < 3; ++cycle) {
        gpio_set_level(DIR_Z, dir_down);
        esp_rom_delay_us(5);
        int32_t search_steps = 0;
        while (!hardware_is_z_switch_pressed() && search_steps < search_limits[cycle]) {
            hardware_step_pulse(STEP_Z, approach_delays[cycle]);
            ++search_steps;
            if ((search_steps & 0x3FU) == 0U) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        if (search_steps >= search_limits[cycle] && !hardware_is_z_switch_pressed()) {
            ESP_LOGE(APP_TAG, "Home Z falhou no teste %d/3: switch nao acionado", cycle + 1);
            ctx->state.em_homing_z = false;
            ctx->state.z_bloqueado = true;
            hardware_rmt_reacquire_pin(STEP_Z);
            xSemaphoreGive(ctx->motion_mutex);
            return ESP_ERR_TIMEOUT;
        }

        gpio_set_level(DIR_Z, dir_up);
        esp_rom_delay_us(5);
        int32_t release_steps = 0;
        while (hardware_is_z_switch_pressed() && release_steps < Z_HOME_RELEASE_LIMIT_STEPS) {
            hardware_step_pulse(STEP_Z, 600);
            ++release_steps;
            if ((release_steps & 0x3FU) == 0U) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        if (release_steps >= Z_HOME_RELEASE_LIMIT_STEPS && hardware_is_z_switch_pressed()) {
            ESP_LOGE(APP_TAG, "Home Z falhou no teste %d/3: switch nao liberou", cycle + 1);
            ctx->state.em_homing_z = false;
            ctx->state.z_bloqueado = true;
            hardware_rmt_reacquire_pin(STEP_Z);
            xSemaphoreGive(ctx->motion_mutex);
            return ESP_ERR_TIMEOUT;
        }

        for (int32_t i = 0; i < extra_backoffs[cycle]; ++i) {
            hardware_step_pulse(STEP_Z, 600);
        }

        ESP_LOGI(APP_TAG, "Home Z teste %d/3 concluido com sucesso.", cycle + 1);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    ctx->state.atual_z = 0;
    ctx->state.z_bloqueado = false;
    ctx->state.em_homing_z = false;
    hardware_rmt_reacquire_pin(STEP_Z);
    xSemaphoreGive(ctx->motion_mutex);
    ESP_LOGI(APP_TAG, "Home Z finalizado com 3 testagens de fim de curso e elevacao de seguranca de 5.0 mm (%ld passos).", (long)lift_5mm_steps);
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
        if (requested_steps > 0) {
            if (ctx->state.atual_z + steps > (int32_t)ctx->settings.max_passos_z) {
                steps = ((int32_t)ctx->settings.max_passos_z > ctx->state.atual_z) ?
                        ((int32_t)ctx->settings.max_passos_z - ctx->state.atual_z) : 0;
            }
        } else {
            if (ctx->state.atual_z - steps < 0) {
                steps = (ctx->state.atual_z > 0) ? ctx->state.atual_z : 0;
            }
        }
        if (steps == 0) {
            hardware_rmt_reacquire_pin(STEP_Z);
            xSemaphoreGive(ctx->motion_mutex);
            return ESP_OK;
        }

        gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);
        esp_rom_delay_us(5);
        ESP_LOGI(APP_TAG, "MOVE_F Z steps=%d dir_pin=%d (Z_DIR_UP=%d Z_DIR_DOWN=%d)",
                 (int)steps,
                 (int)(move_up ? Z_DIR_UP : Z_DIR_DOWN),
                 (int)Z_DIR_UP, (int)Z_DIR_DOWN);

        uint32_t target_delay = 0;
        uint32_t start_delay = 0;
        uint32_t z_ramp = 0;
        compute_z_motion_profile(ctx, steps, speed_override, accel_override,
                                 &start_delay, &target_delay, &z_ramp);

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

    const float MIN_STEP_DELTA_DEG = deg_per_step * 2.0f; // Minimal threshold (~0.22 deg)

    // Direct comparison against exact configured limits
    if (requested_steps > 0) {
        // Moving in positive direction
        if (actual_deg >= (max_limit_deg - MIN_STEP_DELTA_DEG)) {
            return 0; // Already at max limit threshold
        }
        float requested_delta_deg = (float)requested_steps * deg_per_step;
        float target_deg = actual_deg + requested_delta_deg;
        if (target_deg > max_limit_deg) {
            target_deg = max_limit_deg;
        }
        float permitted_delta_deg = target_deg - actual_deg;
        if (permitted_delta_deg < MIN_STEP_DELTA_DEG) {
            return 0;
        }
        int32_t permitted_steps = (int32_t)floorf(permitted_delta_deg / deg_per_step);
        if (permitted_steps > requested_steps) {
            permitted_steps = requested_steps;
        }
        return permitted_steps;
    } else {
        // Moving in negative direction
        if (actual_deg <= (min_limit_deg + MIN_STEP_DELTA_DEG)) {
            return 0; // Already at min limit threshold
        }
        float requested_delta_deg = (float)requested_steps * deg_per_step; // negative
        float target_deg = actual_deg + requested_delta_deg;
        if (target_deg < min_limit_deg) {
            target_deg = min_limit_deg;
        }
        float permitted_delta_deg = target_deg - actual_deg; // negative
        if (permitted_delta_deg > -MIN_STEP_DELTA_DEG) {
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
        // Drain any consecutive queued moves in the same blocked direction
        motion_cmd_t pending;
        while (ctx->motion_queue && xQueuePeek(ctx->motion_queue, &pending, 0) == pdTRUE) {
            if ((pending.type == MOTION_CMD_MOVE_REL || pending.type == MOTION_CMD_MOVE_FORCE) &&
                pending.axis == axis_upper &&
                ((pending.steps > 0 && requested_steps > 0) || (pending.steps < 0 && requested_steps < 0))) {
                motion_cmd_t discarded;
                xQueueReceive(ctx->motion_queue, &discarded, 0);
            } else {
                break;
            }
        }
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
    if (requested_steps > 0) {
        if (ctx->state.atual_z + steps > (int32_t)ctx->settings.max_passos_z) {
            steps = ((int32_t)ctx->settings.max_passos_z > ctx->state.atual_z) ?
                    ((int32_t)ctx->settings.max_passos_z - ctx->state.atual_z) : 0;
            ESP_LOGW(APP_TAG, "MOVE Z ajustado por limite superior: %ld passos", (long)steps);
        }
    } else {
        if (ctx->state.atual_z - steps < 0) {
            steps = (ctx->state.atual_z > 0) ? ctx->state.atual_z : 0;
            ESP_LOGW(APP_TAG, "MOVE Z ajustado por limite inferior: %ld passos", (long)steps);
        }
    }
    if (steps == 0) {
        hardware_rmt_reacquire_pin(STEP_Z);
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_OK;
    }

    gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);
    esp_rom_delay_us(2);
    ESP_LOGI(APP_TAG, "MOVE Z steps=%d dir_pin=%d (Z_DIR_UP=%d Z_DIR_DOWN=%d)",
             (int)steps,
             (int)(move_up ? Z_DIR_UP : Z_DIR_DOWN),
             (int)Z_DIR_UP, (int)Z_DIR_DOWN);

    uint32_t target_delay = 0;
    uint32_t start_delay = 0;
    uint32_t z_ramp = 0;
    compute_z_motion_profile(ctx, steps, speed_override, accel_override,
                             &start_delay, &target_delay, &z_ramp);

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

static esp_err_t do_motion_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z,
                                     float speed_c_override, float speed_a_override, float speed_z_override,
                                     float accel_override, bool force_no_encoder)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    if (steps_c == 0 && steps_a == 0 && steps_z == 0) {
        return ESP_OK;
    }

    if (!force_no_encoder) {
        // Closed loop limits check for Axis C
        if (steps_c != 0) {
            float actual_deg_c = 0.0f;
            esp_err_t enc_err_c = hardware_read_axis_encoder('C', &actual_deg_c);
            if (enc_err_c != ESP_OK) {
                ESP_LOGE(APP_TAG, "MOVE_SYNC rejeitado: encoder C indisponivel (%s). Use MOVE_SYNC_F para forçar.",
                         esp_err_to_name(enc_err_c));
                return enc_err_c;
            }
            float deg_per_step_c = get_deg_per_step(ctx, 'C');
            int32_t planned_c = motion_plan_limited_steps(actual_deg_c, ctx->settings.home_c_deg,
                                                          ctx->settings.limit_min_c_deg, ctx->settings.limit_max_c_deg,
                                                          deg_per_step_c, steps_c);
            if (planned_c != steps_c) {
                ESP_LOGW(APP_TAG, "MOVE_SYNC C ajustado por limite: pedido=%ld planned=%ld (pos=%.2f limites=[%.2f, %.2f])",
                         (long)steps_c, (long)planned_c, actual_deg_c, ctx->settings.limit_min_c_deg, ctx->settings.limit_max_c_deg);
            }
            steps_c = planned_c;
        }

        // Closed loop limits check for Axis A
        if (steps_a != 0) {
            float actual_deg_a = 0.0f;
            esp_err_t enc_err_a = hardware_read_axis_encoder('A', &actual_deg_a);
            if (enc_err_a != ESP_OK) {
                ESP_LOGE(APP_TAG, "MOVE_SYNC rejeitado: encoder A indisponivel (%s). Use MOVE_SYNC_F para forçar.",
                         esp_err_to_name(enc_err_a));
                return enc_err_a;
            }
            float deg_per_step_a = get_deg_per_step(ctx, 'A');
            int32_t planned_a = motion_plan_limited_steps(actual_deg_a, ctx->settings.home_a_deg,
                                                          ctx->settings.limit_min_a_deg, ctx->settings.limit_max_a_deg,
                                                          deg_per_step_a, steps_a);
            if (planned_a != steps_a) {
                ESP_LOGW(APP_TAG, "MOVE_SYNC A ajustado por limite: pedido=%ld planned=%ld (pos=%.2f limites=[%.2f, %.2f])",
                         (long)steps_a, (long)planned_a, actual_deg_a, ctx->settings.limit_min_a_deg, ctx->settings.limit_max_a_deg);
            }
            steps_a = planned_a;
        }

        // Limits check for Axis Z
        if (steps_z != 0) {
            if (ctx->state.z_bloqueado) {
                ESP_LOGW(APP_TAG, "MOVE_SYNC Z bloqueado por seguranca");
                return ESP_ERR_INVALID_STATE;
            }
            int32_t cur_z = ctx->state.atual_z;
            int32_t max_z = (int32_t)ctx->settings.max_passos_z;
            if (steps_z > 0) {
                if (cur_z + steps_z > max_z) {
                    int32_t allowed_z = (max_z > cur_z) ? (max_z - cur_z) : 0;
                    ESP_LOGW(APP_TAG, "MOVE_SYNC Z ajustado por limite: pedido=%ld planned=%ld", (long)steps_z, (long)allowed_z);
                    steps_z = allowed_z;
                }
            } else if (steps_z < 0) {
                if (cur_z + steps_z < 0) {
                    int32_t allowed_z = (cur_z > 0) ? -cur_z : 0;
                    ESP_LOGW(APP_TAG, "MOVE_SYNC Z ajustado por limite: pedido=%ld planned=%ld", (long)steps_z, (long)allowed_z);
                    steps_z = allowed_z;
                }
            }
        }
    } else {
        if (steps_z != 0 && ctx->state.z_bloqueado) {
            ESP_LOGW(APP_TAG, "MOVE_SYNC Z bloqueado por seguranca");
            return ESP_ERR_INVALID_STATE;
        }
    }

    if (steps_c == 0 && steps_a == 0 && steps_z == 0) {
        ESP_LOGW(APP_TAG, "MOVE_SYNC ignorado: todos os eixos estao nos limites");
        return ESP_OK;
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
    float max_accel_z = (ctx->settings.accel_max[AXIS_Z_ID] > 10.0f) ?
                        ctx->settings.accel_max[AXIS_Z_ID] : DEFAULT_ACCEL_MAX_MM_S2_Z;
    if (accel_z > max_accel_z) accel_z = max_accel_z;

    // Speeds individual per axis
    float speed_c = (speed_c_override > 0.0f) ? speed_c_override :
                    (ctx->settings.speed[AXIS_C_ID] > 0.0f ? ctx->settings.speed[AXIS_C_ID] :
                    (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_C_ID]) * step_size_c));
    float speed_a = (speed_a_override > 0.0f) ? speed_a_override :
                    (ctx->settings.speed[AXIS_A_ID] > 0.0f ? ctx->settings.speed[AXIS_A_ID] :
                    (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_A_ID]) * step_size_a));
    float speed_z = (speed_z_override > 0.0f) ? speed_z_override :
                    (ctx->settings.speed[AXIS_Z_ID] > 0.0f ? ctx->settings.speed[AXIS_Z_ID] :
                    (1000000.0f / (2.0f * (float)ctx->state.speed_delay_us[AXIS_Z_ID]) * step_size_z));
    if (speed_c < 1.0f) speed_c = 1.0f;
    if (speed_a < 1.0f) speed_a = 1.0f;
    if (speed_z < 1.0f) speed_z = 1.0f;
    float max_speed_z = (ctx->settings.speed_max[AXIS_Z_ID] > 0.1f) ?
                        ctx->settings.speed_max[AXIS_Z_ID] : DEFAULT_SPEED_MAX_MM_S_Z;
    if (speed_z > max_speed_z) speed_z = max_speed_z;

    // Nominal target frequencies per axis
    float nom_freq_c = speed_c / step_size_c;
    float nom_freq_a = speed_a / step_size_a;
    float nom_freq_z = speed_z / step_size_z;
    if (nom_freq_c < (float)RMT_MIN_STEP_FREQ_HZ) nom_freq_c = (float)RMT_MIN_STEP_FREQ_HZ;
    if (nom_freq_a < (float)RMT_MIN_STEP_FREQ_HZ) nom_freq_a = (float)RMT_MIN_STEP_FREQ_HZ;
    if (nom_freq_z < (float)RMT_MIN_STEP_FREQ_HZ) nom_freq_z = (float)RMT_MIN_STEP_FREQ_HZ;
    if (nom_freq_c > (float)RMT_MAX_STEP_FREQ_HZ) nom_freq_c = (float)RMT_MAX_STEP_FREQ_HZ;
    if (nom_freq_a > (float)RMT_MAX_STEP_FREQ_HZ) nom_freq_a = (float)RMT_MAX_STEP_FREQ_HZ;
    if (nom_freq_z > (float)RMT_MAX_STEP_FREQ_HZ) nom_freq_z = (float)RMT_MAX_STEP_FREQ_HZ;

    // Coordinated Total Trajectory Time (dominant/slowest active axis determines T)
    float time_c = (abs_c > 0) ? ((float)abs_c / nom_freq_c) : 0.0f;
    float time_a = (abs_a > 0) ? ((float)abs_a / nom_freq_a) : 0.0f;
    float time_z = (abs_z > 0) ? ((float)abs_z / nom_freq_z) : 0.0f;

    float t_max = time_c;
    char dom_axis = 'C';
    if (time_a > t_max) { t_max = time_a; dom_axis = 'A'; }
    if (time_z > t_max) { t_max = time_z; dom_axis = 'Z'; }
    if (t_max < 0.01f) t_max = 0.01f;

    // Proportional Coordinated Target Frequencies so all active axes complete in t_max
    uint32_t target_freq_c = 0U;
    uint32_t target_freq_a = 0U;
    uint32_t target_freq_z = 0U;

    if (abs_c > 0) {
        float f_c = (float)abs_c / t_max;
        if (f_c < (float)RMT_MIN_STEP_FREQ_HZ) f_c = (float)RMT_MIN_STEP_FREQ_HZ;
        if (f_c > nom_freq_c) f_c = nom_freq_c;
        target_freq_c = (uint32_t)(f_c + 0.5f);
    }
    if (abs_a > 0) {
        float f_a = (float)abs_a / t_max;
        if (f_a < (float)RMT_MIN_STEP_FREQ_HZ) f_a = (float)RMT_MIN_STEP_FREQ_HZ;
        if (f_a > nom_freq_a) f_a = nom_freq_a;
        target_freq_a = (uint32_t)(f_a + 0.5f);
    }
    if (abs_z > 0) {
        float f_z = (float)abs_z / t_max;
        if (f_z < (float)RMT_MIN_STEP_FREQ_HZ) f_z = (float)RMT_MIN_STEP_FREQ_HZ;
        if (f_z > nom_freq_z) f_z = nom_freq_z;
        target_freq_z = (uint32_t)(f_z + 0.5f);
    }

    // Dominant axis parameters for proportional ramp scaling
    uint32_t dom_target_freq = (dom_axis == 'C') ? target_freq_c : (dom_axis == 'A' ? target_freq_a : target_freq_z);
    uint32_t dom_steps = (dom_axis == 'C') ? abs_c : (dom_axis == 'A' ? abs_a : abs_z);
    float dom_accel = (dom_axis == 'C') ? accel_c : (dom_axis == 'A' ? accel_a : accel_z);
    float dom_step_size = (dom_axis == 'C') ? step_size_c : (dom_axis == 'A' ? step_size_a : step_size_z);

    // Dominant axis ramp
    uint32_t dom_start_freq = (dom_target_freq > 300U) ? (dom_target_freq / 2U) : (dom_target_freq * 2U / 3U);
    if (dom_start_freq < 25U) dom_start_freq = 25U;
    if (dom_start_freq >= dom_target_freq) dom_start_freq = (dom_target_freq > 30U) ? (dom_target_freq - 15U) : (dom_target_freq / 2U);

    float dom_ramp_f = ((float)dom_target_freq * (float)dom_target_freq - (float)dom_start_freq * (float)dom_start_freq) /
                       (2.0f * (dom_accel / dom_step_size));
    uint32_t dom_ramp = (uint32_t)(dom_ramp_f + 0.5f);
    if (dom_ramp > dom_steps / 3U) dom_ramp = dom_steps / 3U;

    float ramp_ratio = (dom_steps > 0) ? ((float)dom_ramp / (float)dom_steps) : 0.0f;
    float start_freq_ratio = (dom_target_freq > 0) ? ((float)dom_start_freq / (float)dom_target_freq) : 0.5f;

    // Scale ramps for all active axes proportionally
    uint32_t start_freq_c = 0U, ramp_c = 0U;
    uint32_t start_freq_a = 0U, ramp_a = 0U;
    uint32_t start_freq_z = 0U, ramp_z = 0U;

    if (abs_c > 0) {
        start_freq_c = (uint32_t)((float)target_freq_c * start_freq_ratio + 0.5f);
        if (start_freq_c < 16U) start_freq_c = 16U;
        if (start_freq_c >= target_freq_c) start_freq_c = target_freq_c / 2U;
        ramp_c = (uint32_t)((float)abs_c * ramp_ratio + 0.5f);
        if (ramp_c > abs_c / 3U) ramp_c = abs_c / 3U;
    }
    if (abs_a > 0) {
        start_freq_a = (uint32_t)((float)target_freq_a * start_freq_ratio + 0.5f);
        if (start_freq_a < 16U) start_freq_a = 16U;
        if (start_freq_a >= target_freq_a) start_freq_a = target_freq_a / 2U;
        ramp_a = (uint32_t)((float)abs_a * ramp_ratio + 0.5f);
        if (ramp_a > abs_a / 3U) ramp_a = abs_a / 3U;
    }
    if (abs_z > 0) {
        start_freq_z = (uint32_t)((float)target_freq_z * start_freq_ratio + 0.5f);
        if (start_freq_z < 16U) start_freq_z = 16U;
        if (start_freq_z >= target_freq_z) start_freq_z = target_freq_z / 2U;
        ramp_z = (uint32_t)((float)abs_z * ramp_ratio + 0.5f);
        if (ramp_z > abs_z / 3U) ramp_z = abs_z / 3U;
    }

    ESP_LOGI(APP_TAG, "MOVE_SYNC Coordinated T=%.2fs (Dom=%c): C=%lu@%luHz A=%lu@%luHz Z=%lu@%luHz force=%d",
             t_max, dom_axis,
             (unsigned long)abs_c, (unsigned long)target_freq_c,
             (unsigned long)abs_a, (unsigned long)target_freq_a,
             (unsigned long)abs_z, (unsigned long)target_freq_z,
             (int)force_no_encoder);
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
            bool is_homing = (cmd.type == MOTION_CMD_HOME);

            if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ctx->state.in_motion = true;
                ctx->state.in_homing = is_homing;
                xSemaphoreGive(ctx->state_mutex);
            }

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
                err = do_motion_move_sync(ctx, cmd.steps_c, cmd.steps_a, cmd.steps_z,
                                          cmd.speed_c, cmd.speed_a, cmd.speed_z,
                                          cmd.accel_override, cmd.force_no_encoder);
                break;
            case MOTION_CMD_HOME:
                if (cmd.axis == 'Z') {
                    err = do_motion_home_z(ctx);
                } else {
                    err = do_motion_adjust_axis_to_home(ctx, cmd.axis);
                }
                break;
            }

            if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ctx->state.in_motion = false;
                ctx->state.in_homing = false;
                if (err == ESP_OK && cmd.type == MOTION_CMD_HOME) {
                    char ax = (char)toupper((unsigned char)cmd.axis);
                    if (ax == 'C' || ax == 'X') {
                        ctx->state.homed[0] = true;
                    } else if (ax == 'A' || ax == 'Y') {
                        ctx->state.homed[1] = true;
                    } else if (ax == 'Z') {
                        ctx->state.homed[2] = true;
                    }
                    printf("Home %c finalizado.\n", ax);
                }
                xSemaphoreGive(ctx->state_mutex);
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
    // Homing tem prioridade imediata: descarta movimentos antigos pendentes na fila
    if (cmd->type == MOTION_CMD_HOME) {
        xQueueReset(ctx->motion_queue);
    }
    // If the new command reverses direction on the same axis, purge stale opposing moves in queue
    if (cmd->type == MOTION_CMD_MOVE_REL || cmd->type == MOTION_CMD_MOVE_FORCE) {
        motion_cmd_t peek_cmd;
        UBaseType_t count = uxQueueMessagesWaiting(ctx->motion_queue);
        for (UBaseType_t i = 0; i < count; i++) {
            if (xQueuePeek(ctx->motion_queue, &peek_cmd, 0) == pdTRUE) {
                if ((peek_cmd.type == MOTION_CMD_MOVE_REL || peek_cmd.type == MOTION_CMD_MOVE_FORCE) &&
                    peek_cmd.axis == cmd->axis &&
                    ((peek_cmd.steps > 0 && cmd->steps < 0) || (peek_cmd.steps < 0 && cmd->steps > 0))) {
                    motion_cmd_t discarded;
                    xQueueReceive(ctx->motion_queue, &discarded, 0);
                } else {
                    break;
                }
            }
        }
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

esp_err_t motion_post_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z,
                                float speed_c, float speed_a, float speed_z, float accel,
                                bool force_no_encoder, uint8_t sender_id, uint8_t opcode)
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
        .speed_override = -1.0f,
        .accel_override = accel,
        .speed_c = speed_c,
        .speed_a = speed_a,
        .speed_z = speed_z,
        .force_no_encoder = force_no_encoder
    };
    return enqueue_motion_cmd(ctx, &cmd);
}
