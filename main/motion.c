#include "motion.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
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
static void compute_axis_rmt_profile(float step_size, float speed, float accel, float start_speed_cfg, uint32_t total_steps,
                                     uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                     uint32_t *ramp_steps);

// Geracao de STOP: incrementada por motion_request_stop(). Cada comando carrega a
// geracao vigente no enfileiramento; se divergir durante a execucao, e abortado.
static volatile uint32_t s_stop_gen = 0U;
static volatile uint32_t s_active_gen = 0U;

static bool motion_abort_hook(void)
{
    return s_active_gen != s_stop_gen;
}

static void jog_cmd_discarded(void);

static void notify_discarded_cmd(app_context_t *ctx, const motion_cmd_t *cmd, bool as_error)
{
    if (cmd->type == MOTION_CMD_JOG) {
        jog_cmd_discarded(); // STOP/HOME esvaziou a fila: o proximo JOG enfileira de novo
    }
    if (cmd->opcode == 0U || !ctx->state.can_online) {
        return;
    }
    // Comandos de streaming rapido (MOVE_UNIFIED) descartados por atualizacao mais recente nao geram flood no CAN
    if (cmd->opcode == CAN_OP_MOVE_UNIFIED && !as_error) {
        return;
    }
    (void)can_send_event(ctx, as_error ? CAN_EVT_ERROR : CAN_EVT_DONE, cmd->opcode,
                         (uint8_t)(ESP_ERR_NOT_FINISHED & 0xFF));
}

/*
 * Laços bit-bang ocupam o core 1: cede CPU ao idle (task WDT de 5 s) sem inserir
 * pausas perceptiveis em alta velocidade — so pausa com periodo de passo longo
 * (>= 0,8 ms) ou, como ultimo recurso, a cada 3 s.
 */
static void bitbang_maybe_yield(int64_t *last_yield_us, uint32_t half_period_us)
{
    int64_t since = esp_timer_get_time() - *last_yield_us;
    if ((since > 250000 && half_period_us >= 400U) || since > 3000000) {
        vTaskDelay(1);
        *last_yield_us = esp_timer_get_time();
    }
}

static char axis_char_from_index(size_t idx)
{
    return (idx == AXIS_C_ID) ? 'C' : ((idx == AXIS_A_ID) ? 'A' : 'Z');
}

/* Velocidade nominal do eixo: override do comando ou valor em uso, limitada por SPEED_MAX. */
static float axis_nominal_speed(app_context_t *ctx, size_t idx, float override)
{
    float v = (override > 0.0f && isfinite(override)) ? override : ctx->state.speed[idx];
    if (!(v > 0.0f) || !isfinite(v)) {
        v = (ctx->settings.speed[idx] > 0.0f) ? ctx->settings.speed[idx]
                                               : motion_delay_us_to_speed(ctx, axis_char_from_index(idx),
                                                                          ctx->state.speed_delay_us[idx]);
    }
    float vmax = ctx->settings.speed_max[idx];
    if (isfinite(vmax) && vmax > 0.0f && v > vmax) {
        v = vmax;
    }
    return (v < 1.0f) ? 1.0f : v;
}

/* Aceleracao do eixo: override do comando ou valor configurado, limitada por ACCEL_MAX. */
static float axis_accel(app_context_t *ctx, size_t idx, float override)
{
    float a = (override > 0.0f && isfinite(override)) ? override : ctx->settings.accel[idx];
    float amax = ctx->settings.accel_max[idx];
    if (isfinite(amax) && amax > 0.0f && a > amax) {
        a = amax;
    }
    return (a >= 10.0f) ? a : 10.0f;
}

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

static void compute_axis_rmt_profile(float step_size, float speed, float accel, float start_speed_cfg, uint32_t total_steps,
                                     uint32_t *start_freq_hz, uint32_t *target_freq_hz,
                                     uint32_t *ramp_steps)
{
    float safe_step_size = (isfinite(step_size) && step_size > 0.0f) ? step_size : 1.0f;
    float safe_speed = (isfinite(speed) && speed > 0.0f) ? speed : safe_step_size;
    float safe_accel = (isfinite(accel) && accel > 0.0f) ? accel : safe_step_size;

    uint32_t nominal_target = (uint32_t)fmaxf(1.0f, floorf((safe_speed / safe_step_size) + 0.5f));
    uint32_t target = nominal_target;

    float safe_start_speed = (isfinite(start_speed_cfg) && start_speed_cfg >= 0.5f) ? start_speed_cfg : 10.0f;
    if (safe_start_speed > safe_speed * 0.8f) {
        safe_start_speed = safe_speed * 0.5f;
    }
    uint32_t nominal_start = (uint32_t)fmaxf(16.0f, floorf((safe_start_speed / safe_step_size) + 0.5f));
    uint32_t start = nominal_start;

    // Scale peak speed and start frequency for short displacements on C/A:
    if (total_steps <= 8) {
        // <= 0.9 deg (e.g. 0.5 deg = 4-5 steps)
        if (target > 180U) target = 180U; // max ~20 deg/s
        if (start > 50U) start = 50U;
    } else if (total_steps <= 24) {
        // <= 2.7 deg (e.g. 1.0 deg = 9 steps)
        if (target > 350U) target = 350U; // max ~39 deg/s
        if (start > 65U) start = 65U;
    } else if (total_steps <= 80) {
        // <= 9.0 deg (e.g. 5.0 deg = 44 steps)
        if (target > 700U) target = 700U; // max ~78 deg/s
        if (start > 80U) start = 80U;
    } else {
        // Large moves (> 9 deg: 15 deg, 45 deg, 90 deg)
        target = nominal_target;
        start = nominal_start;
    }

    if (start >= target) {
        start = (target > 30U) ? (target / 2U) : target;
    }

    uint32_t ramp = 0U;
    uint32_t ramp_needed = 0U;
    float accel_steps_per_s2 = safe_accel / safe_step_size;
    if (target > start) {
        float ramp_f = ((float)target * (float)target - (float)start * (float)start) /
                       (2.0f * accel_steps_per_s2);
        if (isfinite(ramp_f) && ramp_f > 0.0f) {
            ramp = (uint32_t)fmaxf(1.0f, floorf(ramp_f + 0.5f));
        }
    }
    ramp_needed = ramp;

    // For short moves, ensure ramp takes at least half or one third of move (triangular S-curve)
    if (total_steps <= 24) {
        ramp = total_steps / 2U;
    } else if (total_steps <= 80) {
        if (ramp < total_steps / 3U) ramp = total_steps / 3U;
    } else {
        // For large moves, ensure at least 40 ramp steps for a perceptible, silky S-curve swell
        if (ramp < 40U && total_steps > 120U) ramp = 40U;
    }

    if (ramp < 1U && total_steps > 1U) ramp = 1U;
    if (ramp > total_steps / 2U) ramp = total_steps / 2U;
    if (ramp > RMT_MAX_RAMP_SAMPLES) ramp = RMT_MAX_RAMP_SAMPLES;

    // Movimentos longos cuja rampa foi truncada (perfil triangular ou limite de amostras do
    // encoder RMT): reduz o pico para v = sqrt(v0^2 + 2*a*rampa), em vez de atingir o alvo
    // numa rampa mais curta — o que excederia a aceleracao configurada e arriscaria perda de passos.
    if (total_steps > 80U && ramp > 0U && ramp_needed > ramp) {
        float v_peak = sqrtf((float)start * (float)start + 2.0f * accel_steps_per_s2 * (float)ramp);
        if (isfinite(v_peak) && v_peak < (float)target) {
            uint32_t peak = (uint32_t)v_peak;
            target = (peak > start + 1U) ? peak : (start + 1U);
        }
    }

    *start_freq_hz = start;
    *target_freq_hz = target;
    *ramp_steps = ramp;
}

static void compute_z_motion_profile(app_context_t *ctx, int32_t steps, float speed_override, float accel_override,
                                     uint32_t *out_start_delay, uint32_t *out_target_delay, uint32_t *out_ramp_steps)
{
    float step_size = get_step_size(ctx, 'Z');
    if (step_size <= 0.0f) {
        step_size = 0.01f;
    }

    // 1. Accel: configuracao do usuario (ou override), limitada por ACCEL_MAX
    float accel = axis_accel(ctx, AXIS_Z_ID, accel_override);
    if (accel < 50.0f) accel = 50.0f;
    if (accel > 5000.0f) accel = 5000.0f;
    float accel_steps_per_s2 = accel / step_size;

    // 2. Cruise Speed: velocidade em uso (ou override), limitada por SPEED_MAX
    float nominal_speed = axis_nominal_speed(ctx, AXIS_Z_ID, speed_override);
    if (nominal_speed > 400.0f) nominal_speed = 400.0f;
    float max_speed_steps = nominal_speed / step_size;

    // 3. Start speed: user-configurable start speed from NVS (default 15.0 mm/s)
    float start_speed_mm = (ctx->settings.z_start_speed_mm >= 0.5f) ? ctx->settings.z_start_speed_mm : 15.0f;
    if (nominal_speed < start_speed_mm * 1.2f) {
        start_speed_mm = nominal_speed * 0.4f;
    }
    float start_speed_steps = start_speed_mm / step_size;
    if (start_speed_steps < 30.0f) start_speed_steps = 30.0f;

    // 4. Kinematic calculation: check if move is triangular (steps cannot reach nominal cruise speed)
    // Needed ramp steps to reach full cruise speed: s_ramp = (v_max^2 - v_start^2) / (2 * a)
    float v0_sq = start_speed_steps * start_speed_steps;
    float vmax_sq = max_speed_steps * max_speed_steps;
    float needed_ramp_steps = (vmax_sq - v0_sq) / (2.0f * accel_steps_per_s2);

    uint32_t total_steps = (uint32_t)abs(steps);
    uint32_t ramp_steps = 0;
    float actual_cruise_steps = max_speed_steps;

    if (total_steps <= 2) {
        ramp_steps = 1;
        actual_cruise_steps = start_speed_steps;
    } else if (needed_ramp_steps * 2.0f >= (float)total_steps) {
        // Triangular profile! Peak speed is dynamically matched to travel:
        // v_peak^2 = v0^2 + 2 * a * (total_steps / 2)
        ramp_steps = total_steps / 2;
        float v_peak_sq = v0_sq + 2.0f * accel_steps_per_s2 * (float)ramp_steps;
        if (v_peak_sq > v0_sq) {
            actual_cruise_steps = sqrtf(v_peak_sq);
        } else {
            actual_cruise_steps = start_speed_steps;
        }
    } else {
        // Trapezoidal profile: ramp to full cruise speed, then cruise
        ramp_steps = (uint32_t)floorf(needed_ramp_steps + 0.5f);
        if (ramp_steps < 1) ramp_steps = 1;
        if (ramp_steps > total_steps / 2) ramp_steps = total_steps / 2;
        actual_cruise_steps = max_speed_steps;
    }

    // Convert speeds (steps/s) to half-period delays (us):
    // delay_us = 1000000 / (2 * speed_steps_per_s)
    uint32_t start_delay = (uint32_t)(1000000.0f / (2.0f * start_speed_steps));
    uint32_t target_delay = (uint32_t)(1000000.0f / (2.0f * actual_cruise_steps));

    if (target_delay < 10U) target_delay = 10U;
    if (start_delay < target_delay) start_delay = target_delay;
    if (start_delay > 32767U) start_delay = 32767U;

    *out_start_delay = start_delay;
    *out_target_delay = target_delay;
    *out_ramp_steps = ramp_steps;
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
    bool aborted = false;

    while (fabsf(error_deg) > TOLERANCE_DEG && total_steps_executed < max_allowed_steps) {
        if (hardware_abort_requested()) {
            aborted = true;
            break;
        }
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

    if (aborted) {
        ESP_LOGW(APP_TAG, "Home %c interrompido por STOP (pos=%.2f deg)", axis_upper, actual_deg);
        return ESP_ERR_NOT_FINISHED;
    }
    if (fabsf(error_deg) <= TOLERANCE_DEG) {
        ESP_LOGI(APP_TAG, "Home %c concluido com precisao (pos=%.2f deg, erro=%.2f deg, passos=%lu)",
                 axis_upper, actual_deg, error_deg, (unsigned long)total_steps_executed);
        return ESP_OK;
    }
    // Fora da tolerancia (limite, inversao de sentido ou passos esgotados): nao marca o eixo como homed
    ESP_LOGW(APP_TAG, "Home %c finalizado com desvio (pos=%.2f deg, erro=%.2f deg, passos=%lu)",
             axis_upper, actual_deg, error_deg, (unsigned long)total_steps_executed);
    return ESP_ERR_INVALID_RESPONSE;
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
            if (hardware_abort_requested()) {
                goto aborted;
            }
            hardware_step_pulse(STEP_Z, 600);
            ++clear_steps;
            if ((clear_steps & 0x3FU) == 0U) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        for (int32_t i = 0; i < 400; ++i) {
            if (hardware_abort_requested()) {
                goto aborted;
            }
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
            if (hardware_abort_requested()) {
                goto aborted;
            }
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
            if (hardware_abort_requested()) {
                goto aborted;
            }
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
            if (hardware_abort_requested()) {
                goto aborted;
            }
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

aborted:
    // Posicao Z desconhecida apos interrupcao: exige novo HOME Z
    ctx->state.em_homing_z = false;
    hardware_rmt_reacquire_pin(STEP_Z);
    xSemaphoreGive(ctx->motion_mutex);
    ESP_LOGW(APP_TAG, "Home Z interrompido por STOP.");
    return ESP_ERR_NOT_FINISHED;
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
         float accel_val = axis_accel(ctx, axis_idx, accel_override);
         float speed_val = axis_nominal_speed(ctx, axis_idx, speed_override);

         float start_speed_cfg = (axis_upper == 'C' || axis_upper == 'X') ?
                                 ctx->settings.c_start_speed_deg : ctx->settings.a_start_speed_deg;
         if (start_speed_cfg < 0.5f) start_speed_cfg = 10.0f;

         uint32_t abs_steps = (uint32_t)labs(requested_steps);
         uint32_t start_freq_hz = 0U;
         uint32_t target_freq_hz = 0U;
         uint32_t ramp_steps = 0U;
         compute_axis_rmt_profile(step_size, speed_val, accel_val, start_speed_cfg, abs_steps,
                                  &start_freq_hz, &target_freq_hz, &ramp_steps);

         esp_err_t rmt_err = hardware_step_pulse_rmt_move(axis_upper, abs_steps, start_freq_hz, target_freq_hz, ramp_steps);
         ESP_LOGI(APP_TAG, "MOVE_F %c concluido: %s", axis_upper, esp_err_to_name(rmt_err));

         if (rmt_err == ESP_OK) {
             float deg_moved = (float)requested_steps * step_size;
             if (axis_upper == 'C' || axis_upper == 'X') {
                 ctx->state.pos_c_deg += deg_moved;
                 float enc = 0.0f;
                 if (hardware_read_axis_encoder('C', &enc) == ESP_OK && isfinite(enc)) {
                     ctx->state.pos_c_deg = enc;
                 }
             } else {
                 ctx->state.pos_a_deg += deg_moved;
                 float enc = 0.0f;
                 if (hardware_read_axis_encoder('A', &enc) == ESP_OK && isfinite(enc)) {
                     ctx->state.pos_a_deg = enc;
                 }
             }
         }

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

        bool aborted = false;
        int64_t last_yield_us = esp_timer_get_time();
        for (int32_t i = 0; i < steps; ++i) {
            if (hardware_abort_requested()) {
                aborted = true;
                break;
            }
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
            bitbang_maybe_yield(&last_yield_us, delay);
        }

        hardware_rmt_reacquire_pin(STEP_Z);
        xSemaphoreGive(ctx->motion_mutex);
        return aborted ? ESP_ERR_NOT_FINISHED : ESP_OK;
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
    (void)home_deg;

    float actual_deg = 0.0f;
    esp_err_t enc_err = hardware_read_axis_encoder(axis_upper, &actual_deg);
    if (enc_err != ESP_OK) {
        ESP_LOGE(APP_TAG, "MOVE %c rejeitado: encoder indisponivel (%s). Use MOVE_F somente se a malha aberta for intencional.",
                 axis_upper, esp_err_to_name(enc_err));
        return enc_err;
    }

    float deg_per_step = get_deg_per_step(ctx, axis_upper);
    float target_deg = (float)requested_steps * deg_per_step;
    if (target_deg > max_limit_deg) target_deg = max_limit_deg;
    if (target_deg < min_limit_deg) target_deg = min_limit_deg;
    float delta_deg = target_deg - actual_deg;
    int32_t planned_steps = (fabsf(delta_deg) < (deg_per_step * 0.5f)) ? 0 : (int32_t)lroundf(delta_deg / deg_per_step);
    if (planned_steps == 0) {
        ESP_LOGI(APP_TAG, "MOVE %c ja na posicao absoluta alvo: pos=%.2f target=%.2f",
                 axis_upper, actual_deg, target_deg);
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
    ESP_LOGI(APP_TAG, "MOVE %c alvo_passos=%ld delta_passos=%ld dir_pin=%d invert=%d delta=%.2f pos=%.2f limites=[%.2f, %.2f]",
             axis_upper, (long)requested_steps, (long)planned_steps,
             (int)(dir_level ? 1 : 0),
             (int)invert, permitted_move_deg, actual_deg, min_limit_deg, max_limit_deg);
    size_t axis_idx = axis_to_index(axis_upper);
    float step_size = get_step_size(ctx, axis_upper);
    float accel_val = axis_accel(ctx, axis_idx, accel_override);
    float speed_val = axis_nominal_speed(ctx, axis_idx, speed_override);

    float start_speed_cfg = (axis_upper == 'C' || axis_upper == 'X') ?
                            ctx->settings.c_start_speed_deg : ctx->settings.a_start_speed_deg;
    if (start_speed_cfg < 0.5f) start_speed_cfg = 10.0f;

    uint32_t start_freq_hz = 0U;
    uint32_t target_freq_hz = 0U;
    uint32_t ramp_steps = 0U;
    compute_axis_rmt_profile(step_size, speed_val, accel_val, start_speed_cfg, steps_to_execute,
                             &start_freq_hz, &target_freq_hz, &ramp_steps);

    esp_err_t rmt_err = hardware_step_pulse_rmt_move(axis_upper, steps_to_execute,
                                                     start_freq_hz, target_freq_hz, ramp_steps);

    if (rmt_err == ESP_OK) {
        if (axis_upper == 'C' || axis_upper == 'X') {
            ctx->state.pos_c_deg += permitted_move_deg;
            float enc = 0.0f;
            if (hardware_read_axis_encoder('C', &enc) == ESP_OK && isfinite(enc)) {
                ctx->state.pos_c_deg = enc;
            }
        } else {
            ctx->state.pos_a_deg += permitted_move_deg;
            float enc = 0.0f;
            if (hardware_read_axis_encoder('A', &enc) == ESP_OK && isfinite(enc)) {
                ctx->state.pos_a_deg = enc;
            }
        }
    }

    xSemaphoreGive(ctx->motion_mutex);
    return rmt_err;
}

static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps,
                                           float speed_override, float accel_override)
{
    int32_t target_z = requested_steps;
    if (target_z > (int32_t)ctx->settings.max_passos_z) target_z = (int32_t)ctx->settings.max_passos_z;
    if (target_z < 0) target_z = 0;
    int32_t delta_z = target_z - ctx->state.atual_z;

    if (delta_z == 0) {
        return ESP_OK;
    }
    if (ctx->state.z_bloqueado) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    hardware_rmt_release_pin(STEP_Z);

    bool move_up = delta_z > 0;
    if (ctx->state.inverter[AXIS_Z_ID]) {
        move_up = !move_up;
    }
    int32_t steps = labs(delta_z);
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

    bool aborted = false;
    int64_t last_yield_us = esp_timer_get_time();
    for (int32_t i = 0; i < steps; ++i) {
        if (hardware_abort_requested()) {
            aborted = true;
            break;
        }
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
        bitbang_maybe_yield(&last_yield_us, delay);
    }

    hardware_rmt_reacquire_pin(STEP_Z);
    xSemaphoreGive(ctx->motion_mutex);
    return aborted ? ESP_ERR_NOT_FINISHED : ESP_OK;
}

static esp_err_t do_motion_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z,
                                     float speed_c_override, float speed_a_override, float speed_z_override,
                                     float accel_override, bool force_no_encoder)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    float deg_per_step_c = get_deg_per_step(ctx, 'C');
    float deg_per_step_a = get_deg_per_step(ctx, 'A');

    // Closed loop limits e deltas para Eixo C (coordenadas absolutas)
    float actual_deg_c = ctx->state.pos_c_deg;
    esp_err_t enc_err_c = hardware_read_axis_encoder('C', &actual_deg_c);
    if (enc_err_c != ESP_OK && !force_no_encoder) {
        ESP_LOGE(APP_TAG, "MOVE_SYNC rejeitado: encoder C indisponivel (%s). Use MOVE_SYNC_F para forcar.",
                 esp_err_to_name(enc_err_c));
        return enc_err_c;
    }
    float target_deg_c = (float)steps_c * deg_per_step_c;
    if (!force_no_encoder) {
        if (target_deg_c > ctx->settings.limit_max_c_deg) target_deg_c = ctx->settings.limit_max_c_deg;
        if (target_deg_c < ctx->settings.limit_min_c_deg) target_deg_c = ctx->settings.limit_min_c_deg;
    }
    float delta_deg_c = target_deg_c - actual_deg_c;
    int32_t delta_steps_c = (fabsf(delta_deg_c) < (deg_per_step_c * 0.5f)) ? 0 : (int32_t)lroundf(delta_deg_c / deg_per_step_c);

    // Closed loop limits e deltas para Eixo A (coordenadas absolutas)
    float actual_deg_a = ctx->state.pos_a_deg;
    esp_err_t enc_err_a = hardware_read_axis_encoder('A', &actual_deg_a);
    if (enc_err_a != ESP_OK && !force_no_encoder) {
        ESP_LOGE(APP_TAG, "MOVE_SYNC rejeitado: encoder A indisponivel (%s). Use MOVE_SYNC_F para forcar.",
                 esp_err_to_name(enc_err_a));
        return enc_err_a;
    }
    float target_deg_a = (float)steps_a * deg_per_step_a;
    if (!force_no_encoder) {
        if (target_deg_a > ctx->settings.limit_max_a_deg) target_deg_a = ctx->settings.limit_max_a_deg;
        if (target_deg_a < ctx->settings.limit_min_a_deg) target_deg_a = ctx->settings.limit_min_a_deg;
    }
    float delta_deg_a = target_deg_a - actual_deg_a;
    int32_t delta_steps_a = (fabsf(delta_deg_a) < (deg_per_step_a * 0.5f)) ? 0 : (int32_t)lroundf(delta_deg_a / deg_per_step_a);

    // Limites e deltas para Eixo Z (coordenadas absolutas)
    int32_t target_z = steps_z;
    if (target_z > (int32_t)ctx->settings.max_passos_z) target_z = (int32_t)ctx->settings.max_passos_z;
    if (target_z < 0) target_z = 0;
    int32_t delta_steps_z = target_z - ctx->state.atual_z;
    if (delta_steps_z != 0 && ctx->state.z_bloqueado) {
        ESP_LOGW(APP_TAG, "MOVE_SYNC Z bloqueado por seguranca");
        return ESP_ERR_INVALID_STATE;
    }

    steps_c = delta_steps_c;
    steps_a = delta_steps_a;
    steps_z = delta_steps_z;

    if (steps_c == 0 && steps_a == 0 && steps_z == 0) {
        ESP_LOGI(APP_TAG, "MOVE_SYNC ja nas coordenadas absolutas alvo.");
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

    // Accel e velocidade por eixo (override do comando ou valor em uso), limitados por ACCEL_MAX/SPEED_MAX
    float accel_c = axis_accel(ctx, AXIS_C_ID, accel_override);
    float accel_a = axis_accel(ctx, AXIS_A_ID, accel_override);
    float accel_z = axis_accel(ctx, AXIS_Z_ID, accel_override);
    float speed_c = axis_nominal_speed(ctx, AXIS_C_ID, speed_c_override);
    float speed_a = axis_nominal_speed(ctx, AXIS_A_ID, speed_a_override);
    float speed_z = axis_nominal_speed(ctx, AXIS_Z_ID, speed_z_override);

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

    // Dominant axis ramp: parte da velocidade inicial configurada (RAMP C/A/Z), como nos
    // movimentos de eixo unico. Antes partia em target/2 — a 720 deg/s isso era um salto
    // instantaneo de 360 deg/s na partida e na parada.
    float dom_start_cfg = (dom_axis == 'C') ? ctx->settings.c_start_speed_deg :
                          ((dom_axis == 'A') ? ctx->settings.a_start_speed_deg : ctx->settings.z_start_speed_mm);
    if (!(dom_start_cfg >= 0.5f)) {
        dom_start_cfg = (dom_axis == 'Z') ? 15.0f : 10.0f;
    }
    uint32_t dom_start_freq = (uint32_t)(dom_start_cfg / dom_step_size);
    if (dom_start_freq < 25U) dom_start_freq = 25U;
    if (dom_start_freq >= dom_target_freq) dom_start_freq = (dom_target_freq > 30U) ? (dom_target_freq / 2U) : dom_target_freq;

    float dom_accel_steps = dom_accel / dom_step_size;
    float dom_ramp_f = ((float)dom_target_freq * (float)dom_target_freq - (float)dom_start_freq * (float)dom_start_freq) /
                       (2.0f * dom_accel_steps);
    uint32_t dom_ramp_limit = dom_steps / 3U;
    if (dom_ramp_limit > RMT_MAX_RAMP_SAMPLES) dom_ramp_limit = RMT_MAX_RAMP_SAMPLES;
    uint32_t dom_ramp = (uint32_t)(dom_ramp_f + 0.5f);
    if (dom_ramp > dom_ramp_limit) {
        dom_ramp = dom_ramp_limit;
        // Rampa truncada: reduz o pico do eixo dominante para respeitar a aceleracao e escala
        // os demais eixos na mesma proporcao (mantem a coordenacao de tempo entre eixos).
        float v_peak = sqrtf((float)dom_start_freq * (float)dom_start_freq + 2.0f * dom_accel_steps * (float)dom_ramp);
        if (isfinite(v_peak) && v_peak < (float)dom_target_freq && dom_target_freq > 0U) {
            float k = v_peak / (float)dom_target_freq;
            uint32_t *targets[AXIS_COUNT] = {&target_freq_c, &target_freq_a, &target_freq_z};
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                if (*targets[i] > 0U) {
                    uint32_t scaled = (uint32_t)((float)*targets[i] * k + 0.5f);
                    *targets[i] = (scaled < RMT_MIN_STEP_FREQ_HZ) ? RMT_MIN_STEP_FREQ_HZ : scaled;
                }
            }
            dom_target_freq = (dom_axis == 'C') ? target_freq_c : (dom_axis == 'A' ? target_freq_a : target_freq_z);
            if (dom_start_freq >= dom_target_freq) dom_start_freq = dom_target_freq / 2U;
        }
    }

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
    if (err == ESP_OK) {
        if (steps_z != 0) {
            ctx->state.atual_z += steps_z;
        }
        ctx->state.pos_c_deg += (float)steps_c * deg_per_step_c;
        ctx->state.pos_a_deg += (float)steps_a * deg_per_step_a;
        float enc_c = 0.0f, enc_a = 0.0f;
        if (hardware_read_axis_encoder('C', &enc_c) == ESP_OK && isfinite(enc_c)) {
            ctx->state.pos_c_deg = enc_c;
        }
        if (hardware_read_axis_encoder('A', &enc_a) == ESP_OK && isfinite(enc_a)) {
            ctx->state.pos_a_deg = enc_a;
        }
    } else if (err != ESP_OK && steps_z != 0) {
        // O RMT nao informa quantos passos sairam antes da interrupcao: posicao Z desconhecida
        ctx->state.homed[AXIS_Z_ID] = false;
        ESP_LOGW(APP_TAG, "MOVE_SYNC interrompido (%s): posicao Z incerta, execute HOME Z.", esp_err_to_name(err));
    }

    xSemaphoreGive(ctx->motion_mutex);
    return err;
}


/* ==================================================================================== */
/* Motor "stream" (padrao): perfis S-curve no dominio do tempo gerados no ISR do RMT,    */
/* tres canais sempre alinhados e encadeamento de movimentos (lookahead com jerk por     */
/* eixo, como as impressoras 3D). HOME continua no caminho de malha fechada/bit-bang.    */
/* ==================================================================================== */

#define STREAM_TICK_HZ 1000000U
#define STREAM_POOL_WORDS 6144U /* por plano; dois planos em voo = 48 KB */

static uint32_t s_stream_pool[2][STREAM_POOL_WORDS];
static mp_plan_t s_stream_plan[2];

typedef struct {
    motion_cmd_t cmd;
    mp_request_t req;
    bool empty;      /* bloqueado pelos limites: nada a executar (DONE imediato, como no legado) */
} stream_item_t;

typedef struct {
    uint8_t opcode;
    int32_t steps_c;
    int32_t steps_a;
    int32_t steps_z;
} stream_done_info_t;

typedef struct {
    float deg[2];     /* posicao prevista de C e A ao fim dos movimentos ja planejados */
    bool deg_valid[2];
    bool deg_read[2]; /* encoder ja consultado (leitura sob demanda: MOVE_F/Z nao dependem dele) */
    int32_t z;
} stream_prediction_t;

static bool motion_cmd_streamable(const motion_cmd_t *cmd)
{
    return cmd->type == MOTION_CMD_MOVE_REL || cmd->type == MOTION_CMD_MOVE_FORCE ||
           cmd->type == MOTION_CMD_MOVE_SYNC;
}

/* true se `cmd` precisa de um encoder ainda nao lido nesta cadeia. A leitura tem de ser feita
 * com os motores parados (no inicio de uma cadeia), nunca com movimentos em voo. */
static bool stream_needs_encoder_read(const motion_cmd_t *cmd, const stream_prediction_t *pred)
{
    if (cmd->type == MOTION_CMD_MOVE_FORCE || (cmd->type == MOTION_CMD_MOVE_SYNC && cmd->force_no_encoder)) {
        return false;
    }
    if (cmd->type == MOTION_CMD_MOVE_SYNC) {
        return (!pred->deg_read[AXIS_C_ID] || !pred->deg_read[AXIS_A_ID]);
    }
    size_t idx = axis_to_index(cmd->axis);
    if (idx <= AXIS_A_ID && !pred->deg_read[idx]) {
        return true;
    }
    return false;
}

static void stream_send_event(app_context_t *ctx, uint8_t opcode, esp_err_t err)
{
    if (opcode != 0U && ctx->state.can_online) {
        if (opcode == CAN_OP_MOVE_UNIFIED && err == ESP_OK) {
            return; // Streaming unificado de alta taxa nao inunda CAN com DONE
        }
        (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_DONE : CAN_EVT_ERROR, opcode, (uint8_t)err);
    }
}

static float axis_start_speed(app_context_t *ctx, size_t idx)
{
    float v = (idx == AXIS_C_ID) ? ctx->settings.c_start_speed_deg :
              (idx == AXIS_A_ID) ? ctx->settings.a_start_speed_deg : ctx->settings.z_start_speed_mm;
    return (v >= 0.5f) ? v : ((idx == AXIS_Z_ID) ? 15.0f : 10.0f);
}

/* Converte um comando em pedido de perfil com COORDENADAS ABSOLUTAS sobre a posicao PREVISTA. */
static esp_err_t stream_prepare(app_context_t *ctx, const motion_cmd_t *cmd, stream_prediction_t *pred,
                                stream_item_t *item)
{
    memset(item, 0, sizeof(*item));
    item->cmd = *cmd;

    int32_t target_steps[AXIS_COUNT] = {0, 0, 0};
    bool axis_active[AXIS_COUNT] = {false, false, false};
    float speed_ovr[AXIS_COUNT] = {-1.0f, -1.0f, -1.0f};
    float accel_ovr = cmd->accel_override;
    bool force = (cmd->type == MOTION_CMD_MOVE_FORCE) ||
                 (cmd->type == MOTION_CMD_MOVE_SYNC && cmd->force_no_encoder);

    if (cmd->type == MOTION_CMD_MOVE_SYNC) {
        target_steps[AXIS_C_ID] = cmd->steps_c;
        target_steps[AXIS_A_ID] = cmd->steps_a;
        target_steps[AXIS_Z_ID] = cmd->steps_z;
        axis_active[AXIS_C_ID] = true;
        axis_active[AXIS_A_ID] = true;
        axis_active[AXIS_Z_ID] = true;
        speed_ovr[AXIS_C_ID] = cmd->speed_c;
        speed_ovr[AXIS_A_ID] = cmd->speed_a;
        speed_ovr[AXIS_Z_ID] = cmd->speed_z;
    } else {
        size_t idx = axis_to_index(cmd->axis);
        target_steps[idx] = cmd->steps;
        axis_active[idx] = true;
        speed_ovr[idx] = cmd->speed_override;
    }

    int32_t steps[AXIS_COUNT] = {0, 0, 0};

    // Coordenadas absolutas angulares de C e A (sobre a posicao prevista)
    for (size_t i = AXIS_C_ID; i <= AXIS_A_ID; ++i) {
        if (!pred->deg_read[i]) {
            pred->deg_read[i] = true;
            pred->deg_valid[i] = (hardware_read_axis_encoder(axis_char_from_index(i), &pred->deg[i]) == ESP_OK);
        }
        if (!axis_active[i]) {
            continue;
        }
        if (!pred->deg_valid[i]) {
            if (!force) {
                ESP_LOGE(APP_TAG, "MOVE %c rejeitado: encoder indisponivel. Use MOVE_F para malha aberta.",
                         axis_char_from_index(i));
                return ESP_ERR_INVALID_RESPONSE;
            }
            pred->deg[i] = (i == AXIS_C_ID) ? ctx->state.pos_c_deg : ctx->state.pos_a_deg;
            pred->deg_valid[i] = true;
        }

        float deg_per_step = get_deg_per_step(ctx, axis_char_from_index(i));
        float target_deg = (float)target_steps[i] * deg_per_step;
        if (!force) {
            float min_deg = (i == AXIS_C_ID) ? ctx->settings.limit_min_c_deg : ctx->settings.limit_min_a_deg;
            float max_deg = (i == AXIS_C_ID) ? ctx->settings.limit_max_c_deg : ctx->settings.limit_max_a_deg;
            if (target_deg > max_deg) target_deg = max_deg;
            if (target_deg < min_deg) target_deg = min_deg;
        }

        float delta_deg = target_deg - pred->deg[i];
        if (fabsf(delta_deg) >= (deg_per_step * 0.5f)) {
            steps[i] = (int32_t)lroundf(delta_deg / deg_per_step);
            pred->deg[i] += (float)steps[i] * deg_per_step;
        } else {
            steps[i] = 0;
            pred->deg[i] = target_deg;
        }
    }

    // Coordenadas absolutas de Z (sobre a cota prevista)
    if (axis_active[AXIS_Z_ID]) {
        int32_t target_z = target_steps[AXIS_Z_ID];
        if (target_z > (int32_t)ctx->settings.max_passos_z) target_z = (int32_t)ctx->settings.max_passos_z;
        if (target_z < 0) target_z = 0;

        int32_t delta_z = target_z - pred->z;
        if (delta_z != 0 && ctx->state.z_bloqueado) {
            return ESP_ERR_INVALID_STATE;
        }
        steps[AXIS_Z_ID] = delta_z;
        pred->z = target_z;
    }

    if (steps[0] == 0 && steps[1] == 0 && steps[2] == 0) {
        item->empty = true;
        return ESP_OK;
    }

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        float step_size = get_step_size(ctx, axis_char_from_index(i));
        float speed = axis_nominal_speed(ctx, i, speed_ovr[i]);
        float accel = axis_accel(ctx, i, accel_ovr);
        if (i == AXIS_Z_ID) {
            if (speed > 400.0f) speed = 400.0f;
            if (accel < 50.0f) accel = 50.0f;
            if (accel > 5000.0f) accel = 5000.0f;
        }
        float vmax = speed / step_size;
        if (vmax > (float)RMT_MAX_STEP_FREQ_HZ) vmax = (float)RMT_MAX_STEP_FREQ_HZ;
        float vfloor = axis_start_speed(ctx, i) / step_size;

        // Suavizacao de micro-deslocamentos: elimina o "soquinho" no Z e rotativos
        uint32_t abs_steps = (uint32_t)abs(steps[i]);
        if (abs_steps > 0 && abs_steps < 48) {
            float v_reach = sqrtf(2.0f * (accel / step_size) * (float)abs_steps);
            if (vmax > v_reach) {
                vmax = v_reach;
            }
            float scale = (float)abs_steps / 48.0f;
            vfloor *= (scale * scale);
            if (vfloor < 10.0f) {
                vfloor = 10.0f;
            }
        }
        if (vfloor > vmax) vfloor = vmax;
        item->req.steps[i] = steps[i];
        item->req.vmax[i] = vmax;
        item->req.accel[i] = accel / step_size;
        item->req.v_floor[i] = vfloor;
    }
    return ESP_OK;
}

static void stream_dir_levels(app_context_t *ctx, const mp_request_t *req, int8_t dir[AXIS_COUNT])
{
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (req->steps[i] == 0) {
            dir[i] = -1;
            continue;
        }
        bool positive = req->steps[i] > 0;
        if (ctx->state.inverter[i]) {
            positive = !positive;
        }
        if (i == AXIS_Z_ID) {
            dir[i] = (int8_t)(positive ? Z_DIR_UP : Z_DIR_DOWN);
        } else {
            dir[i] = positive ? 1 : 0;
        }
    }
}

/* Movimentos concluidos no hardware: atualiza Z, C, A e emite DONE na ordem. */
static void stream_report_completed(app_context_t *ctx, stream_done_info_t info[2], uint32_t *reported)
{
    uint32_t completed = hardware_stream_completed();
    while (*reported < completed) {
        stream_done_info_t *d = &info[*reported % 2U];
        ctx->state.atual_z += d->steps_z;
        ctx->state.pos_c_deg += (float)d->steps_c * get_deg_per_step(ctx, 'C');
        ctx->state.pos_a_deg += (float)d->steps_a * get_deg_per_step(ctx, 'A');
        float enc_c = 0.0f, enc_a = 0.0f;
        if (hardware_read_axis_encoder('C', &enc_c) == ESP_OK && isfinite(enc_c)) {
            ctx->state.pos_c_deg = enc_c;
        }
        if (hardware_read_axis_encoder('A', &enc_a) == ESP_OK && isfinite(enc_a)) {
            ctx->state.pos_a_deg = enc_a;
        }
        stream_send_event(ctx, d->opcode, ESP_OK);
        (*reported)++;
    }
}

/* Executa `first` e, com lookahead, os comandos seguintes da fila sem parar entre eles.
 * Retorna o resultado do primeiro comando (ou o erro que interrompeu a cadeia). */
static esp_err_t motion_stream_chain(app_context_t *ctx, const motion_cmd_t *first)
{
    // Encoders sao lidos sob demanda em stream_prepare: MOVE_F e Z nao esperam por um encoder offline
    stream_prediction_t pred = {.z = ctx->state.atual_z};
    float jerk_steps[AXIS_COUNT];
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        jerk_steps[i] = ctx->ext.jerk[i] / get_step_size(ctx, axis_char_from_index(i));
    }

    stream_item_t cur, nxt;
    esp_err_t err = stream_prepare(ctx, first, &pred, &cur);
    if (err != ESP_OK || cur.empty) {
        stream_send_event(ctx, first->opcode, err);
        return err;
    }
    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        stream_send_event(ctx, first->opcode, ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }
    if (hardware_stream_begin() != ESP_OK) {
        xSemaphoreGive(ctx->motion_mutex);
        stream_send_event(ctx, first->opcode, ESP_ERR_INVALID_STATE);
        return ESP_FAIL;
    }

    stream_done_info_t info[2] = {0};
    uint32_t committed = 0, reported = 0, pool_idx = 0, chained = 0;
    float entry_rate = 0.0f;       // do repouso
    bool have_trailing = false;    // comando reservado que nao entrou na cadeia
    motion_cmd_t trailing = {0};
    esp_err_t trailing_err = ESP_OK;
    bool has_next = false;         // `nxt` reservado da fila (ja fora dela)
    bool cur_committed = false;    // `cur` ja foi entregue ao RMT

    while (true) {
        // Lookahead: reserva o proximo comando da fila para calcular a juncao
        has_next = false;
        cur_committed = false;
        float exit_rate = 0.0f, next_entry = 0.0f;
        motion_cmd_t peek;
        if (ctx->ext.lookahead && xQueuePeek(ctx->motion_queue, &peek, 0) == pdTRUE &&
            motion_cmd_streamable(&peek) && peek.stop_gen == s_stop_gen &&
            !stream_needs_encoder_read(&peek, &pred) &&
            xQueueReceive(ctx->motion_queue, &peek, 0) == pdTRUE) {
            stream_prediction_t pred_next = pred;
            esp_err_t perr = stream_prepare(ctx, &peek, &pred_next, &nxt);
            if (perr == ESP_OK && !nxt.empty) {
                pred = pred_next;
                has_next = true;
                if (!mp_junction(&cur.req, &nxt.req, jerk_steps, &exit_rate, &next_entry)) {
                    exit_rate = 0.0f;
                    next_entry = 0.0f;
                }
            } else {
                // Invalido ou vazio: encerra a cadeia apos o atual e reporta depois
                pred = pred_next;
                have_trailing = true;
                trailing = peek;
                trailing_err = perr;
            }
        }

        // No maximo 2 movimentos em voo: o pool deste plano (usado 2 movimentos atras) precisa estar livre
        err = hardware_stream_wait(committed > 0U ? committed - 1U : 0U);
        stream_report_completed(ctx, info, &reported);
        if (err != ESP_OK) {
            break;
        }

        cur.req.rate_entry = entry_rate;
        cur.req.rate_exit = exit_rate;
        mp_result_t pr = mp_plan_move(&cur.req, s_stream_pool[pool_idx], STREAM_POOL_WORDS, STREAM_TICK_HZ,
                                      &s_stream_plan[pool_idx]);
        if (pr != MP_OK) {
            ESP_LOGE(APP_TAG, "stream: falha ao planejar movimento (%d)", (int)pr);
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        const mp_plan_t *plan = &s_stream_plan[pool_idx];
        int8_t dir[AXIS_COUNT];
        stream_dir_levels(ctx, &cur.req, dir);
        err = hardware_stream_commit(plan, dir);
        if (err != ESP_OK) {
            break;
        }
        cur_committed = true;
        ESP_LOGD(APP_TAG, "stream: mov %lu dom=%u v=%.0f->%.0f->%.0f passos/s T=%.3fs",
                 (unsigned long)committed, plan->dom, plan->v_entry, plan->v_cruise, plan->v_exit, plan->duration_s);
        info[committed % 2U] = (stream_done_info_t){
            .opcode = cur.cmd.opcode,
            .steps_c = cur.req.steps[AXIS_C_ID],
            .steps_a = cur.req.steps[AXIS_A_ID],
            .steps_z = cur.req.steps[AXIS_Z_ID]
        };
        committed++;
        pool_idx ^= 1U;

        if (!has_next) {
            // Se o hardware ainda esta executando o movimento atual, aguarda ate 15 ms
            // para absorver o proximo comando de streaming continuo sem interromper o hardware stream:
            int64_t t0 = esp_timer_get_time();
            while (hardware_stream_completed() < committed && (esp_timer_get_time() - t0) < 15000LL) {
                if (xQueuePeek(ctx->motion_queue, &peek, 0) == pdTRUE &&
                    motion_cmd_streamable(&peek) && peek.stop_gen == s_stop_gen) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (xQueuePeek(ctx->motion_queue, &peek, 0) == pdTRUE &&
                motion_cmd_streamable(&peek) && peek.stop_gen == s_stop_gen &&
                xQueueReceive(ctx->motion_queue, &peek, 0) == pdTRUE) {
                stream_prediction_t pred_next = pred;
                esp_err_t perr = stream_prepare(ctx, &peek, &pred_next, &nxt);
                if (perr == ESP_OK && !nxt.empty) {
                    pred = pred_next;
                    entry_rate = 0.0f;
                    cur = nxt;
                    continue;
                } else {
                    pred = pred_next;
                    have_trailing = true;
                    trailing = peek;
                    trailing_err = perr;
                }
            }
            break;
        }
        // A saida efetiva pode ter sido limitada pelo planejador: entrada do proximo na mesma proporcao
        next_entry = (exit_rate > 0.0f) ? next_entry * (plan->rate_exit / exit_rate) : 0.0f;
        if (exit_rate > 0.0f) {
            chained++;
        }
        entry_rate = next_entry;
        cur = nxt;
    }

    if (err == ESP_OK) {
        err = hardware_stream_wait(committed);
    }
    stream_report_completed(ctx, info, &reported);
    if (err != ESP_OK) {
        // STOP ou falha: movimentos em voo nao terminaram; posicao do Z incerta se havia Z
        for (uint32_t k = reported; k < committed; ++k) {
            if (info[k % 2U].steps_z != 0) {
                ctx->state.homed[AXIS_Z_ID] = false;
                ESP_LOGW(APP_TAG, "Movimento com Z interrompido: posicao Z incerta, execute HOME Z.");
            }
            stream_send_event(ctx, info[k % 2U].opcode, err);
        }
        // Comandos que nao chegaram ao RMT (STOP durante a espera do pool)
        if (!cur_committed) {
            stream_send_event(ctx, cur.cmd.opcode, err);
        }
        if (has_next) {
            stream_send_event(ctx, nxt.cmd.opcode, err);
        }
    }
    hardware_stream_end();
    xSemaphoreGive(ctx->motion_mutex);

    if (have_trailing) {
        // Comando reservado que nao entrou na cadeia: vazio (DONE) ou invalido (ERROR)
        stream_send_event(ctx, trailing.opcode, (err == ESP_OK) ? trailing_err : ESP_ERR_NOT_FINISHED);
    }
    if (chained > 0U) {
        ESP_LOGI(APP_TAG, "stream: %lu movimento(s), %lu juncao(oes) sem parada.",
                 (unsigned long)committed, (unsigned long)chained);
    }
    return err;
}

/* ==================================================================================== */
/* Jog continuo (controle por mouse): o host so manda incrementos do alvo; aqui um       */
/* seguidor de posicao por eixo (velocidade/aceleracao da NVS, curva de frenagem) gera   */
/* segmentos de velocidade constante de JOG_DT_S no stream RMT, no maximo 2 em voo.      */
/* Sem fila entre host e motor: latencia de ~2 segmentos e o alvo e sempre alcancado.    */
/* ==================================================================================== */

#define JOG_DT_S 0.010f           /* periodo do seguidor = duracao minima de um segmento */
#define JOG_MAX_SUBPERIODS 8U     /* baixa velocidade: um segmento junta ate 8 periodos */
#define JOG_IDLE_EXIT_US 500000   /* ocioso e sem incrementos por 0,5 s: encerra o jog */
#define JOG_MAX_LAG_S 0.75f       /* alvo a frente da maquina limitado a 0,75 s de vmax */

static portMUX_TYPE s_jog_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_jog_accum[AXIS_COUNT];     /* incrementos (graus/mm) ainda nao absorvidos */
static int64_t s_jog_last_update_us;
static bool s_jog_running;                /* motion_task dentro do laco de jog */
static bool s_jog_queued;                 /* MOTION_CMD_JOG ja na fila */

static esp_err_t enqueue_motion_cmd(app_context_t *ctx, motion_cmd_t *cmd);

esp_err_t motion_jog_add(app_context_t *ctx, float d_c, float d_a, float d_z)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    if (ctx->state.ota_in_progress) {
        return ESP_ERR_INVALID_STATE;
    }
    bool need_cmd = false;
    portENTER_CRITICAL(&s_jog_mux);
    s_jog_accum[AXIS_C_ID] += d_c;
    s_jog_accum[AXIS_A_ID] += d_a;
    s_jog_accum[AXIS_Z_ID] += d_z;
    s_jog_last_update_us = esp_timer_get_time();
    if (!s_jog_running && !s_jog_queued) {
        s_jog_queued = true;
        need_cmd = true;
    }
    portEXIT_CRITICAL(&s_jog_mux);

    if (need_cmd) {
        motion_cmd_t cmd = {.type = MOTION_CMD_JOG, .axis = 'C', .speed_override = -1.0f,
                            .accel_override = -1.0f, .speed_c = -1.0f, .speed_a = -1.0f, .speed_z = -1.0f};
        esp_err_t err = enqueue_motion_cmd(ctx, &cmd);
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_jog_mux);
            s_jog_queued = false;
            portEXIT_CRITICAL(&s_jog_mux);
            return err;
        }
    }
    return ESP_OK;
}

static void jog_cmd_discarded(void)
{
    portENTER_CRITICAL(&s_jog_mux);
    s_jog_queued = false;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        s_jog_accum[i] = 0.0f;
    }
    portEXIT_CRITICAL(&s_jog_mux);
}

static void jog_clear_accum(void)
{
    portENTER_CRITICAL(&s_jog_mux);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        s_jog_accum[i] = 0.0f;
    }
    portEXIT_CRITICAL(&s_jog_mux);
}

typedef struct {
    float step_size;   /* graus ou mm por passo */
    float vmax;        /* passos/s */
    float accel;       /* passos/s^2 */
    float lo, hi;      /* alvo permitido, em passos relativos ao inicio do jog */
    float target;      /* alvo (passos, relativo) */
    float pos;         /* posicao comandada com fracao (passos) */
    int32_t pos_int;   /* passos inteiros ja emitidos */
    float vel;         /* velocidade do seguidor (passos/s) */
    bool blocked;
    bool warned;
} jog_axis_t;

/* Um periodo do seguidor: velocidade em direcao ao alvo com aceleracao limitada e
 * frenagem v <= sqrt(2 a |erro|), pousando no alvo sem ultrapassar. Retorna passos inteiros. */
static int32_t jog_follow(jog_axis_t *ax, float dt)
{
    float e = ax->target - ax->pos;
    float dist = fabsf(e);
    float dv = ax->accel * dt;
    float v_des = fminf(ax->vmax, sqrtf(2.0f * ax->accel * dist));
    v_des = (e < 0.0f) ? -v_des : v_des;
    float v = ax->vel + fmaxf(-dv, fminf(dv, v_des - ax->vel));
    float d = v * dt;
    if (d * e > 0.0f && fabsf(d) > dist) {
        d = e;             /* chegaria alem do alvo: pousa nele */
        v = d / dt;
    }
    if (dist < 1e-4f && fabsf(v) <= dv) {
        v = 0.0f;
        d = 0.0f;
    }
    ax->vel = v;
    ax->pos += d;
    int32_t n = (int32_t)lroundf(ax->pos) - ax->pos_int;
    ax->pos_int += n;
    return n;
}

static esp_err_t motion_jog_run(app_context_t *ctx)
{
    portENTER_CRITICAL(&s_jog_mux);
    s_jog_queued = false;
    s_jog_running = true;
    portEXIT_CRITICAL(&s_jog_mux);

    jog_axis_t ax[AXIS_COUNT] = {0};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        char name = axis_char_from_index(i);
        ax[i].step_size = get_step_size(ctx, name);
        float speed = axis_nominal_speed(ctx, i, -1.0f);
        float accel = axis_accel(ctx, i, -1.0f);
        if (i == AXIS_Z_ID) {
            speed = fminf(speed, 400.0f);
            accel = fmaxf(50.0f, fminf(accel, 5000.0f));
        }
        ax[i].vmax = fminf(speed / ax[i].step_size, (float)RMT_MAX_STEP_FREQ_HZ);
        ax[i].accel = accel / ax[i].step_size;
    }
    // Limites de C/A sobre a posicao do encoder no inicio; sem encoder o eixo fica travado
    for (size_t i = AXIS_C_ID; i <= AXIS_A_ID; ++i) {
        float deg = 0.0f;
        if (hardware_read_axis_encoder(axis_char_from_index(i), &deg) == ESP_OK) {
            float min_deg = (i == AXIS_C_ID) ? ctx->settings.limit_min_c_deg : ctx->settings.limit_min_a_deg;
            float max_deg = (i == AXIS_C_ID) ? ctx->settings.limit_max_c_deg : ctx->settings.limit_max_a_deg;
            ax[i].lo = fminf(0.0f, (min_deg - deg) / ax[i].step_size);
            ax[i].hi = fmaxf(0.0f, (max_deg - deg) / ax[i].step_size);
        } else {
            ax[i].blocked = true;
        }
    }
    ax[AXIS_Z_ID].blocked = ctx->state.z_bloqueado;
    ax[AXIS_Z_ID].lo = (float)(-ctx->state.atual_z);
    ax[AXIS_Z_ID].hi = (float)(ctx->settings.max_passos_z - ctx->state.atual_z);

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        portENTER_CRITICAL(&s_jog_mux);
        s_jog_running = false;
        portEXIT_CRITICAL(&s_jog_mux);
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = hardware_stream_begin();
    if (err != ESP_OK) {
        xSemaphoreGive(ctx->motion_mutex);
        portENTER_CRITICAL(&s_jog_mux);
        s_jog_running = false;
        portEXIT_CRITICAL(&s_jog_mux);
        return err;
    }

    int32_t z_ring[4] = {0};
    uint32_t committed = 0, reported = 0, pool_idx = 0;
    int32_t seg_steps[AXIS_COUNT] = {0, 0, 0};
    uint32_t seg_periods = 0;

    while (true) {
        if (motion_abort_hook()) {
            err = ESP_ERR_NOT_FINISHED;
            break;
        }
        // No maximo 2 segmentos em voo: o plano/pool deste indice (usado 2 segmentos atras) esta livre
        if (committed >= 2U) {
            err = hardware_stream_wait(committed - 1U);
            if (err != ESP_OK) {
                break;
            }
        }
        uint32_t completed = hardware_stream_completed();
        while (reported < completed) {
            ctx->state.atual_z += z_ring[reported % 4U];
            reported++;
        }

        // Absorve os incrementos do host no alvo (limites e atraso maximo aplicados aqui:
        // o que passa do limite e descartado, nao "guardado" para depois)
        bool other_cmd = uxQueueMessagesWaiting(ctx->motion_queue) > 0U;
        float inc[AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
        int64_t last_update;
        portENTER_CRITICAL(&s_jog_mux);
        if (!other_cmd) {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                inc[i] = s_jog_accum[i];
                s_jog_accum[i] = 0.0f;
            }
        }
        last_update = s_jog_last_update_us;
        portEXIT_CRITICAL(&s_jog_mux);
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (inc[i] == 0.0f) {
                continue;
            }
            if (ax[i].blocked) {
                if (!ax[i].warned) {
                    ax[i].warned = true;
                    printf("AVISO: JOG %c ignorado: %s.\n", axis_char_from_index(i),
                           (i == AXIS_Z_ID) ? "eixo Z bloqueado (ALARM OFF ou HOME Z)"
                                            : "encoder indisponivel (use DIAG)");
                }
                continue;
            }
            float lag = ax[i].vmax * JOG_MAX_LAG_S;
            float t = ax[i].target + inc[i] / ax[i].step_size;
            t = fmaxf(ax[i].pos - lag, fminf(ax[i].pos + lag, t));
            ax[i].target = fmaxf(ax[i].lo, fminf(ax[i].hi, t));
        }

        // Um periodo do seguidor
        bool moving = false;
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            seg_steps[i] += jog_follow(&ax[i], JOG_DT_S);
            moving = moving || ax[i].vel != 0.0f || fabsf(ax[i].target - ax[i].pos) > 0.5f;
        }
        seg_periods++;

        if (seg_steps[0] != 0 || seg_steps[1] != 0 || seg_steps[2] != 0) {
            // Segmento de velocidade constante: v_floor = vmax -> sem rampas, dura exatamente
            // seg_periods * JOG_DT_S e todos os eixos terminam juntos
            float seg_s = (float)seg_periods * JOG_DT_S;
            mp_request_t req = {0};
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                float v = fabsf((float)seg_steps[i]) / seg_s;
                req.steps[i] = seg_steps[i];
                req.vmax[i] = v;
                req.v_floor[i] = v;
                req.accel[i] = 1e9f;
            }
            mp_result_t pr = mp_plan_move(&req, s_stream_pool[pool_idx], STREAM_POOL_WORDS, STREAM_TICK_HZ,
                                          &s_stream_plan[pool_idx]);
            if (pr != MP_OK) {
                ESP_LOGE(APP_TAG, "jog: falha ao planejar segmento (%d)", (int)pr);
                err = ESP_ERR_INVALID_ARG;
                break;
            }
            int8_t dir[AXIS_COUNT];
            stream_dir_levels(ctx, &req, dir);
            err = hardware_stream_commit(&s_stream_plan[pool_idx], dir);
            if (err != ESP_OK) {
                break;
            }
            z_ring[committed % 4U] = seg_steps[AXIS_Z_ID];
            committed++;
            pool_idx ^= 1U;
            seg_steps[0] = seg_steps[1] = seg_steps[2] = 0;
            seg_periods = 0;
            continue;
        }

        if (moving && seg_periods < JOG_MAX_SUBPERIODS) {
            continue; // velocidade abaixo de 1 passo/periodo: junta periodos num segmento so
        }
        // Nada a emitir: deixa o tempo passar (o seguidor "gastou" esses periodos parado)
        uint32_t idle_ms = (uint32_t)lroundf((float)seg_periods * JOG_DT_S * 1000.0f);
        seg_periods = 0;
        if (!moving) {
            ax[0].vel = ax[1].vel = ax[2].vel = 0.0f;
            bool in_flight = hardware_stream_completed() < committed;
            bool timed_out = (esp_timer_get_time() - last_update) > JOG_IDLE_EXIT_US;
            if (!in_flight && (other_cmd || timed_out)) {
                bool exit_now = false;
                portENTER_CRITICAL(&s_jog_mux);
                if (other_cmd || (s_jog_accum[0] == 0.0f && s_jog_accum[1] == 0.0f && s_jog_accum[2] == 0.0f)) {
                    s_jog_running = false; // um JOG novo a partir daqui enfileira outro comando
                    exit_now = true;
                }
                portEXIT_CRITICAL(&s_jog_mux);
                if (exit_now) {
                    break;
                }
            }
            idle_ms = 2U; // ocioso: responde rapido ao proximo incremento
        }
        if (hardware_stream_completed() < committed) {
            err = hardware_stream_wait(committed);
            if (err != ESP_OK) {
                break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(idle_ms > 0U ? idle_ms : 1U));
        }
    }

    if (err == ESP_OK) {
        err = hardware_stream_wait(committed);
    }
    uint32_t completed = hardware_stream_completed();
    while (reported < completed) {
        ctx->state.atual_z += z_ring[reported % 4U];
        reported++;
    }
    if (err != ESP_OK) {
        for (uint32_t k = reported; k < committed; ++k) {
            if (z_ring[k % 4U] != 0) {
                ctx->state.homed[AXIS_Z_ID] = false;
                ESP_LOGW(APP_TAG, "Jog com Z interrompido: posicao Z incerta, execute HOME Z.");
                break;
            }
        }
        jog_clear_accum();
    }
    hardware_stream_end();
    xSemaphoreGive(ctx->motion_mutex);
    portENTER_CRITICAL(&s_jog_mux);
    s_jog_running = false;
    portEXIT_CRITICAL(&s_jog_mux);
    return err;
}

void motion_print_pos_line(app_context_t *ctx)
{
    // Linha estruturada para o app (chave=valor, "nan" = invalido); as linhas "Eixo ..." seguem
    // existindo para compatibilidade com terminais e scripts antigos.
    float c = 0.0f, a = 0.0f;
    char c_txt[16] = "nan", a_txt[16] = "nan";
    if (hardware_read_axis_encoder('C', &c) == ESP_OK) {
        snprintf(c_txt, sizeof(c_txt), "%.2f", c);
    }
    if (hardware_read_axis_encoder('A', &a) == ESP_OK) {
        snprintf(a_txt, sizeof(a_txt), "%.2f", a);
    }
    printf("@POS C=%s A=%s Z=%ld ZMAX=%ld HOMED=%u%u%u MOVING=%u ZLOCK=%u ALARM=%u\n", c_txt, a_txt,
           (long)ctx->state.atual_z, (long)ctx->settings.max_passos_z,
           ctx->state.homed[0] ? 1U : 0U, ctx->state.homed[1] ? 1U : 0U, ctx->state.homed[2] ? 1U : 0U,
           ctx->state.in_motion ? 1U : 0U, ctx->state.z_bloqueado ? 1U : 0U,
           ctx->state.alarme_z_ativo ? 1U : 0U);
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
            s_active_gen = cmd.stop_gen;
            bool events_sent = false;
            if (motion_abort_hook()) {
                // Enfileirado antes de um STOP que ocorreu enquanto aguardava na fila
                err = ESP_ERR_NOT_FINISHED;
                if (cmd.type == MOTION_CMD_JOG) {
                    jog_cmd_discarded();
                }
            } else if (ctx->ext.motion_engine == MOTION_ENGINE_STREAM && motion_cmd_streamable(&cmd)) {
                // Cadeia de movimentos: emite DONE/ERROR de cada comando por conta propria
                err = motion_stream_chain(ctx, &cmd);
                events_sent = true;
            } else switch (cmd.type) {
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
            case MOTION_CMD_JOG:
                err = motion_jog_run(ctx);
                break;
            }

            if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ctx->state.in_motion = false;
                ctx->state.in_homing = false;
                if (cmd.type == MOTION_CMD_HOME) {
                    char ax = (char)toupper((unsigned char)cmd.axis);
                    size_t hidx = (ax == 'C' || ax == 'X') ? AXIS_C_ID : ((ax == 'A' || ax == 'Y') ? AXIS_A_ID : AXIS_Z_ID);
                    ctx->state.homed[hidx] = (err == ESP_OK);
                    if (err == ESP_OK) {
                        printf("Home %c finalizado.\n", ax);
                    } else {
                        printf("Home %c falhou: %s\n", ax, esp_err_to_name(err));
                    }
                }
                if (err == ESP_ERR_NOT_FINISHED) {
                    printf("Movimento interrompido por STOP.\n");
                } else if (err == ESP_ERR_INVALID_STATE && cmd.type != MOTION_CMD_HOME) {
                    printf("AVISO: Eixo Z bloqueado por seguranca. Use 'ALARM OFF' ou 'HOME Z'.\n");
                } else if (err == ESP_ERR_INVALID_RESPONSE && cmd.type != MOTION_CMD_HOME) {
                    printf("ERRO: encoder indisponivel; movimento rejeitado. Use MOVE_F (malha aberta) ou DIAG.\n");
                } else if (err != ESP_OK && cmd.type != MOTION_CMD_HOME) {
                    printf("ERRO: movimento falhou: %s\n", esp_err_to_name(err));
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
            motion_print_pos_line(ctx);

            bool can_online = false;
            if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                can_online = ctx->state.can_online;
                xSemaphoreGive(ctx->state_mutex);
            }

            if (can_online && cmd.opcode != 0 && !events_sent) {
                if (cmd.opcode != CAN_OP_MOVE_UNIFIED || err != ESP_OK) {
                    (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_DONE : CAN_EVT_ERROR, cmd.opcode, (uint8_t)err);
                }
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

esp_err_t motion_apply_speed_level(app_context_t *ctx, uint8_t level)
{
    static const uint32_t k_level_delay_us[5] = {2000U, 800U, 400U, 150U, 50U};
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    if (level < 1U || level > 5U) {
        return ESP_ERR_INVALID_ARG;
    }
    // O nivel vale para os eixos rotativos C/A (como antes nos MOVE de eixo unico); o Z
    // mantem sua velocidade em mm/s — 400 us no Z seriam apenas ~15 mm/s.
    uint32_t delay_us = k_level_delay_us[level - 1U];
    const size_t axes[2] = {AXIS_C_ID, AXIS_A_ID};
    for (size_t i = 0; i < 2; ++i) {
        ctx->state.speed_delay_us[axes[i]] = delay_us;
        ctx->state.speed[axes[i]] = motion_delay_us_to_speed(ctx, axis_char_from_index(axes[i]), delay_us);
    }
    return ESP_OK;
}

esp_err_t motion_request_stop(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    // Invalida o comando em execucao e todos os ja enfileirados (checado a cada passo/poll de 5 ms)
    (void)__atomic_add_fetch(&s_stop_gen, 1U, __ATOMIC_SEQ_CST);
    jog_clear_accum(); // incrementos de jog ainda nao absorvidos tambem sao descartados

    uint32_t dropped_count = 0U;
    if (ctx->motion_queue != NULL) {
        motion_cmd_t dropped;
        while (xQueueReceive(ctx->motion_queue, &dropped, 0) == pdTRUE) {
            notify_discarded_cmd(ctx, &dropped, true);
            ++dropped_count;
        }
    }
    ESP_LOGW(APP_TAG, "STOP: movimento em curso interrompido, %lu comando(s) descartado(s) da fila.",
             (unsigned long)dropped_count);
    return ESP_OK;
}

esp_err_t motion_init(app_context_t *ctx)
{
    ctx->motion_queue = xQueueCreate(10, sizeof(motion_cmd_t));
    if (ctx->motion_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    hardware_set_abort_hook(motion_abort_hook);
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
    if (ctx->state.ota_in_progress) {
        return ESP_ERR_INVALID_STATE; // sessao OTA ativa: nenhum movimento (serial ou CAN)
    }
    cmd->stop_gen = s_stop_gen;
    // Homing tem prioridade imediata: descarta movimentos antigos pendentes na fila
    if (cmd->type == MOTION_CMD_HOME) {
        motion_cmd_t dropped;
        while (xQueueReceive(ctx->motion_queue, &dropped, 0) == pdTRUE) {
            notify_discarded_cmd(ctx, &dropped, true);
        }
    }
    // Para movimentos sincronizados continuos (ex: TouchDesigner / CAN_OP_MOVE_UNIFIED):
    // Se ja houver comandos MOVE_SYNC pendentes na fila aguardando execucao,
    // descarta os intermediarios obsoletos para manter a latencia minima (tempo real).
    if (cmd->type == MOTION_CMD_MOVE_SYNC) {
        motion_cmd_t peek_cmd;
        UBaseType_t count = uxQueueMessagesWaiting(ctx->motion_queue);
        for (UBaseType_t i = 0; i < count; i++) {
            if (xQueuePeek(ctx->motion_queue, &peek_cmd, 0) == pdTRUE) {
                if (peek_cmd.type == MOTION_CMD_MOVE_SYNC) {
                    motion_cmd_t discarded;
                    if (xQueueReceive(ctx->motion_queue, &discarded, 0) == pdTRUE) {
                        notify_discarded_cmd(ctx, &discarded, false);
                    }
                } else {
                    break;
                }
            }
        }
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
                    if (xQueueReceive(ctx->motion_queue, &discarded, 0) == pdTRUE) {
                        notify_discarded_cmd(ctx, &discarded, false); // substituido pela reversao do jog
                    }
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
