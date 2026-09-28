#pragma once

#include <stddef.h>

#include "esp_err.h"

#include "app_defs.h"
#include "motion_profile.h"

#define RMT_MIN_STEP_FREQ_HZ 16U
#define RMT_MAX_STEP_FREQ_HZ 60000U
#define RMT_MAX_RAMP_SAMPLES 1024U

/* Hook consultado durante movimentos RMT/bit-bang; retorna true quando um STOP foi pedido. */
typedef bool (*hardware_abort_hook_t)(void);

esp_err_t hardware_init(app_context_t *ctx);
void hardware_deinit(void);
void hardware_set_abort_hook(hardware_abort_hook_t hook);
bool hardware_abort_requested(void);

/* Motor "stream": um movimento = uma transacao RMT por canal (os tres canais, sempre).
 * begin -> commit* -> wait -> end. Ate 2 movimentos em voo: o DIR do seguinte e trocado
 * no ISR, na fronteira, permitindo encadear movimentos sem parar. */
esp_err_t hardware_stream_begin(void);
/* dir_level[i]: nivel do pino DIR do eixo i neste movimento, ou -1 para manter */
esp_err_t hardware_stream_commit(const mp_plan_t *plan, const int8_t dir_level[AXIS_COUNT]);
/* Movimentos concluidos (em todos os canais) desde hardware_stream_begin() */
uint32_t hardware_stream_completed(void);
/* Aguarda ate `target_completed` movimentos concluidos; STOP -> ESP_ERR_NOT_FINISHED */
esp_err_t hardware_stream_wait(uint32_t target_completed);
void hardware_stream_end(void);
void hardware_step_pulse(gpio_num_t step_pin, uint32_t delay_us);
void hardware_rmt_release_pin(gpio_num_t step_pin);
void hardware_rmt_reacquire_pin(gpio_num_t step_pin);
esp_err_t hardware_step_pulse_rmt_move(char axis, uint32_t total_steps, uint32_t start_freq_hz, uint32_t target_freq_hz, uint32_t ramp_steps);
esp_err_t hardware_step_pulse_rmt_move_dual(uint32_t steps_c, uint32_t start_freq_c, uint32_t target_freq_c, uint32_t ramp_c,
                                            uint32_t steps_a, uint32_t start_freq_a, uint32_t target_freq_a, uint32_t ramp_a);
esp_err_t hardware_step_pulse_rmt_move_sync3(uint32_t steps_c, uint32_t start_freq_c, uint32_t target_freq_c, uint32_t ramp_c,
                                             uint32_t steps_a, uint32_t start_freq_a, uint32_t target_freq_a, uint32_t ramp_a,
                                             uint32_t steps_z, uint32_t start_freq_z, uint32_t target_freq_z, uint32_t ramp_z);
uint32_t hardware_compute_step_delay(uint32_t start_delay, uint32_t end_delay, uint32_t step_num, uint32_t total_steps, uint32_t ramp_steps);
void hardware_set_driver_enable(app_context_t *ctx, bool enable);
void hardware_set_fan_output(app_context_t *ctx, bool on);
esp_err_t hardware_set_laser_level(app_context_t *ctx, size_t laser_index, uint16_t level);
bool hardware_is_z_switch_pressed(void);
esp_err_t hardware_read_axis_encoder(char axis, float *angle_deg);
esp_err_t hardware_encoder_set_zero(app_context_t *ctx, char axis);
void hardware_update_encoders(void);
/* Diagnostico: nivel do fim de curso Z, linhas I2C, resposta e ima de cada encoder. */
void hardware_print_diag(void);
esp_err_t hardware_read_temperature_c(float *temp_c);
