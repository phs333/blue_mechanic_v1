#pragma once

#include <stddef.h>

#include "esp_err.h"

#include "app_defs.h"

#define RMT_MIN_STEP_FREQ_HZ 16U
#define RMT_MAX_STEP_FREQ_HZ 60000U
#define RMT_MAX_RAMP_SAMPLES 1024U

esp_err_t hardware_init(app_context_t *ctx);
void hardware_deinit(void);
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
esp_err_t hardware_read_temperature_c(float *temp_c);
