#pragma once

#include <stddef.h>

#include "esp_err.h"

#include "app_defs.h"

esp_err_t hardware_init(app_context_t *ctx);
void hardware_deinit(void);
void hardware_step_pulse(gpio_num_t step_pin, uint32_t delay_us);
esp_err_t hardware_step_pulse_rmt(char axis, uint32_t steps, uint32_t delay_us);
void hardware_set_driver_enable(app_context_t *ctx, bool enable);
void hardware_set_fan_output(app_context_t *ctx, bool on);
esp_err_t hardware_set_laser_level(app_context_t *ctx, size_t laser_index, uint8_t level);
bool hardware_is_z_switch_pressed(void);
esp_err_t hardware_read_axis_encoder(char axis, float *angle_deg);
esp_err_t hardware_read_temperature_c(float *temp_c);
