#pragma once

#include "esp_err.h"
#include "app_defs.h"

esp_err_t motion_init(app_context_t *ctx);

esp_err_t motion_post_move_axis(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_axis_force(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_home_axis(app_context_t *ctx, char axis, uint8_t sender_id, uint8_t opcode);

uint32_t motion_speed_to_delay_us(app_context_t *ctx, char axis, float speed);
void compute_trapezoidal_profile(app_context_t *ctx, char axis, uint32_t start_delay, uint32_t end_delay, uint32_t total_steps, uint32_t ramp_steps, uint32_t *delay_us);
esp_err_t motion_post_move_axis_with_params(app_context_t *ctx, char axis, int32_t steps, float speed, float accel, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z, float speed, float accel, uint8_t sender_id, uint8_t opcode);
