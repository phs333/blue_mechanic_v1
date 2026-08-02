#pragma once

#include "esp_err.h"
#include "app_defs.h"

esp_err_t motion_init(app_context_t *ctx);

esp_err_t motion_post_move_axis(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_axis_force(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_home_axis(app_context_t *ctx, char axis, uint8_t sender_id, uint8_t opcode);

uint32_t motion_speed_to_delay_us(app_context_t *ctx, char axis, float speed);
