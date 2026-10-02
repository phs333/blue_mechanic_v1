#pragma once

#include "esp_err.h"
#include "app_defs.h"

esp_err_t motion_init(app_context_t *ctx);

esp_err_t motion_post_move_axis(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_axis_force(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_home_axis(app_context_t *ctx, char axis, uint8_t sender_id, uint8_t opcode);

/* Parada imediata: interrompe o movimento em curso (RMT e bit-bang) e esvazia a fila.
 * Pode ser chamada de qualquer tarefa (console, CAN, safety). */
esp_err_t motion_request_stop(app_context_t *ctx);

/* Jog continuo (controle por mouse): soma incrementos (graus C/A, mm Z) ao alvo. O motion_task
 * persegue o alvo com a velocidade/aceleracao da NVS, em segmentos de 10 ms no stream RMT,
 * respeitando limites de C/A (encoder) e o curso/bloqueio do Z. Encerra sozinho quando ocioso. */
esp_err_t motion_jog_add(app_context_t *ctx, float d_c, float d_a, float d_z);
/* Alvo absoluto em streaming (U/TouchDesigner): C/A em graus, Z em passos. O motor
 * persegue sempre o alvo mais recente, sem fila nem parada entre alvos. */
esp_err_t motion_track_set(app_context_t *ctx, float c_deg, float a_deg, int32_t z_steps, bool force_no_encoder);

/* Nivel de velocidade 1..5 (mesma tabela de CAN_OP_SPEED) aplicado aos eixos C/A em uso. */
esp_err_t motion_apply_speed_level(app_context_t *ctx, uint8_t level);

/* Linha "@POS C=.. A=.. Z=.. ZMAX=.. HOMED=CAZ MOVING=.." para o app. */
void motion_print_pos_line(app_context_t *ctx);

uint32_t motion_speed_to_delay_us(app_context_t *ctx, char axis, float speed);
float motion_delay_us_to_speed(app_context_t *ctx, char axis, uint32_t delay_us);
void compute_trapezoidal_profile(app_context_t *ctx, char axis, uint32_t start_delay, uint32_t end_delay, uint32_t total_steps, uint32_t ramp_steps, uint32_t *delay_us);
esp_err_t motion_post_move_axis_with_params(app_context_t *ctx, char axis, int32_t steps, float speed, float accel, uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_axis_profile(app_context_t *ctx, char axis, int32_t steps,
                                        float speed, float accel, bool force_no_encoder,
                                        uint8_t sender_id, uint8_t opcode);
esp_err_t motion_post_move_sync(app_context_t *ctx, int32_t steps_c, int32_t steps_a, int32_t steps_z,
                                float speed_c, float speed_a, float speed_z, float accel,
                                bool force_no_encoder, uint8_t sender_id, uint8_t opcode);

/* Pure planner helper: clamps a relative request to limits relative to home angle. */
int32_t motion_plan_limited_steps(float actual_deg, float home_deg,
                                  float min_limit_deg, float max_limit_deg,
                                  float deg_per_step, int32_t requested_steps);
