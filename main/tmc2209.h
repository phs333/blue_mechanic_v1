#pragma once

#include "esp_err.h"

#include "app_defs.h"

/* mA RMS por degrau de CS (0..31): (CS+1)/32 * Vfs/(Rsense+20mOhm)/sqrt(2), Rsense 0,1 Ohm.
 * A corrente e quantizada nesses degraus: o valor efetivo e (CS+1) * 59,846 mA. */
#define TMC2209_MA_PER_CS 59.846f

esp_err_t tmc2209_init(app_context_t *ctx);
esp_err_t tmc2209_apply_settings(app_context_t *ctx);
/* Monitor periodico: reaplica a configuracao se o driver reiniciar (GSTAT.reset) ou voltar a responder. */
esp_err_t tmc2209_start_monitor(app_context_t *ctx);
esp_err_t tmc2209_read_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t *value);
/* Escrita confirmada pelo contador IFCNT do driver. */
esp_err_t tmc2209_write_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t value);
void tmc2209_print_status(const app_context_t *ctx);
uint8_t tmc2209_ma_to_cs(uint16_t ma);
uint16_t tmc2209_cs_to_ma(uint8_t cs);
esp_err_t tmc2209_set_spreadcycle(app_context_t *ctx, char axis, bool enabled);
esp_err_t tmc2209_set_microsteps(app_context_t *ctx, char axis, uint16_t microsteps);
/* Escreve IHOLD_IRUN (com confirmacao) no driver do eixo; ESP_ERR_INVALID_STATE se offline. */
esp_err_t tmc2209_apply_current(app_context_t *ctx, char axis);
/* Velocidade acima da qual o driver troca stealthChop -> spreadCycle (0 = sempre stealthChop). */
esp_err_t tmc2209_set_stealth_max_speed(app_context_t *ctx, char axis, float speed);
