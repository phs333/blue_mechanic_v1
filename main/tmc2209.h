#pragma once

#include "esp_err.h"

#include "app_defs.h"

esp_err_t tmc2209_init(app_context_t *ctx);
esp_err_t tmc2209_apply_settings(app_context_t *ctx);
esp_err_t tmc2209_read_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t *value);
esp_err_t tmc2209_write_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t value);
void tmc2209_print_status(const app_context_t *ctx);
uint8_t tmc2209_ma_to_cs(uint16_t ma);
uint16_t tmc2209_cs_to_ma(uint8_t cs);
