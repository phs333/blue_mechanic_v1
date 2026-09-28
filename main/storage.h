#pragma once

#include "esp_err.h"

#include "app_defs.h"

esp_err_t storage_init(void);
esp_err_t storage_load_settings(persisted_settings_t *settings);

/* Grava imediatamente — mas so se o conteudo mudou desde a ultima gravacao. */
esp_err_t storage_save_settings(const persisted_settings_t *settings);

/* Agenda a gravacao (300 ms apos o ultimo pedido): uma rajada de comandos vira uma
 * unica escrita na flash. Retorna na hora; use storage_flush() para forcar. */
void storage_request_save(const persisted_settings_t *settings);

/* Executa agora as gravacoes pendentes (comando SAVE, antes de reiniciar, OTA). */
esp_err_t storage_flush(void);

esp_err_t storage_load_ext(ext_settings_t *ext);
esp_err_t storage_save_ext(const ext_settings_t *ext);
void storage_request_save_ext(const ext_settings_t *ext);
