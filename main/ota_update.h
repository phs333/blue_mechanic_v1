#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "app_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Executa verificacao de inicializacao da particao ativa e trata validacao de rollback.
 *        Se o firmware atual estiver em estado ESP_OTA_IMG_PENDING_VERIFY, valida e cancela rollback.
 */
esp_err_t ota_update_boot_check(app_context_t *ctx);

/**
 * @brief Entra no modo seguro de preparo para atualizacao OTA:
 *        - Desliga imediatamente os lasers (seguranca).
 *        - Desenergiza os drivers de passo e interrompe movimentos.
 *        - Esvazia filas residuais de movimento.
 *        - Suspende leituras termicas 1-Wire e telemetria periodica CAN para liberar a CPU e barramento.
 *        - Sinaliza visualmente no LED RGB (Laranja/Ambar pulsante rapido).
 *        - Abre a particao flash de destino com esp_ota_begin().
 * @param ctx Contexto da aplicacao
 * @param image_size Tamanho esperado do binario (ou 0 para OTA_SIZE_UNKNOWN)
 */
esp_err_t ota_prepare_for_update(app_context_t *ctx, uint32_t image_size);

/**
 * @brief Grava um bloco de dados binarios na particao flash aberta.
 */
esp_err_t ota_write_chunk(const void *data, size_t length);

/**
 * @brief Finaliza a gravacao, valida os dados na flash, define a particao de boot e agenda reboot.
 */
esp_err_t ota_finalize_and_reboot(app_context_t *ctx);

/**
 * @brief Aborta o processo de atualizacao OTA, fecha a particao flash e restaura o funcionamento normal.
 */
esp_err_t ota_abort(app_context_t *ctx);

/**
 * @brief Valida explicitamente o firmware atual e cancela rollback.
 */
esp_err_t ota_mark_valid(void);

/**
 * @brief Marca o firmware atual como invalido, forca rollback para a versao anterior e reinicia.
 */
esp_err_t ota_rollback_and_reboot(void);

/**
 * @brief Exibe informacoes detalhadas da particao em execucao, proxima particao e estado de rollback.
 */
void ota_print_status(void);

/**
 * @brief Retorna true se houver uma sessao de atualizacao OTA em andamento.
 */
bool ota_is_in_progress(void);

/**
 * @brief Processa comandos de atualizacao OTA recebidos pelo barramento CAN via Teensy.
 * @return true se o comando foi tratado pelo subsistema OTA.
 */
bool ota_handle_can_cmd(app_context_t *ctx, uint8_t opcode, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
