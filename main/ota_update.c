#include "ota_update.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_bus.h"
#include "hardware.h"
#include "status_led.h"

#define TAG "ota_update"

static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_target_partition = NULL;
static bool s_ota_in_progress = false;
static uint32_t s_bytes_written = 0;
static uint32_t s_expected_size = 0;
static uint32_t s_last_progress_bytes = 0;

esp_err_t ota_update_boot_check(app_context_t *ctx)
{
    (void)ctx;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGW(TAG, "Nao foi possivel obter particao em execucao");
        return ESP_FAIL;
    }

    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);

    if (err == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "Boot a partir de particao OTA pendente de verificacao (%s). Validando aplicativo...",
                 running->label);
        esp_err_t valid_err = esp_ota_mark_app_valid_cancel_rollback();
        if (valid_err == ESP_OK) {
            ESP_LOGI(TAG, "Firmware validado com sucesso! Rollback cancelado. Particao: %s", running->label);
        } else {
            ESP_LOGE(TAG, "Falha ao validar particao OTA: %s", esp_err_to_name(valid_err));
        }
    } else {
        ESP_LOGI(TAG, "Particao ativa: %s (offset 0x%08" PRIX32 ", tamanho %" PRIu32 " KB, estado_ota=%d)",
                 running->label, running->address, running->size / 1024, (int)ota_state);
    }

    return ESP_OK;
}

esp_err_t ota_prepare_for_update(app_context_t *ctx, uint32_t image_size)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "ctx nulo");

    ESP_LOGW(TAG, "=====================================================");
    ESP_LOGW(TAG, "ENTRANDO EM MODO SEGURO DE ATUALIZACAO OTA");
    ESP_LOGW(TAG, "Parando motores, desligando lasers e liberando barramento");
    ESP_LOGW(TAG, "=====================================================");

    // 1. Desliga imediatamente os lasers (seguranca maxima)
    hardware_set_laser_level(ctx, 0, 0);
    hardware_set_laser_level(ctx, 1, 0);

    // 2. Desenergiza drivers de passo e interrompe movimentos
    hardware_set_driver_enable(ctx, false);

    // 3. Atualiza estado e cancela movimentos na fila
    if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ctx->state.in_motion = false;
        ctx->state.in_homing = false;
        ctx->state.em_homing_z = false;
        ctx->state.laser_level[0] = 0;
        ctx->state.laser_level[1] = 0;
        ctx->state.ota_in_progress = true;
        xSemaphoreGive(ctx->state_mutex);
    } else {
        ctx->state.ota_in_progress = true;
    }

    // 4. Limpa fila de comandos pendentes de movimento
    if (ctx->motion_queue) {
        motion_cmd_t dummy;
        while (xQueueReceive(ctx->motion_queue, &dummy, 0) == pdTRUE) {}
    }

    // 5. Sinalizacao no LED RGB (Laranja / Ambar fixo de atencao)
    status_led_set_color(255, 120, 0);

    // 6. Aborta sessao anterior se tiver ficado aberta
    if (s_ota_in_progress && s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
        s_ota_in_progress = false;
    }

    // 7. Descobre a proxima particao de destino (ota_0 ou ota_1)
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Nenhuma particao OTA de destino encontrada na tabela de particoes!");
        ctx->state.ota_in_progress = false;
        status_led_set_mode(STATUS_LED_MODE_AUTO);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Particao de destino: %s (offset 0x%08" PRIX32 ", tamanho %" PRIu32 " KB)",
             update_partition->label, update_partition->address, update_partition->size / 1024);

    // 8. Inicializa gravacao OTA na particao flash
    esp_err_t err = esp_ota_begin(update_partition, image_size > 0 ? image_size : OTA_SIZE_UNKNOWN, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha em esp_ota_begin(): %s", esp_err_to_name(err));
        ctx->state.ota_in_progress = false;
        status_led_set_mode(STATUS_LED_MODE_AUTO);
        return err;
    }

    s_target_partition = update_partition;
    s_ota_in_progress = true;
    s_bytes_written = 0;
    s_expected_size = image_size;
    s_last_progress_bytes = 0;

    ESP_LOGI(TAG, "Nó pronto para receber dados binarios de firmware via CAN/Serial!");
    return ESP_OK;
}

esp_err_t ota_write_chunk(const void *data, size_t length)
{
    if (!s_ota_in_progress || s_ota_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, data, length);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao gravar bloco OTA: %s", esp_err_to_name(err));
        return err;
    }

    s_bytes_written += length;
    return ESP_OK;
}

esp_err_t ota_finalize_and_reboot(app_context_t *ctx)
{
    if (!s_ota_in_progress || s_ota_handle == 0 || s_target_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Finalizando gravacao OTA. Total gravado: %" PRIu32 " bytes. Validando hash...", s_bytes_written);

    esp_err_t err = esp_ota_end(s_ota_handle);
    s_ota_handle = 0;
    s_ota_in_progress = false;

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Validacao do firmware falhou: %s", esp_err_to_name(err));
        if (ctx) {
            ctx->state.ota_in_progress = false;
            (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, (uint8_t)(err & 0xFF));
        }
        status_led_set_mode(STATUS_LED_MODE_AUTO);
        return err;
    }

    err = esp_ota_set_boot_partition(s_target_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao definir proxima particao de boot: %s", esp_err_to_name(err));
        if (ctx) {
            ctx->state.ota_in_progress = false;
            (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, (uint8_t)(err & 0xFF));
        }
        status_led_set_mode(STATUS_LED_MODE_AUTO);
        return err;
    }

    ESP_LOGI(TAG, "Gravacao concluida com sucesso! Particao %s definida para proximo boot.",
             s_target_partition->label);

    if (ctx) {
        (void)can_send_event(ctx, CAN_EVT_OTA_DONE, ctx->settings.node_id, 100);
    }

    // Sinaliza LED verde antes do reboot
    status_led_set_color(0, 255, 0);

    ESP_LOGW(TAG, "Reiniciando sistema em 1 segundo para carregar novo firmware...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

esp_err_t ota_abort(app_context_t *ctx)
{
    if (s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }
    s_ota_in_progress = false;
    s_bytes_written = 0;

    if (ctx) {
        if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctx->state.ota_in_progress = false;
            xSemaphoreGive(ctx->state_mutex);
        } else {
            ctx->state.ota_in_progress = false;
        }
        (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, 0xFF);
    }

    status_led_set_mode(STATUS_LED_MODE_AUTO);
    ESP_LOGW(TAG, "Atualizacao OTA abortada. Operacao normal restaurada.");
    return ESP_OK;
}

esp_err_t ota_mark_valid(void)
{
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firmware atual validado manualmente. Rollback cancelado.");
    } else {
        ESP_LOGE(TAG, "Erro ao validar firmware: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ota_rollback_and_reboot(void)
{
    ESP_LOGW(TAG, "Forcando rollback para a particao anterior...");
    esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao executar rollback: %s", esp_err_to_name(err));
    }
    return err;
}

void ota_print_status(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next_update = esp_ota_get_next_update_partition(NULL);
    const esp_app_desc_t *app_desc = esp_app_get_description();

    printf("\n=== STATUS OTA & PARTICIONAMENTO ===\n");
    if (app_desc) {
        printf("Versao do App: %s | Compilado: %s %s\n",
               app_desc->version, app_desc->date, app_desc->time);
        printf("IDF Version: %s | Nome do Projeto: %s\n",
               app_desc->idf_ver, app_desc->project_name);
    }

    if (running) {
        esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
        (void)esp_ota_get_state_partition(running, &state);
        const char *state_str = "DESCONHECIDO";
        switch (state) {
        case ESP_OTA_IMG_NEW: state_str = "NOVO (NOVO BOOT)"; break;
        case ESP_OTA_IMG_PENDING_VERIFY: state_str = "PENDENTE DE VALIDACAO (ROLLBACK ATIVO)"; break;
        case ESP_OTA_IMG_VALID: state_str = "VALIDADO / SEGURO"; break;
        case ESP_OTA_IMG_INVALID: state_str = "INVALIDO"; break;
        case ESP_OTA_IMG_ABORTED: state_str = "ABORTADO"; break;
        default: state_str = "PADRAO (NAO-OTA / FACTORY)"; break;
        }
        printf("Particao Atual (Running): %s (0x%08" PRIX32 ", %" PRIu32 " KB) [Estado: %s]\n",
               running->label, running->address, running->size / 1024, state_str);
    } else {
        printf("Particao Atual: Indisponivel\n");
    }

    if (next_update) {
        printf("Proxima Particao de Update: %s (0x%08" PRIX32 ", %" PRIu32 " KB)\n",
               next_update->label, next_update->address, next_update->size / 1024);
    } else {
        printf("Proxima Particao de Update: Nenhuma disponivel\n");
    }

    printf("Sessao OTA em andamento: %s\n", s_ota_in_progress ? "SIM (AGUARDANDO/RECEBENDO)" : "NAO (OPERACAO NORMAL)");
    if (s_ota_in_progress) {
        printf("Bytes gravados: %" PRIu32 " / %" PRIu32 "\n", s_bytes_written, s_expected_size);
    }
    printf("=====================================\n");
}

bool ota_is_in_progress(void)
{
    return s_ota_in_progress;
}

bool ota_handle_can_cmd(app_context_t *ctx, uint8_t opcode, const uint8_t *data, size_t len)
{
    if (ctx == NULL || data == NULL || len == 0) {
        return false;
    }

    switch (opcode) {
    case CAN_OP_OTA_START: {
        // Formato: [0x40, target_node, size_b0, size_b1, size_b2, size_b3, flags]
        uint8_t target_node = (len > 1) ? data[1] : 0xFF;
        // target_node == 0 significa broadcast para todos os 10 nos simultaneamente!
        if (target_node != 0 && target_node != ctx->settings.node_id) {
            return true; // Mensagem destinada a outro no especifico
        }

        uint32_t image_size = 0;
        if (len >= 6) {
            image_size = (uint32_t)data[2] |
                         ((uint32_t)data[3] << 8) |
                         ((uint32_t)data[4] << 16) |
                         ((uint32_t)data[5] << 24);
        }

        esp_err_t err = ota_prepare_for_update(ctx, image_size);
        (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_OTA_READY : CAN_EVT_OTA_ERROR,
                             ctx->settings.node_id, (uint8_t)(err & 0xFF));
        return true;
    }

    case CAN_OP_OTA_DATA: {
        // Formato: [0x41, seq_num, byte0..byte5]
        if (!s_ota_in_progress) {
            return true;
        }
        if (len > 2) {
            esp_err_t err = ota_write_chunk(&data[2], len - 2);
            if (err != ESP_OK) {
                (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, (uint8_t)(err & 0xFF));
            } else if ((s_bytes_written - s_last_progress_bytes) >= 16384U) {
                s_last_progress_bytes = s_bytes_written;
                uint8_t pct = (s_expected_size > 0) ? (uint8_t)((s_bytes_written * 100U) / s_expected_size) : 0;
                (void)can_send_event(ctx, CAN_EVT_OTA_PROGRESS, ctx->settings.node_id, pct);
            }
        }
        return true;
    }

    case CAN_OP_OTA_END: {
        // Formato: [0x42, target_node, checksum_lo, checksum_hi]
        uint8_t target_node = (len > 1) ? data[1] : 0xFF;
        if (target_node != 0 && target_node != ctx->settings.node_id) {
            return true;
        }
        (void)ota_finalize_and_reboot(ctx);
        return true;
    }

    case CAN_OP_OTA_ABORT: {
        uint8_t target_node = (len > 1) ? data[1] : 0xFF;
        if (target_node != 0 && target_node != ctx->settings.node_id) {
            return true;
        }
        (void)ota_abort(ctx);
        return true;
    }

    default:
        return false;
    }
}
