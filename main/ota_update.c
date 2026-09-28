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
#include "motion.h"
#include "status_led.h"
#include "storage.h"

#define TAG "ota_update"

// Tempo de operacao saudavel antes de confirmar um firmware recem-gravado. Se ele
// travar/reiniciar antes disso, o bootloader volta automaticamente para o anterior.
#define OTA_VALIDATE_DELAY_MS 20000
// Blocos de 6 bytes do CAN sao acumulados e gravados na flash em paginas de 4 KB
#define OTA_WRITE_BUF_SIZE 4096

static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_target_partition = NULL;
static bool s_ota_in_progress = false;
static bool s_ota_failed = false;
static bool s_drivers_were_enabled = false;
static int s_expected_seq = -1;
static uint32_t s_bytes_written = 0;
static uint32_t s_expected_size = 0;
static uint32_t s_last_progress_bytes = 0;
static uint8_t s_write_buf[OTA_WRITE_BUF_SIZE];
static size_t s_write_buf_len = 0;

static void ota_restore_operation(app_context_t *ctx);

static void ota_delayed_validate_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(OTA_VALIDATE_DELAY_MS));
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firmware validado apos %d s de operacao estavel. Rollback cancelado.",
                 OTA_VALIDATE_DELAY_MS / 1000);
    } else {
        ESP_LOGE(TAG, "Falha ao validar firmware: %s", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

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
        // Validar logo no boot tornaria o rollback inutil: um firmware que trava segundos
        // depois (CAN, motion, etc.) ficaria gravado. Confirma so apos operacao estavel.
        ESP_LOGW(TAG, "Boot de particao OTA pendente de verificacao (%s). Validacao automatica em %d s "
                      "(ou use OTA CONFIRM).", running->label, OTA_VALIDATE_DELAY_MS / 1000);
        if (xTaskCreate(ota_delayed_validate_task, "ota_validate", 3072, NULL, 2, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar tarefa de validacao; validando imediatamente.");
            (void)esp_ota_mark_app_valid_cancel_rollback();
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

    // 2. Interrompe o movimento em curso (nao so a fila) e so entao desenergiza os drivers
    if (!s_ota_in_progress) {
        s_drivers_were_enabled = ctx->state.drivers_enabled;
    }
    (void)motion_request_stop(ctx);
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
        ota_restore_operation(ctx);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Particao de destino: %s (offset 0x%08" PRIX32 ", tamanho %" PRIu32 " KB)",
             update_partition->label, update_partition->address, update_partition->size / 1024);

    // 8. Inicializa gravacao OTA na particao flash
    // Tamanho conhecido: apaga so a area da imagem antes (OTA_READY sai depois disso).
    // Desconhecido: apaga setor a setor durante a escrita — OTA_SIZE_UNKNOWN apagaria os
    // 6 MB da particao inteira antes de aceitar dados.
    esp_err_t err = esp_ota_begin(update_partition, image_size > 0 ? image_size : OTA_WITH_SEQUENTIAL_WRITES,
                                  &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha em esp_ota_begin(): %s", esp_err_to_name(err));
        ota_restore_operation(ctx);
        return err;
    }

    s_target_partition = update_partition;
    s_ota_in_progress = true;
    s_ota_failed = false;
    s_expected_seq = -1;
    s_write_buf_len = 0;
    s_bytes_written = 0;
    s_expected_size = image_size;
    s_last_progress_bytes = 0;

    ESP_LOGI(TAG, "Nó pronto para receber dados binarios de firmware via CAN/Serial!");
    return ESP_OK;
}

static esp_err_t ota_flush_write_buffer(void)
{
    if (s_write_buf_len == 0) {
        return ESP_OK;
    }
    esp_err_t err = esp_ota_write(s_ota_handle, s_write_buf, s_write_buf_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao gravar bloco OTA: %s", esp_err_to_name(err));
        return err;
    }
    s_write_buf_len = 0;
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

    // Uma escrita de flash por frame de 6 bytes custaria ~170 mil operacoes por imagem
    // (cada uma pausando o cache). Acumula em 4 KB e grava de uma vez.
    const uint8_t *src = (const uint8_t *)data;
    size_t remaining = length;
    while (remaining > 0) {
        size_t room = OTA_WRITE_BUF_SIZE - s_write_buf_len;
        size_t n = (remaining < room) ? remaining : room;
        memcpy(&s_write_buf[s_write_buf_len], src, n);
        s_write_buf_len += n;
        src += n;
        remaining -= n;
        if (s_write_buf_len == OTA_WRITE_BUF_SIZE) {
            ESP_RETURN_ON_ERROR(ota_flush_write_buffer(), TAG, "flush OTA");
        }
    }

    s_bytes_written += length;
    return ESP_OK;
}

/* Volta a operacao normal apos OTA abortado/falho: reenergiza os drivers se estavam ligados
 * antes e invalida o home do Z (desenergizado, o eixo pode ter se deslocado). */
static void ota_restore_operation(app_context_t *ctx)
{
    if (ctx != NULL) {
        if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctx->state.ota_in_progress = false;
            ctx->state.homed[AXIS_Z_ID] = false;
            xSemaphoreGive(ctx->state_mutex);
        } else {
            ctx->state.ota_in_progress = false;
            ctx->state.homed[AXIS_Z_ID] = false;
        }
        if (s_drivers_were_enabled) {
            hardware_set_driver_enable(ctx, true);
        }
    }
    status_led_set_mode(STATUS_LED_MODE_AUTO);
}

esp_err_t ota_finalize_and_reboot(app_context_t *ctx)
{
    if (!s_ota_in_progress || s_ota_handle == 0 || s_target_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Finalizando gravacao OTA. Total gravado: %" PRIu32 " bytes. Validando hash...", s_bytes_written);

    esp_err_t err = s_ota_failed ? ESP_ERR_INVALID_CRC : ota_flush_write_buffer();
    if (err == ESP_OK && s_expected_size > 0 && s_bytes_written != s_expected_size) {
        ESP_LOGE(TAG, "Tamanho recebido (%" PRIu32 ") difere do anunciado (%" PRIu32 ")",
                 s_bytes_written, s_expected_size);
        err = ESP_ERR_INVALID_SIZE;
    }
    if (err == ESP_OK) {
        err = esp_ota_end(s_ota_handle);
    } else {
        esp_ota_abort(s_ota_handle);
    }
    s_ota_handle = 0;
    s_ota_in_progress = false;
    s_write_buf_len = 0;

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Validacao do firmware falhou: %s", esp_err_to_name(err));
        ota_restore_operation(ctx);
        if (ctx) {
            (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, (uint8_t)(err & 0xFF));
        }
        return err;
    }

    err = esp_ota_set_boot_partition(s_target_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao definir proxima particao de boot: %s", esp_err_to_name(err));
        ota_restore_operation(ctx);
        if (ctx) {
            (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, (uint8_t)(err & 0xFF));
        }
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
    (void)storage_flush(); // nao perder alteracoes ainda pendentes na janela de 300 ms
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
    s_write_buf_len = 0;

    ota_restore_operation(ctx);
    if (ctx) {
        (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id, 0xFF);
    }
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
    (void)storage_flush();
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
        if (!s_ota_in_progress || s_ota_failed) {
            return true;
        }
        if (len > 2) {
            // Sequencia de 8 bits (wrap 255->0): duplicata e ignorada; lacuna significa
            // frame perdido (ex.: RX lotado durante o erase) e a imagem ficaria corrompida.
            uint8_t seq = data[1];
            if (s_expected_seq >= 0 && seq != (uint8_t)s_expected_seq) {
                if (seq == (uint8_t)(s_expected_seq - 1)) {
                    return true;
                }
                ESP_LOGE(TAG, "OTA: frame perdido (esperado seq=%d, recebido %u apos %" PRIu32 " bytes). Sessao invalidada.",
                         s_expected_seq, (unsigned)seq, s_bytes_written);
                s_ota_failed = true;
                (void)can_send_event(ctx, CAN_EVT_OTA_ERROR, ctx->settings.node_id,
                                     (uint8_t)(ESP_ERR_INVALID_CRC & 0xFF));
                return true;
            }
            s_expected_seq = (int)((seq + 1U) & 0xFFU);

            esp_err_t err = ota_write_chunk(&data[2], len - 2);
            if (err != ESP_OK) {
                s_ota_failed = true;
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
