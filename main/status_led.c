#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "status_led.h"
#include "app_defs.h"

#define TAG "status_led"
#define RMT_WS2812_RESOLUTION_HZ 10000000 // 10MHz (0.1us por tick)

static rmt_channel_handle_t s_led_chan = NULL;
static rmt_encoder_handle_t s_led_encoder = NULL;
static SemaphoreHandle_t s_led_mutex = NULL;

static status_led_mode_t s_led_mode = STATUS_LED_MODE_AUTO;
static uint8_t s_manual_r = 0;
static uint8_t s_manual_g = 0;
static uint8_t s_manual_b = 0;
static uint8_t s_brightness_pct = DEFAULT_STATUS_LED_BRIGHTNESS;
static char s_current_desc[64] = "INICIALIZANDO";

static esp_err_t ws2812_write_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_chan == NULL || s_led_encoder == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // WS2812 utiliza ordem de bytes GRB: Green, Red, Blue
    const uint8_t grb[3] = {g, r, b};
    rmt_symbol_word_t symbols[25];
    size_t sym_idx = 0;

    for (int byte_idx = 0; byte_idx < 3; ++byte_idx) {
        uint8_t byte_val = grb[byte_idx];
        for (int bit = 7; bit >= 0; --bit) {
            if (byte_val & (1U << bit)) {
                // T1: 0.9us Alto, 0.35us Baixo (a 10MHz -> 9 ticks e 3 ticks)
                symbols[sym_idx++] = (rmt_symbol_word_t){
                    .level0 = 1,
                    .duration0 = 9,
                    .level1 = 0,
                    .duration1 = 3,
                };
            } else {
                // T0: 0.35us Alto, 0.9us Baixo (a 10MHz -> 3 ticks e 9 ticks)
                symbols[sym_idx++] = (rmt_symbol_word_t){
                    .level0 = 1,
                    .duration0 = 3,
                    .level1 = 0,
                    .duration1 = 9,
                };
            }
        }
    }

    // Reset code (> 280us em nível baixo para latch seguro em todas as revisões WS2812/WS2812B/SK6812)
    // 300us = 3000 ticks a 10MHz
    symbols[sym_idx++] = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = 1500,
        .level1 = 0,
        .duration1 = 1500,
    };

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    esp_err_t ret = rmt_transmit(s_led_chan, s_led_encoder, symbols, sizeof(symbols), &tx_config);
    if (ret == ESP_OK) {
        (void)rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(20));
    }
    return ret;
}

static void get_auto_color_and_effect(const runtime_state_t *st, uint8_t *r, uint8_t *g, uint8_t *b,
                                      status_led_effect_t *eff, const char **desc)
{
    // 0. Modo OTA: Atualizacao de Firmware em Andamento
    if (st->ota_in_progress) {
        *r = 255; *g = 120; *b = 0; // Laranja / Âmbar
        *eff = STATUS_LED_EFFECT_BLINK_FAST;
        if (desc) *desc = "MODO OTA SEGURO (AGUARDANDO/GRAVANDO FLASH)";
        return;
    }

    // 1. Alarme Crítico: Eixo Z Bloqueado (fim de curso acionado inesperadamente ou erro Z)
    if (st->z_bloqueado) {
        *r = 255; *g = 0; *b = 0; // Vermelho
        *eff = STATUS_LED_EFFECT_BLINK_FAST; // 4 Hz
        if (desc) *desc = "ALARME Z BLOQUEADO";
        return;
    }

    // 2. Homing ativo em qualquer eixo
    if (st->in_homing || st->em_homing_z) {
        *r = 255; *g = 160; *b = 0; // Amarelo / Âmbar
        *eff = STATUS_LED_EFFECT_BLINK_MED; // 2 Hz
        if (desc) *desc = "HOMING / CALIBRACAO ATIVA";
        return;
    }

    // 3. Movimento ativo nos eixos
    if (st->in_motion) {
        *r = 0; *g = 220; *b = 255; // Ciano
        *eff = STATUS_LED_EFFECT_BREATHE;
        if (desc) *desc = "MOVIMENTO ATIVO";
        return;
    }

    // 4. Drivers desabilitados / Standby
    if (!st->drivers_enabled) {
        *r = 100; *g = 100; *b = 100; // Branco suave
        *eff = STATUS_LED_EFFECT_DIM; // Brilho atenuado
        if (desc) *desc = "STANDBY (DRIVERS OFF)";
        return;
    }

    // 5. Sistema pronto: verificar se eixos C e A finalizaram o home
    bool homed_ca = (st->homed[0] && st->homed[1]);

    if (homed_ca) {
        if (st->can_online) {
            // Pronto + CAN Online + Home Feito
            *r = 0; *g = 255; *b = 30; // Verde vivo
            *eff = STATUS_LED_EFFECT_SOLID;
            if (desc) *desc = "PRONTO (CAN ONLINE + HOME OK)";
        } else {
            // Pronto + Modo Local (CAN Offline) + Home Feito
            *r = 0; *g = 80; *b = 255; // Azul Royal
            *eff = STATUS_LED_EFFECT_BREATHE; // Respiração suave
            if (desc) *desc = "PRONTO LOCAL (CAN OFFLINE + HOME OK)";
        }
    } else {
        // Inicializado, mas aguardando Home
        *r = 200; *g = 0; *b = 255; // Magenta / Roxo
        *eff = STATUS_LED_EFFECT_BREATHE;
        if (desc) *desc = "INICIALIZADO (AGUARDANDO HOME)";
    }
}

static void status_led_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    uint32_t tick_count = 0;

    ESP_LOGI(TAG, "Tarefa de LED de status iniciada (GPIO %d)", BOARD_RGB_LED_PIN);

    while (true) {
        uint8_t target_r = 0, target_g = 0, target_b = 0;
        status_led_effect_t effect = STATUS_LED_EFFECT_SOLID;
        status_led_mode_t mode = STATUS_LED_MODE_AUTO;
        uint8_t brightness = DEFAULT_STATUS_LED_BRIGHTNESS;
        const char *desc = "DESCONHECIDO";

        if (s_led_mutex && xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            mode = s_led_mode;
            brightness = s_brightness_pct;
            if (mode == STATUS_LED_MODE_MANUAL) {
                target_r = s_manual_r;
                target_g = s_manual_g;
                target_b = s_manual_b;
                effect = STATUS_LED_EFFECT_SOLID;
                desc = "MODO MANUAL";
            }
            xSemaphoreGive(s_led_mutex);
        }

        if (mode == STATUS_LED_MODE_AUTO) {
            runtime_state_t st_copy = {0};
            if (ctx->state_mutex && xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                st_copy = ctx->state;
                xSemaphoreGive(ctx->state_mutex);
            }
            get_auto_color_and_effect(&st_copy, &target_r, &target_g, &target_b, &effect, &desc);
        }

        // Salva descrição atual para o comando LED STATUS
        if (s_led_mutex && xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            strncpy(s_current_desc, desc, sizeof(s_current_desc) - 1);
            s_current_desc[sizeof(s_current_desc) - 1] = '\0';
            xSemaphoreGive(s_led_mutex);
        }

        // Calcular fator de animação (0 a 100%)
        uint32_t factor = 100;
        switch (effect) {
        case STATUS_LED_EFFECT_SOLID:
            factor = 100;
            break;
        case STATUS_LED_EFFECT_BLINK_FAST: // 4 Hz (periodo 250ms = 5 ticks a 50ms)
            factor = ((tick_count % 5) < 3) ? 100 : 0;
            break;
        case STATUS_LED_EFFECT_BLINK_MED:  // 2 Hz (periodo 500ms = 10 ticks a 50ms)
            factor = ((tick_count % 10) < 5) ? 100 : 0;
            break;
        case STATUS_LED_EFFECT_BREATHE: {  // Respiracao suave (ciclo 2 segundos = 40 ticks)
            float angle = (float)(tick_count % 40) * (2.0f * 3.14159265f / 40.0f);
            float sin_val = (sinf(angle) + 1.0f) * 0.5f; // 0.0 .. 1.0
            factor = 15 + (uint32_t)(85.0f * sin_val);   // 15% .. 100%
            break;
        }
        case STATUS_LED_EFFECT_DIM:
            factor = 12; // 12%
            break;
        }

        // Aplicar brilho global e fator de efeito
        uint8_t final_r = (uint8_t)(((uint32_t)target_r * brightness * factor) / 10000U);
        uint8_t final_g = (uint8_t)(((uint32_t)target_g * brightness * factor) / 10000U);
        uint8_t final_b = (uint8_t)(((uint32_t)target_b * brightness * factor) / 10000U);

        (void)ws2812_write_pixel(final_r, final_g, final_b);

        tick_count++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t status_led_init(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "ctx nulo");

    if (s_led_mutex == NULL) {
        s_led_mutex = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_led_mutex != NULL, ESP_ERR_NO_MEM, TAG, "Falha ao criar mutex do LED");
    }

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = BOARD_RGB_LED_PIN,
        .mem_block_symbols = 48,
        .resolution_hz = RMT_WS2812_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &s_led_chan), TAG,
                        "Falha ao criar canal RMT para LED de status");

    rmt_copy_encoder_config_t copy_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_config, &s_led_encoder), TAG,
                        "Falha ao criar encoder RMT para LED WS2812");

    ESP_RETURN_ON_ERROR(rmt_enable(s_led_chan), TAG,
                        "Falha ao habilitar canal RMT do LED de status");

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        status_led_task,
        "status_led",
        3072,
        ctx,
        3, // Prioridade 3 (adequada para UI/LED)
        NULL,
        0  // Core 0
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar task status_led");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LED RGB WS2812 de status inicializado no GPIO %d", BOARD_RGB_LED_PIN);
    return ESP_OK;
}

esp_err_t status_led_set_mode(status_led_mode_t mode)
{
    if (s_led_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_led_mode = mode;
        xSemaphoreGive(s_led_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

status_led_mode_t status_led_get_mode(void)
{
    status_led_mode_t mode = STATUS_LED_MODE_AUTO;
    if (s_led_mutex && xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        mode = s_led_mode;
        xSemaphoreGive(s_led_mutex);
    }
    return mode;
}

esp_err_t status_led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_manual_r = r;
        s_manual_g = g;
        s_manual_b = b;
        s_led_mode = STATUS_LED_MODE_MANUAL;
        xSemaphoreGive(s_led_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t status_led_set_brightness(uint8_t brightness_pct)
{
    if (brightness_pct > 100) {
        brightness_pct = 100;
    }
    if (s_led_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_brightness_pct = brightness_pct;
        xSemaphoreGive(s_led_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

uint8_t status_led_get_brightness(void)
{
    uint8_t b = DEFAULT_STATUS_LED_BRIGHTNESS;
    if (s_led_mutex && xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        b = s_brightness_pct;
        xSemaphoreGive(s_led_mutex);
    }
    return b;
}

void status_led_get_current_status(char *out_buf, size_t max_len)
{
    if (out_buf == NULL || max_len == 0) {
        return;
    }
    status_led_mode_t mode = STATUS_LED_MODE_AUTO;
    uint8_t bright = DEFAULT_STATUS_LED_BRIGHTNESS;
    char desc_copy[64] = "DESCONHECIDO";

    if (s_led_mutex && xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        mode = s_led_mode;
        bright = s_brightness_pct;
        strncpy(desc_copy, s_current_desc, sizeof(desc_copy) - 1);
        desc_copy[sizeof(desc_copy) - 1] = '\0';
        xSemaphoreGive(s_led_mutex);
    }

    snprintf(out_buf, max_len, "Modo: %s | Brilho: %u%% | Estado: %s",
             (mode == STATUS_LED_MODE_AUTO) ? "AUTO" : "MANUAL",
             (unsigned)bright,
             desc_copy);
}
