#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "app_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATUS_LED_MODE_AUTO = 0, // Segue automaticamente o estado operacional do nó
    STATUS_LED_MODE_MANUAL,   // Fixado pelo usuário (via comando ou teste)
} status_led_mode_t;

typedef enum {
    STATUS_LED_EFFECT_SOLID = 0,
    STATUS_LED_EFFECT_BLINK_FAST, // ~4 Hz (Alarme / Erro)
    STATUS_LED_EFFECT_BLINK_MED,  // ~2 Hz (Homing)
    STATUS_LED_EFFECT_BREATHE,    // Pulsação suave (Movimento / Aguardando)
    STATUS_LED_EFFECT_DIM,        // Brilho reduzido (Standby)
} status_led_effect_t;

/**
 * @brief Inicializa o canal RMT para controle do LED RGB WS2812 onboard (GPIO 48)
 *        e cria a tarefa de atualizacao dinamica de status.
 */
esp_err_t status_led_init(app_context_t *ctx);

/**
 * @brief Define o modo de operacao do LED de status (AUTO ou MANUAL).
 */
esp_err_t status_led_set_mode(status_led_mode_t mode);

/**
 * @brief Retorna o modo de operacao atual do LED de status.
 */
status_led_mode_t status_led_get_mode(void);

/**
 * @brief Define manualmente a cor RGB do LED (alterna para modo MANUAL).
 * @param r Vermelho (0..255)
 * @param g Verde (0..255)
 * @param b Azul (0..255)
 */
esp_err_t status_led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Ajusta o percentual de brilho global do LED.
 * @param brightness_pct Percentual de brilho (0 a 100%)
 */
esp_err_t status_led_set_brightness(uint8_t brightness_pct);

/**
 * @brief Retorna o percentual de brilho atual do LED.
 */
uint8_t status_led_get_brightness(void);

/**
 * @brief Formata uma string descritiva com o status atual do LED (cor, modo, brilho e significado).
 */
void status_led_get_current_status(char *out_buf, size_t max_len);

#ifdef __cplusplus
}
#endif
