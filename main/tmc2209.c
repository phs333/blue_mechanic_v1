#include "tmc2209.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "storage.h"


#define TMC_UART_PORT UART_NUM_1
#define TMC_UART_BAUDRATE 115200
#define TMC_UART_RX_BUF_SIZE 256
#define TMC_UART_REPLY_TIMEOUT_MS 100

#define TMC_SYNC_BYTE 0x05U
#define TMC_WRITE_BIT 0x80U

#define TMC_REG_GCONF 0x00U
#define TMC_REG_IOIN 0x06U
#define TMC_REG_IHOLD_IRUN 0x10U
#define TMC_REG_TPOWERDOWN 0x11U
#define TMC_REG_CHOPCONF 0x6CU

static volatile bool s_uart_installed;

static bool axis_to_index(char axis, size_t *axis_index);
static const char *axis_name_from_index(size_t axis_index);

static const char *driver_mode_to_string(driver_bus_mode_t mode);
static void mark_step_dir_fallback(app_context_t *ctx);
static void reset_axis_online_flags(app_context_t *ctx);
static void tmc_uart_deinit(void);
static esp_err_t tmc_uart_init(const persisted_settings_t *settings);
static uint8_t tmc_crc8(const uint8_t *data, size_t len);
static esp_err_t tmc_write_register_raw(uint8_t slave_addr, uint8_t reg_addr, uint32_t value);
static esp_err_t tmc_read_register_raw(uint8_t slave_addr, uint8_t reg_addr, uint32_t *value);
static esp_err_t tmc_apply_axis_defaults(app_context_t *ctx, size_t axis_index);

esp_err_t tmc2209_init(app_context_t *ctx)
{
    return tmc2209_apply_settings(ctx);
}

esp_err_t tmc2209_apply_settings(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    ctx->state.driver_mode_requested = (driver_bus_mode_t)ctx->settings.driver_bus_mode;
    reset_axis_online_flags(ctx);
    tmc_uart_deinit();

    if (ctx->state.driver_mode_requested == DRIVER_BUS_MODE_STEP_DIR_ONLY) {
        mark_step_dir_fallback(ctx);
        ESP_LOGI(APP_TAG, "Drivers configurados para STEP/DIR puro.");
        return ESP_OK;
    }

    esp_err_t err = tmc_uart_init(&ctx->settings);
    if (err != ESP_OK) {
        mark_step_dir_fallback(ctx);
        ESP_LOGW(APP_TAG, "UART TMC indisponivel (%s). Mantendo STEP/DIR.", esp_err_to_name(err));
        return err;
    }

    size_t online_count = 0;
    for (size_t axis_index = 0; axis_index < AXIS_COUNT; ++axis_index) {
        err = tmc_apply_axis_defaults(ctx, axis_index);
        ctx->state.tmc_axis_online[axis_index] = (err == ESP_OK);
        if (err == ESP_OK) {
            ++online_count;
        } else {
            ESP_LOGW(APP_TAG, "Eixo %s sem resposta via UART TMC (%s).", axis_name_from_index(axis_index), esp_err_to_name(err));
        }
    }

    ctx->state.tmc_uart_ready = (online_count > 0);
    ctx->state.driver_mode_active = ctx->state.tmc_uart_ready ? DRIVER_BUS_MODE_UART_OPTIONAL : DRIVER_BUS_MODE_STEP_DIR_ONLY;

    if (!ctx->state.tmc_uart_ready) {
        ESP_LOGW(APP_TAG, "Nenhum TMC2209 respondeu na UART. Movimento continua em STEP/DIR.");
    } else {
        ESP_LOGI(APP_TAG, "UART TMC ativa em %u/%u drivers.", (unsigned)online_count, (unsigned)AXIS_COUNT);
    }

    return ctx->state.tmc_uart_ready ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t tmc2209_read_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t *value)
{
    size_t axis_index = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL && value != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "arg invalido");
    ESP_RETURN_ON_FALSE(axis_to_index(axis, &axis_index), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao ativa");
    return tmc_read_register_raw(ctx->settings.tmc_slave_addr[axis_index], reg_addr, value);
}

esp_err_t tmc2209_write_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t value)
{
    size_t axis_index = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    ESP_RETURN_ON_FALSE(axis_to_index(axis, &axis_index), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao ativa");
    return tmc_write_register_raw(ctx->settings.tmc_slave_addr[axis_index], reg_addr, value);
}

void tmc2209_print_status(const app_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    printf("Driver bus: solicitado=%s ativo=%s\n",
           driver_mode_to_string(ctx->state.driver_mode_requested),
           driver_mode_to_string(ctx->state.driver_mode_active));
    printf("TMC UART: %s, TX=%u RX=%u\n",
           ctx->state.tmc_uart_ready ? "ATIVA" : "INATIVA",
           (unsigned)TMC_UART_TX_PIN,
           (unsigned)TMC_UART_RX_PIN);

    for (size_t axis_index = 0; axis_index < AXIS_COUNT; ++axis_index) {
        printf("TMC %s: addr=%u ihold=%umA irun=%umA delay=%u status=%s\n",
               axis_name_from_index(axis_index),
               (unsigned)ctx->settings.tmc_slave_addr[axis_index],
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[axis_index]),
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[axis_index]),
               (unsigned)ctx->settings.tmc_ihold_delay[axis_index],
               ctx->state.tmc_axis_online[axis_index] ? "UART OK" : "STEP/DIR");
    }
}

static bool axis_to_index(char axis, size_t *axis_index)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'X') {
        *axis_index = AXIS_X_ID;
        return true;
    }
    if (axis_upper == 'Y') {
        *axis_index = AXIS_Y_ID;
        return true;
    }
    if (axis_upper == 'Z') {
        *axis_index = AXIS_Z_ID;
        return true;
    }
    return false;
}

static const char *axis_name_from_index(size_t axis_index)
{
    static const char *const names[AXIS_COUNT] = {"X", "Y", "Z"};
    return (axis_index < AXIS_COUNT) ? names[axis_index] : "?";
}



static const char *driver_mode_to_string(driver_bus_mode_t mode)
{
    switch (mode) {
    case DRIVER_BUS_MODE_STEP_DIR_ONLY:
        return "STEP/DIR";
    case DRIVER_BUS_MODE_UART_OPTIONAL:
        return "STEP/DIR + UART";
    default:
        return "DESCONHECIDO";
    }
}

static void mark_step_dir_fallback(app_context_t *ctx)
{
    ctx->state.driver_mode_active = DRIVER_BUS_MODE_STEP_DIR_ONLY;
    ctx->state.tmc_uart_ready = false;
    reset_axis_online_flags(ctx);
}

static void reset_axis_online_flags(app_context_t *ctx)
{
    for (size_t axis_index = 0; axis_index < AXIS_COUNT; ++axis_index) {
        ctx->state.tmc_axis_online[axis_index] = false;
    }
}

static void tmc_uart_deinit(void)
{
    if (s_uart_installed) {
        esp_err_t del_err = uart_driver_delete(TMC_UART_PORT);
        if (del_err == ESP_OK || del_err == ESP_ERR_INVALID_STATE) {
            s_uart_installed = false;
        }

        if (TMC_UART_TX_PIN == TMC_UART_RX_PIN) {
            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << TMC_UART_TX_PIN),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&io_conf);
            gpio_set_level(TMC_UART_TX_PIN, 0);
        }
    }
}

static esp_err_t tmc_uart_init(const persisted_settings_t *settings)
{
    esp_err_t ret = ESP_OK;
    gpio_num_t tx_pin = TMC_UART_TX_PIN;
    gpio_num_t rx_pin = TMC_UART_RX_PIN;

    ESP_RETURN_ON_FALSE(tx_pin != GPIO_NUM_NC && rx_pin != GPIO_NUM_NC, ESP_ERR_INVALID_STATE, APP_TAG, "Pinos UART TMC nao configurados");

    if (tx_pin == rx_pin) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << tx_pin),
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(APP_TAG, "Falha ao configurar GPIO PDN/UART: %s", esp_err_to_name(ret));
            return ret;
        }

        gpio_set_level(tx_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uart_config_t uart_cfg = {
        .baud_rate = TMC_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_param_config(TMC_UART_PORT, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "Falha ao configurar UART TMC: %s", esp_err_to_name(ret));
        goto err;
    }

    ret = uart_driver_install(TMC_UART_PORT, TMC_UART_RX_BUF_SIZE, TMC_UART_RX_BUF_SIZE, 10, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "Falha ao instalar UART TMC: %s (0x%x)", esp_err_to_name(ret), ret);
        goto err;
    }

    if (tx_pin == rx_pin) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << tx_pin),
            .mode = GPIO_MODE_INPUT_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(APP_TAG, "Falha ao reconfigurar GPIO Open-Drain para UART: %s", esp_err_to_name(ret));
            goto err;
        }

        ret = uart_set_pin(TMC_UART_PORT, tx_pin, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(APP_TAG, "Falha ao configurar pinos UART TMC: %s", esp_err_to_name(ret));
            goto err;
        }
    } else {
        ret = uart_set_pin(TMC_UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(APP_TAG, "Falha ao configurar pinos UART TMC: %s", esp_err_to_name(ret));
            goto err;
        }
    }

    ret = uart_flush_input(TMC_UART_PORT);
    if (ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "Falha ao limpar RX UART TMC: %s", esp_err_to_name(ret));
        goto err;
    }
    s_uart_installed = true;
    return ESP_OK;

err:
    (void)uart_driver_delete(TMC_UART_PORT);
    return ret;
}

static uint8_t tmc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;

    for (size_t byte_index = 0; byte_index < len; ++byte_index) {
        uint8_t current = data[byte_index];
        for (int bit = 0; bit < 8; ++bit) {
            if (((crc >> 7) ^ (current & 0x01U)) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            current >>= 1;
        }
    }

    return crc;
}

static esp_err_t tmc_write_register_raw(uint8_t slave_addr, uint8_t reg_addr, uint32_t value)
{
    uint8_t frame[8] = {
        TMC_SYNC_BYTE,
        slave_addr,
        (uint8_t)(reg_addr | TMC_WRITE_BIT),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)value,
        0,
    };
    frame[7] = tmc_crc8(frame, 7);

    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao instalada");
    ESP_RETURN_ON_ERROR(uart_flush_input(TMC_UART_PORT), APP_TAG, "Falha ao limpar RX UART");
    ESP_RETURN_ON_FALSE(uart_write_bytes(TMC_UART_PORT, frame, sizeof(frame)) == (int)sizeof(frame), ESP_FAIL, APP_TAG, "Falha ao escrever UART TMC");
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20)), APP_TAG, "Timeout TX UART TMC");
    return ESP_OK;
}

static esp_err_t tmc_read_register_raw(uint8_t slave_addr, uint8_t reg_addr, uint32_t *value)
{
    uint8_t request[4] = {
        TMC_SYNC_BYTE,
        slave_addr,
        reg_addr,
        0,
    };
    uint8_t rx_buf[24];

    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "value nulo");
    request[3] = tmc_crc8(request, 3);

    ESP_RETURN_ON_ERROR(uart_flush_input(TMC_UART_PORT), APP_TAG, "Falha ao limpar RX UART");
    ESP_RETURN_ON_FALSE(uart_write_bytes(TMC_UART_PORT, request, sizeof(request)) == (int)sizeof(request), ESP_FAIL, APP_TAG, "Falha ao enviar read UART");
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20)), APP_TAG, "Timeout TX UART TMC");

    if (TMC_UART_TX_PIN == TMC_UART_RX_PIN) {
        uint8_t dummy[4];
        (void)uart_read_bytes(TMC_UART_PORT, dummy, sizeof(request), pdMS_TO_TICKS(10));
    }

    int len = uart_read_bytes(TMC_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(TMC_UART_REPLY_TIMEOUT_MS));
    ESP_RETURN_ON_FALSE(len >= 8, ESP_ERR_TIMEOUT, APP_TAG, "Sem resposta UART TMC");

    for (int start = 0; start <= (len - 8); ++start) {
        if (rx_buf[start] != TMC_SYNC_BYTE) {
            continue;
        }
        if (rx_buf[start + 2] != reg_addr) {
            continue;
        }
        if (tmc_crc8(&rx_buf[start], 7) != rx_buf[start + 7]) {
            continue;
        }

        *value = ((uint32_t)rx_buf[start + 3] << 24) |
                 ((uint32_t)rx_buf[start + 4] << 16) |
                 ((uint32_t)rx_buf[start + 5] << 8) |
                 (uint32_t)rx_buf[start + 6];
        return ESP_OK;
    }

    return ESP_ERR_INVALID_CRC;
}

static int microsteps_to_mres(uint16_t microsteps)
{
    switch (microsteps) {
    case 256: return 0;
    case 128: return 1;
    case 64:  return 2;
    case 32:  return 3;
    case 16:  return 4;
    case 8:   return 5;
    case 4:   return 6;
    case 2:   return 7;
    case 1:   return 8;
    default:  return 4; // fallback 16
    }
}

static esp_err_t tmc_apply_axis_defaults(app_context_t *ctx, size_t axis_index)
{
    uint32_t verify_value = 0;
    uint32_t gconf = (1U << 6) | (1U << 7);
    int mres = microsteps_to_mres(ctx->settings.tmc_microsteps[axis_index]);
    uint32_t chopconf = 0x10000053U | ((uint32_t)mres << 24); // 16 micropassos (mres=4) com interpolacao para 256 (intpol=1)
    uint32_t ihold_irun =
        ((uint32_t)(ctx->settings.tmc_ihold_delay[axis_index] & 0x0FU) << 16) |
        ((uint32_t)(ctx->settings.tmc_irun[axis_index] & 0x1FU) << 8) |
        (uint32_t)(ctx->settings.tmc_ihold[axis_index] & 0x1FU);

    ESP_RETURN_ON_ERROR(
        tmc_read_register_raw(ctx->settings.tmc_slave_addr[axis_index], TMC_REG_IOIN, &verify_value),
        APP_TAG,
        "Falha ao ler IOIN");
    ESP_RETURN_ON_ERROR(
        tmc_write_register_raw(ctx->settings.tmc_slave_addr[axis_index], TMC_REG_GCONF, gconf),
        APP_TAG,
        "Falha ao gravar GCONF");
    ESP_RETURN_ON_ERROR(
        tmc_write_register_raw(ctx->settings.tmc_slave_addr[axis_index], TMC_REG_CHOPCONF, chopconf),
        APP_TAG,
        "Falha ao gravar CHOPCONF");
    ESP_RETURN_ON_ERROR(
        tmc_write_register_raw(ctx->settings.tmc_slave_addr[axis_index], TMC_REG_IHOLD_IRUN, ihold_irun),
        APP_TAG,
        "Falha ao gravar IHOLD_IRUN");
    ESP_RETURN_ON_ERROR(
        tmc_write_register_raw(ctx->settings.tmc_slave_addr[axis_index], TMC_REG_TPOWERDOWN, 20U),
        APP_TAG,
        "Falha ao gravar TPOWERDOWN");
    return ESP_OK;
}

uint8_t tmc2209_ma_to_cs(uint16_t ma)
{
    float cs_f = ((float)ma / 59.846f) - 1.0f;
    int cs = (int)(cs_f + 0.5f);
    if (cs < 0) {
        return 0;
    }
    if (cs > 31) {
        return 31;
    }
    return (uint8_t)cs;
}

uint16_t tmc2209_cs_to_ma(uint8_t cs)
{
    if (cs > 31) {
        cs = 31;
    }
    return (uint16_t)(((float)cs + 1.0f) * 59.846f);
}

esp_err_t tmc2209_set_spreadcycle(app_context_t *ctx, char axis, bool enabled)
{
    uint32_t gconf = 0;
    ESP_RETURN_ON_ERROR(tmc2209_read_register(ctx, axis, TMC_REG_GCONF, &gconf), APP_TAG, "Erro ao ler GCONF");
    if (enabled) {
        gconf |= (1U << 2);
    } else {
        gconf &= ~(1U << 2);
    }
    ESP_RETURN_ON_ERROR(tmc2209_write_register(ctx, axis, TMC_REG_GCONF, gconf), APP_TAG, "Erro ao gravar GCONF");
    return ESP_OK;
}

esp_err_t tmc2209_set_microsteps(app_context_t *ctx, char axis, uint16_t microsteps)
{
    size_t axis_index = 0;
    if (!axis_to_index(axis, &axis_index)) {
        return ESP_ERR_INVALID_ARG;
    }
    int mres = microsteps_to_mres(microsteps);
    if (mres < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t chopconf = 0;
    ESP_RETURN_ON_ERROR(tmc2209_read_register(ctx, axis, TMC_REG_CHOPCONF, &chopconf), APP_TAG, "Erro ao ler CHOPCONF");
    chopconf &= ~(0x0FU << 24);
    chopconf |= ((uint32_t)mres << 24);
    ESP_RETURN_ON_ERROR(tmc2209_write_register(ctx, axis, TMC_REG_CHOPCONF, chopconf), APP_TAG, "Erro ao gravar CHOPCONF");

    ctx->settings.tmc_microsteps[axis_index] = microsteps;
    (void)storage_save_settings(&ctx->settings);
    return ESP_OK;
}
