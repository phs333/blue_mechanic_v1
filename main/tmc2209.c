#include "tmc2209.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "storage.h"


#define TMC_UART_PORT UART_NUM_1
#define TMC_UART_BAUDRATE 115200
#define TMC_UART_RX_BUF_SIZE 256
#define TMC_UART_REPLY_TIMEOUT_MS 100

#define TMC_SYNC_BYTE 0x05U
#define TMC_WRITE_BIT 0x80U

#define TMC_REG_GCONF 0x00U
#define TMC_REG_GSTAT 0x01U
#define TMC_REG_IFCNT 0x02U
#define TMC_REG_IOIN 0x06U
#define TMC_REG_IHOLD_IRUN 0x10U
#define TMC_REG_TPOWERDOWN 0x11U
#define TMC_REG_TPWMTHRS 0x13U
#define TMC_REG_CHOPCONF 0x6CU
#define TMC_REG_PWMCONF 0x70U

#define TMC_GSTAT_RESET  0x01U  // driver reiniciou (queda de VM): registradores voltaram ao padrao
#define TMC_GSTAT_DRV_ERR 0x02U // sobretemperatura ou curto: driver desligado ate limpar o flag

#define TMC_FCLK_HZ 12000000.0f
#define TMC_APPLY_ATTEMPTS 3
#define TMC_MONITOR_PERIOD_MS 2000

static volatile bool s_uart_installed;
// Serializa as transacoes da UART single-wire: console (DRIVER REG/APPLY) e o monitor
static SemaphoreHandle_t s_tmc_lock;
static TaskHandle_t s_monitor_task;
static bool s_drv_err_reported[AXIS_COUNT];

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

static void tmc_lock(void)
{
    if (s_tmc_lock == NULL) {
        s_tmc_lock = xSemaphoreCreateRecursiveMutex();
    }
    (void)xSemaphoreTakeRecursive(s_tmc_lock, portMAX_DELAY);
}

static void tmc_unlock(void)
{
    (void)xSemaphoreGiveRecursive(s_tmc_lock);
}

esp_err_t tmc2209_init(app_context_t *ctx)
{
    return tmc2209_apply_settings(ctx);
}

esp_err_t tmc2209_apply_settings(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    tmc_lock();
    ctx->state.driver_mode_requested = (driver_bus_mode_t)ctx->settings.driver_bus_mode;
    reset_axis_online_flags(ctx);
    tmc_uart_deinit();

    if (ctx->state.driver_mode_requested == DRIVER_BUS_MODE_STEP_DIR_ONLY) {
        mark_step_dir_fallback(ctx);
        tmc_unlock();
        ESP_LOGI(APP_TAG, "Drivers configurados para STEP/DIR puro.");
        return ESP_OK;
    }

    esp_err_t err = tmc_uart_init(&ctx->settings);
    if (err != ESP_OK) {
        mark_step_dir_fallback(ctx);
        tmc_unlock();
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
            ESP_LOGW(APP_TAG, "Eixo %s sem resposta/confirmacao via UART TMC (%s).",
                     axis_name_from_index(axis_index), esp_err_to_name(err));
        }
    }

    ctx->state.tmc_uart_ready = (online_count > 0);
    ctx->state.driver_mode_active = ctx->state.tmc_uart_ready ? DRIVER_BUS_MODE_UART_OPTIONAL : DRIVER_BUS_MODE_STEP_DIR_ONLY;
    tmc_unlock();

    if (!ctx->state.tmc_uart_ready) {
        ESP_LOGW(APP_TAG, "Nenhum TMC2209 respondeu na UART. Movimento continua em STEP/DIR; "
                          "o monitor tenta de novo a cada %d s.", TMC_MONITOR_PERIOD_MS / 1000);
    } else {
        ESP_LOGI(APP_TAG, "UART TMC ativa em %u/%u drivers (escritas confirmadas por IFCNT).",
                 (unsigned)online_count, (unsigned)AXIS_COUNT);
    }

    return ctx->state.tmc_uart_ready ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/*
 * Os registradores do TMC2209 sao volateis: se a alimentacao dos motores (VM) sobe
 * depois do ESP32, ou oscila, o driver reinicia com a corrente do VREF/OTP e a
 * configuracao UART se perde. O monitor detecta isso por GSTAT.reset (ou pelo driver
 * voltar a responder) e reaplica correntes, microsteps e modo de chopper.
 */
static void tmc_monitor_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(TMC_MONITOR_PERIOD_MS));
        if (ctx->state.ota_in_progress ||
            ctx->state.driver_mode_requested != DRIVER_BUS_MODE_UART_OPTIONAL || !s_uart_installed) {
            continue;
        }

        tmc_lock();
        size_t online_count = 0;
        for (size_t idx = 0; idx < AXIS_COUNT; ++idx) {
            uint8_t addr = ctx->settings.tmc_slave_addr[idx];
            uint32_t gstat = 0;
            esp_err_t err = tmc_read_register_raw(addr, TMC_REG_GSTAT, &gstat);
            if (err != ESP_OK) {
                if (ctx->state.tmc_axis_online[idx]) {
                    ESP_LOGW(APP_TAG, "TMC %s parou de responder na UART (VM desligado?).",
                             axis_name_from_index(idx));
                }
                ctx->state.tmc_axis_online[idx] = false;
                continue;
            }

            bool needs_apply = !ctx->state.tmc_axis_online[idx] || (gstat & TMC_GSTAT_RESET);
            if (needs_apply) {
                esp_err_t aerr = tmc_apply_axis_defaults(ctx, idx);
                ctx->state.tmc_axis_online[idx] = (aerr == ESP_OK);
                if (aerr == ESP_OK) {
                    ESP_LOGW(APP_TAG, "TMC %s %s: correntes e modo reaplicados (IRUN=%umA IHOLD=%umA).",
                             axis_name_from_index(idx),
                             (gstat & TMC_GSTAT_RESET) ? "reiniciou" : "voltou a responder",
                             (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[idx]),
                             (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[idx]));
                } else {
                    ESP_LOGW(APP_TAG, "TMC %s: falha ao reaplicar configuracao (%s).",
                             axis_name_from_index(idx), esp_err_to_name(aerr));
                }
            }

            if (gstat & TMC_GSTAT_DRV_ERR) {
                if (!s_drv_err_reported[idx]) {
                    ESP_LOGE(APP_TAG, "TMC %s: drv_err (sobretemperatura ou curto na bobina). "
                                      "Driver desligado pelo proprio TMC; verifique fiacao/dissipacao.",
                             axis_name_from_index(idx));
                    s_drv_err_reported[idx] = true;
                }
            } else {
                s_drv_err_reported[idx] = false;
            }

            if (ctx->state.tmc_axis_online[idx]) {
                ++online_count;
            }
        }
        ctx->state.tmc_uart_ready = (online_count > 0);
        ctx->state.driver_mode_active = ctx->state.tmc_uart_ready ? DRIVER_BUS_MODE_UART_OPTIONAL
                                                                   : DRIVER_BUS_MODE_STEP_DIR_ONLY;
        tmc_unlock();
    }
}

esp_err_t tmc2209_start_monitor(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    if (s_monitor_task != NULL) {
        return ESP_OK;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(tmc_monitor_task, "tmc_monitor", 3584, ctx, 3,
                                            &s_monitor_task, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t tmc2209_read_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t *value)
{
    size_t axis_index = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL && value != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "arg invalido");
    ESP_RETURN_ON_FALSE(axis_to_index(axis, &axis_index), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao ativa");
    tmc_lock();
    esp_err_t err = tmc_read_register_raw(ctx->settings.tmc_slave_addr[axis_index], reg_addr, value);
    tmc_unlock();
    return err;
}

/* Escrita confirmada: o TMC2209 nao responde a escritas, mas incrementa IFCNT a cada
 * datagrama valido. Lido antes e depois, prova que o driver recebeu o valor. */
static esp_err_t tmc_write_verified(uint8_t addr, uint8_t reg_addr, uint32_t value)
{
    uint32_t before = 0, after = 0;
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_IFCNT, &before), APP_TAG, "IFCNT");
    ESP_RETURN_ON_ERROR(tmc_write_register_raw(addr, reg_addr, value), APP_TAG, "escrita TMC");
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_IFCNT, &after), APP_TAG, "IFCNT");
    return (((after - before) & 0xFFU) == 1U) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t tmc2209_write_register(app_context_t *ctx, char axis, uint8_t reg_addr, uint32_t value)
{
    size_t axis_index = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    ESP_RETURN_ON_FALSE(axis_to_index(axis, &axis_index), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao ativa");
    tmc_lock();
    esp_err_t err = tmc_write_verified(ctx->settings.tmc_slave_addr[axis_index], reg_addr, value);
    tmc_unlock();
    return err;
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
        printf("TMC %s: addr=%u ihold=%umA irun=%umA delay=%u stealth_ate=%.1f status=%s\n",
               axis_name_from_index(axis_index),
               (unsigned)ctx->settings.tmc_slave_addr[axis_index],
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[axis_index]),
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[axis_index]),
               (unsigned)ctx->settings.tmc_ihold_delay[axis_index],
               ctx->ext.stealth_max_speed[axis_index],
               ctx->state.tmc_axis_online[axis_index] ? "UART OK" : "STEP/DIR");
    }
}

static bool axis_to_index(char axis, size_t *axis_index)
{
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'C' || axis_upper == 'X') {
        *axis_index = AXIS_C_ID;
        return true;
    }
    if (axis_upper == 'A' || axis_upper == 'Y') {
        *axis_index = AXIS_A_ID;
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
    static const char *const names[AXIS_COUNT] = {"C", "A", "Z"};
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
    }
}

static esp_err_t tmc_uart_init(const persisted_settings_t *settings)
{
    (void)settings;
    esp_err_t ret = ESP_OK;
    gpio_num_t tx_pin = TMC_UART_TX_PIN;
    gpio_num_t rx_pin = TMC_UART_RX_PIN;

    ESP_RETURN_ON_FALSE(tx_pin != GPIO_NUM_NC && rx_pin != GPIO_NUM_NC, ESP_ERR_INVALID_STATE, APP_TAG, "Pinos UART TMC nao configurados");

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

    // TX == RX: uart_set_pin roteia ambos pela matriz de GPIO no mesmo pino. Antes o
    // pino tambem passava por gpio_config(), que o reservava de novo e gerava os avisos
    // "gpio: conflict found for GPIO[8]" a cada DRIVER APPLY.
    ret = uart_set_pin(TMC_UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "Falha ao configurar pinos UART TMC: %s", esp_err_to_name(ret));
        goto err;
    }
    if (tx_pin == rx_pin) {
        // Single-wire (PDN_UART): dreno aberto + pull-up para o TMC poder responder na mesma linha
        (void)gpio_od_enable(tx_pin);
        (void)gpio_pullup_en(tx_pin);
        (void)gpio_input_enable(tx_pin);
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

    for (int attempt = 0; attempt < 2; ++attempt) {
        (void)uart_flush_input(TMC_UART_PORT);
        if (uart_write_bytes(TMC_UART_PORT, frame, sizeof(frame)) == (int)sizeof(frame)) {
            esp_err_t err = uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20));
            if (err == ESP_OK) {
                esp_rom_delay_us(500);
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_FAIL;
}

static esp_err_t tmc_read_register_raw(uint8_t slave_addr, uint8_t reg_addr, uint32_t *value)
{
    uint8_t request[4] = {
        TMC_SYNC_BYTE,
        slave_addr,
        reg_addr,
        0,
    };
    request[3] = tmc_crc8(request, 3);

    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "value nulo");
    ESP_RETURN_ON_FALSE(s_uart_installed, ESP_ERR_INVALID_STATE, APP_TAG, "UART TMC nao instalada");

    for (int attempt = 0; attempt < 3; ++attempt) {
        uint8_t rx_buf[32] = {0};
        (void)uart_flush_input(TMC_UART_PORT);

        if (uart_write_bytes(TMC_UART_PORT, request, sizeof(request)) != (int)sizeof(request)) {
            continue;
        }
        (void)uart_wait_tx_done(TMC_UART_PORT, pdMS_TO_TICKS(20));

        int len = uart_read_bytes(TMC_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(TMC_UART_REPLY_TIMEOUT_MS));
        if (len >= 8) {
            for (int start = 0; start <= (len - 8); ++start) {
                if (rx_buf[start] != TMC_SYNC_BYTE) {
                    continue;
                }
                // Resposta do TMC2209 sempre tem byte 1 = 0xFF (Master Address)
                if (rx_buf[start + 1] != 0xFF) {
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
        }
        vTaskDelay(pdMS_TO_TICKS(4));
    }

    return ESP_ERR_TIMEOUT;
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
    default:  return -1; // invalido: o chamador decide (tmc_apply_axis_defaults usa 16)
    }
}

/*
 * TPWMTHRS: o TMC usa stealthChop enquanto TSTEP >= TPWMTHRS e troca para spreadCycle
 * acima dessa velocidade (como nas impressoras 3D: silencioso devagar, torque em alta).
 * TSTEP = tempo entre 1/256 micropassos em ciclos de 12 MHz.
 */
static uint32_t stealth_speed_to_tpwmthrs(const app_context_t *ctx, size_t idx)
{
    float speed = ctx->ext.stealth_max_speed[idx];
    if (!(speed > 0.0f)) {
        return 0U; // 0 = stealthChop em qualquer velocidade
    }
    float spr = ctx->settings.steps_per_rev[idx] ? (float)ctx->settings.steps_per_rev[idx] : 200.0f;
    float msteps = ctx->settings.tmc_microsteps[idx] ? (float)ctx->settings.tmc_microsteps[idx] : 16.0f;
    float units_per_rev = (idx == AXIS_Z_ID)
                              ? (float)(ctx->settings.z_pulley_teeth ? ctx->settings.z_pulley_teeth : DEFAULT_Z_PULLEY_TEETH) * Z_BELT_PITCH_MM
                              : 360.0f;
    float step_freq = speed * spr * msteps / units_per_rev;      // micropassos/s
    float ustep256_freq = step_freq * (256.0f / msteps);          // 1/256 micropassos/s
    float tstep = TMC_FCLK_HZ / ustep256_freq;
    if (!isfinite(tstep) || tstep < 1.0f) {
        return 1U;
    }
    return (tstep > 0xFFFFFUL) ? 0xFFFFFUL : (uint32_t)tstep;
}

static esp_err_t tmc_apply_axis_once(app_context_t *ctx, size_t axis_index)
{
    uint8_t addr = ctx->settings.tmc_slave_addr[axis_index];
    uint32_t verify_value = 0;
    // GCONF: pdn_disable=1 (bit 6), mstep_reg_select=1 (bit 7), multistep_filt=1 (bit 8)
    uint32_t gconf = (1U << 6) | (1U << 7) | (1U << 8);
    if (ctx->settings.tmc_spreadcycle[axis_index]) {
        gconf |= (1U << 2); // en_spreadCycle = 1
    }

    int mres = microsteps_to_mres(ctx->settings.tmc_microsteps[axis_index]);
    if (mres < 0) {
        mres = 4; // 16 microsteps
    }
    // CHOPCONF: intpol=1 (bit 28), mres (bits 24-27), tbl=2 (bits 15-16), hend=1 (bits 7-10), hstrt=4 (bits 4-6), toff=3 (bits 0-3)
    uint32_t chopconf = (1U << 28) | ((uint32_t)mres << 24) | (2U << 15) | (1U << 7) | (4U << 4) | (3U);
    uint32_t ihold_irun =
        ((uint32_t)(ctx->settings.tmc_ihold_delay[axis_index] & 0x0FU) << 16) |
        ((uint32_t)(ctx->settings.tmc_irun[axis_index] & 0x1FU) << 8) |
        (uint32_t)(ctx->settings.tmc_ihold[axis_index] & 0x1FU);

    // PWMCONF: autoscale=1, autograd=1, freq=1, grad=14, ofs=36
    const uint32_t pwmconf = 0xC10D0024U;
    const struct {
        uint8_t reg;
        uint32_t value;
    } writes[] = {
        {TMC_REG_GCONF, gconf},
        {TMC_REG_CHOPCONF, chopconf},
        {TMC_REG_IHOLD_IRUN, ihold_irun},
        {TMC_REG_PWMCONF, pwmconf},
        {TMC_REG_TPOWERDOWN, 20U},
        {TMC_REG_TPWMTHRS, stealth_speed_to_tpwmthrs(ctx, axis_index)},
        {TMC_REG_GSTAT, TMC_GSTAT_RESET | TMC_GSTAT_DRV_ERR | 0x04U}, // limpa flags (write-1-to-clear)
    };

    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_IOIN, &verify_value), APP_TAG, "Falha ao ler IOIN");

    uint32_t ifcnt_before = 0;
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_IFCNT, &ifcnt_before), APP_TAG, "Falha ao ler IFCNT");
    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); ++i) {
        ESP_RETURN_ON_ERROR(tmc_write_register_raw(addr, writes[i].reg, writes[i].value), APP_TAG, "Falha de escrita TMC");
    }

    // Confirmacao: IFCNT conta cada datagrama de escrita aceito (IHOLD_IRUN e write-only,
    // entao esta e a unica forma de provar que a corrente chegou ao driver)
    uint32_t ifcnt_after = 0;
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_IFCNT, &ifcnt_after), APP_TAG, "Falha ao ler IFCNT");
    uint32_t accepted = (ifcnt_after - ifcnt_before) & 0xFFU;
    if (accepted != (uint32_t)(sizeof(writes) / sizeof(writes[0]))) {
        ESP_LOGW(APP_TAG, "TMC %s aceitou %lu de %u escritas.", axis_name_from_index(axis_index),
                 (unsigned long)accepted, (unsigned)(sizeof(writes) / sizeof(writes[0])));
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Leitura de volta dos registradores legiveis
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_GCONF, &verify_value), APP_TAG, "Falha ao ler GCONF");
    if ((verify_value & 0x3FFU) != gconf) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_RETURN_ON_ERROR(tmc_read_register_raw(addr, TMC_REG_CHOPCONF, &verify_value), APP_TAG, "Falha ao ler CHOPCONF");
    if (verify_value != chopconf) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t tmc_apply_axis_defaults(app_context_t *ctx, size_t axis_index)
{
    esp_err_t err = ESP_FAIL;
    tmc_lock();
    for (int attempt = 0; attempt < TMC_APPLY_ATTEMPTS; ++attempt) {
        err = tmc_apply_axis_once(ctx, axis_index);
        if (err == ESP_OK || err == ESP_ERR_TIMEOUT) {
            break; // sucesso, ou driver ausente (nao adianta repetir agora)
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    tmc_unlock();
    return err;
}

uint8_t tmc2209_ma_to_cs(uint16_t ma)
{
    float cs_f = ((float)ma / TMC2209_MA_PER_CS) - 1.0f;
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
    // Arredonda (antes truncava): 9 * 59,846 = 538,6 -> 539, simetrico ao ma_to_cs
    return (uint16_t)(((float)cs + 1.0f) * TMC2209_MA_PER_CS + 0.5f);
}

esp_err_t tmc2209_set_spreadcycle(app_context_t *ctx, char axis, bool enabled)
{
    size_t axis_index = 0;
    if (!axis_to_index(axis, &axis_index)) {
        return ESP_ERR_INVALID_ARG;
    }
    tmc_lock();
    uint32_t gconf = (1U << 6) | (1U << 7) | (1U << 8); // pdn_disable=1, mstep_reg_select=1, multistep_filt=1
    (void)tmc2209_read_register(ctx, axis, TMC_REG_GCONF, &gconf);
    if (enabled) {
        gconf |= (1U << 2);
    } else {
        gconf &= ~(1U << 2);
    }
    esp_err_t err = tmc2209_write_register(ctx, axis, TMC_REG_GCONF, gconf);
    tmc_unlock();
    ESP_RETURN_ON_ERROR(err, APP_TAG, "Erro ao gravar GCONF");
    ctx->settings.tmc_spreadcycle[axis_index] = enabled ? 1U : 0U;
    storage_request_save(&ctx->settings);
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

    tmc_lock();
    uint32_t chopconf = 0x10000053U; // Default CHOPCONF com intpol=1, toff=3, tbl=2, hend=0, hstrt=5
    (void)tmc2209_read_register(ctx, axis, TMC_REG_CHOPCONF, &chopconf);
    chopconf &= ~(0x0FU << 24);
    chopconf |= ((uint32_t)mres << 24);
    esp_err_t err = tmc2209_write_register(ctx, axis, TMC_REG_CHOPCONF, chopconf);
    tmc_unlock();
    ESP_RETURN_ON_ERROR(err, APP_TAG, "Erro ao gravar CHOPCONF");

    ctx->settings.tmc_microsteps[axis_index] = microsteps;
    storage_request_save(&ctx->settings);
    return ESP_OK;
}

esp_err_t tmc2209_apply_current(app_context_t *ctx, char axis)
{
    size_t idx = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL && axis_to_index(axis, &idx), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    if (!s_uart_installed || !ctx->state.tmc_axis_online[idx]) {
        return ESP_ERR_INVALID_STATE; // gravada; sera aplicada quando o driver responder
    }
    uint32_t ihold_irun =
        ((uint32_t)(ctx->settings.tmc_ihold_delay[idx] & 0x0FU) << 16) |
        ((uint32_t)(ctx->settings.tmc_irun[idx] & 0x1FU) << 8) |
        (uint32_t)(ctx->settings.tmc_ihold[idx] & 0x1FU);
    return tmc2209_write_register(ctx, axis, TMC_REG_IHOLD_IRUN, ihold_irun);
}

esp_err_t tmc2209_set_stealth_max_speed(app_context_t *ctx, char axis, float speed)
{
    size_t axis_index = 0;
    ESP_RETURN_ON_FALSE(ctx != NULL && axis_to_index(axis, &axis_index), ESP_ERR_INVALID_ARG, APP_TAG, "eixo invalido");
    ESP_RETURN_ON_FALSE(isfinite(speed) && speed >= 0.0f && speed <= 10000.0f, ESP_ERR_INVALID_ARG, APP_TAG, "velocidade invalida");
    ctx->ext.stealth_max_speed[axis_index] = speed;
    storage_request_save_ext(&ctx->ext);
    if (!s_uart_installed || !ctx->state.tmc_axis_online[axis_index]) {
        return ESP_OK; // aplicado no proximo DRIVER APPLY / reconexao do driver
    }
    return tmc2209_write_register(ctx, axis, TMC_REG_TPWMTHRS, stealth_speed_to_tpwmthrs(ctx, axis_index));
}
