#include "can_bus.h"

#include <inttypes.h>
#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "hardware.h"
#include "motion.h"
#include "storage.h"

#define CAN_RX_POOL_DEPTH 16
#define CAN_HEARTBEAT_PERIOD_MS 1000
#define CAN_TX_TIMEOUT_MS 50
typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} can_rx_slot_t;

static app_context_t *s_ctx;
static twai_node_handle_t s_node;
static TaskHandle_t s_can_task_handle;
static SemaphoreHandle_t s_rx_free_sem;
static SemaphoreHandle_t s_rx_ready_sem;
static can_rx_slot_t s_rx_pool[CAN_RX_POOL_DEPTH];
static volatile uint32_t s_rx_write_index;
static uint32_t s_rx_read_index;

static bool validate_can_settings(const persisted_settings_t *settings);
static void prepare_rx_pool_once(void);
static void can_task(void *arg);
static esp_err_t can_node_start(app_context_t *ctx);
static void can_node_stop(app_context_t *ctx);
static esp_err_t can_send_payload(app_context_t *ctx, uint16_t frame_id, const uint8_t *payload, size_t payload_len);
esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1);
static uint8_t speed_level_from_delay(uint32_t delay_us);
static void process_can_frame(app_context_t *ctx, const twai_frame_t *frame);
static bool can_on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);
static bool can_on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx);
static bool can_on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx);

esp_err_t can_bus_init(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    s_ctx = ctx;
    prepare_rx_pool_once();

    if (s_can_task_handle == NULL) {
        BaseType_t created = xTaskCreate(can_task, "can_task", 6144, ctx, 6, &s_can_task_handle);
        ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, APP_TAG, "Falha ao criar tarefa CAN");
    }

    return can_bus_apply_settings(ctx);
}

esp_err_t can_bus_apply_settings(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    can_node_stop(ctx);
    ctx->state.can_rx_count = 0;
    ctx->state.can_tx_count = 0;
    ctx->state.can_last_error_flags = 0;

    if (!ctx->settings.can_enabled) {
        ctx->state.can_online = false;
        ESP_LOGI(APP_TAG, "CAN/TWAI desativado por configuracao.");
        return ESP_OK;
    }

    if (!validate_can_settings(&ctx->settings)) {
        ctx->state.can_online = false;
        return ESP_ERR_INVALID_ARG;
    }

    return can_node_start(ctx);
}

esp_err_t can_bus_send_status(app_context_t *ctx)
{
    uint8_t payload[8];
    int temp = (int)ctx->state.last_temp_c;
    if (temp > 127) {
        temp = 127;
    } else if (temp < -128) {
        temp = -128;
    }

    payload[0] = CAN_EVT_STATUS;
    payload[1] = ctx->settings.node_id;
    payload[2] =
        (ctx->state.drivers_enabled ? 0x01U : 0x00U) |
        (ctx->state.z_bloqueado ? 0x02U : 0x00U) |
        (ctx->state.alarme_z_ativo ? 0x04U : 0x00U) |
        (ctx->state.temp_valid ? 0x08U : 0x00U) |
        (ctx->state.tmc_uart_ready ? 0x10U : 0x00U) |
        (ctx->state.can_online ? 0x20U : 0x00U);
    payload[3] = ctx->state.laser_level[0];
    payload[4] = ctx->state.laser_level[1];
    payload[5] = (uint8_t)((ctx->state.fan_output_on ? 0x01U : 0x00U) | ((uint8_t)ctx->state.fan_mode << 1));
    payload[6] = (uint8_t)(int8_t)temp;
    payload[7] = speed_level_from_delay(ctx->state.move_delay_us);

    return can_send_payload(ctx, (uint16_t)(ctx->settings.can_status_base_id + ctx->settings.node_id), payload, sizeof(payload));
}

void can_bus_print_status(const app_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    printf("CAN: %s node=%u bitrate=%" PRIu32 "\n",
           ctx->state.can_online ? "ONLINE" : (ctx->settings.can_enabled ? "CONFIGURADO/OFFLINE" : "OFF"),
           (unsigned)ctx->settings.node_id,
           ctx->settings.can_bitrate);
    printf("CAN IDs: cmd=0x%03X status=0x%03X event=0x%03X\n",
           ctx->settings.can_command_base_id + ctx->settings.node_id,
           ctx->settings.can_status_base_id + ctx->settings.node_id,
           ctx->settings.can_event_base_id + ctx->settings.node_id);
    printf("CAN bases: cmd=0x%03X status=0x%03X event=0x%03X\n",
           ctx->settings.can_command_base_id,
           ctx->settings.can_status_base_id,
           ctx->settings.can_event_base_id);
    printf("CAN contadores: rx=%" PRIu32 " tx=%" PRIu32 " ultimo_erro=0x%08" PRIX32 "\n",
           ctx->state.can_rx_count,
           ctx->state.can_tx_count,
           ctx->state.can_last_error_flags);
}

static bool validate_can_settings(const persisted_settings_t *settings)
{
    if (settings->node_id == 0U || settings->node_id > 127U) {
        ESP_LOGW(APP_TAG, "Node ID CAN invalido: %u", (unsigned)settings->node_id);
        return false;
    }
    if ((settings->can_command_base_id + 127U) > TWAI_STD_ID_MASK ||
        (settings->can_status_base_id + 127U) > TWAI_STD_ID_MASK ||
        (settings->can_event_base_id + 127U) > TWAI_STD_ID_MASK) {
        ESP_LOGW(APP_TAG, "Bases CAN excedem o range de 11 bits.");
        return false;
    }

    switch (settings->can_bitrate) {
    case 125000U:
    case 250000U:
    case 500000U:
    case 1000000U:
        return true;
    default:
        ESP_LOGW(APP_TAG, "Bitrate CAN nao suportado: %" PRIu32, settings->can_bitrate);
        return false;
    }
}

static void prepare_rx_pool_once(void)
{
    if (s_rx_free_sem == NULL) {
        s_rx_free_sem = xSemaphoreCreateCounting(CAN_RX_POOL_DEPTH, CAN_RX_POOL_DEPTH);
    }
    if (s_rx_ready_sem == NULL) {
        s_rx_ready_sem = xSemaphoreCreateCounting(CAN_RX_POOL_DEPTH, 0);
    }

    for (size_t index = 0; index < CAN_RX_POOL_DEPTH; ++index) {
        s_rx_pool[index].frame.buffer = s_rx_pool[index].data;
        s_rx_pool[index].frame.buffer_len = sizeof(s_rx_pool[index].data);
    }
}

static void can_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    TickType_t last_heartbeat = xTaskGetTickCount();

    while (true) {
        if (xSemaphoreTake(s_rx_ready_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
            process_can_frame(ctx, &s_rx_pool[s_rx_read_index].frame);
            s_rx_read_index = (s_rx_read_index + 1U) % CAN_RX_POOL_DEPTH;
            xSemaphoreGive(s_rx_free_sem);
        }

        if (ctx->state.can_online && (xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS)) {
            (void)can_send_event(ctx, CAN_EVT_HEARTBEAT, ctx->settings.node_id, 0);
            last_heartbeat = xTaskGetTickCount();
        }
    }
}

static esp_err_t can_node_start(app_context_t *ctx)
{
    esp_err_t ret = ESP_OK;
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = ctx->settings.can_bitrate,
        },
        .timestamp_resolution_hz = 1000000,
        .fail_retry_cnt = 3,
        .tx_queue_depth = 8,
        .intr_priority = 0,
    };

    twai_event_callbacks_t callbacks = {
        .on_rx_done = can_on_rx_done,
        .on_error = can_on_error,
        .on_state_change = can_on_state_change,
    };

    twai_mask_filter_config_t cmd_filter = {
        .id = 0,
        .mask = 0,
        .is_ext = false,
    };

    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_config, &s_node), APP_TAG, "Falha ao criar node TWAI");
    ESP_GOTO_ON_ERROR(twai_node_config_mask_filter(s_node, 0, &cmd_filter), err, APP_TAG, "Falha no filtro de comando");
    ESP_GOTO_ON_ERROR(twai_node_register_event_callbacks(s_node, &callbacks, ctx), err, APP_TAG, "Falha ao registrar callbacks TWAI");
    ESP_GOTO_ON_ERROR(twai_node_enable(s_node), err, APP_TAG, "Falha ao habilitar TWAI");

    ctx->state.can_online = true;
    ESP_LOGI(APP_TAG, "CAN/TWAI ativo: node=%u cmd=0x%03X status=0x%03X event=0x%03X bitrate=%" PRIu32,
             (unsigned)ctx->settings.node_id,
             ctx->settings.can_command_base_id + ctx->settings.node_id,
             ctx->settings.can_status_base_id + ctx->settings.node_id,
             ctx->settings.can_event_base_id + ctx->settings.node_id,
             ctx->settings.can_bitrate);
    return ESP_OK;

err:
    can_node_stop(ctx);
    return ret;
}

static void can_node_stop(app_context_t *ctx)
{
    if (s_node != NULL) {
        (void)twai_node_disable(s_node);
        (void)twai_node_delete(s_node);
        s_node = NULL;
    }
    if (ctx != NULL) {
        ctx->state.can_online = false;
    }
}

static esp_err_t can_send_payload(app_context_t *ctx, uint16_t frame_id, const uint8_t *payload, size_t payload_len)
{
    twai_frame_t frame = {
        .header.id = frame_id,
        .buffer = (uint8_t *)payload,
        .buffer_len = payload_len,
    };

    ESP_RETURN_ON_FALSE(ctx != NULL && payload != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "arg invalido");
    ESP_RETURN_ON_FALSE(s_node != NULL && ctx->state.can_online, ESP_ERR_INVALID_STATE, APP_TAG, "CAN offline");
    ESP_RETURN_ON_FALSE(payload_len <= TWAI_FRAME_MAX_LEN, ESP_ERR_INVALID_SIZE, APP_TAG, "Payload CAN grande demais");

    ESP_RETURN_ON_ERROR(twai_node_transmit(s_node, &frame, CAN_TX_TIMEOUT_MS), APP_TAG, "Falha ao enfileirar TX TWAI");
    ESP_RETURN_ON_ERROR(twai_node_transmit_wait_all_done(s_node, CAN_TX_TIMEOUT_MS), APP_TAG, "Timeout TX TWAI");
    ctx->state.can_tx_count++;
    return ESP_OK;
}

esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1)
{
    uint8_t payload[8] = {
        (uint8_t)event_id,
        ctx->settings.node_id,
        arg0,
        arg1,
        0,
        0,
        0,
        0,
    };
    return can_send_payload(ctx, (uint16_t)(ctx->settings.can_event_base_id + ctx->settings.node_id), payload, sizeof(payload));
}

static uint8_t speed_level_from_delay(uint32_t delay_us)
{
    if (delay_us >= 2000U) {
        return 1;
    }
    if (delay_us >= 800U) {
        return 2;
    }
    if (delay_us >= 400U) {
        return 3;
    }
    if (delay_us >= 150U) {
        return 4;
    }
    return 5;
}

static void process_can_frame(app_context_t *ctx, const twai_frame_t *frame)
{
    uint16_t rx_id = frame->header.id;
    uint16_t own_cmd_id = (uint16_t)(ctx->settings.can_command_base_id + ctx->settings.node_id);
    uint16_t broadcast_cmd_id = ctx->settings.can_command_base_id;
    size_t len = frame->header.dlc;
    const uint8_t *buf = frame->buffer;

    if (rx_id != own_cmd_id && rx_id != broadcast_cmd_id) {
        return;
    }

    ctx->state.can_rx_count++;
    if (len == 0U) {
        return;
    }

    esp_err_t err = ESP_OK;
    switch (buf[0]) {
    case CAN_OP_PING:
        (void)can_send_event(ctx, CAN_EVT_PONG, buf[1], buf[2]);
        break;

    case CAN_OP_STATUS_REQUEST:
        (void)can_bus_send_status(ctx);
        break;

    case CAN_OP_ENABLE:
        if (len >= 2U) {
            hardware_set_driver_enable(ctx, buf[1] != 0U);
            (void)can_send_event(ctx, CAN_EVT_ACK, CAN_OP_ENABLE, 0);
        }
        break;

    case CAN_OP_SPEED:
        if (len >= 2U) {
            switch (buf[1]) {
            case 1:
                ctx->state.move_delay_us = 2000;
                err = ESP_OK;
                break;
            case 2:
                ctx->state.move_delay_us = 800;
                err = ESP_OK;
                break;
            case 3:
                ctx->state.move_delay_us = 400;
                err = ESP_OK;
                break;
            case 4:
                ctx->state.move_delay_us = 150;
                err = ESP_OK;
                break;
            case 5:
                ctx->state.move_delay_us = 50;
                err = ESP_OK;
                break;
            default:
                err = ESP_ERR_INVALID_ARG;
                break;
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_SPEED, (uint8_t)err);
        }
        break;

    case CAN_OP_MOVE:
        if (len >= 6U) {
            int32_t steps = (int32_t)((uint32_t)buf[2] |
                                      ((uint32_t)buf[3] << 8) |
                                      ((uint32_t)buf[4] << 16) |
                                      ((uint32_t)buf[5] << 24));
            err = motion_post_move_axis(ctx, (char)buf[1], steps, ctx->settings.node_id, CAN_OP_MOVE);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_MOVE, (uint8_t)err);
        }
        break;

    case CAN_OP_HOME:
        if (len >= 2U) {
            char axis = (char)buf[1];
            if (axis == 'X' || axis == 'x' || axis == 'Y' || axis == 'y') {
                err = motion_post_home_axis(ctx, axis, ctx->settings.node_id, CAN_OP_HOME);
            } else if (axis == 'Z' || axis == 'z') {
                err = motion_post_home_axis(ctx, 'Z', ctx->settings.node_id, CAN_OP_HOME);
            } else {
                err = ESP_ERR_INVALID_ARG;
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_HOME, (uint8_t)err);
        }
        break;

    case CAN_OP_LASER:
        if (len >= 3U && buf[1] >= 1U && buf[1] <= 2U) {
            err = hardware_set_laser_level(ctx, (size_t)(buf[1] - 1U), buf[2]);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_LASER, (uint8_t)err);
        }
        break;

    case CAN_OP_FAN:
        if (len >= 2U) {
            switch (buf[1]) {
            case 0:
                ctx->state.fan_mode = FAN_MODE_MANUAL_OFF;
                hardware_set_fan_output(ctx, false);
                err = ESP_OK;
                break;
            case 1:
                ctx->state.fan_mode = FAN_MODE_MANUAL_ON;
                hardware_set_fan_output(ctx, true);
                err = ESP_OK;
                break;
            case 2:
                ctx->state.fan_mode = FAN_MODE_AUTO;
                err = ESP_OK;
                break;
            default:
                err = ESP_ERR_INVALID_ARG;
                break;
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_FAN, (uint8_t)err);
        }
        break;

    default:
        (void)can_send_event(ctx, CAN_EVT_ERROR, buf[0], (uint8_t)ESP_ERR_NOT_SUPPORTED);
        break;
    }
}

static bool IRAM_ATTR can_on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t task_woken = pdFALSE;
    (void)edata;
    (void)user_ctx;

    if (xSemaphoreTakeFromISR(s_rx_free_sem, &task_woken) != pdTRUE) {
        return (task_woken == pdTRUE);
    }

    if (twai_node_receive_from_isr(handle, &s_rx_pool[s_rx_write_index].frame) == ESP_OK) {
        s_rx_write_index = (s_rx_write_index + 1U) % CAN_RX_POOL_DEPTH;
        xSemaphoreGiveFromISR(s_rx_ready_sem, &task_woken);
    } else {
        xSemaphoreGiveFromISR(s_rx_free_sem, &task_woken);
    }
    return (task_woken == pdTRUE);
}

static bool IRAM_ATTR can_on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)user_ctx;
    if (s_ctx != NULL) {
        s_ctx->state.can_last_error_flags = edata->err_flags.val;
    }
    return false;
}

static bool IRAM_ATTR can_on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)user_ctx;
    if (s_ctx != NULL) {
        s_ctx->state.can_online = (edata->new_sta != TWAI_ERROR_BUS_OFF);
    }
    return false;
}
