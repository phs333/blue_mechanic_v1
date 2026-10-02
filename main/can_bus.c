#include "can_bus.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "hardware.h"
#include "motion.h"
#include "storage.h"
#include "ota_update.h"

// Folga para gravacoes de 4 KB na flash durante OTA (~11 ms com a can_task ocupada)
#define CAN_RX_POOL_DEPTH 64
#define CAN_HEARTBEAT_PERIOD_MS 1000
#define CAN_TX_TIMEOUT_MS 50
typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} can_rx_slot_t;

typedef struct {
    bool valid;
    float speed;
    float accel;
} can_motion_profile_t;

static volatile app_context_t *s_ctx;
static twai_node_handle_t s_node;
static TaskHandle_t s_can_task_handle;
static SemaphoreHandle_t s_rx_free_sem;
static SemaphoreHandle_t s_rx_ready_sem;
static can_rx_slot_t s_rx_pool[CAN_RX_POOL_DEPTH];
static volatile uint32_t s_rx_write_index;
static uint32_t s_rx_read_index;
static can_motion_profile_t s_motion_profiles[AXIS_COUNT];
static volatile bool s_bus_off;
static volatile uint32_t s_pending_bitrate; // CAN_OP_SET_BITRATE: aplicado pela can_task apos o ACK sair

/*
 * U com commit sincronizado: o Teensy envia os U de todos os nos em sequencia (o no 10
 * recebe ~1-2 ms depois do no 1) e no fim da rajada um CAN_OP_SYNC_COMMIT em broadcast.
 * Todos os nos aplicam o alvo no mesmo instante, e o feedforward mede o intervalo entre
 * frames sem o jitter da posicao de cada no na rajada. So a can_task acessa este estado.
 */
#define CAN_LATCH_FLAG 0x80U
#define CAN_LATCH_TIMEOUT_US 10000 // commit perdido: aplica sozinho depois disto

typedef struct {
    bool valid;
    float c_deg;
    float a_deg;
    int32_t z_steps;
    bool force;
    int64_t t_us;
} can_latched_target_t;

static can_latched_target_t s_latch;

static esp_err_t can_latch_apply(app_context_t *ctx)
{
    if (!s_latch.valid) {
        return ESP_OK;
    }
    s_latch.valid = false;
    return motion_track_set(ctx, s_latch.c_deg, s_latch.a_deg, s_latch.z_steps, s_latch.force);
}

#define CAN_BUS_OFF_RECOVER_PERIOD_MS 1000

static bool validate_can_settings(const persisted_settings_t *settings);
static void prepare_rx_pool_once(void);
static void can_task(void *arg);
static esp_err_t can_node_start(app_context_t *ctx);
static void can_node_stop(app_context_t *ctx);
static twai_timing_basic_config_t can_get_bit_timing_config(uint32_t bitrate);
static esp_err_t can_send_payload(app_context_t *ctx, uint16_t frame_id, const uint8_t *payload, size_t payload_len, bool can_online);
esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1);
static uint8_t speed_level_from_delay(uint32_t delay_us);
static bool can_parse_axis(uint8_t token, size_t *axis_index, char *canonical_axis);
static float can_decode_float_le(const uint8_t *data);
static int16_t can_decode_i16_le(const uint8_t *data);
static int32_t can_sync_angle_to_steps(const app_context_t *ctx, size_t axis_index,
                                       int16_t angle_deci_deg);
static int32_t can_sync_angle_to_steps_centi(const app_context_t *ctx, size_t axis_index,
                                             int32_t angle_centi_deg);
static int32_t can_sync_z_to_steps(const app_context_t *ctx, int16_t distance_z_units);
static bool can_speed_is_valid(char axis, float speed);
static bool can_accel_is_valid(char axis, float accel);
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
    memset(s_motion_profiles, 0, sizeof(s_motion_profiles));

    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        ctx->state.can_rx_count = 0;
        ctx->state.can_tx_count = 0;
        ctx->state.can_last_error_flags = 0;
        xSemaphoreGive(ctx->state_mutex);
    }

    if (!ctx->settings.can_enabled) {
        if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctx->state.can_online = false;
            xSemaphoreGive(ctx->state_mutex);
        }
        ESP_LOGI(APP_TAG, "CAN/TWAI desativado por configuracao.");
        return ESP_OK;
    }

    if (!validate_can_settings(&ctx->settings)) {
        if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctx->state.can_online = false;
            xSemaphoreGive(ctx->state_mutex);
        }
        return ESP_ERR_INVALID_ARG;
    }

    return can_node_start(ctx);
}

static int16_t can_encode_deci_deg(float deg)
{
    float deci = deg * 10.0f;
    if (deci > 32767.0f) {
        return INT16_MAX;       // saturado (alem de +3276,7 deg)
    }
    if (deci < -32767.0f) {
        return INT16_MIN + 1;   // saturado; INT16_MIN fica reservado para "invalido"
    }
    return (int16_t)lroundf(deci);
}

esp_err_t can_bus_send_status(app_context_t *ctx)
{
    uint8_t payload[8];
    bool drivers_enabled = false;
    bool z_bloqueado = false;
    bool alarme_z_ativo = false;
    bool temp_valid = false;
    bool tmc_uart_ready = false;
    bool can_online = false;
    uint16_t laser_level[2] = {0, 0};
    bool fan_output_on = false;
    fan_mode_t fan_mode = FAN_MODE_MANUAL_OFF;
    float last_temp_c = 0.0f;
    int32_t current_z = 0;

    if (ctx->state.ota_in_progress) {
        return ESP_OK;
    }

    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        drivers_enabled = ctx->state.drivers_enabled;
        z_bloqueado = ctx->state.z_bloqueado;
        alarme_z_ativo = ctx->state.alarme_z_ativo;
        temp_valid = ctx->state.temp_valid;
        tmc_uart_ready = ctx->state.tmc_uart_ready;
        can_online = ctx->state.can_online;
        laser_level[0] = ctx->state.laser_level[0];
        laser_level[1] = ctx->state.laser_level[1];
        fan_output_on = ctx->state.fan_output_on;
        fan_mode = ctx->state.fan_mode;
        last_temp_c = ctx->state.last_temp_c;
        current_z = ctx->state.atual_z;
        xSemaphoreGive(ctx->state_mutex);
    }

    uint8_t speed_lvl = speed_level_from_delay(ctx->state.speed_delay_us[AXIS_C_ID]);

    payload[0] = CAN_EVT_STATUS;
    payload[1] = ctx->settings.node_id;
    payload[2] =
        (drivers_enabled ? 0x01U : 0x00U) |
        (z_bloqueado ? 0x02U : 0x00U) |
        (alarme_z_ativo ? 0x04U : 0x00U) |
        (temp_valid ? 0x08U : 0x00U) |
        (tmc_uart_ready ? 0x10U : 0x00U) |
        (can_online ? 0x20U : 0x00U) |
        CAN_STATUS_FLAG_POS_V2; // frame de posicao seguinte usa angulos com sinal (v2)
    payload[3] = (uint8_t)(laser_level[0] & 0xFF);
    payload[4] = (uint8_t)((laser_level[0] >> 8) & 0xFF);
    payload[5] = (uint8_t)(laser_level[1] & 0xFF);
    payload[6] = (uint8_t)((laser_level[1] >> 8) & 0xFF);
    payload[7] = (uint8_t)((fan_output_on ? 0x01U : 0x00U) | ((uint8_t)fan_mode << 1) | ((uint8_t)speed_lvl << 4));

    // Capture live telemetry before any CAN transmission. This keeps the
    // encoder path identical to the direct serial STATUS path and avoids
    // sampling I2C immediately after switching the external CAN transceiver.
    // Posicao v2: int16 em decimos de grau, com sinal (+-3276,6 deg; INT16_MIN = invalido).
    // A v1 usava 0..360 deg sem sinal: com limites de +-540 deg, todo angulo negativo ou
    // acima de uma volta chegava ao host como "invalido".
    float cur_c = 0.0f, cur_a = 0.0f;
    int16_t c_deci = INT16_MIN;
    int16_t a_deci = INT16_MIN;
    if (hardware_read_axis_encoder('C', &cur_c) == ESP_OK && isfinite(cur_c)) {
        c_deci = can_encode_deci_deg(cur_c);
    }
    if (hardware_read_axis_encoder('A', &cur_a) == ESP_OK && isfinite(cur_a)) {
        a_deci = can_encode_deci_deg(cur_a);
    }
    uint16_t z_pos = (current_z < 0) ? 0U :
                     (current_z >= (int32_t)UINT16_MAX ? UINT16_MAX - 1U : (uint16_t)current_z);
    int16_t temp_deci = INT16_MIN;
    if (temp_valid && isfinite(last_temp_c) && last_temp_c >= -55.0f && last_temp_c <= 125.0f) {
        temp_deci = (int16_t)lroundf(last_temp_c * 10.0f);
    }

    uint8_t payload_pos[8];
    payload_pos[0] = (uint8_t)((uint16_t)c_deci & 0xFF);
    payload_pos[1] = (uint8_t)(((uint16_t)c_deci >> 8) & 0xFF);
    payload_pos[2] = (uint8_t)((uint16_t)a_deci & 0xFF);
    payload_pos[3] = (uint8_t)(((uint16_t)a_deci >> 8) & 0xFF);
    payload_pos[4] = (uint8_t)(z_pos & 0xFF);
    payload_pos[5] = (uint8_t)((z_pos >> 8) & 0xFF);
    payload_pos[6] = (uint8_t)(temp_deci & 0xFF);
    payload_pos[7] = (uint8_t)((temp_deci >> 8) & 0xFF);

    esp_err_t err1 = can_send_payload(ctx,
                                      (uint16_t)(ctx->settings.can_status_base_id +
                                                 ctx->settings.node_id),
                                      payload, sizeof(payload), can_online);
    if (err1 != ESP_OK) {
        return err1;
    }
    esp_err_t err2 = can_send_payload(ctx, (uint16_t)(ctx->settings.can_status_base_id + 0x10U + ctx->settings.node_id), payload_pos, sizeof(payload_pos), can_online);

    return err2;
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
    if (settings->node_id < CAN_NODE_ID_MIN ||
        settings->node_id > CAN_NODE_ID_MAX) {
        ESP_LOGW(APP_TAG, "Node ID CAN invalido: %u", (unsigned)settings->node_id);
        return false;
    }
    if ((settings->can_command_base_id + CAN_NODE_ID_MAX) > TWAI_STD_ID_MASK ||
        (settings->can_status_base_id + CAN_NODE_ID_MAX) > TWAI_STD_ID_MASK ||
        (settings->can_status_base_id + 0x10U + CAN_NODE_ID_MAX) > TWAI_STD_ID_MASK ||
        (settings->can_event_base_id + CAN_NODE_ID_MAX) > TWAI_STD_ID_MASK) {
        ESP_LOGW(APP_TAG, "Bases CAN excedem o range de 11 bits.");
        return false;
    }
    if ((settings->can_status_base_id + CAN_NODE_ID_MAX) >=
            (settings->can_status_base_id + 0x10U + CAN_NODE_ID_MIN) ||
        (settings->can_status_base_id + 0x10U + CAN_NODE_ID_MAX) >=
            (settings->can_event_base_id + CAN_NODE_ID_MIN)) {
        ESP_LOGW(APP_TAG, "Faixas CAN de status, posicao e eventos se sobrepoem.");
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
    TickType_t last_recover = 0;

    while (true) {
        TickType_t wait = s_latch.valid ? pdMS_TO_TICKS(2) : pdMS_TO_TICKS(100);
        if (xSemaphoreTake(s_rx_ready_sem, wait > 0 ? wait : 1) == pdTRUE) {
            process_can_frame(ctx, &s_rx_pool[s_rx_read_index].frame);
            s_rx_read_index = (s_rx_read_index + 1U) % CAN_RX_POOL_DEPTH;
            xSemaphoreGive(s_rx_free_sem);
        }
        if (s_latch.valid && (esp_timer_get_time() - s_latch.t_us) > CAN_LATCH_TIMEOUT_US) {
            (void)can_latch_apply(ctx); // commit perdido: nao deixa o alvo parado
        }

        if (s_pending_bitrate != 0U) {
            // Da tempo ao ACK (e aos ACKs dos outros nos, no broadcast) de sair no bitrate antigo
            uint32_t bitrate = s_pending_bitrate;
            s_pending_bitrate = 0U;
            vTaskDelay(pdMS_TO_TICKS(30));
            ctx->settings.can_bitrate = bitrate;
            storage_request_save(&ctx->settings);
            esp_err_t br_err = can_bus_apply_settings(ctx);
            ESP_LOGW(APP_TAG, "CAN: bitrate alterado para %" PRIu32 " bps (%s)", bitrate, esp_err_to_name(br_err));
        }

        // Bus-off (ex.: cabo solto, terminacao ausente): o TWAI nao se recupera sozinho.
        // Inicia a recuperacao periodicamente; on_state_change volta can_online=true ao concluir.
        if (s_bus_off && s_node != NULL &&
            (xTaskGetTickCount() - last_recover) >= pdMS_TO_TICKS(CAN_BUS_OFF_RECOVER_PERIOD_MS)) {
            last_recover = xTaskGetTickCount();
            esp_err_t rec_err = twai_node_recover(s_node);
            ESP_LOGW(APP_TAG, "CAN em bus-off: tentando recuperar (%s)", esp_err_to_name(rec_err));
        }

        bool can_online = false;
        if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            can_online = ctx->state.can_online;
            xSemaphoreGive(ctx->state_mutex);
        }

        if (can_online && !ctx->state.ota_in_progress && (xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS)) {
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
    node_config.bit_timing = can_get_bit_timing_config(ctx->settings.can_bitrate);

    twai_event_callbacks_t callbacks = {
        .on_rx_done = can_on_rx_done,
        .on_error = can_on_error,
        .on_state_change = can_on_state_change,
    };

    twai_mask_filter_config_t cmd_filter = {
        .id = ctx->settings.can_command_base_id,
        .mask = 0x780, // Aceita 0x200 (broadcast) e 0x201..0x27F (node especÃ­fico)
        .is_ext = false,
    };

    s_bus_off = false;
    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&node_config, &s_node), APP_TAG, "Falha ao criar node TWAI");
    ESP_GOTO_ON_ERROR(twai_node_config_mask_filter(s_node, 0, &cmd_filter), err, APP_TAG, "Falha no filtro de comando");
    ESP_GOTO_ON_ERROR(twai_node_register_event_callbacks(s_node, &callbacks, ctx), err, APP_TAG, "Falha ao registrar callbacks TWAI");
    ESP_GOTO_ON_ERROR(twai_node_enable(s_node), err, APP_TAG, "Falha ao habilitar TWAI");

    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        ctx->state.can_online = true;
        xSemaphoreGive(ctx->state_mutex);
    }
    ESP_LOGI(APP_TAG, "CAN/TWAI ativo: node=%u cmd=0x%03X status=0x%03X event=0x%03X bitrate=%" PRIu32 " sample=%" PRIu32 ".%" PRIu32 "%%",
             (unsigned)ctx->settings.node_id,
             ctx->settings.can_command_base_id + ctx->settings.node_id,
             ctx->settings.can_status_base_id + ctx->settings.node_id,
             ctx->settings.can_event_base_id + ctx->settings.node_id,
             ctx->settings.can_bitrate,
             node_config.bit_timing.sp_permill / 10U,
             node_config.bit_timing.sp_permill % 10U);
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
        if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ctx->state.can_online = false;
            xSemaphoreGive(ctx->state_mutex);
        }
    }
}

static esp_err_t can_send_payload(app_context_t *ctx, uint16_t frame_id, const uint8_t *payload, size_t payload_len, bool can_online)
{
    ESP_RETURN_ON_FALSE(ctx != NULL && payload != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "arg invalido");
    ESP_RETURN_ON_FALSE(s_node != NULL && can_online, ESP_ERR_INVALID_STATE, APP_TAG, "CAN offline");
    ESP_RETURN_ON_FALSE(payload_len <= TWAI_FRAME_MAX_LEN, ESP_ERR_INVALID_SIZE, APP_TAG, "Payload CAN grande demais");

    twai_frame_t frame = {
        .header.id = frame_id,
        .buffer = (uint8_t *)payload,
        .buffer_len = payload_len,
    };

    ESP_RETURN_ON_ERROR(twai_node_transmit(s_node, &frame, CAN_TX_TIMEOUT_MS), APP_TAG, "Falha ao enfileirar TX TWAI");
    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        ctx->state.can_tx_count++;
        xSemaphoreGive(ctx->state_mutex);
    }
    return ESP_OK;
}

esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1)
{
    bool can_online = false;
    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        can_online = ctx->state.can_online;
        xSemaphoreGive(ctx->state_mutex);
    }

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
    return can_send_payload(ctx, (uint16_t)(ctx->settings.can_event_base_id + ctx->settings.node_id), payload, sizeof(payload), can_online);
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

static bool can_parse_axis(uint8_t token, size_t *axis_index, char *canonical_axis)
{
    char axis = (char)token;
    if (axis == 'C' || axis == 'c' || axis == 'X' || axis == 'x') {
        *axis_index = AXIS_C_ID;
        *canonical_axis = 'C';
        return true;
    }
    if (axis == 'A' || axis == 'a' || axis == 'Y' || axis == 'y') {
        *axis_index = AXIS_A_ID;
        *canonical_axis = 'A';
        return true;
    }
    if (axis == 'Z' || axis == 'z') {
        *axis_index = AXIS_Z_ID;
        *canonical_axis = 'Z';
        return true;
    }
    return false;
}

static float can_decode_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] |
                    ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 24);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static int16_t can_decode_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t can_sync_angle_to_steps(const app_context_t *ctx, size_t axis_index,
                                       int16_t angle_deci_deg)
{
    uint16_t steps_per_rev = ctx->settings.steps_per_rev[axis_index];
    uint16_t microsteps = ctx->settings.tmc_microsteps[axis_index];
    if (steps_per_rev == 0U) {
        steps_per_rev = 200U;
    }
    if (microsteps == 0U) {
        microsteps = 16U;
    }

    float angle_deg = (float)angle_deci_deg / 10.0f;
    return (int32_t)lroundf(angle_deg * (float)steps_per_rev *
                           (float)microsteps / 360.0f);
}

static int32_t can_sync_angle_to_steps_centi(const app_context_t *ctx, size_t axis_index,
                                             int32_t angle_centi_deg)
{
    uint16_t steps_per_rev = ctx->settings.steps_per_rev[axis_index];
    uint16_t microsteps = ctx->settings.tmc_microsteps[axis_index];
    if (steps_per_rev == 0U) {
        steps_per_rev = 200U;
    }
    if (microsteps == 0U) {
        microsteps = 16U;
    }

    float angle_deg = (float)angle_centi_deg / 100.0f;
    return (int32_t)lroundf(angle_deg * (float)steps_per_rev *
                           (float)microsteps / 360.0f);
}

/* Z nos frames 0x23/0x25: int16 em unidades de 0,02 mm (+-655,34 mm; cobre o curso de 480 mm) */
#define CAN_Z_MM_PER_UNIT 0.02f

static int32_t can_sync_z_to_steps(const app_context_t *ctx, int16_t distance_z_units)
{
    uint16_t steps_per_rev = ctx->settings.steps_per_rev[AXIS_Z_ID];
    uint16_t microsteps = ctx->settings.tmc_microsteps[AXIS_Z_ID];
    uint16_t pulley_teeth = ctx->settings.z_pulley_teeth;
    if (steps_per_rev == 0U) {
        steps_per_rev = 200U;
    }
    if (microsteps == 0U) {
        microsteps = 16U;
    }
    if (pulley_teeth == 0U) {
        pulley_teeth = DEFAULT_Z_PULLEY_TEETH;
    }

    float distance_mm = (float)distance_z_units * CAN_Z_MM_PER_UNIT;
    float mm_per_rev = (float)pulley_teeth * Z_BELT_PITCH_MM;
    return (int32_t)lroundf(distance_mm * (float)steps_per_rev *
                           (float)microsteps / mm_per_rev);
}

static bool can_speed_is_valid(char axis, float speed)
{
    if (!isfinite(speed)) {
        return false;
    }
    if (axis == 'Z') {
        return speed >= SPEED_MIN_MM_S_Z && speed <= SPEED_MAX_MM_S_Z;
    }
    return speed >= SPEED_MIN_DEG_S_CA && speed <= SPEED_MAX_DEG_S_CA;
}

static bool can_accel_is_valid(char axis, float accel)
{
    if (!isfinite(accel)) {
        return false;
    }
    if (axis == 'Z') {
        return accel >= ACCEL_MIN_MM_S2_Z && accel <= ACCEL_MAX_MM_S2_Z;
    }
    return accel >= ACCEL_MIN_DEG_S2_CA && accel <= ACCEL_MAX_DEG_S2_CA;
}

static void process_can_frame(app_context_t *ctx, const twai_frame_t *frame)
{
    uint16_t rx_id = frame->header.id;
    uint16_t own_cmd_id = (uint16_t)(ctx->settings.can_command_base_id + ctx->settings.node_id);
    uint16_t broadcast_cmd_id = (uint16_t)ctx->settings.can_command_base_id;
    size_t len = frame->header.dlc;
    const uint8_t *buf = frame->buffer;

    if (rx_id != own_cmd_id && rx_id != broadcast_cmd_id) {
        return;
    }

    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    ctx->state.can_rx_count++;
    if (len == 0U) {
        xSemaphoreGive(ctx->state_mutex);
        return;
    }

    // Intercepta comandos de atualizacao OTA prioritariamente
    if (buf[0] == CAN_OP_OTA_START || buf[0] == CAN_OP_OTA_DATA ||
        buf[0] == CAN_OP_OTA_END || buf[0] == CAN_OP_OTA_ABORT) {
        xSemaphoreGive(ctx->state_mutex);
        (void)ota_handle_can_cmd(ctx, buf[0], buf, len);
        return;
    }

    // STOP e aceito em qualquer estado (inclusive OTA) e tem prioridade sobre os demais comandos
    if (buf[0] == CAN_OP_STOP) {
        bool lasers_off = (len >= 2U) && ((buf[1] & 0x01U) != 0U);
        s_latch.valid = false; // um U travado nao pode reativar o movimento depois do STOP
        xSemaphoreGive(ctx->state_mutex);
        (void)motion_request_stop(ctx);
        if (lasers_off) {
            (void)hardware_set_laser_level(ctx, 0, 0);
            (void)hardware_set_laser_level(ctx, 1, 0);
        }
        (void)can_send_event(ctx, CAN_EVT_ACK, CAN_OP_STOP, 0);
        return;
    }

    // Se uma sessao OTA estiver ativa, bloqueia quaisquer outros comandos para evitar corrupcao/interrupcao
    if (ctx->state.ota_in_progress) {
        xSemaphoreGive(ctx->state_mutex);
        ESP_LOGW(APP_TAG, "Comando CAN 0x%02X rejeitado: sessao OTA em andamento", buf[0]);
        (void)can_send_event(ctx, CAN_EVT_ERROR, buf[0], (uint8_t)(ESP_ERR_INVALID_STATE & 0xFF));
        return;
    }

    esp_err_t err = ESP_OK;
    switch (buf[0]) {
    case CAN_OP_PING:
        if (len >= 3U) {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_PONG, buf[1], buf[2]);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_STATUS_REQUEST:
        xSemaphoreGive(ctx->state_mutex);
        (void)can_bus_send_status(ctx);
        break;

    case CAN_OP_ENABLE:
        if (len >= 2U) {
            hardware_set_driver_enable(ctx, buf[1] != 0U);
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ACK, CAN_OP_ENABLE, 0);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_SPEED:
        if (len >= 2U) {
            // Mesma tabela do comando serial VELOCIDADE; vale para C/A em todos os tipos de MOVE
            err = motion_apply_speed_level(ctx, buf[1]);
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_SPEED, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_AXIS_SPEED:
        if (len >= 6U) {
            size_t axis_index = 0U;
            char axis = '\0';
            float speed = can_decode_float_le(&buf[2]);
            if (!can_parse_axis(buf[1], &axis_index, &axis) || !can_speed_is_valid(axis, speed)) {
                err = ESP_ERR_INVALID_ARG;
            } else {
                uint32_t delay_us = motion_speed_to_delay_us(ctx, axis, speed);
                ctx->settings.speed_delay_us[axis_index] = delay_us;
                ctx->state.speed_delay_us[axis_index] = delay_us;
                // Antes so o delay era atualizado: Z e MOVE_SYNC usavam settings.speed e ignoravam o comando
                ctx->settings.speed[axis_index] = speed;
                ctx->state.speed[axis_index] = speed;
                if (speed > ctx->settings.speed_max[axis_index]) {
                    ctx->settings.speed_max[axis_index] = speed;
                    ctx->state.speed_max[axis_index] = speed;
                }
            }
            xSemaphoreGive(ctx->state_mutex);
            if (err == ESP_OK) {
                storage_request_save(&ctx->settings);
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR,
                                 CAN_OP_AXIS_SPEED, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_AXIS_SPEED,
                                 (uint8_t)ESP_ERR_INVALID_SIZE);
        }
        break;

    case CAN_OP_AXIS_ACCEL:
        if (len >= 6U) {
            size_t axis_index = 0U;
            char axis = '\0';
            float accel = can_decode_float_le(&buf[2]);
            if (!can_parse_axis(buf[1], &axis_index, &axis) || !can_accel_is_valid(axis, accel)) {
                err = ESP_ERR_INVALID_ARG;
            } else {
                ctx->settings.accel[axis_index] = accel;
                if (accel > ctx->settings.accel_max[axis_index]) {
                    ctx->settings.accel_max[axis_index] = accel;
                    ctx->state.accel_max[axis_index] = accel;
                }
            }
            xSemaphoreGive(ctx->state_mutex);
            if (err == ESP_OK) {
                storage_request_save(&ctx->settings);
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR,
                                 CAN_OP_AXIS_ACCEL, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_AXIS_ACCEL,
                                 (uint8_t)ESP_ERR_INVALID_SIZE);
        }
        break;

    case CAN_OP_MOVE_PROFILE:
        if (len >= 8U) {
            size_t axis_index = 0U;
            char axis = '\0';
            float speed = can_decode_float_le(&buf[2]);
            uint16_t accel_raw = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
            float accel = (float)accel_raw;
            bool speed_ok = false;
            bool accel_ok = false;
            if (!can_parse_axis(buf[1], &axis_index, &axis)) {
                err = ESP_ERR_INVALID_ARG;
            } else {
                speed_ok = (speed == 0.0f) || can_speed_is_valid(axis, speed);
                accel_ok = (accel_raw == 0U) || can_accel_is_valid(axis, accel);
                if (!speed_ok || !accel_ok) {
                    s_motion_profiles[axis_index].valid = false;
                    err = ESP_ERR_INVALID_ARG;
                } else {
                    s_motion_profiles[axis_index] = (can_motion_profile_t) {
                        .valid = true,
                        .speed = (speed > 0.0f) ? speed : -1.0f,
                        .accel = (accel_raw > 0U) ? accel : -1.0f,
                    };
                }
            }
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR,
                                 CAN_OP_MOVE_PROFILE, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_MOVE_PROFILE,
                                 (uint8_t)ESP_ERR_INVALID_SIZE);
        }
        break;

    case CAN_OP_SYNC_COMMIT:
        xSemaphoreGive(ctx->state_mutex);
        err = can_latch_apply(ctx);
        if (err != ESP_OK) {
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_MOVE_UNIFIED, (uint8_t)err);
        }
        break;

    case CAN_OP_SET_BITRATE:
        if (len >= 5U) {
            uint32_t bitrate = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
                               ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
            bool valid = bitrate == 125000U || bitrate == 250000U || bitrate == 500000U || bitrate == 1000000U;
            xSemaphoreGive(ctx->state_mutex);
            if (valid && bitrate != ctx->settings.can_bitrate) {
                s_pending_bitrate = bitrate;
            }
            (void)can_send_event(ctx, valid ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_SET_BITRATE,
                                 valid ? 0U : (uint8_t)ESP_ERR_INVALID_ARG);
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_SET_BITRATE, (uint8_t)ESP_ERR_INVALID_SIZE);
        }
        break;

    case CAN_OP_MOVE:
        if (len >= 6U) {
            int32_t steps = (int32_t)((uint32_t)buf[2] |
                                      ((uint32_t)buf[3] << 8) |
                                      ((uint32_t)buf[4] << 16) |
                                      ((uint32_t)buf[5] << 24));
            size_t axis_index = 0U;
            char axis = '\0';
            can_motion_profile_t profile = {0};
            if (can_parse_axis(buf[1], &axis_index, &axis) && s_motion_profiles[axis_index].valid) {
                profile = s_motion_profiles[axis_index];
                s_motion_profiles[axis_index].valid = false;
            }
            xSemaphoreGive(ctx->state_mutex);
            if (profile.valid) {
                err = motion_post_move_axis_profile(ctx, axis, steps, profile.speed, profile.accel,
                                                    false, ctx->settings.node_id, CAN_OP_MOVE);
            } else {
                err = motion_post_move_axis(ctx, (char)buf[1], steps, ctx->settings.node_id, CAN_OP_MOVE);
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_MOVE, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_MOVE_FORCE:
        if (len >= 6U) {
            int32_t steps = (int32_t)((uint32_t)buf[2] |
                                      ((uint32_t)buf[3] << 8) |
                                      ((uint32_t)buf[4] << 16) |
                                      ((uint32_t)buf[5] << 24));
            size_t axis_index = 0U;
            char axis = '\0';
            can_motion_profile_t profile = {0};
            if (can_parse_axis(buf[1], &axis_index, &axis) && s_motion_profiles[axis_index].valid) {
                profile = s_motion_profiles[axis_index];
                s_motion_profiles[axis_index].valid = false;
            }
            xSemaphoreGive(ctx->state_mutex);
            if (profile.valid) {
                err = motion_post_move_axis_profile(ctx, axis, steps, profile.speed, profile.accel,
                                                    true, ctx->settings.node_id, CAN_OP_MOVE_FORCE);
            } else {
                err = motion_post_move_axis_force(ctx, (char)buf[1], steps, ctx->settings.node_id, CAN_OP_MOVE_FORCE);
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_MOVE_FORCE, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_MOVE_SYNC:
    case CAN_OP_MOVE_UNIFIED:
        if (len == 8U) {
            uint8_t flags = buf[7];
            int32_t angle_c_centi = 0;
            int32_t angle_a_centi = 0;
            int16_t angle_c_deci = 0;
            int16_t angle_a_deci = 0;
            int16_t distance_z_units = can_decode_i16_le(&buf[5]);
            float speed_c = -1.0f;
            float speed_a = -1.0f;
            float speed_z = -1.0f;
            float accel = -1.0f;

            if (buf[0] == CAN_OP_MOVE_UNIFIED) {
                // Decodifica angulos de 19 bits assinados em centesimos de grau (+-2621.43 deg)
                // Byte 7: bit0=force_no_encoder, bits 1..3=C bits 16..18, bits 4..6=A bits 16..18,
                // bit 7=aguardar CAN_OP_SYNC_COMMIT
                uint32_t c_u32 = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) | (((uint32_t)(flags >> 1) & 0x07U) << 16);
                if ((c_u32 & (1U << 18)) != 0U) {
                    c_u32 |= ~0x7FFFFU;
                }
                angle_c_centi = (int32_t)c_u32;

                uint32_t a_u32 = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | (((uint32_t)(flags >> 4) & 0x07U) << 16);
                if ((a_u32 & (1U << 18)) != 0U) {
                    a_u32 |= ~0x7FFFFU;
                }
                angle_a_centi = (int32_t)a_u32;
            } else {
                if ((flags & ~0x01U) != 0U) {
                    err = ESP_ERR_INVALID_ARG;
                }
                angle_c_deci = can_decode_i16_le(&buf[1]);
                angle_a_deci = can_decode_i16_le(&buf[3]);
            }

            if (err == ESP_OK) {
                float *sync_speeds[AXIS_COUNT] = {&speed_c, &speed_a, &speed_z};
                for (size_t i = 0; i < AXIS_COUNT; ++i) {
                    if (s_motion_profiles[i].valid) {
                        *sync_speeds[i] = s_motion_profiles[i].speed;
                        if (s_motion_profiles[i].accel > 0.0f) {
                            if (accel > 0.0f && fabsf(accel - s_motion_profiles[i].accel) > 0.5f) {
                                err = ESP_ERR_INVALID_ARG;
                            } else {
                                accel = s_motion_profiles[i].accel;
                            }
                        }
                    }
                    s_motion_profiles[i].valid = false;
                }
            }

            int32_t steps_c;
            int32_t steps_a;
            if (buf[0] == CAN_OP_MOVE_UNIFIED) {
                steps_c = can_sync_angle_to_steps_centi(ctx, AXIS_C_ID, angle_c_centi);
                steps_a = can_sync_angle_to_steps_centi(ctx, AXIS_A_ID, angle_a_centi);
            } else {
                steps_c = can_sync_angle_to_steps(ctx, AXIS_C_ID, angle_c_deci);
                steps_a = can_sync_angle_to_steps(ctx, AXIS_A_ID, angle_a_deci);
            }
            int32_t steps_z = can_sync_z_to_steps(ctx, distance_z_units);
            xSemaphoreGive(ctx->state_mutex);

            if (err == ESP_OK && buf[0] == CAN_OP_MOVE_UNIFIED && (flags & CAN_LATCH_FLAG) != 0U) {
                // Commit sincronizado: guarda o alvo (o mais recente substitui um anterior
                // cujo commit se perdeu) e aplica no CAN_OP_SYNC_COMMIT
                s_latch = (can_latched_target_t){
                    .valid = true,
                    .c_deg = (float)angle_c_centi / 100.0f,
                    .a_deg = (float)angle_a_centi / 100.0f,
                    .z_steps = steps_z,
                    .force = (flags & 0x01U) != 0U,
                    .t_us = esp_timer_get_time(),
                };
            } else if (err == ESP_OK && buf[0] == CAN_OP_MOVE_UNIFIED) {
                // Streaming: so atualiza o alvo do seguidor (sem fila, sem parada entre frames)
                err = motion_track_set(ctx, (float)angle_c_centi / 100.0f, (float)angle_a_centi / 100.0f,
                                       steps_z, (flags & 0x01U) != 0U);
            } else if (err == ESP_OK) {
                err = motion_post_move_sync(ctx, steps_c, steps_a, steps_z,
                                            speed_c, speed_a, speed_z, accel,
                                            (flags & 0x01U) != 0U,
                                            ctx->settings.node_id, buf[0]);
            }
            // Para CAN_OP_MOVE_UNIFIED (streaming continuo do TouchDesigner), suprime CAN_EVT_ACK
            // de sucesso para nao saturar o barramento CAN nem gerar overhead reverso no canal USB.
            if (buf[0] != CAN_OP_MOVE_UNIFIED || err != ESP_OK) {
                (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR,
                                     buf[0], (uint8_t)err);
            }
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, buf[0],
                                 (uint8_t)ESP_ERR_INVALID_SIZE);
        }
        break;

    case CAN_OP_HOME:
        if (len >= 2U) {
            char axis = (char)buf[1];
            xSemaphoreGive(ctx->state_mutex);
            if (axis == 'C' || axis == 'c' || axis == 'A' || axis == 'a' ||
                axis == 'X' || axis == 'x' || axis == 'Y' || axis == 'y') {
                err = motion_post_home_axis(ctx, axis, ctx->settings.node_id, CAN_OP_HOME);
            } else if (axis == 'Z' || axis == 'z') {
                err = motion_post_home_axis(ctx, 'Z', ctx->settings.node_id, CAN_OP_HOME);
            } else {
                err = ESP_ERR_INVALID_ARG;
            }
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_HOME, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_LASER:
        if (len >= 3U && buf[1] >= 1U && buf[1] <= 2U) {
            uint16_t lvl = 0;
            if (len >= 4U) {
                lvl = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
            } else {
                lvl = (uint16_t)buf[2];
            }
            ESP_LOGD(APP_TAG, "CAN comando LASER %u -> level=%u (12-bit)", (unsigned)buf[1], (unsigned)lvl);
            err = hardware_set_laser_level(ctx, (size_t)(buf[1] - 1U), lvl);
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_LASER, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    case CAN_OP_LASER_DUAL:
        if (len >= 5U) {
            uint16_t lvl1 = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
            uint16_t lvl2 = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
            if (lvl1 > 4095U) lvl1 = 4095U;
            if (lvl2 > 4095U) lvl2 = 4095U;
            // LOGD: um LOGI aqui bloqueava a can_task ~5 ms na UART a cada frame do streaming U
            ESP_LOGD(APP_TAG, "CAN comando LASER_DUAL -> L1=%u L2=%u (12-bit)", (unsigned)lvl1, (unsigned)lvl2);
            err = hardware_set_laser_level(ctx, 0, lvl1);
            if (err == ESP_OK) {
                err = hardware_set_laser_level(ctx, 1, lvl2);
            }
            xSemaphoreGive(ctx->state_mutex);
            // Frame de streaming (par do MOVE_UNIFIED): so falhas geram evento, como no 0x25
            if (err != ESP_OK) {
                (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_LASER_DUAL, (uint8_t)err);
            }
        } else {
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, CAN_EVT_ERROR, CAN_OP_LASER_DUAL, (uint8_t)ESP_ERR_INVALID_SIZE);
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
            xSemaphoreGive(ctx->state_mutex);
            (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_ACK : CAN_EVT_ERROR, CAN_OP_FAN, (uint8_t)err);
        } else {
            xSemaphoreGive(ctx->state_mutex);
        }
        break;

    default:
        xSemaphoreGive(ctx->state_mutex);
        ESP_LOGW(APP_TAG, "CAN opcode desconhecido: 0x%02X", buf[0]);
        (void)can_send_event(ctx, CAN_EVT_ERROR, buf[0], (uint8_t)ESP_ERR_NOT_SUPPORTED);
        break;
    }
}

/* ------------------------------------------------------------------------------------ */
/* CAN TEST: separa falha do controlador, do transceiver e do barramento entre as placas */
/* ------------------------------------------------------------------------------------ */

static volatile uint32_t s_test_rx;
static volatile uint32_t s_test_err;

static bool IRAM_ATTR can_test_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    (void)user_ctx;
    uint8_t data[TWAI_FRAME_MAX_LEN];
    twai_frame_t frame = {.buffer = data, .buffer_len = sizeof(data)};
    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        s_test_rx++;
    }
    return false;
}

static bool IRAM_ATTR can_test_err_cb(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)user_ctx;
    s_test_err |= edata->err_flags.val;
    return false;
}

/* Envia 3 frames em modo self-test (sem exigir ACK) + loopback e conta quantos voltam. */
static esp_err_t can_test_run(uint32_t bitrate, gpio_num_t rx_pin, uint32_t *rx, uint32_t *errs, uint16_t *tec)
{
    twai_onchip_node_config_t cfg = {
        .io_cfg = {.tx = CAN_TX_PIN, .rx = rx_pin, .quanta_clk_out = GPIO_NUM_NC, .bus_off_indicator = GPIO_NUM_NC},
        .bit_timing = can_get_bit_timing_config(bitrate),
        .fail_retry_cnt = 0,
        .tx_queue_depth = 4,
        .flags = {.enable_self_test = 1, .enable_loopback = 1},
    };
    twai_event_callbacks_t cbs = {.on_rx_done = can_test_rx_cb, .on_error = can_test_err_cb};
    twai_node_handle_t node = NULL;
    s_test_rx = 0U;
    s_test_err = 0U;
    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&cfg, &node), APP_TAG, "CAN TEST: falha ao criar node");
    esp_err_t err = twai_node_register_event_callbacks(node, &cbs, NULL);
    if (err == ESP_OK) {
        err = twai_node_enable(node);
    }
    if (err == ESP_OK) {
        uint8_t data[4] = {'T', 'E', 'S', 'T'};
        for (int i = 0; i < 3; ++i) {
            twai_frame_t frame = {.header.id = 0x7F0U, .buffer = data, .buffer_len = sizeof(data)};
            (void)twai_node_transmit(node, &frame, 20);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        twai_node_status_t st = {0};
        if (twai_node_get_info(node, &st, NULL) == ESP_OK) {
            *tec = st.tx_error_count;
        }
        (void)twai_node_disable(node);
    }
    (void)twai_node_delete(node);
    *rx = s_test_rx;
    *errs = s_test_err;
    return err;
}

esp_err_t can_bus_self_test(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");
    uint32_t bitrate = ctx->settings.can_bitrate ? ctx->settings.can_bitrate : DEFAULT_CAN_BITRATE;
    can_node_stop(ctx);

    uint32_t rx1 = 0, err1 = 0, rx2 = 0, err2 = 0;
    uint16_t tec1 = 0, tec2 = 0;
    // 1) TX e RX no mesmo GPIO: o sinal volta por dentro do chip, sem depender do transceiver
    esp_err_t e1 = can_test_run(bitrate, CAN_TX_PIN, &rx1, &err1, &tec1);
    // 2) Pinos reais: o frame sai pelo transceiver e precisa voltar lido do barramento
    esp_err_t e2 = can_test_run(bitrate, CAN_RX_PIN, &rx2, &err2, &tec2);
    bool ok1 = (e1 == ESP_OK && rx1 >= 3U);
    bool ok2 = (e2 == ESP_OK && rx2 >= 3U && err2 == 0U);

    printf("CAN TEST bitrate=%" PRIu32 " TX=GPIO%d RX=GPIO%d\n", bitrate, (int)CAN_TX_PIN, (int)CAN_RX_PIN);
    printf("CAN TEST 1 controlador (loopback interno): %s rx=%" PRIu32 "/3 erros=0x%02" PRIX32 " TEC=%u\n",
           ok1 ? "OK" : "FALHOU", rx1, err1, (unsigned)tec1);
    printf("CAN TEST 2 transceiver (sai e volta pelo barramento): %s rx=%" PRIu32 "/3 erros=0x%02" PRIX32 " TEC=%u\n",
           ok2 ? "OK" : "FALHOU", rx2, err2, (unsigned)tec2);
    if (!ok1) {
        puts("CAN TEST diagnostico: falha no proprio controlador/GPIO do ESP32 (verifique conflito no GPIO39).");
    } else if (!ok2) {
        puts("CAN TEST diagnostico: o ESP nao le de volta o que transmite -> transceiver sem alimentacao (3V3/5V), "
             "pino de standby (S/RS) em nivel alto, TX/RX trocados entre ESP e transceiver, ou GPIO39/40 "
             "sem contato. Erro 0x02 = bit error (nivel lido diferente do transmitido).");
    } else {
        puts("CAN TEST diagnostico: ESP e transceiver OK. Se o Teensy nao recebe ACK, o problema esta no "
             "barramento entre as placas: CANH/CANL abertos ou invertidos, falta de GND comum ou de "
             "terminacao 120R nas pontas, ou o transceiver do Teensy (pinos 22=CTX1 / 23=CRX1).");
    }

    esp_err_t restore = can_bus_apply_settings(ctx);
    printf("CAN TEST: configuracao normal restaurada (%s).\n", esp_err_to_name(restore));
    return (ok1 && ok2) ? ESP_OK : ESP_FAIL;
}

static twai_timing_basic_config_t can_get_bit_timing_config(uint32_t bitrate)
{
    twai_timing_basic_config_t timing = {
        .bitrate = bitrate,
        .sp_permill = 800,
        .ssp_permill = 0,
    };
    return timing;
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
    s_bus_off = (edata->new_sta == TWAI_ERROR_BUS_OFF);
    if (s_ctx != NULL) {
        s_ctx->state.can_online = !s_bus_off;
    }
    return false;
}
