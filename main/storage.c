#include "storage.h"

#include <string.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define EXT_SETTINGS_KEY "cfg_ext"

// Rajadas de comandos (ex.: "Gravar Todas" do app) viram UMA escrita na flash.
// Cada escrita/erase de pagina desliga o cache e pausa ISRs fora da IRAM; gravar a
// cada comando fazia a UART do console perder bytes no meio da rajada.
#define SAVE_DEBOUNCE_MS 300

static SemaphoreHandle_t s_save_lock;
static TaskHandle_t s_storage_task;
static const persisted_settings_t *volatile s_pending_settings;
static const ext_settings_t *volatile s_pending_ext;
// Ultimo conteudo efetivamente gravado: escrita identica e descartada (sem desgaste da flash)
static persisted_settings_t s_last_saved;
static bool s_last_saved_valid;
static ext_settings_t s_last_ext;
static bool s_last_ext_valid;

static void storage_task(void *arg)
{
    (void)arg;
    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Aguarda a rajada terminar: cada novo pedido reinicia a janela de silencio
        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SAVE_DEBOUNCE_MS)) > 0) {
        }
        (void)storage_flush();
    }
}

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err == ESP_OK && s_save_lock == NULL) {
        s_save_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_save_lock != NULL, ESP_ERR_NO_MEM, APP_TAG, "mutex NVS");
        ESP_RETURN_ON_FALSE(xTaskCreate(storage_task, "storage", 3584, NULL, 3, &s_storage_task) == pdPASS,
                            ESP_ERR_NO_MEM, APP_TAG, "tarefa NVS");
    }
    return err;
}

void storage_request_save(const persisted_settings_t *settings)
{
    s_pending_settings = settings;
    if (s_storage_task != NULL) {
        xTaskNotifyGive(s_storage_task);
    } else {
        (void)storage_flush();
    }
}

void storage_request_save_ext(const ext_settings_t *ext)
{
    s_pending_ext = ext;
    if (s_storage_task != NULL) {
        xTaskNotifyGive(s_storage_task);
    } else {
        (void)storage_flush();
    }
}

esp_err_t storage_flush(void)
{
    esp_err_t err = ESP_OK;
    const persisted_settings_t *settings = s_pending_settings;
    const ext_settings_t *ext = s_pending_ext;
    s_pending_settings = NULL;
    s_pending_ext = NULL;
    if (settings != NULL) {
        err = storage_save_settings(settings);
    }
    if (ext != NULL) {
        esp_err_t ext_err = storage_save_ext(ext);
        if (err == ESP_OK) {
            err = ext_err;
        }
    }
    return err;
}

esp_err_t storage_load_ext(ext_settings_t *ext)
{
    ESP_RETURN_ON_FALSE(ext != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ext nulo");
    const ext_settings_t defaults = APP_EXT_SETTINGS_DEFAULT_INIT;
    *ext = defaults;

    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return ESP_OK;
    }
    // Buffer maior que a struct: aceita blobs de firmwares mais novos (usa o prefixo conhecido)
    uint8_t raw[256];
    size_t len = sizeof(raw);
    esp_err_t err = nvs_get_blob(handle, EXT_SETTINGS_KEY, raw, &len);
    nvs_close(handle);
    if (err == ESP_OK && len >= sizeof(uint32_t)) {
        // Blob de firmware antigo (menor): copia so os campos existentes; o resto fica no padrao
        memcpy(ext, raw, len < sizeof(*ext) ? len : sizeof(*ext));
        ext->size = sizeof(*ext);
    }
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!(ext->stealth_max_speed[i] >= 0.0f) || ext->stealth_max_speed[i] > 10000.0f) {
            ext->stealth_max_speed[i] = defaults.stealth_max_speed[i];
        }
        if (!(ext->jerk[i] >= 0.0f) || ext->jerk[i] > 1000.0f) {
            ext->jerk[i] = defaults.jerk[i];
        }
    }
    if (ext->motion_engine > MOTION_ENGINE_STREAM) {
        ext->motion_engine = defaults.motion_engine;
    }
    ext->lookahead = ext->lookahead ? 1U : 0U;
    memcpy(&s_last_ext, ext, sizeof(s_last_ext));
    s_last_ext_valid = (err == ESP_OK && len == sizeof(*ext));
    return ESP_OK;
}

esp_err_t storage_save_ext(const ext_settings_t *ext)
{
    ESP_RETURN_ON_FALSE(ext != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ext nulo");
    ext_settings_t snapshot;
    memcpy(&snapshot, ext, sizeof(snapshot));
    snapshot.size = sizeof(snapshot);

    if (s_save_lock) xSemaphoreTake(s_save_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!s_last_ext_valid || memcmp(&snapshot, &s_last_ext, sizeof(snapshot)) != 0) {
        nvs_handle_t handle;
        err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
        if (err == ESP_OK) {
            err = nvs_set_blob(handle, EXT_SETTINGS_KEY, &snapshot, sizeof(snapshot));
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }
        if (err == ESP_OK) {
            memcpy(&s_last_ext, &snapshot, sizeof(s_last_ext));
            s_last_ext_valid = true;
        }
    }
    if (s_save_lock) xSemaphoreGive(s_save_lock);
    return err;
}

// Identidade do no gravada em chaves proprias, fora do blob versionado: sobrevive a
// mudancas de SETTINGS_VERSION. Sem isso, um OTA que altere a struct zeraria node_id e
// desligaria o CAN (DEFAULT_CAN_ENABLED=0) — o no sumiria do barramento apos o update.
#define ID_KEY_NODE     "id_node"
#define ID_KEY_CAN_EN   "id_can_en"
#define ID_KEY_BITRATE  "id_can_br"
#define ID_KEY_CMD_BASE "id_can_cmd"
#define ID_KEY_ST_BASE  "id_can_st"
#define ID_KEY_EV_BASE  "id_can_ev"

static void restore_node_identity(persisted_settings_t *settings)
{
    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    uint8_t node = 0, can_en = 0;
    uint32_t bitrate = 0;
    uint16_t cmd_base = 0, st_base = 0, ev_base = 0;
    if (nvs_get_u8(handle, ID_KEY_NODE, &node) == ESP_OK &&
        nvs_get_u8(handle, ID_KEY_CAN_EN, &can_en) == ESP_OK &&
        nvs_get_u32(handle, ID_KEY_BITRATE, &bitrate) == ESP_OK &&
        nvs_get_u16(handle, ID_KEY_CMD_BASE, &cmd_base) == ESP_OK &&
        nvs_get_u16(handle, ID_KEY_ST_BASE, &st_base) == ESP_OK &&
        nvs_get_u16(handle, ID_KEY_EV_BASE, &ev_base) == ESP_OK) {
        settings->node_id = node;
        settings->can_enabled = can_en;
        settings->can_bitrate = bitrate;
        settings->can_command_base_id = cmd_base;
        settings->can_status_base_id = st_base;
        settings->can_event_base_id = ev_base;
        ESP_LOGW(APP_TAG, "Identidade do no preservada apos reset de configuracao: node=%u CAN=%s %lu bps",
                 (unsigned)node, can_en ? "ON" : "OFF", (unsigned long)bitrate);
    }
    nvs_close(handle);
}

static esp_err_t save_node_identity(nvs_handle_t handle, const persisted_settings_t *settings)
{
    esp_err_t err = nvs_set_u8(handle, ID_KEY_NODE, settings->node_id);
    if (err == ESP_OK) err = nvs_set_u8(handle, ID_KEY_CAN_EN, settings->can_enabled);
    if (err == ESP_OK) err = nvs_set_u32(handle, ID_KEY_BITRATE, settings->can_bitrate);
    if (err == ESP_OK) err = nvs_set_u16(handle, ID_KEY_CMD_BASE, settings->can_command_base_id);
    if (err == ESP_OK) err = nvs_set_u16(handle, ID_KEY_ST_BASE, settings->can_status_base_id);
    if (err == ESP_OK) err = nvs_set_u16(handle, ID_KEY_EV_BASE, settings->can_event_base_id);
    return err;
}

esp_err_t storage_load_settings(persisted_settings_t *settings)
{
    ESP_RETURN_ON_FALSE(settings != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "settings nulo");

    nvs_handle_t handle;
    esp_err_t open_err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (open_err != ESP_OK) {
        ESP_LOGW(APP_TAG, "NVS namespace '%s' nao encontrado ou erro (%s). Carregando padroes.",
                 SETTINGS_NAMESPACE, esp_err_to_name(open_err));
        *settings = (persisted_settings_t)APP_SETTINGS_DEFAULT_INIT;
        return storage_save_settings(settings);
    }

    size_t required_size = sizeof(*settings);
    esp_err_t err = nvs_get_blob(handle, SETTINGS_KEY, settings, &required_size);
    nvs_close(handle);

    if (err != ESP_OK || required_size != sizeof(*settings) || settings->version != SETTINGS_VERSION) {
        ESP_LOGI(APP_TAG, "NVS blob inexistente, tamanho divergente ou versao antiga. Gravando padroes versao %u.",
                 (unsigned)SETTINGS_VERSION);
        *settings = (persisted_settings_t)APP_SETTINGS_DEFAULT_INIT;
        restore_node_identity(settings);
        return storage_save_settings(settings);
    }

    if (settings->max_passos_z <= 0) {
        settings->max_passos_z = DEFAULT_MAX_Z_STEPS;
    }

    if (settings->z_pulley_teeth < 6 || settings->z_pulley_teeth > 200) {
        settings->z_pulley_teeth = DEFAULT_Z_PULLEY_TEETH;
    }

    if (settings->z_start_speed_mm < 0.5f || settings->z_start_speed_mm > 150.0f) {
        settings->z_start_speed_mm = 15.0f;
    }

    if (settings->c_start_speed_deg < 0.5f || settings->c_start_speed_deg > 100.0f) {
        settings->c_start_speed_deg = 10.0f;
    }

    if (settings->a_start_speed_deg < 0.5f || settings->a_start_speed_deg > 100.0f) {
        settings->a_start_speed_deg = 10.0f;
    }

    if (settings->limit_max_c_deg <= settings->limit_min_c_deg || settings->limit_min_c_deg < -3600.0f || settings->limit_max_c_deg > 3600.0f) {
        settings->limit_min_c_deg = DEFAULT_LIMIT_MIN_C_DEG;
        settings->limit_max_c_deg = DEFAULT_LIMIT_MAX_C_DEG;
    }

    if (settings->limit_max_a_deg <= settings->limit_min_a_deg || settings->limit_min_a_deg < -3600.0f || settings->limit_max_a_deg > 3600.0f) {
        settings->limit_min_a_deg = DEFAULT_LIMIT_MIN_A_DEG;
        settings->limit_max_a_deg = DEFAULT_LIMIT_MAX_A_DEG;
    }

    if (settings->home_c_deg < settings->limit_min_c_deg || settings->home_c_deg > settings->limit_max_c_deg) {
        settings->home_c_deg = 0.0f;
    }

    if (settings->home_a_deg < settings->limit_min_a_deg || settings->home_a_deg > settings->limit_max_a_deg) {
        settings->home_a_deg = 0.0f;
    }

    if (settings->speed[0] <= 0.0f) settings->speed[0] = 140.0f;
    if (settings->speed[1] <= 0.0f) settings->speed[1] = 140.0f;
    if (settings->speed[2] <= 0.0f) settings->speed[2] = 12.5f;

    // Nos ja em campo (gravados por firmwares anteriores) ainda nao tem as chaves de
    // identidade: grava-as agora para que sobrevivam a um futuro OTA com nova versao.
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        uint8_t probe = 0;
        if (nvs_get_u8(handle, ID_KEY_NODE, &probe) == ESP_ERR_NVS_NOT_FOUND &&
            save_node_identity(handle, settings) == ESP_OK) {
            (void)nvs_commit(handle);
        }
        nvs_close(handle);
    }

    memcpy(&s_last_saved, settings, sizeof(s_last_saved));
    s_last_saved_valid = true;
    return ESP_OK;
}

esp_err_t storage_save_settings(const persisted_settings_t *settings)
{
    ESP_RETURN_ON_FALSE(settings != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "settings nulo");

    // Copia antes de comparar/gravar: outras tarefas podem alterar a struct viva durante a
    // escrita; nesse caso o proprio alterador agenda uma nova gravacao.
    persisted_settings_t snapshot;
    memcpy(&snapshot, settings, sizeof(snapshot));

    if (s_save_lock) xSemaphoreTake(s_save_lock, portMAX_DELAY);
    if (s_last_saved_valid && memcmp(&snapshot, &s_last_saved, sizeof(snapshot)) == 0) {
        if (s_save_lock) xSemaphoreGive(s_save_lock);
        return ESP_OK; // nada mudou: nao reescreve a flash
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, SETTINGS_KEY, &snapshot, sizeof(snapshot));
        if (err == ESP_OK) {
            err = save_node_identity(handle, &snapshot);
        }
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    if (err == ESP_OK) {
        memcpy(&s_last_saved, &snapshot, sizeof(s_last_saved));
        s_last_saved_valid = true;
        ESP_LOGI(APP_TAG, "NVS: configuracao gravada.");
    } else {
        ESP_LOGE(APP_TAG, "NVS: falha ao gravar configuracao (%s)", esp_err_to_name(err));
    }
    if (s_save_lock) xSemaphoreGive(s_save_lock);
    return err;
}
