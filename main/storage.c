#include "storage.h"

#include "esp_check.h"
#include "nvs.h"
#include "nvs_flash.h"

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t storage_load_settings(persisted_settings_t *settings)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_FALSE(settings != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "settings nulo");
    ESP_RETURN_ON_ERROR(nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle), APP_TAG, "Falha ao abrir NVS");

    size_t required_size = sizeof(*settings);
    esp_err_t err = nvs_get_blob(handle, SETTINGS_KEY, settings, &required_size);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND || required_size != sizeof(*settings) || settings->version != SETTINGS_VERSION) {
        *settings = (persisted_settings_t)APP_SETTINGS_DEFAULT_INIT;
        return storage_save_settings(settings);
    }

    if (err != ESP_OK) {
        return err;
    }

    if (settings->max_passos_z <= 0) {
        settings->max_passos_z = DEFAULT_MAX_Z_STEPS;
    }

    return ESP_OK;
}

esp_err_t storage_save_settings(const persisted_settings_t *settings)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_FALSE(settings != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "settings nulo");
    ESP_RETURN_ON_ERROR(nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle), APP_TAG, "Falha ao abrir NVS");

    esp_err_t err = nvs_set_blob(handle, SETTINGS_KEY, settings, sizeof(*settings));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
