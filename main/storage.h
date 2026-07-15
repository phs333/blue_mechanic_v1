#pragma once

#include "esp_err.h"

#include "app_defs.h"

esp_err_t storage_init(void);
esp_err_t storage_load_settings(persisted_settings_t *settings);
esp_err_t storage_save_settings(const persisted_settings_t *settings);
