#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_defs.h"
#include "can_bus.h"
#include "commands.h"
#include "hardware.h"
#include "motion.h"
#include "storage.h"
#include "tmc2209.h"

static app_context_t g_app = {
    .settings = APP_SETTINGS_DEFAULT_INIT,
    .state = APP_RUNTIME_DEFAULT_INIT,
    .motion_mutex = NULL,
};

static void console_task(void *arg) {
  app_context_t *ctx = (app_context_t *)arg;
  char line[96];
  char buf[96];
  size_t buf_len = 0;

  commands_print_help();
  puts("\n=== SISTEMA PRONTO PARA COMANDOS ===");

  while (true) {
    int c = fgetc(stdin);
    if (c == EOF) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (c == '\n') {
      if (buf_len > 0) {
        buf[buf_len] = '\0';
        snprintf(line, sizeof(line), "%s", buf);
        ESP_LOGI(APP_TAG, "RX line=[%s]", line);
        commands_handle_line(ctx, line);
        buf_len = 0;
      }
    } else if (c == '\r') {
      // ignore CR
    } else {
      if (buf_len < sizeof(buf) - 1) {
        buf[buf_len++] = (char)c;
      }
    }
  }
}

static void safety_task(void *arg) {
  app_context_t *ctx = (app_context_t *)arg;

  while (true) {
    if (ctx->state.alarme_z_ativo && !ctx->state.em_homing_z &&
        !ctx->state.z_bloqueado && hardware_is_z_switch_pressed()) {
      ctx->state.z_bloqueado = true;
      ESP_LOGE(APP_TAG, "ALARME: fim de curso Z acionado inesperadamente. "
                        "Movimentos no eixo Z suspensos.");
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void thermal_task(void *arg) {
  app_context_t *ctx = (app_context_t *)arg;
  uint8_t consecutive_failures = 0U;

  while (true) {
    float temp_c = 0.0f;
    esp_err_t err = hardware_read_temperature_c(&temp_c);
    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (err == ESP_OK) {
        ctx->state.last_temp_c = temp_c;
        ctx->state.temp_valid = true;
        consecutive_failures = 0U;
      } else {
        if (consecutive_failures < 3U) {
          ++consecutive_failures;
        }
        if (consecutive_failures >= 3U) {
          ctx->state.temp_valid = false;
        }
      }
      xSemaphoreGive(ctx->state_mutex);
    }

    if (err == ESP_OK && ctx->state.fan_mode == FAN_MODE_AUTO) {
      if (!ctx->state.fan_output_on && temp_c >= FAN_AUTO_ON_TEMP_C) {
        hardware_set_fan_output(ctx, true);
      } else if (ctx->state.fan_output_on && temp_c <= FAN_AUTO_OFF_TEMP_C) {
        hardware_set_fan_output(ctx, false);
      }
    }

    if (err != ESP_OK) {
      ESP_LOGW(APP_TAG, "Falha ao ler DS18B20 (%s), tentativa consecutiva %u/3",
               esp_err_to_name(err), (unsigned)consecutive_failures);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  g_app.motion_mutex = xSemaphoreCreateMutex();
  if (g_app.motion_mutex == NULL) {
    ESP_LOGE(APP_TAG, "Falha ao criar mutex de movimento");
    return;
  }

  g_app.state_mutex = xSemaphoreCreateMutex();
  if (g_app.state_mutex == NULL) {
    ESP_LOGE(APP_TAG, "Falha ao criar mutex de estado");
    return;
  }

  ESP_ERROR_CHECK(storage_init());
  esp_err_t store_err = storage_load_settings(&g_app.settings);
  if (store_err != ESP_OK) {
      ESP_LOGE(APP_TAG, "Aviso ao carregar settings da NVS (%s). Usando padroes.", esp_err_to_name(store_err));
      g_app.settings = (persisted_settings_t)APP_SETTINGS_DEFAULT_INIT;
  }
  for (size_t i = 0; i < AXIS_COUNT; ++i) {
      g_app.state.speed_delay_us[i] = g_app.settings.speed_delay_us[i];
      g_app.state.inverter[i] = (bool)g_app.settings.inverter[i];
  }
  ESP_ERROR_CHECK(hardware_init(&g_app));
  g_app.state.driver_mode_requested =
      (driver_bus_mode_t)g_app.settings.driver_bus_mode;

  ESP_ERROR_CHECK(motion_init(&g_app));

  if (tmc2209_init(&g_app) != ESP_OK) {
    ESP_LOGW(APP_TAG,
             "TMC UART nao entrou totalmente. STEP/DIR continua habilitado.");
  }
  if (can_bus_init(&g_app) != ESP_OK) {
    ESP_LOGW(APP_TAG, "CAN/TWAI nao entrou totalmente. Firmware segue local.");
  }

  ESP_LOGI(APP_TAG, "Postando pedido de auto-ajuste C e A...");
  (void)motion_post_home_axis(&g_app, 'C', 0, 0);
  (void)motion_post_home_axis(&g_app, 'A', 0, 0);

  xTaskCreatePinnedToCore(safety_task, "safety_task", 4096, &g_app, 10, NULL,
                          1);
  xTaskCreatePinnedToCore(thermal_task, "thermal_task", 4096, &g_app, 5, NULL,
                          0);
  xTaskCreatePinnedToCore(console_task, "console_task", 6144, &g_app, 4, NULL,
                          0);
}
