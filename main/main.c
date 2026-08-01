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

  commands_print_help();
  puts("\n=== SISTEMA PRONTO PARA COMANDOS ===");

  while (true) {
    if (fgets(line, sizeof(line), stdin) != NULL) {
      commands_handle_line(ctx, line);
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
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

  while (true) {
    float temp_c = 0.0f;
    esp_err_t err = hardware_read_temperature_c(&temp_c);
    if (err == ESP_OK) {
      ctx->state.last_temp_c = temp_c;
      ctx->state.temp_valid = true;

      if (ctx->state.fan_mode == FAN_MODE_AUTO) {
        if (!ctx->state.fan_output_on && temp_c >= FAN_AUTO_ON_TEMP_C) {
          hardware_set_fan_output(ctx, true);
        } else if (ctx->state.fan_output_on && temp_c <= FAN_AUTO_OFF_TEMP_C) {
          hardware_set_fan_output(ctx, false);
        }
      }
    } else {
      ctx->state.temp_valid = false;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
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
  ESP_ERROR_CHECK(storage_load_settings(&g_app.settings));
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

  ESP_LOGI(APP_TAG, "Postando pedido de auto-ajuste X e Y...");
  (void)motion_post_home_axis(&g_app, 'X', 0, 0);
  (void)motion_post_home_axis(&g_app, 'Y', 0, 0);

  xTaskCreatePinnedToCore(safety_task, "safety_task", 4096, &g_app, 10, NULL,
                          1);
  xTaskCreatePinnedToCore(thermal_task, "thermal_task", 4096, &g_app, 5, NULL,
                          0);
  xTaskCreatePinnedToCore(console_task, "console_task", 6144, &g_app, 4, NULL,
                          0);
}
