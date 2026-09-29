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
#include "status_led.h"
#include "ota_update.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/gpio.h"

static TaskHandle_t s_safety_task;

/* Borda de subida do fim de curso Z: acorda a safety_task na hora (antes: polling de 20 ms).
 * Os passos do Z via RMT nao checam o switch a cada passo, entao o STOP depende disso. */
static void IRAM_ATTR z_switch_isr(void *arg)
{
  (void)arg;
  BaseType_t woken = pdFALSE;
  if (s_safety_task != NULL) {
    vTaskNotifyGiveFromISR(s_safety_task, &woken);
  }
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static app_context_t g_app = {
    .settings = APP_SETTINGS_DEFAULT_INIT,
    .ext = APP_EXT_SETTINGS_DEFAULT_INIT,
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
      clearerr(stdin);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (buf_len > 0) {
        buf[buf_len] = '\0';
        snprintf(line, sizeof(line), "%s", buf);
        if (strncmp(line, "JOG ", 4) != 0 && strncmp(line, "jog ", 4) != 0) {
          ESP_LOGI(APP_TAG, "RX line=[%s]", line); // JOG chega a ~50/s: nao polui o console
        }
        commands_handle_line(ctx, line);
        buf_len = 0;
      }
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
    hardware_update_encoders();
    if (ctx->state.alarme_z_ativo && !ctx->state.em_homing_z &&
        !ctx->state.z_bloqueado && hardware_is_z_switch_pressed()) {
      ctx->state.z_bloqueado = true;
      ctx->state.homed[AXIS_Z_ID] = false;
      // MOVE_SYNC gera os passos do Z via RMT sem checar o switch a cada passo:
      // interrompe qualquer movimento em curso, como um endstop de impressora 3D.
      (void)motion_request_stop(ctx);
      ESP_LOGE(APP_TAG, "ALARME: fim de curso Z acionado inesperadamente. "
                        "Movimentos interrompidos e eixo Z suspenso.");
      printf("Estado Z: BLOQUEADO\n"); // o app atualiza o card sem esperar um STATUS
    }
    // Periodico (encoders) ou imediato quando a ISR do fim de curso Z dispara
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
  }
}

static void thermal_task(void *arg) {
  app_context_t *ctx = (app_context_t *)arg;
  uint8_t consecutive_failures = 0U;
  bool reported_offline = false;

  while (true) {
    if (ctx->state.ota_in_progress) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    float temp_c = 0.0f;
    esp_err_t err = hardware_read_temperature_c(&temp_c);
    bool log_online = false;
    bool log_offline = false;

    if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (err == ESP_OK) {
        if (!ctx->state.temp_valid || reported_offline) {
          log_online = true;
        }
        ctx->state.last_temp_c = temp_c;
        ctx->state.temp_valid = true;
        consecutive_failures = 0U;
        reported_offline = false;
      } else {
        if (consecutive_failures < 3U) {
          ++consecutive_failures;
          ESP_LOGD(APP_TAG, "Tentativa de leitura DS18B20 falhou (%s), %u/3",
                   esp_err_to_name(err), (unsigned)consecutive_failures);
          if (consecutive_failures >= 3U) {
            ctx->state.temp_valid = false;
            if (!reported_offline) {
              log_offline = true;
              reported_offline = true;
            }
          }
        }
      }
      xSemaphoreGive(ctx->state_mutex);
    }

    if (log_online) {
      ESP_LOGI(APP_TAG, "Sensor DS18B20 detectado/online. Temperatura: %.2f C", temp_c);
    } else if (log_offline) {
      ESP_LOGW(APP_TAG,
               "Sensor DS18B20 nao detectado (%s). Telemetria termica desativada (logs suprimidos).",
               esp_err_to_name(err));
    }

    if (err == ESP_OK && ctx->state.fan_mode == FAN_MODE_AUTO) {
      if (!ctx->state.fan_output_on && temp_c >= FAN_AUTO_ON_TEMP_C) {
        hardware_set_fan_output(ctx, true);
      } else if (ctx->state.fan_output_on && temp_c <= FAN_AUTO_OFF_TEMP_C) {
        hardware_set_fan_output(ctx, false);
      }
    }

    // Se o sensor nao esta presente/conectado, faz a sondagem em intervalo mais espacado (3s)
    vTaskDelay(pdMS_TO_TICKS(consecutive_failures >= 3U ? 3000 : 1000));
  }
}

void app_main(void) {
  uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
  uart_vfs_dev_use_driver(UART_NUM_0);
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
  (void)ota_update_boot_check(&g_app);
  esp_err_t store_err = storage_load_settings(&g_app.settings);
  if (store_err != ESP_OK) {
      ESP_LOGE(APP_TAG, "Aviso ao carregar settings da NVS (%s). Usando padroes.", esp_err_to_name(store_err));
      g_app.settings = (persisted_settings_t)APP_SETTINGS_DEFAULT_INIT;
  }
  (void)storage_load_ext(&g_app.ext);
  for (size_t i = 0; i < AXIS_COUNT; ++i) {
      g_app.state.speed_delay_us[i] = g_app.settings.speed_delay_us[i];
      g_app.state.speed[i] = g_app.settings.speed[i];
      g_app.state.speed_max[i] = g_app.settings.speed_max[i];
      g_app.state.accel_max[i] = g_app.settings.accel_max[i];
      g_app.state.inverter[i] = (bool)g_app.settings.inverter[i];
  }
  ESP_ERROR_CHECK(hardware_init(&g_app));
  if (status_led_init(&g_app) != ESP_OK) {
    ESP_LOGW(APP_TAG, "LED de status (GPIO %d) nao inicializou.", BOARD_RGB_LED_PIN);
  }
  g_app.state.driver_mode_requested =
      (driver_bus_mode_t)g_app.settings.driver_bus_mode;

  ESP_ERROR_CHECK(motion_init(&g_app));

  if (tmc2209_init(&g_app) != ESP_OK) {
    ESP_LOGW(APP_TAG,
             "TMC UART nao entrou totalmente. STEP/DIR continua habilitado.");
  }
  // VM dos motores pode subir depois do ESP32 (ou oscilar): o monitor reaplica a
  // configuracao quando o driver volta a responder ou reporta reset.
  (void)tmc2209_start_monitor(&g_app);
  if (can_bus_init(&g_app) != ESP_OK) {
    ESP_LOGW(APP_TAG, "CAN/TWAI nao entrou totalmente. Firmware segue local.");
  }


  xTaskCreatePinnedToCore(safety_task, "safety_task", 4096, &g_app, 10, &s_safety_task,
                          1);
  esp_err_t isr_err = gpio_install_isr_service(0);
  if (isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE) {
    (void)gpio_set_intr_type(SWITCH_Z, GPIO_INTR_POSEDGE);
    if (gpio_isr_handler_add(SWITCH_Z, z_switch_isr, NULL) != ESP_OK) {
      ESP_LOGW(APP_TAG, "ISR do fim de curso Z indisponivel; safety_task segue por polling.");
    }
  }
  xTaskCreatePinnedToCore(thermal_task, "thermal_task", 4096, &g_app, 5, NULL,
                          0);
  xTaskCreatePinnedToCore(console_task, "console_task", 6144, &g_app, 4, NULL,
                          0);
}
