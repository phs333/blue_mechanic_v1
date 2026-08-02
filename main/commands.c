#include "commands.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_bus.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hardware.h"
#include "motion.h"
#include "storage.h"
#include "tmc2209.h"

static esp_err_t persist_settings(app_context_t *ctx);
static bool parse_u32_token(const char *text, uint32_t *value);
static void trim_ascii(char *text);
static void to_upper_ascii(char *text);

void commands_print_help(void)
{
    puts("\nComandos disponiveis:");
    puts("STATUS");
    puts("HELP");
    puts("DRIVER ENABLED ON / OFF");
    puts("ALARM ON / ALARM OFF");
    puts("SET_HOME X / Y");
    puts("HOME X / Y / Z");
    puts("SET_LENGTH Z <steps>");
    puts("STEPS X|Y|Z <steps_per_rev>");
    puts("SPEED X|Y <deg/s> | Z <mm/s>");
    puts("ACCEL X|Y <deg/s^2> | Z <mm/s^2>");
    puts("MOVE X|Y|Z <steps>");
    puts("MOVE_F X|Y|Z <steps>");
    puts("LASER 1|2 ON|OFF|0..255");
    puts("FAN 0|1|AUTO");
    puts("TEMP");
    puts("DRIVER STATUS");
    puts("DRIVER INVERT X|Y|Z ON|OFF");
    puts("DRIVER MODE STEPDIR|UART");
    puts("DRIVER UART ADDR X|Y|Z <0..3>");
    puts("DRIVER UART CURRENT X|Y|Z <ihold_mA> <irun_mA> <delay>");
    puts("DRIVER UART SPREADCYCLE X|Y|Z ON|OFF");
    puts("DRIVER UART MICROSTEPS X|Y|Z <1..256>");
    puts("DRIVER REG READ X|Y|Z <reg>");
    puts("DRIVER REG WRITE X|Y|Z <reg> <value>");
    puts("DRIVER APPLY");
    puts("CAN STATUS");
    puts("CAN ON / CAN OFF / CAN APPLY");
    puts("CAN NODE <1..127>");
    puts("CAN BITRATE <125000|250000|500000|1000000>");
    puts("CAN BASE CMD|STATUS|EVENT <id>");
    puts("CAN SEND STATUS");
}

void commands_print_status(app_context_t *ctx)
{
    float encoder_x = 0.0f;
    float encoder_y = 0.0f;
    esp_err_t err_x = hardware_read_axis_encoder('X', &encoder_x);
    esp_err_t err_y = hardware_read_axis_encoder('Y', &encoder_y);

    printf("\n=== STATUS ===\n");
    if (err_x == ESP_OK) {
        printf("Eixo X: %.2f deg\n", encoder_x);
    } else {
        printf("Eixo X: erro de leitura (%s)\n", esp_err_to_name(err_x));
    }
    if (err_y == ESP_OK) {
        printf("Eixo Y: %.2f deg\n", encoder_y);
    } else {
        printf("Eixo Y: erro de leitura (%s)\n", esp_err_to_name(err_y));
    }

    printf("Eixo Z: %ld / %ld passos\n", (long)ctx->state.atual_z, (long)ctx->settings.max_passos_z);
    printf("Alarme Z: %s\n", ctx->state.alarme_z_ativo ? "ON" : "OFF");
    printf("Estado Z: %s\n", ctx->state.z_bloqueado ? "BLOQUEADO" : "LIVRE");
    printf("Drivers: %s\n", ctx->state.drivers_enabled ? "ENERGIZADOS" : "DESLIGADOS");
    printf("Laser 1: %u/255\n", ctx->state.laser_level[0]);
    printf("Laser 2: %u/255\n", ctx->state.laser_level[1]);

    const char *fan_mode = "MANUAL OFF";
    if (ctx->state.fan_mode == FAN_MODE_MANUAL_ON) {
        fan_mode = "MANUAL ON";
    } else if (ctx->state.fan_mode == FAN_MODE_AUTO) {
        fan_mode = "AUTO";
    }
    printf("Fan: %s (%s)\n", ctx->state.fan_output_on ? "ON" : "OFF", fan_mode);

    if (ctx->state.temp_valid) {
        printf("Temperatura: %.2f C\n", ctx->state.last_temp_c);
    } else {
        printf("Temperatura: indisponivel\n");
    }
    tmc2209_print_status(ctx);
    can_bus_print_status(ctx);
    printf("================\n");
}

void commands_handle_line(app_context_t *ctx, const char *line)
{
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "%s", line);
    trim_ascii(cmd);
    to_upper_ascii(cmd);

    if (cmd[0] == '\0') {
        return;
    }

    if (strcmp(cmd, "STATUS") == 0) {
        commands_print_status(ctx);
        return;
    }

    if (strcmp(cmd, "HELP") == 0) {
        commands_print_help();
        return;
    }

    if (strcmp(cmd, "ALARM OFF") == 0) {
        ctx->state.alarme_z_ativo = false;
        ctx->state.z_bloqueado = false;
        puts("ALARME Z DESATIVADO. Fim de curso ignorado.");
        return;
    }

    if (strcmp(cmd, "ALARM ON") == 0) {
        ctx->state.alarme_z_ativo = true;
        puts("ALARME Z ATIVADO. Fim de curso em vigia.");
        return;
    }

    if (strcmp(cmd, "DRIVER ENABLED ON") == 0) {
        hardware_set_driver_enable(ctx, true);
        puts("Drivers energizados.");
        return;
    }

    if (strcmp(cmd, "DRIVER ENABLED OFF") == 0) {
        hardware_set_driver_enable(ctx, false);
        puts("Drivers desligados.");
        return;
    }

    if (strncmp(cmd, "DRIVER INVERT ", 14) == 0) {
        char axis = cmd[14];
        char state_str[8] = {0};
        if (sscanf(cmd + 14, "%c %7s", &axis, state_str) != 2) {
            puts("Uso: DRIVER INVERT X|Y|Z ON|OFF");
            return;
        }
        axis = (char)toupper((unsigned char)axis);
        size_t axis_index = 0;
        if (axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido. Use X, Y ou Z.");
            return;
        }
        bool enable = (strcmp(state_str, "ON") == 0);
        ctx->state.inverter[axis_index] = enable;
        printf("Inversao %c %s.\n", axis, enable ? "ATIVADA" : "DESATIVADA");
        return;
    }

    if (strcmp(cmd, "SETHOME X") == 0 || strcmp(cmd, "SETHOME Y") == 0) {
        char axis = cmd[8];
        float current_deg = 0.0f;
        esp_err_t err = hardware_read_axis_encoder(axis, &current_deg);
        if (err != ESP_OK) {
            printf("ERRO: nao foi possivel ler encoder %c (%s)\n", axis, esp_err_to_name(err));
            return;
        }

        if (axis == 'X') {
            ctx->settings.home_x_deg = current_deg;
        } else {
            ctx->settings.home_y_deg = current_deg;
        }

        err = storage_save_settings(&ctx->settings);
        if (err == ESP_OK) {
            printf("Home %c gravado em %.2f deg.\n", axis, current_deg);
        } else {
            printf("ERRO ao salvar home %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "HOME X") == 0 || strcmp(cmd, "HOME Y") == 0) {
        char axis = cmd[5];
        esp_err_t err = motion_post_home_axis(ctx, axis, 0, 0);
        if (err == ESP_OK) {
            printf("Home %c finalizado.\n", axis);
        } else {
            printf("ERRO no home %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "HOME Z") == 0) {
        esp_err_t err = motion_post_home_axis(ctx, 'Z', 0, 0);
        if (err == ESP_OK) {
            puts("Home Z finalizado.");
        } else {
            printf("ERRO no HOME Z: %s\n", esp_err_to_name(err));
        }
        return;
    }

    long steps_z = 0;
    if (sscanf(cmd, "SETLENGTH Z %ld", &steps_z) == 1) {
        if (steps_z <= 0 || steps_z > 100000) {
            puts("Valor invalido para limite Z (deve ser entre 1 e 100000).");
            return;
        }
        ctx->settings.max_passos_z = (int32_t)steps_z;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Limite Z definido manualmente para %ld passos.\n", steps_z);
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "TEMP") == 0) {
        if (ctx->state.temp_valid) {
            printf("Temperatura atual: %.2f C\n", ctx->state.last_temp_c);
        } else {
            puts("Temperatura indisponivel.");
        }
        return;
    }

    if (strcmp(cmd, "FAN AUTO") == 0) {
        ctx->state.fan_mode = FAN_MODE_AUTO;
        puts("Fan em modo AUTO.");
        return;
    }

    char fan_arg[16];
    if (sscanf(cmd, "FAN %15s", fan_arg) == 1) {
        if (strcmp(fan_arg, "1") == 0 || strcmp(fan_arg, "ON") == 0) {
            ctx->state.fan_mode = FAN_MODE_MANUAL_ON;
            hardware_set_fan_output(ctx, true);
            puts("Fan ligado.");
            return;
        }
        if (strcmp(fan_arg, "0") == 0 || strcmp(fan_arg, "OFF") == 0) {
            ctx->state.fan_mode = FAN_MODE_MANUAL_OFF;
            hardware_set_fan_output(ctx, false);
            puts("Fan desligado.");
            return;
        }
    }

    char axis = '\0';
    long steps_per_rev = 0;
    if (sscanf(cmd, "STEPS %c %ld", &axis, &steps_per_rev) == 2) {
        axis = (char)toupper((unsigned char)axis);
        size_t axis_index = 0;
        if (axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido. Use X, Y ou Z.");
            return;
        }
        if (steps_per_rev <= 0 || steps_per_rev > 10000) {
            puts("Valor invalido para passos por rev (deve ser entre 1 e 10000).");
            return;
        }
        ctx->settings.steps_per_rev[axis_index] = (uint16_t)steps_per_rev;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Passos por rev do eixo %c definidos para %lu.\n", axis, (unsigned long)steps_per_rev);
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    float speed_val = 0.0f;
    if (sscanf(cmd, "SPEED %c %f", &axis, &speed_val) == 2) {
        axis = (char)toupper((unsigned char)axis);
        size_t axis_index = 0;
        if (axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido. Use X, Y ou Z.");
            return;
        }
        if (axis == 'Z') {
            if (speed_val < 0.01f || speed_val > 100.0f) {
                puts("Velocidade Z invalida. Use mm/s entre 0.01 e 100.");
                return;
            }
        } else {
            if (speed_val < 0.1f || speed_val > 1000.0f) {
                puts("Velocidade XY invalida. Use deg/s entre 0.1 e 1000.");
                return;
            }
        }
        uint32_t delay_us = motion_speed_to_delay_us(ctx, axis, speed_val);
        ctx->settings.speed_delay_us[axis_index] = delay_us;
        ctx->state.speed_delay_us[axis_index] = delay_us;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            if (axis == 'Z') {
                printf("Velocidade Z definida para %.2f mm/s (%lu us).\n", speed_val, (unsigned long)delay_us);
            } else {
                printf("Velocidade %c definida para %.2f deg/s (%lu us).\n", axis, speed_val, (unsigned long)delay_us);
            }
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    float accel_val = 0.0f;
    if (sscanf(cmd, "ACCEL %c %f", &axis, &accel_val) == 2) {
        axis = (char)toupper((unsigned char)axis);
        size_t axis_index = 0;
        if (axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido. Use X, Y ou Z.");
            return;
        }
        if (accel_val < 0.1f || accel_val > 5000.0f) {
            puts("Aceleracao invalida. Use 0.1 a 5000.");
            return;
        }
        ctx->settings.accel[axis_index] = accel_val;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            if (axis == 'Z') {
                printf("Aceleracao Z definida para %.2f mm/s^2.\n", accel_val);
            } else {
                printf("Aceleracao %c definida para %.2f deg/s^2.\n", axis, accel_val);
            }
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    int laser_number = 0;
    char laser_value[16] = {0};
    if (sscanf(cmd, "LASER %d %15s", &laser_number, laser_value) == 2) {
        if (laser_number < 1 || laser_number > 2) {
            puts("Laser invalido. Use 1 ou 2.");
            return;
        }

        uint8_t level = 0;
        if (strcmp(laser_value, "ON") == 0) {
            level = 255;
        } else if (strcmp(laser_value, "OFF") == 0) {
            level = 0;
        } else {
            char *endptr = NULL;
            long numeric = strtol(laser_value, &endptr, 10);
            if (endptr == laser_value || *endptr != '\0' || numeric < 0 || numeric > 255) {
                puts("Valor de laser invalido. Use ON, OFF ou 0..255.");
                return;
            }
            level = (uint8_t)numeric;
        }

        esp_err_t err = hardware_set_laser_level(ctx, (size_t)(laser_number - 1), level);
        if (err == ESP_OK) {
            printf("Laser %d ajustado para %u/255.\n", laser_number, level);
        } else {
            printf("ERRO ao ajustar laser %d: %s\n", laser_number, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "DRIVER STATUS") == 0) {
        tmc2209_print_status(ctx);
        return;
    }

    if (strcmp(cmd, "DRIVER APPLY") == 0) {
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            err = tmc2209_apply_settings(ctx);
        }
        if (err == ESP_OK) {
            puts("Configuracao dos drivers reaplicada.");
        } else {
            printf("ERRO ao aplicar driver UART: %s\n", esp_err_to_name(err));
        }
        return;
    }

    char driver_mode[16];
    if (sscanf(cmd, "DRIVER MODE %15s", driver_mode) == 1) {
        if (strcmp(driver_mode, "STEPDIR") == 0) {
            ctx->settings.driver_bus_mode = DRIVER_BUS_MODE_STEP_DIR_ONLY;
        } else if (strcmp(driver_mode, "UART") == 0) {
            ctx->settings.driver_bus_mode = DRIVER_BUS_MODE_UART_OPTIONAL;
        } else {
            puts("Modo invalido. Use STEPDIR ou UART.");
            return;
        }

        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            ctx->state.driver_mode_requested = (driver_bus_mode_t)ctx->settings.driver_bus_mode;
            puts("Modo de driver salvo. Use DRIVER APPLY para reconfigurar a UART.");
        } else {
            printf("ERRO ao salvar modo do driver: %s\n", esp_err_to_name(err));
        }
        return;
    }

    char driver_axis = '\0';
    unsigned addr = 0;
    if (sscanf(cmd, "DRIVER UART ADDR %c %u", &driver_axis, &addr) == 2) {
        size_t axis_index = 0;
        driver_axis = (char)toupper((unsigned char)driver_axis);
        if (driver_axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (driver_axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (driver_axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido para endereco TMC.");
            return;
        }
        if (addr > 3U) {
            puts("Endereco TMC invalido. Use 0..3.");
            return;
        }

        ctx->settings.tmc_slave_addr[axis_index] = (uint8_t)addr;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Endereco TMC do eixo %c salvo em %u.\n", driver_axis, addr);
        } else {
            printf("ERRO ao salvar endereco TMC: %s\n", esp_err_to_name(err));
        }
        return;
    }

    unsigned ihold_ma = 0;
    unsigned irun_ma = 0;
    unsigned ihold_delay = 0;
    if (sscanf(cmd, "DRIVER UART CURRENT %c %u %u %u", &driver_axis, &ihold_ma, &irun_ma, &ihold_delay) == 4) {
        size_t axis_index = 0;
        driver_axis = (char)toupper((unsigned char)driver_axis);
        if (driver_axis == 'X') {
            axis_index = AXIS_X_ID;
        } else if (driver_axis == 'Y') {
            axis_index = AXIS_Y_ID;
        } else if (driver_axis == 'Z') {
            axis_index = AXIS_Z_ID;
        } else {
            puts("Eixo invalido para corrente TMC.");
            return;
        }
        if (ihold_ma > 2000U || irun_ma > 2000U || ihold_delay > 15U) {
            puts("Valores invalidos. mA 0..2000, delay 0..15.");
            return;
        }

        ctx->settings.tmc_ihold[axis_index] = tmc2209_ma_to_cs((uint16_t)ihold_ma);
        ctx->settings.tmc_irun[axis_index] = tmc2209_ma_to_cs((uint16_t)irun_ma);
        ctx->settings.tmc_ihold_delay[axis_index] = (uint8_t)ihold_delay;

        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Corrente TMC do eixo %c salva.\n", driver_axis);
        } else {
            printf("ERRO ao salvar corrente TMC: %s\n", esp_err_to_name(err));
        }
        return;
    }

    char mode_token[24];
    if (sscanf(cmd, "DRIVER UART SPREADCYCLE %c %23s", &driver_axis, mode_token) == 2) {
        driver_axis = (char)toupper((unsigned char)driver_axis);
        to_upper_ascii(mode_token);
        bool enable = false;
        if (strcmp(mode_token, "ON") == 0) {
            enable = true;
        } else if (strcmp(mode_token, "OFF") == 0) {
            enable = false;
        } else {
            puts("Modo invalido. Use ON ou OFF.");
            return;
        }
        esp_err_t err = tmc2209_set_spreadcycle(ctx, driver_axis, enable);
        if (err == ESP_OK) {
            printf("Eixo %c SPREADCYCLE definido para %s\n", driver_axis, mode_token);
        } else {
            printf("ERRO ao configurar SPREADCYCLE: %s\n", esp_err_to_name(err));
        }
        return;
    }

    unsigned microsteps_val = 0;
    if (sscanf(cmd, "DRIVER UART MICROSTEPS %c %u", &driver_axis, &microsteps_val) == 2) {
        driver_axis = (char)toupper((unsigned char)driver_axis);
        esp_err_t err = tmc2209_set_microsteps(ctx, driver_axis, (uint16_t)microsteps_val);
        if (err == ESP_OK) {
            printf("Eixo %c MICROSTEPS definido para %u\n", driver_axis, microsteps_val);
        } else {
            printf("ERRO ao configurar MICROSTEPS: %s\n", esp_err_to_name(err));
        }
        return;
    }

    char reg_token[24];
    char value_token[24];
    if (sscanf(cmd, "DRIVER REG READ %c %23s", &driver_axis, reg_token) == 2) {
        uint32_t reg_addr = 0;
        uint32_t value = 0;
        driver_axis = (char)toupper((unsigned char)driver_axis);
        if (!parse_u32_token(reg_token, &reg_addr) || reg_addr > 0x7FU) {
            puts("Endereco de registrador invalido.");
            return;
        }

        esp_err_t err = tmc2209_read_register(ctx, driver_axis, (uint8_t)reg_addr, &value);
        if (err == ESP_OK) {
            printf("TMC %c REG 0x%02" PRIX32 " = 0x%08" PRIX32 "\n", driver_axis, reg_addr, value);
        } else {
            printf("ERRO na leitura TMC %c: %s\n", driver_axis, esp_err_to_name(err));
        }
        return;
    }

    if (sscanf(cmd, "DRIVER REG WRITE %c %23s %23s", &driver_axis, reg_token, value_token) == 3) {
        uint32_t reg_addr = 0;
        uint32_t reg_value = 0;
        driver_axis = (char)toupper((unsigned char)driver_axis);
        if (!parse_u32_token(reg_token, &reg_addr) || reg_addr > 0x7FU || !parse_u32_token(value_token, &reg_value)) {
            puts("Registrador ou valor invalido.");
            return;
        }

        esp_err_t err = tmc2209_write_register(ctx, driver_axis, (uint8_t)reg_addr, reg_value);
        if (err == ESP_OK) {
            printf("TMC %c REG 0x%02" PRIX32 " <= 0x%08" PRIX32 "\n", driver_axis, reg_addr, reg_value);
        } else {
            printf("ERRO na escrita TMC %c: %s\n", driver_axis, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "CAN STATUS") == 0) {
        can_bus_print_status(ctx);
        return;
    }

    if (strcmp(cmd, "CAN APPLY") == 0) {
        esp_err_t err = can_bus_apply_settings(ctx);
        if (err == ESP_OK) {
            puts("Configuracao CAN reaplicada.");
        } else {
            printf("ERRO ao aplicar configuracao CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "CAN ON") == 0 || strcmp(cmd, "CAN OFF") == 0) {
        ctx->settings.can_enabled = (strcmp(cmd, "CAN ON") == 0) ? 1U : 0U;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            puts("Estado do CAN salvo. Use CAN APPLY para reiniciar o barramento.");
        } else {
            printf("ERRO ao salvar estado do CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    unsigned node_id = 0;
    if (sscanf(cmd, "CAN NODE %u", &node_id) == 1) {
        if (node_id == 0U || node_id > 127U) {
            puts("Node ID invalido. Use 1..127.");
            return;
        }
        ctx->settings.node_id = (uint8_t)node_id;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Node ID CAN salvo em %u.\n", node_id);
        } else {
            printf("ERRO ao salvar node ID CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    uint32_t bitrate = 0;
    if (sscanf(cmd, "CAN BITRATE %" SCNu32, &bitrate) == 1) {
        switch (bitrate) {
        case 125000U:
        case 250000U:
        case 500000U:
        case 1000000U:
            break;
        default:
            puts("Bitrate invalido. Use 125000, 250000, 500000 ou 1000000.");
            return;
        }
        ctx->settings.can_bitrate = bitrate;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Bitrate CAN salvo em %" PRIu32 ".\n", bitrate);
        } else {
            printf("ERRO ao salvar bitrate CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    char base_type[16];
    if (sscanf(cmd, "CAN BASE %15s %23s", base_type, value_token) == 2) {
        uint32_t base_id = 0;
        if (!parse_u32_token(value_token, &base_id) || base_id > 0x7FFU) {
            puts("Base ID CAN invalida.");
            return;
        }

        if (strcmp(base_type, "CMD") == 0) {
            ctx->settings.can_command_base_id = (uint16_t)base_id;
        } else if (strcmp(base_type, "STATUS") == 0) {
            ctx->settings.can_status_base_id = (uint16_t)base_id;
        } else if (strcmp(base_type, "EVENT") == 0) {
            ctx->settings.can_event_base_id = (uint16_t)base_id;
        } else {
            puts("Tipo de base CAN invalido. Use CMD, STATUS ou EVENT.");
            return;
        }

        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Base CAN %s salva em 0x%03" PRIX32 ".\n", base_type, base_id);
        } else {
            printf("ERRO ao salvar base CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "CAN SEND STATUS") == 0) {
        esp_err_t err = can_bus_send_status(ctx);
        if (err == ESP_OK) {
            puts("Status enviado no CAN.");
        } else {
            printf("ERRO ao enviar status no CAN: %s\n", esp_err_to_name(err));
        }
        return;
    }

    long steps = 0;
    if (sscanf(cmd, "MOVE %c %ld", &axis, &steps) == 2) {
        axis = (char)toupper((unsigned char)axis);
        esp_err_t err = motion_post_move_axis(ctx, axis, (int32_t)steps, 0, 0);
        if (axis == 'Z' && err == ESP_ERR_INVALID_STATE) {
            puts("AVISO: Eixo Z bloqueado por seguranca. Use 'ALARM OFF' ou 'HOME Z'.");
            return;
        }

        if (err == ESP_OK) {
            printf("MOVE %c enfileirado.\n", axis);
        } else {
            printf("ERRO no MOVE %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (sscanf(cmd, "MOVE_F %c %ld", &axis, &steps) == 2) {
        axis = (char)toupper((unsigned char)axis);
        ESP_LOGI(APP_TAG, "PARSER MOVE_F axis=%c raw_steps=%ld", axis, steps);
        esp_err_t err = motion_post_move_axis_force(ctx, axis, (int32_t)steps, 0, 0);
        if (axis == 'Z' && err == ESP_ERR_INVALID_STATE) {
            puts("AVISO: Eixo Z bloqueado por seguranca. Use 'ALARM OFF' ou 'HOME Z'.");
            return;
        }

        if (err == ESP_OK) {
            printf("MOVE_F %c enfileirado (sem encoder).\n", axis);
        } else {
            printf("ERRO no MOVE_F %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    puts("Comando desconhecido. Use HELP.");
}

static esp_err_t persist_settings(app_context_t *ctx)
{
    return storage_save_settings(&ctx->settings);
}

static bool parse_u32_token(const char *text, uint32_t *value)
{
    char *endptr = NULL;
    unsigned long parsed = strtoul(text, &endptr, 0);
    if (text == NULL || value == NULL || endptr == text || *endptr != '\0') {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static void trim_ascii(char *text)
{
    char *start = text;
    while (*start != '\0' && isspace((unsigned char)*start)) {
        ++start;
    }

    if (start != text) {
        memmove(text, start, strlen(start) + 1U);
    }

    size_t len = strlen(text);
    while (len > 0 && isspace((unsigned char)text[len - 1U])) {
        text[len - 1U] = '\0';
        --len;
    }
}

static void to_upper_ascii(char *text)
{
    while (*text != '\0') {
        *text = (char)toupper((unsigned char)*text);
        ++text;
    }
}
