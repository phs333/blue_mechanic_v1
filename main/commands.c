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
static bool parse_laser_level_token(const char *text, uint8_t *level);
static uint8_t percent_to_laser_level(uint32_t percent);
static uint32_t laser_level_to_percent(uint8_t level);
static void trim_ascii(char *text);
static void to_upper_ascii(char *text);

static bool parse_axis_token(char axis_char, size_t *axis_index, char *canonical_axis)
{
    char u = (char)toupper((unsigned char)axis_char);
    if (u == 'C' || u == 'X') {
        if (axis_index) *axis_index = AXIS_C_ID;
        if (canonical_axis) *canonical_axis = 'C';
        return true;
    }
    if (u == 'A' || u == 'Y') {
        if (axis_index) *axis_index = AXIS_A_ID;
        if (canonical_axis) *canonical_axis = 'A';
        return true;
    }
    if (u == 'Z') {
        if (axis_index) *axis_index = AXIS_Z_ID;
        if (canonical_axis) *canonical_axis = 'Z';
        return true;
    }
    return false;
}

void commands_print_help(void)
{
    puts("\nComandos disponiveis:");
    puts("STATUS");
    puts("HELP");
    puts("DRIVER ENABLED ON / OFF");
    puts("ALARM ON / ALARM OFF");
    puts("SETHOME C / A (ou X / Y)");
    puts("HOME C / A / Z (ou X / Y / Z)");
    puts("SET_LENGTH Z <steps>");
    puts("PULLEY Z <dentes> (ex: 16, 20)");
    puts("STEPS C|A|Z <steps_per_rev>");
    puts("SPEED C|A <deg/s> | Z <mm/s>");
    puts("ACCEL C|A <deg/s^2> | Z <mm/s^2>");
    puts("SPEED_MAX C|A|Z <value> | ACCEL_MAX C|A|Z <value>");
    puts("MOVE C|A|Z <steps> [S<speed>] [F<accel>]");
    puts("MOVE_F C|A|Z <steps> [S<speed>] [F<accel>]");
    puts("LASER 1|2 ON|OFF|0..100%|0..255");
    puts("FAN 0|1|AUTO");
    puts("TEMP");
    puts("DRIVER STATUS");
    puts("DRIVER INVERT C|A|Z ON|OFF");
    puts("DRIVER MODE STEPDIR|UART");
    puts("DRIVER UART ADDR C|A|Z <0..3>");
    puts("DRIVER UART CURRENT C|A|Z <ihold_mA> <irun_mA> <delay>");
    puts("DRIVER UART SPREADCYCLE C|A|Z ON|OFF");
    puts("DRIVER UART MICROSTEPS C|A|Z <1..256>");
    puts("DRIVER REG READ C|A|Z <reg>");
    puts("DRIVER REG WRITE C|A|Z <reg> <value>");
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
    float encoder_c = 0.0f;
    float encoder_a = 0.0f;
    esp_err_t err_c = hardware_read_axis_encoder('C', &encoder_c);
    esp_err_t err_a = hardware_read_axis_encoder('A', &encoder_a);

    printf("\n=== STATUS ===\n");
    if (err_c == ESP_OK) {
        printf("Eixo C (Base): %.2f deg\n", encoder_c);
    } else {
        printf("Eixo C (Base): erro de leitura (%s)\n", esp_err_to_name(err_c));
    }
    if (err_a == ESP_OK) {
        printf("Eixo A (Pivot): %.2f deg\n", encoder_a);
    } else {
        printf("Eixo A (Pivot): erro de leitura (%s)\n", esp_err_to_name(err_a));
    }

    uint16_t z_teeth = ctx->settings.z_pulley_teeth ? ctx->settings.z_pulley_teeth : DEFAULT_Z_PULLEY_TEETH;
    uint16_t msteps_z = ctx->settings.tmc_microsteps[AXIS_Z_ID] ? ctx->settings.tmc_microsteps[AXIS_Z_ID] : 16;
    float spr_z = (float)ctx->settings.steps_per_rev[AXIS_Z_ID] > 0.0f ? (float)ctx->settings.steps_per_rev[AXIS_Z_ID] : 200.0f;
    float mm_per_step_z = (float)(z_teeth * Z_BELT_PITCH_MM) / (spr_z * (float)msteps_z);
    float pos_z_mm = (float)ctx->state.atual_z * mm_per_step_z;
    float max_z_mm = (float)ctx->settings.max_passos_z * mm_per_step_z;

    printf("Eixo Z: %ld / %ld passos (%.2f / %.2f mm | Polia: %uT GT2)\n",
           (long)ctx->state.atual_z, (long)ctx->settings.max_passos_z, pos_z_mm, max_z_mm, (unsigned)z_teeth);
    printf("Alarme Z: %s\n", ctx->state.alarme_z_ativo ? "ON" : "OFF");
    printf("Estado Z: %s\n", ctx->state.z_bloqueado ? "BLOQUEADO" : "LIVRE");
    printf("Drivers: %s\n", ctx->state.drivers_enabled ? "ENERGIZADOS" : "DESLIGADOS");
    printf("Laser 1: %" PRIu32 "%% (%u/255)\n",
           laser_level_to_percent(ctx->state.laser_level[0]),
           ctx->state.laser_level[0]);
    printf("Laser 2: %" PRIu32 "%% (%u/255)\n",
           laser_level_to_percent(ctx->state.laser_level[1]),
           ctx->state.laser_level[1]);

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

void commands_print_config(const app_context_t *ctx)
{
    if (ctx == NULL) return;
    printf("\n=== CONFIG DUMP ===\n");
    printf("CONFIG STEPS C=%lu A=%lu Z=%lu\n",
           (unsigned long)ctx->settings.steps_per_rev[0],
           (unsigned long)ctx->settings.steps_per_rev[1],
           (unsigned long)ctx->settings.steps_per_rev[2]);
    printf("CONFIG SPEED_MAX C=%.2f A=%.2f Z=%.2f\n",
           ctx->settings.speed_max[0], ctx->settings.speed_max[1], ctx->settings.speed_max[2]);
    printf("CONFIG ACCEL_MAX C=%.2f A=%.2f Z=%.2f\n",
           ctx->settings.accel_max[0], ctx->settings.accel_max[1], ctx->settings.accel_max[2]);
    printf("CONFIG ACCEL C=%.2f A=%.2f Z=%.2f\n",
           ctx->settings.accel[0], ctx->settings.accel[1], ctx->settings.accel[2]);
    printf("CONFIG INVERT C=%d A=%d Z=%d\n",
           ctx->state.inverter[0] ? 1 : 0,
           ctx->state.inverter[1] ? 1 : 0,
           ctx->state.inverter[2] ? 1 : 0);
    printf("CONFIG PULLEY_Z=%u MAX_PASSOS_Z=%lu\n",
           (unsigned)ctx->settings.z_pulley_teeth,
           (unsigned long)ctx->settings.max_passos_z);
    printf("CONFIG HOME_DEG C=%.2f A=%.2f\n",
           ctx->settings.home_c_deg, ctx->settings.home_a_deg);
    printf("CONFIG DRIVER_BUS_MODE=%u\n",
           (unsigned)ctx->settings.driver_bus_mode);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        char ax = (i == 0) ? 'C' : ((i == 1) ? 'A' : 'Z');
        printf("CONFIG TMC %c addr=%u ihold=%u irun=%u delay=%u usteps=%u spread=0\n",
               ax,
               (unsigned)ctx->settings.tmc_slave_addr[i],
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[i]),
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[i]),
               (unsigned)ctx->settings.tmc_ihold_delay[i],
               (unsigned)ctx->settings.tmc_microsteps[i]);
    }
    printf("CONFIG CAN node=%u bitrate=%lu cmd=0x%03lX status=0x%03lX event=0x%03lX\n",
           (unsigned)ctx->settings.node_id,
           (unsigned long)ctx->settings.can_bitrate,
           (unsigned long)ctx->settings.can_command_base_id,
           (unsigned long)ctx->settings.can_status_base_id,
           (unsigned long)ctx->settings.can_event_base_id);
    printf("===================\n");
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

    if (strcmp(cmd, "CONFIG DUMP") == 0 || strcmp(cmd, "CONFIG") == 0 || strcmp(cmd, "NVS DUMP") == 0) {
        commands_print_config(ctx);
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
        char raw_axis = cmd[14];
        char state_str[8] = {0};
        if (sscanf(cmd + 14, "%c %7s", &raw_axis, state_str) != 2) {
            puts("Uso: DRIVER INVERT C|A|Z ON|OFF");
            return;
        }
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        bool enable = (strcmp(state_str, "ON") == 0);
        ctx->state.inverter[axis_index] = enable;
        printf("Inversao %c %s.\n", axis, enable ? "ATIVADA" : "DESATIVADA");
        return;
    }

    if (strcmp(cmd, "SETHOME C") == 0 || strcmp(cmd, "SETHOME A") == 0 ||
        strcmp(cmd, "SETHOME X") == 0 || strcmp(cmd, "SETHOME Y") == 0) {
        char raw_axis = cmd[8];
        size_t axis_index = 0;
        char axis = '\0';
        parse_axis_token(raw_axis, &axis_index, &axis);

        float current_deg = 0.0f;
        esp_err_t err = hardware_read_axis_encoder(axis, &current_deg);
        if (err != ESP_OK) {
            printf("ERRO: nao foi possivel ler encoder %c (%s)\n", axis, esp_err_to_name(err));
            return;
        }

        if (axis == 'C') {
            ctx->settings.home_c_deg = current_deg;
        } else {
            ctx->settings.home_a_deg = current_deg;
        }

        err = storage_save_settings(&ctx->settings);
        if (err == ESP_OK) {
            printf("Home %c gravado em %.2f deg.\n", axis, current_deg);
        } else {
            printf("ERRO ao salvar home %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "HOME C") == 0 || strcmp(cmd, "HOME A") == 0 ||
        strcmp(cmd, "HOME X") == 0 || strcmp(cmd, "HOME Y") == 0) {
        char raw_axis = cmd[5];
        char axis = '\0';
        parse_axis_token(raw_axis, NULL, &axis);
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

    long teeth_z = 0;
    if (sscanf(cmd, "PULLEY Z %ld", &teeth_z) == 1 || sscanf(cmd, "SET_PULLEY Z %ld", &teeth_z) == 1 || sscanf(cmd, "PULLEY_TEETH Z %ld", &teeth_z) == 1) {
        if (teeth_z < 6 || teeth_z > 200) {
            puts("Numero de dentes da polia Z invalido (use entre 6 e 200).");
            return;
        }
        ctx->settings.z_pulley_teeth = (uint16_t)teeth_z;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            float mm_rev = (float)teeth_z * Z_BELT_PITCH_MM;
            printf("Polia do motor Z configurada para %ld dentes GT2 (passo %.1fmm -> %.2f mm/volta).\n",
                   teeth_z, Z_BELT_PITCH_MM, mm_rev);
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

    char raw_axis = '\0';
    long steps_per_rev = 0;
    if (sscanf(cmd, "STEPS %c %ld", &raw_axis, &steps_per_rev) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
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
    if (sscanf(cmd, "SPEED %c %f", &raw_axis, &speed_val) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        if (axis == 'Z') {
            if (speed_val < SPEED_MIN_MM_S_Z || speed_val > SPEED_MAX_MM_S_Z) {
                printf("Velocidade Z invalida. Use mm/s entre %.2f e %.2f.\n", SPEED_MIN_MM_S_Z, SPEED_MAX_MM_S_Z);
                return;
            }
            if (speed_val > ctx->settings.speed_max[axis_index]) {
                ctx->settings.speed_max[axis_index] = speed_val;
                ctx->state.speed_max[axis_index] = speed_val;
            }
        } else {
            if (speed_val < SPEED_MIN_DEG_S_CA || speed_val > SPEED_MAX_DEG_S_CA) {
                printf("Velocidade %c invalida. Use deg/s entre %.2f e %.2f.\n", axis, SPEED_MIN_DEG_S_CA, SPEED_MAX_DEG_S_CA);
                return;
            }
            if (speed_val > ctx->settings.speed_max[axis_index]) {
                ctx->settings.speed_max[axis_index] = speed_val;
                ctx->state.speed_max[axis_index] = speed_val;
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
    if (sscanf(cmd, "ACCEL %c %f", &raw_axis, &accel_val) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        if (axis == 'Z') {
            if (accel_val < ACCEL_MIN_MM_S2_Z || accel_val > ACCEL_MAX_MM_S2_Z) {
                printf("Aceleracao Z invalida. Use mm/s^2 entre %.2f e %.2f.\n", ACCEL_MIN_MM_S2_Z, ACCEL_MAX_MM_S2_Z);
                return;
            }
            if (accel_val > ctx->settings.accel_max[axis_index]) {
                ctx->settings.accel_max[axis_index] = accel_val;
                ctx->state.accel_max[axis_index] = accel_val;
            }
        } else {
            if (accel_val < ACCEL_MIN_DEG_S2_CA || accel_val > ACCEL_MAX_DEG_S2_CA) {
                printf("Aceleracao %c invalida. Use deg/s^2 entre %.2f e %.2f.\n", axis, ACCEL_MIN_DEG_S2_CA, ACCEL_MAX_DEG_S2_CA);
                return;
            }
            if (accel_val > ctx->settings.accel_max[axis_index]) {
                ctx->settings.accel_max[axis_index] = accel_val;
                ctx->state.accel_max[axis_index] = accel_val;
            }
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

    float speed_max_val = 0.0f;
    if (sscanf(cmd, "SPEED_MAX %c %f", &raw_axis, &speed_max_val) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        if (axis == 'Z') {
            if (speed_max_val < SPEED_MIN_MM_S_Z || speed_max_val > SPEED_MAX_MM_S_Z) {
                printf("Velocidade maxima Z invalida. Use mm/s entre %.2f e %.2f.\n", SPEED_MIN_MM_S_Z, SPEED_MAX_MM_S_Z);
                return;
            }
        } else {
            if (speed_max_val < SPEED_MIN_DEG_S_CA || speed_max_val > SPEED_MAX_DEG_S_CA) {
                printf("Velocidade maxima %c invalida. Use deg/s entre %.2f e %.2f.\n", axis, SPEED_MIN_DEG_S_CA, SPEED_MAX_DEG_S_CA);
                return;
            }
        }
        ctx->settings.speed_max[axis_index] = speed_max_val;
        ctx->state.speed_max[axis_index] = speed_max_val;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            if (axis == 'Z') {
                printf("Velocidade maxima Z definida para %.2f mm/s.\n", speed_max_val);
            } else {
                printf("Velocidade maxima %c definida para %.2f deg/s.\n", axis, speed_max_val);
            }
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    float accel_max_val = 0.0f;
    if (sscanf(cmd, "ACCEL_MAX %c %f", &raw_axis, &accel_max_val) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        if (axis == 'Z') {
            if (accel_max_val < ACCEL_MIN_MM_S2_Z || accel_max_val > ACCEL_MAX_MM_S2_Z) {
                printf("Aceleracao maxima Z invalida. Use mm/s^2 entre %.2f e %.2f.\n", ACCEL_MIN_MM_S2_Z, ACCEL_MAX_MM_S2_Z);
                return;
            }
        } else {
            if (accel_max_val < ACCEL_MIN_DEG_S2_CA || accel_max_val > ACCEL_MAX_DEG_S2_CA) {
                printf("Aceleracao maxima %c invalida. Use deg/s^2 entre %.2f e %.2f.\n", axis, ACCEL_MIN_DEG_S2_CA, ACCEL_MAX_DEG_S2_CA);
                return;
            }
        }
        ctx->settings.accel_max[axis_index] = accel_max_val;
        ctx->state.accel_max[axis_index] = accel_max_val;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            if (axis == 'Z') {
                printf("Aceleracao maxima Z definida para %.2f mm/s^2.\n", accel_max_val);
            } else {
                printf("Aceleracao maxima %c definida para %.2f deg/s^2.\n", axis, accel_max_val);
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
        to_upper_ascii(laser_value);
        if (!parse_laser_level_token(laser_value, &level)) {
            puts("Valor de laser invalido. Use ON, OFF, 0..100% ou 0..255.");
            return;
        }

        esp_err_t err = hardware_set_laser_level(ctx, (size_t)(laser_number - 1), level);
        if (err == ESP_OK) {
            printf("Laser %d ajustado para %" PRIu32 "%% (%u/255).\n",
                   laser_number, laser_level_to_percent(level), level);
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

    char raw_driver_axis = '\0';
    unsigned addr = 0;
    if (sscanf(cmd, "DRIVER UART ADDR %c %u", &raw_driver_axis, &addr) == 2) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido para endereco TMC. Use C, A ou Z.");
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
    if (sscanf(cmd, "DRIVER UART CURRENT %c %u %u %u", &raw_driver_axis, &ihold_ma, &irun_ma, &ihold_delay) == 4) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido para corrente TMC. Use C, A ou Z.");
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
    if (sscanf(cmd, "DRIVER UART SPREADCYCLE %c %23s", &raw_driver_axis, mode_token) == 2) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido. Use C, A ou Z.");
            return;
        }
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
    if (sscanf(cmd, "DRIVER UART MICROSTEPS %c %u", &raw_driver_axis, &microsteps_val) == 2) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido. Use C, A ou Z.");
            return;
        }
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
    if (sscanf(cmd, "DRIVER REG READ %c %23s", &raw_driver_axis, reg_token) == 2) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido. Use C, A ou Z.");
            return;
        }
        uint32_t reg_addr = 0;
        uint32_t value = 0;
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

    if (sscanf(cmd, "DRIVER REG WRITE %c %23s %23s", &raw_driver_axis, reg_token, value_token) == 3) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido. Use C, A ou Z.");
            return;
        }
        uint32_t reg_addr = 0;
        uint32_t reg_value = 0;
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
    float move_speed_val = -1.0f;
    float move_accel_val = -1.0f;
    char suffix[64];
    suffix[0] = '\0';
    char raw_axis_move = '\0';

    if (sscanf(cmd, "MOVE %c %ld %63[^\n]", &raw_axis_move, &steps, suffix) >= 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis_move, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        // Parse optional S=<speed> F=<accel> params
        char *sp = suffix;
        while (sp && *sp) {
            while (*sp && isspace((unsigned char)*sp)) sp++;
            if (*sp == 'S') {
                sp++;
                if (*sp == '=') sp++;
                char *endptr = NULL;
                move_speed_val = strtof(sp, &endptr);
                if (endptr == sp) {
                    move_speed_val = -1.0f;
                }
                sp = endptr;
            } else if (*sp == 'F') {
                sp++;
                if (*sp == '=') sp++;
                char *endptr = NULL;
                move_accel_val = strtof(sp, &endptr);
                if (endptr == sp) {
                    move_accel_val = -1.0f;
                }
                sp = endptr;
            } else {
                break;
            }
        }
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

    if (sscanf(cmd, "MOVE_F %c %ld %63[^\n]", &raw_axis_move, &steps, suffix) >= 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis_move, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        ESP_LOGI(APP_TAG, "PARSER MOVE_F axis=%c raw_steps=%ld", axis, steps);

        // Parse optional S=<speed> F=<accel> params
        char *sp = suffix;
        while (sp && *sp) {
            while (*sp && isspace((unsigned char)*sp)) sp++;
            if (*sp == 'S') {
                sp++;
                if (*sp == '=') sp++;
                char *endptr = NULL;
                move_speed_val = strtof(sp, &endptr);
                if (endptr == sp) {
                    move_speed_val = -1.0f;
                }
                sp = endptr;
            } else if (*sp == 'F') {
                sp++;
                if (*sp == '=') sp++;
                char *endptr = NULL;
                move_accel_val = strtof(sp, &endptr);
                if (endptr == sp) {
                    move_accel_val = -1.0f;
                }
                sp = endptr;
            } else {
                break;
            }
        }

        esp_err_t err;
        if (move_speed_val > 0.0f || move_accel_val > 0.0f) {
            err = motion_post_move_axis_with_params(ctx, axis, (int32_t)steps, move_speed_val, move_accel_val, 0, 0);
        } else {
            err = motion_post_move_axis_force(ctx, axis, (int32_t)steps, 0, 0);
        }
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

static bool parse_laser_level_token(const char *text, uint8_t *level)
{
    if (text == NULL || level == NULL) {
        return false;
    }

    if (strcmp(text, "ON") == 0) {
        *level = 255U;
        return true;
    }

    if (strcmp(text, "OFF") == 0) {
        *level = 0U;
        return true;
    }

    size_t len = strlen(text);
    if (len > 1U && text[len - 1U] == '%') {
        char percent_text[16];
        if (len >= sizeof(percent_text)) {
            return false;
        }

        memcpy(percent_text, text, len - 1U);
        percent_text[len - 1U] = '\0';

        uint32_t percent = 0;
        if (!parse_u32_token(percent_text, &percent) || percent > 100U) {
            return false;
        }

        *level = percent_to_laser_level(percent);
        return true;
    }

    uint32_t numeric = 0;
    if (!parse_u32_token(text, &numeric) || numeric > 255U) {
        return false;
    }

    *level = (uint8_t)numeric;
    return true;
}

static uint8_t percent_to_laser_level(uint32_t percent)
{
    if (percent >= 100U) {
        return 255U;
    }
    return (uint8_t)((percent * 255U + 50U) / 100U);
}

static uint32_t laser_level_to_percent(uint8_t level)
{
    return ((uint32_t)level * 100U + 127U) / 255U;
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
