#include "commands.h"

#include <ctype.h>
#include <inttypes.h>
#include <math.h>
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
#include "status_led.h"
#include "ota_update.h"

static esp_err_t persist_settings(app_context_t *ctx);
static bool parse_u32_token(const char *text, uint32_t *value);
static bool parse_laser_level_token(const char *text, uint16_t *level);
static uint16_t percent_to_laser_level(uint32_t percent);
static uint32_t laser_level_to_percent(uint16_t level);
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
    puts("STOP | ESTOP (interrompe o movimento imediatamente e esvazia a fila; ESTOP tambem apaga os lasers)");
    puts("STATUS");
    puts("DIAG (diagnostico: fim de curso Z, linhas I2C e ima dos encoders, resposta dos TMC2209)");
    puts("SAVE (grava agora na NVS as alteracoes pendentes; normalmente automatico apos 300 ms)");
    puts("HELP");
    puts("DRIVER ENABLED ON / OFF");
    puts("ALARM ON / ALARM OFF");
    puts("SETHOME C / A (ou X / Y)");
    puts("HOME C / A / Z (ou X / Y / Z)");
    puts("LIMIT C|A <min_deg> <max_deg> (salva limites na NVS)");
    puts("LIMITS");
    puts("SET_LENGTH Z <steps>");
    puts("PULLEY Z <dentes> (ex: 16, 20)");
    puts("STEPS C|A|Z <steps_per_rev>");
    puts("SPEED C|A <deg/s> | Z <mm/s>");
    puts("MOTION ENGINE STREAM|LEGACY (motor de movimento; STREAM = S-curve no ISR + encadeamento)");
    puts("MOTION LOOKAHEAD ON|OFF (encadeia movimentos consecutivos da fila sem parar)");
    puts("MOTION JERK C|A <deg/s> | Z <mm/s> (salto maximo de velocidade numa juncao)");
    puts("VELOCIDADE <1..5> (nivel de velocidade de C/A em uso, nao salvo na NVS)");
    puts("ACCEL C|A <deg/s^2> | Z <mm/s^2>");
    puts("SPEED_MAX C|A|Z <value> | ACCEL_MAX C|A|Z <value>");
    puts("MOVE C|A|Z <target_steps> [S<speed>] [F<accel>] (alvo absoluto em passos)");
    puts("MOVE_F C|A|Z <target_steps> [S<speed>] [F<accel>] (alvo absoluto malha aberta)");
    puts("MOVE_SYNC C <steps_c> A <steps_a> Z <steps_z> [S<speed>] [F<accel>] (coordenadas absolutas)");
    puts("U / UNIFIED <c_deg> <a_deg> <z_mm> <laser1> <laser2> (comando unificado TouchDesigner)");
    puts("JOG C <graus> A <graus> Z <mm> (incrementa o alvo do jog continuo; limites e velocidade da NVS)");
    puts("LASER 1|2 ON|OFF|0..100%|0..4095");
    puts("FAN 0|1|AUTO");
    puts("TEMP");
    puts("DRIVER STATUS");
    puts("DRIVER INVERT C|A|Z ON|OFF");
    puts("DRIVER MODE STEPDIR|UART");
    puts("DRIVER UART ADDR C|A|Z <0..3>");
    puts("DRIVER UART CURRENT C|A|Z <ihold_mA> <irun_mA> <delay>");
    puts("DRIVER UART SPREADCYCLE C|A|Z ON|OFF");
    puts("DRIVER UART MICROSTEPS C|A|Z <1..256>");
    puts("DRIVER UART STEALTH_MAX C|A|Z <vel> (stealthChop ate essa velocidade, spreadCycle acima; 0 = sempre stealth)");
    puts("DRIVER REG READ C|A|Z <reg>");
    puts("DRIVER REG WRITE C|A|Z <reg> <value>");
    puts("DRIVER APPLY");
    puts("CAN STATUS");
    puts("CAN ON / CAN OFF / CAN APPLY");
    puts("CAN TEST (autoteste: controlador e transceiver; diz onde esta a falha)");
    puts("CAN NODE <1..10>");
    puts("CAN BITRATE <125000|250000|500000|1000000>");
    puts("CAN BASE CMD|STATUS|EVENT <id>");
    puts("CAN SEND STATUS");
    puts("LED [STATUS]");
    puts("LED AUTO (modo automatico de status do no)");
    puts("LED OFF | RED | GREEN | BLUE | YELLOW | CYAN | MAGENTA | WHITE");
    puts("LED RGB <r> <g> <b> (0..255)");
    puts("LED BRIGHTNESS <0..100>");
    puts("OTA [STATUS] (exibe particao ativa, rollback e proxima particao)");
    puts("OTA WAIT [tamanho] / OTA PREPARE (entra em modo seguro de espera por gravacao OTA)");
    puts("OTA ABORT / OTA CANCEL (cancela OTA e restaura operacao normal)");
    puts("OTA CONFIRM (valida firmware atual e cancela rollback)");
    puts("OTA ROLLBACK (forca rollback para versao anterior e reinicia)");
}

void commands_print_status(app_context_t *ctx)
{
    float encoder_c = 0.0f;
    float encoder_a = 0.0f;
    esp_err_t err_c = hardware_read_axis_encoder('C', &encoder_c);
    esp_err_t err_a = hardware_read_axis_encoder('A', &encoder_a);

    printf("\n=== STATUS ===\n");
    if (err_c == ESP_OK) {
        printf("Eixo C (Base): %.2f deg (Limites: [%.2f, %.2f] deg)\n", encoder_c, ctx->settings.limit_min_c_deg, ctx->settings.limit_max_c_deg);
    } else {
        printf("Eixo C (Base): erro de leitura (%s)\n", esp_err_to_name(err_c));
    }
    if (err_a == ESP_OK) {
        printf("Eixo A (Pivot): %.2f deg (Limites: [%.2f, %.2f] deg)\n", encoder_a, ctx->settings.limit_min_a_deg, ctx->settings.limit_max_a_deg);
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
    motion_print_pos_line(ctx);
    printf("Alarme Z: %s\n", ctx->state.alarme_z_ativo ? "ON" : "OFF");
    printf("Estado Z: %s\n", ctx->state.z_bloqueado ? "BLOQUEADO" : "LIVRE");
    printf("Drivers: %s\n", ctx->state.drivers_enabled ? "ENERGIZADOS" : "DESLIGADOS");
    printf("Laser 1: %" PRIu32 "%% (%u/4095)\n",
           laser_level_to_percent(ctx->state.laser_level[0]),
           (unsigned)ctx->state.laser_level[0]);
    printf("Laser 2: %" PRIu32 "%% (%u/4095)\n",
           laser_level_to_percent(ctx->state.laser_level[1]),
           (unsigned)ctx->state.laser_level[1]);

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
    char led_buf[128];
    status_led_get_current_status(led_buf, sizeof(led_buf));
    printf("LED RGB (GPIO %d): %s\n", BOARD_RGB_LED_PIN, led_buf);
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
    float spd_c = (ctx->settings.speed[0] > 0.0f) ? ctx->settings.speed[0] : motion_delay_us_to_speed((app_context_t *)ctx, 'C', ctx->settings.speed_delay_us[0]);
    float spd_a = (ctx->settings.speed[1] > 0.0f) ? ctx->settings.speed[1] : motion_delay_us_to_speed((app_context_t *)ctx, 'A', ctx->settings.speed_delay_us[1]);
    float spd_z = (ctx->settings.speed[2] > 0.0f) ? ctx->settings.speed[2] : motion_delay_us_to_speed((app_context_t *)ctx, 'Z', ctx->settings.speed_delay_us[2]);
    printf("CONFIG SPEED C=%.2f A=%.2f Z=%.2f\n", spd_c, spd_a, spd_z);
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
    float z_mm_rev = (float)ctx->settings.z_pulley_teeth * 2.0f;
    float z_spr = (float)ctx->settings.steps_per_rev[AXIS_Z_ID] * (float)ctx->settings.tmc_microsteps[AXIS_Z_ID];
    float z_max_mm = (z_spr > 0.0f) ? ((float)ctx->settings.max_passos_z * z_mm_rev / z_spr) : 0.0f;
    printf("CONFIG PULLEY_Z=%u MAX_PASSOS_Z=%lu MAX_Z_MM=%.2f\n",
           (unsigned)ctx->settings.z_pulley_teeth,
           (unsigned long)ctx->settings.max_passos_z,
           z_max_mm);
    printf("CONFIG RAMP_C=%.2f RAMP_A=%.2f RAMP_Z=%.2f\n",
           ctx->settings.c_start_speed_deg,
           ctx->settings.a_start_speed_deg,
           ctx->settings.z_start_speed_mm);
    printf("CONFIG HOME_DEG C=%.2f A=%.2f\n",
           ctx->settings.home_c_deg, ctx->settings.home_a_deg);
    printf("CONFIG LIMITS C=%.2f..%.2f A=%.2f..%.2f\n",
           ctx->settings.limit_min_c_deg, ctx->settings.limit_max_c_deg,
           ctx->settings.limit_min_a_deg, ctx->settings.limit_max_a_deg);
    printf("CONFIG DRIVER_BUS_MODE=%u\n",
           (unsigned)ctx->settings.driver_bus_mode);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        char ax = (i == 0) ? 'C' : ((i == 1) ? 'A' : 'Z');
        printf("CONFIG TMC %c addr=%u ihold=%u irun=%u delay=%u usteps=%u spread=%u\n",
               ax,
               (unsigned)ctx->settings.tmc_slave_addr[i],
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[i]),
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[i]),
               (unsigned)ctx->settings.tmc_ihold_delay[i],
               (unsigned)ctx->settings.tmc_microsteps[i],
               (unsigned)ctx->settings.tmc_spreadcycle[i]);
    }
    printf("CONFIG MOTION ENGINE=%s LOOKAHEAD=%u JERK C=%.2f A=%.2f Z=%.2f\n",
           ctx->ext.motion_engine == MOTION_ENGINE_STREAM ? "STREAM" : "LEGACY",
           (unsigned)ctx->ext.lookahead, ctx->ext.jerk[0], ctx->ext.jerk[1], ctx->ext.jerk[2]);
    printf("CONFIG STEALTH_MAX C=%.2f A=%.2f Z=%.2f\n",
           ctx->ext.stealth_max_speed[0], ctx->ext.stealth_max_speed[1], ctx->ext.stealth_max_speed[2]);
    printf("CONFIG CAN enabled=%u node=%u bitrate=%lu cmd=0x%03lX status=0x%03lX event=0x%03lX\n",
           (unsigned)ctx->settings.can_enabled,
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

    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "ESTOP") == 0 || strcmp(cmd, "E-STOP") == 0) {
        (void)motion_request_stop(ctx);
        if (strcmp(cmd, "STOP") != 0) {
            (void)hardware_set_laser_level(ctx, 0, 0);
            (void)hardware_set_laser_level(ctx, 1, 0);
            puts("ESTOP: movimento interrompido, fila esvaziada e lasers apagados.");
        } else {
            puts("STOP: movimento interrompido e fila esvaziada.");
        }
        return;
    }

    // JOG C <d_deg> A <d_deg> Z <d_mm>: incrementos do alvo do jog continuo (controle por
    // mouse). Silencioso de proposito: chega a ~50 linhas/s.
    if (strncmp(cmd, "JOG", 3) == 0 && (cmd[3] == ' ' || cmd[3] == '\0')) {
        float delta[AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
        const char *p = cmd + 3;
        bool ok = true;
        while (*p) {
            while (*p == ' ') p++;
            if (!*p) break;
            char ax = *p++;
            size_t idx = (ax == 'C' || ax == 'X') ? AXIS_C_ID : (ax == 'A' || ax == 'Y') ? AXIS_A_ID
                         : (ax == 'Z') ? AXIS_Z_ID : AXIS_COUNT;
            while (*p == ' ' || *p == '=') p++;
            char *end = NULL;
            float v = strtof(p, &end);
            if (idx == AXIS_COUNT || end == p || !isfinite(v)) {
                ok = false;
                break;
            }
            delta[idx] += v;
            p = end;
        }
        if (!ok) {
            puts("ERRO: use JOG C <graus> A <graus> Z <mm> (incrementos).");
            return;
        }
        esp_err_t err = motion_jog_add(ctx, delta[AXIS_C_ID], delta[AXIS_A_ID], delta[AXIS_Z_ID]);
        if (err != ESP_OK) {
            printf("ERRO no JOG: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "SAVE") == 0 || strcmp(cmd, "NVS SAVE") == 0) {
        esp_err_t err = storage_flush();
        if (err == ESP_OK) {
            puts("NVS: configuracao gravada e confirmada.");
        } else {
            printf("NVS ERRO: falha ao gravar (%s)\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "DIAG") == 0) {
        printf("\n=== DIAG ===\n");
        hardware_print_diag();
        // TMC2209: IOIN responde mesmo parado; versao 0x21 confirma o chip certo
        int tmc_online = 0;
        for (int i = 0; i < 3; ++i) {
            const char axis = "CAZ"[i];
            uint32_t ioin = 0;
            esp_err_t err = tmc2209_read_register(ctx, axis, 0x06, &ioin);
            if (err == ESP_OK) {
                ++tmc_online;
                printf("DIAG TMC %c: responde (versao 0x%02lX, ENN=%lu)\n", axis,
                       (unsigned long)(ioin >> 24), (unsigned long)(ioin & 0x01U));
            } else {
                printf("DIAG TMC %c: sem resposta (%s)\n", axis, esp_err_to_name(err));
            }
        }
        if (tmc_online == 0) {
            puts("DIAG -> nenhum TMC responde: verifique a alimentacao dos motores (VM) e o fio PDN/UART (GPIO8).");
        }
        printf("DIAG Drivers: %s | Alarme Z: %s | Z bloqueado: %s\n",
               ctx->state.drivers_enabled ? "ENERGIZADOS" : "DESLIGADOS",
               ctx->state.alarme_z_ativo ? "ON" : "OFF", ctx->state.z_bloqueado ? "SIM" : "NAO");
        printf("============\n");
        return;
    }

    if (strcmp(cmd, "STATUS") == 0) {
        commands_print_status(ctx);
        return;
    }

    if (strcmp(cmd, "LED") == 0 || strcmp(cmd, "LED STATUS") == 0) {
        char st_buf[128];
        status_led_get_current_status(st_buf, sizeof(st_buf));
        printf("LED STATUS: %s\n", st_buf);
        return;
    }

    if (strcmp(cmd, "LED AUTO") == 0) {
        status_led_set_mode(STATUS_LED_MODE_AUTO);
        puts("LED: Modo AUTO ativado (status do no).");
        return;
    }

    if (strcmp(cmd, "LED OFF") == 0) {
        status_led_set_color(0, 0, 0);
        puts("LED: Desligado.");
        return;
    }

    if (strcmp(cmd, "LED RED") == 0) {
        status_led_set_color(255, 0, 0);
        puts("LED: Vermelho manual.");
        return;
    }

    if (strcmp(cmd, "LED GREEN") == 0) {
        status_led_set_color(0, 255, 0);
        puts("LED: Verde manual.");
        return;
    }

    if (strcmp(cmd, "LED BLUE") == 0) {
        status_led_set_color(0, 0, 255);
        puts("LED: Azul manual.");
        return;
    }

    if (strcmp(cmd, "LED YELLOW") == 0) {
        status_led_set_color(255, 180, 0);
        puts("LED: Amarelo manual.");
        return;
    }

    if (strcmp(cmd, "LED CYAN") == 0) {
        status_led_set_color(0, 255, 255);
        puts("LED: Ciano manual.");
        return;
    }

    if (strcmp(cmd, "LED MAGENTA") == 0) {
        status_led_set_color(255, 0, 255);
        puts("LED: Magenta manual.");
        return;
    }

    if (strcmp(cmd, "LED WHITE") == 0) {
        status_led_set_color(255, 255, 255);
        puts("LED: Branco manual.");
        return;
    }

    if (strncmp(cmd, "LED RGB ", 8) == 0) {
        unsigned r = 0, g = 0, b = 0;
        if (sscanf(cmd + 8, "%u %u %u", &r, &g, &b) == 3) {
            status_led_set_color((uint8_t)(r > 255 ? 255 : r),
                                 (uint8_t)(g > 255 ? 255 : g),
                                 (uint8_t)(b > 255 ? 255 : b));
            printf("LED: Cor RGB (%u, %u, %u) definida.\n", r, g, b);
        } else {
            puts("Uso: LED RGB <0..255> <0..255> <0..255>");
        }
        return;
    }

    if (strncmp(cmd, "LED BRIGHTNESS ", 15) == 0 || strncmp(cmd, "LED BRIGHT ", 11) == 0) {
        const char *p = (strncmp(cmd, "LED BRIGHTNESS ", 15) == 0) ? (cmd + 15) : (cmd + 11);
        unsigned br = 0;
        if (sscanf(p, "%u", &br) == 1 && br <= 100) {
            status_led_set_brightness((uint8_t)br);
            printf("LED: Brilho ajustado para %u%%.\n", br);
        } else {
            puts("Uso: LED BRIGHTNESS <0..100>");
        }
        return;
    }

    if (strcmp(cmd, "OTA") == 0 || strcmp(cmd, "OTA STATUS") == 0) {
        ota_print_status();
        return;
    }

    if (strncmp(cmd, "OTA WAIT", 8) == 0 || strncmp(cmd, "OTA PREPARE", 11) == 0 || strncmp(cmd, "OTA START", 9) == 0) {
        // Tamanho opcional: com ele apaga so o necessario; sem ele, apaga setor a setor durante a gravacao
        unsigned long image_size = 0;
        const char *size_arg = strchr(cmd + 4, ' ');
        if (size_arg != NULL) {
            image_size = strtoul(size_arg, NULL, 10);
        }
        esp_err_t err = ota_prepare_for_update(ctx, (uint32_t)image_size);
        if (err == ESP_OK) {
            puts("OTA: Modo de espera segura ativado. Motores e lasers parados. Aguardando dados de firmware...");
        } else {
            printf("OTA ERRO: Falha ao preparar para atualizacao: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "OTA ABORT") == 0 || strcmp(cmd, "OTA CANCEL") == 0) {
        ota_abort(ctx);
        puts("OTA: Sessao cancelada. Operacao normal restaurada.");
        return;
    }

    if (strcmp(cmd, "OTA CONFIRM") == 0 || strcmp(cmd, "OTA VALID") == 0) {
        esp_err_t err = ota_mark_valid();
        if (err == ESP_OK) {
            puts("OTA: Firmware validado com sucesso! Rollback cancelado.");
        } else {
            printf("OTA ERRO: Falha ao validar firmware: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "OTA ROLLBACK") == 0) {
        puts("OTA: Executando rollback para versao anterior...");
        (void)ota_rollback_and_reboot();
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

    if (strncmp(cmd, "DRIVER INVERT ", 14) == 0 || strncmp(cmd, "INVERT ", 7) == 0) {
        const char *p = (strncmp(cmd, "DRIVER INVERT ", 14) == 0) ? (cmd + 14) : (cmd + 7);
        char raw_axis = '\0';
        char state_str[8] = {0};
        if (sscanf(p, " %c %7s", &raw_axis, state_str) != 2) {
            puts("Uso: INVERT C|A|Z 0|1|ON|OFF");
            return;
        }
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(raw_axis, &axis_index, &axis)) {
            puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
            return;
        }
        bool enable = (strcmp(state_str, "ON") == 0 || strcmp(state_str, "1") == 0);
        ctx->state.inverter[axis_index] = enable;
        ctx->settings.inverter[axis_index] = enable;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Inversao de direcao %c %s (salvo na NVS).\n", axis, enable ? "ATIVADA" : "DESATIVADA");
        } else {
            printf("Inversao %c %s, mas erro ao salvar NVS: %s\n", axis, enable ? "ATIVADA" : "DESATIVADA", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "SETHOME C") == 0 || strcmp(cmd, "SETHOME A") == 0 ||
        strcmp(cmd, "SETHOME X") == 0 || strcmp(cmd, "SETHOME Y") == 0) {
        char raw_axis = cmd[8];
        size_t axis_index = 0;
        char axis = '\0';
        parse_axis_token(raw_axis, &axis_index, &axis);

        esp_err_t err = hardware_encoder_set_zero(ctx, axis);
        if (err == ESP_OK) {
            printf("Home %c gravado em 0.00 deg (posicao atual zerada, voltas resetadas na NVS).\n", axis);
            float live_deg = 0.0f;
            if (hardware_read_axis_encoder(axis, &live_deg) == ESP_OK) {
                printf("Eixo %c (%s): %.2f deg\n", axis, (axis == 'C' || axis == 'X') ? "Base" : "Pivot", live_deg);
            }
        } else {
            printf("ERRO ao salvar home %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (strncmp(cmd, "LIMIT ", 6) == 0) {
        char raw_axis = '\0';
        float min_deg = 0.0f, max_deg = 0.0f;
        if (sscanf(cmd + 6, "%c %f %f", &raw_axis, &min_deg, &max_deg) == 3) {
            char axis = '\0';
            parse_axis_token(raw_axis, NULL, &axis);
            if (min_deg < -3600.0f || max_deg > 3600.0f || max_deg <= min_deg) {
                printf("ERRO: limites invalidos para %c (devem estar entre -3600.0 e +3600.0 e min < max).\n", axis);
                return;
            }
            if (axis == 'C') {
                ctx->settings.limit_min_c_deg = min_deg;
                ctx->settings.limit_max_c_deg = max_deg;
            } else if (axis == 'A') {
                ctx->settings.limit_min_a_deg = min_deg;
                ctx->settings.limit_max_a_deg = max_deg;
            } else {
                printf("ERRO: Eixo invalido (%c). Use LIMIT C <min> <max> ou LIMIT A <min> <max>.\n", raw_axis);
                return;
            }
            esp_err_t err = persist_settings(ctx);
            if (err == ESP_OK) {
                printf("LIMIT %c gravado: min=%.2f max=%.2f deg (salvo na NVS).\n", axis, min_deg, max_deg);
            } else {
                printf("ERRO ao salvar LIMIT %c: %s\n", axis, esp_err_to_name(err));
            }
            return;
        }
    }

    if (strcmp(cmd, "LIMITS") == 0) {
        printf("LIMITS C=[%.2f, %.2f] A=[%.2f, %.2f] deg\n",
               ctx->settings.limit_min_c_deg, ctx->settings.limit_max_c_deg,
               ctx->settings.limit_min_a_deg, ctx->settings.limit_max_a_deg);
        return;
    }

    if (strcmp(cmd, "HOME C") == 0 || strcmp(cmd, "HOME A") == 0 ||
        strcmp(cmd, "HOME X") == 0 || strcmp(cmd, "HOME Y") == 0) {
        char raw_axis = cmd[5];
        char axis = '\0';
        parse_axis_token(raw_axis, NULL, &axis);
        esp_err_t err = motion_post_home_axis(ctx, axis, 0, 0);
        if (err == ESP_OK) {
            printf("Home %c enfileirado.\n", axis);
        } else {
            printf("ERRO no home %c: %s\n", axis, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(cmd, "HOME Z") == 0) {
        esp_err_t err = motion_post_home_axis(ctx, 'Z', 0, 0);
        if (err == ESP_OK) {
            puts("Home Z enfileirado.");
        } else {
            printf("ERRO no HOME Z: %s\n", esp_err_to_name(err));
        }
        return;
    }

    float mm_z = 0.0f;
    if (sscanf(cmd, "SETLENGTH_MM Z %f", &mm_z) == 1 || sscanf(cmd, "SET_LENGTH_MM Z %f", &mm_z) == 1 ||
        sscanf(cmd, "LIMIT_MM Z %f", &mm_z) == 1) {
        if (mm_z <= 0.0f || mm_z > 500.0f) {
            puts("Valor invalido para limite Z em mm (deve ser entre 1.0 e 500.0 mm).");
            return;
        }
        float mm_per_rev = (float)ctx->settings.z_pulley_teeth * Z_BELT_PITCH_MM;
        float spr = (float)ctx->settings.steps_per_rev[AXIS_Z_ID] * (float)ctx->settings.tmc_microsteps[AXIS_Z_ID];
        int32_t calc_steps = (int32_t)(mm_z * (spr / mm_per_rev) + 0.5f);
        ctx->settings.max_passos_z = calc_steps;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Limite Z definido para %.2f mm (%ld passos).\n", mm_z, (long)calc_steps);
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    long steps_z = 0;
    if (sscanf(cmd, "SETLENGTH Z %ld", &steps_z) == 1 || sscanf(cmd, "SET_LENGTH Z %ld", &steps_z) == 1) {
        if (steps_z <= 0 || steps_z > 2000000) {
            puts("Valor invalido para limite Z (deve ser entre 1 e 2000000).");
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

    char ramp_axis = '\0';
    float ramp_speed = 0.0f;
    if (sscanf(cmd, "RAMP %c %f", &ramp_axis, &ramp_speed) == 2 ||
        sscanf(cmd, "SET_RAMP %c %f", &ramp_axis, &ramp_speed) == 2 ||
        sscanf(cmd, "SPEED_START %c %f", &ramp_axis, &ramp_speed) == 2) {
        size_t axis_index = 0;
        char canonical_axis = '\0';
        if (!parse_axis_token(ramp_axis, &axis_index, &canonical_axis)) {
            puts("Eixo invalido para rampa. Use C, A ou Z.");
            return;
        }
        if (canonical_axis == 'Z') {
            if (ramp_speed < 0.5f || ramp_speed > 150.0f) {
                puts("Velocidade inicial de rampa Z invalida (use entre 0.5 e 150.0 mm/s).");
                return;
            }
            ctx->settings.z_start_speed_mm = ramp_speed;
        } else if (canonical_axis == 'C') {
            if (ramp_speed < 0.5f || ramp_speed > 100.0f) {
                puts("Velocidade inicial de rampa C invalida (use entre 0.5 e 100.0 deg/s).");
                return;
            }
            ctx->settings.c_start_speed_deg = ramp_speed;
        } else if (canonical_axis == 'A') {
            if (ramp_speed < 0.5f || ramp_speed > 100.0f) {
                puts("Velocidade inicial de rampa A invalida (use entre 0.5 e 100.0 deg/s).");
                return;
            }
            ctx->settings.a_start_speed_deg = ramp_speed;
        }
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Rampa inicial %c definida para %.2f %s.\n",
                   canonical_axis, ramp_speed, (canonical_axis == 'Z') ? "mm/s" : "deg/s");
        } else {
            printf("Erro ao salvar: %s\n", esp_err_to_name(err));
        }
        return;
    }

    float z_ramp_speed = 0.0f;
    if (sscanf(cmd, "RAMP_Z %f", &z_ramp_speed) == 1) {
        if (z_ramp_speed < 0.5f || z_ramp_speed > 150.0f) {
            puts("Velocidade inicial de rampa Z invalida (use entre 0.5 e 150.0 mm/s).");
            return;
        }
        ctx->settings.z_start_speed_mm = z_ramp_speed;
        esp_err_t err = persist_settings(ctx);
        if (err == ESP_OK) {
            printf("Rampa inicial Z definida para %.2f mm/s.\n", z_ramp_speed);
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
        ctx->settings.speed[axis_index] = speed_val;
        ctx->settings.speed_delay_us[axis_index] = delay_us;
        ctx->state.speed_delay_us[axis_index] = delay_us;
        ctx->state.speed[axis_index] = speed_val;
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

    char motion_arg[16] = {0};
    if (sscanf(cmd, "MOTION ENGINE %15s", motion_arg) == 1) {
        if (strcmp(motion_arg, "STREAM") == 0) {
            ctx->ext.motion_engine = MOTION_ENGINE_STREAM;
        } else if (strcmp(motion_arg, "LEGACY") == 0) {
            ctx->ext.motion_engine = MOTION_ENGINE_LEGACY;
        } else {
            puts("Use MOTION ENGINE STREAM ou LEGACY.");
            return;
        }
        storage_request_save_ext(&ctx->ext);
        printf("Motor de movimento: %s (salvo na NVS).\n", motion_arg);
        return;
    }
    if (sscanf(cmd, "MOTION LOOKAHEAD %15s", motion_arg) == 1) {
        bool on = (strcmp(motion_arg, "ON") == 0 || strcmp(motion_arg, "1") == 0);
        ctx->ext.lookahead = on ? 1U : 0U;
        storage_request_save_ext(&ctx->ext);
        printf("Lookahead %s (salvo na NVS).\n", on ? "ATIVADO" : "DESATIVADO");
        return;
    }
    char jerk_axis = '\0';
    float jerk_val = 0.0f;
    if (sscanf(cmd, "MOTION JERK %c %f", &jerk_axis, &jerk_val) == 2) {
        size_t axis_index = 0;
        char axis = '\0';
        if (!parse_axis_token(jerk_axis, &axis_index, &axis) || !(jerk_val >= 0.0f) || jerk_val > 1000.0f) {
            puts("Uso: MOTION JERK C|A|Z <0..1000>");
            return;
        }
        ctx->ext.jerk[axis_index] = jerk_val;
        storage_request_save_ext(&ctx->ext);
        printf("Jerk %c = %.2f %s (salvo na NVS).\n", axis, jerk_val, axis == 'Z' ? "mm/s" : "deg/s");
        return;
    }

    unsigned speed_level = 0;
    if (sscanf(cmd, "VELOCIDADE %u", &speed_level) == 1 || sscanf(cmd, "SPEED_LEVEL %u", &speed_level) == 1) {
        esp_err_t err = motion_apply_speed_level(ctx, (uint8_t)(speed_level > 255U ? 0U : speed_level));
        if (err == ESP_OK) {
            printf("Nivel de velocidade %u aplicado: C=%.1f A=%.1f deg/s.\n", speed_level,
                   ctx->state.speed[AXIS_C_ID], ctx->state.speed[AXIS_A_ID]);
        } else {
            puts("Nivel de velocidade invalido. Use 1..5.");
        }
        return;
    }

    int laser_number = 0;
    char laser_value[16] = {0};
    if (sscanf(cmd, "LASER %d %15s", &laser_number, laser_value) == 2) {
        if (ctx->state.ota_in_progress) {
            puts("ERRO: sessao OTA em andamento; lasers bloqueados.");
            return;
        }
        if (laser_number < 1 || laser_number > 2) {
            puts("Laser invalido. Use 1 ou 2.");
            return;
        }

        uint16_t level = 0;
        to_upper_ascii(laser_value);
        if (!parse_laser_level_token(laser_value, &level)) {
            puts("Valor de laser invalido. Use ON, OFF, 0..100% ou 0..4095.");
            return;
        }

        esp_err_t err = hardware_set_laser_level(ctx, (size_t)(laser_number - 1), level);
        if (err == ESP_OK) {
            printf("Laser %d ajustado para %" PRIu32 "%% (%u/4095).\n",
                   laser_number, laser_level_to_percent(level), (unsigned)level);
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

        (void)persist_settings(ctx);
        // Aplica ja no driver (confirmado por IFCNT); antes so valia apos DRIVER APPLY/reboot.
        // A corrente e quantizada em degraus de ~60 mA: informa o valor efetivo.
        esp_err_t apply_err = tmc2209_apply_current(ctx, driver_axis);
        printf("Corrente TMC do eixo %c salva: irun=%umA ihold=%umA delay=%u (%s).\n", driver_axis,
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_irun[axis_index]),
               (unsigned)tmc2209_cs_to_ma(ctx->settings.tmc_ihold[axis_index]),
               ihold_delay,
               (apply_err == ESP_OK) ? "aplicada no driver" :
               (apply_err == ESP_ERR_INVALID_STATE) ? "driver offline, aplicada quando responder" :
               "FALHA ao confirmar no driver");
        return;
    }

    float stealth_speed = 0.0f;
    if (sscanf(cmd, "DRIVER UART STEALTH_MAX %c %f", &raw_driver_axis, &stealth_speed) == 2 ||
        sscanf(cmd, "STEALTH_MAX %c %f", &raw_driver_axis, &stealth_speed) == 2) {
        size_t axis_index = 0;
        char driver_axis = '\0';
        if (!parse_axis_token(raw_driver_axis, &axis_index, &driver_axis)) {
            puts("Eixo invalido. Use C, A ou Z.");
            return;
        }
        esp_err_t err = tmc2209_set_stealth_max_speed(ctx, driver_axis, stealth_speed);
        if (err == ESP_OK) {
            printf("StealthChop do eixo %c ate %.1f %s (acima: spreadCycle).\n", driver_axis, stealth_speed,
                   driver_axis == 'Z' ? "mm/s" : "deg/s");
        } else {
            printf("ERRO ao ajustar limiar stealthChop %c: %s\n", driver_axis, esp_err_to_name(err));
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

    if (strcmp(cmd, "CAN TEST") == 0) {
        (void)can_bus_self_test(ctx);
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
        if (node_id < CAN_NODE_ID_MIN || node_id > CAN_NODE_ID_MAX) {
            puts("Node ID invalido. Use 1..10.");
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

    bool is_move_cmd = false;
    bool is_force_move = false;
    const char *p_move_args = NULL;

    if (strncmp(cmd, "MOVE_F ", 7) == 0) {
        is_move_cmd = true;
        is_force_move = true;
        p_move_args = cmd + 7;
    } else if (strncmp(cmd, "MSMF ", 5) == 0) {
        is_move_cmd = true;
        is_force_move = true;
        p_move_args = cmd + 5;
    } else if (strncmp(cmd, "MF ", 3) == 0) {
        is_move_cmd = true;
        is_force_move = true;
        p_move_args = cmd + 3;
    } else if (strncmp(cmd, "MOVE ", 5) == 0) {
        is_move_cmd = true;
        is_force_move = false;
        p_move_args = cmd + 5;
    } else if (strncmp(cmd, "MSM ", 4) == 0) {
        is_move_cmd = true;
        is_force_move = false;
        p_move_args = cmd + 4;
    } else if (strncmp(cmd, "M ", 2) == 0) {
        is_move_cmd = true;
        is_force_move = false;
        p_move_args = cmd + 2;
    }

    if (is_move_cmd && p_move_args != NULL) {
        while (*p_move_args && isspace((unsigned char)*p_move_args)) p_move_args++;

        unsigned target_node = 0;
        char raw_axis_move = '\0';
        long steps = 0;
        char suffix[64] = {0};
        bool matched = false;

        // Tenta primeiro formato com node ID: "<node> <axis> <steps> [suffix]"
        if (sscanf(p_move_args, "%u %c %ld %63[^\n]", &target_node, &raw_axis_move, &steps, suffix) >= 3) {
            // Se target_node for 0 (broadcast) ou igual ao nosso node_id, executamos
            if (target_node == 0 || target_node == (unsigned)ctx->settings.node_id) {
                matched = true;
            } else {
                // Comando direcionado a outro no especifico; em conexao serial direta com este no, ignora
                return;
            }
        } else if (sscanf(p_move_args, "%c %ld %63[^\n]", &raw_axis_move, &steps, suffix) >= 2) {
            matched = true;
        }

        if (matched) {
            size_t axis_index = 0;
            char axis = '\0';
            if (!parse_axis_token(raw_axis_move, &axis_index, &axis)) {
                puts("Eixo invalido. Use C, A ou Z (ou X, Y).");
                return;
            }

            // Parse optional S=<speed> F=<accel> params
            float move_speed_val = -1.0f;
            float move_accel_val = -1.0f;
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
            if (is_force_move) {
                ESP_LOGI(APP_TAG, "PARSER MOVE_F axis=%c raw_steps=%ld", axis, steps);
                if (move_speed_val > 0.0f || move_accel_val > 0.0f) {
                    err = motion_post_move_axis_with_params(ctx, axis, (int32_t)steps, move_speed_val, move_accel_val, 0, 0);
                } else {
                    err = motion_post_move_axis_force(ctx, axis, (int32_t)steps, 0, 0);
                }
            } else {
                if (move_speed_val > 0.0f || move_accel_val > 0.0f) {
                    err = motion_post_move_axis_profile(ctx, axis, (int32_t)steps,
                                                        move_speed_val, move_accel_val,
                                                        false, 0, 0);
                } else {
                    err = motion_post_move_axis(ctx, axis, (int32_t)steps, 0, 0);
                }
            }

            if (axis == 'Z' && err == ESP_ERR_INVALID_STATE) {
                puts("AVISO: Eixo Z bloqueado por seguranca. Use 'ALARM OFF' ou 'HOME Z'.");
                return;
            }

            if (err == ESP_OK) {
                printf("%s %c enfileirado%s.\n", is_force_move ? "MOVE_F" : "MOVE", axis, is_force_move ? " (sem encoder)" : "");
            } else {
                printf("ERRO no %s %c: %s\n", is_force_move ? "MOVE_F" : "MOVE", axis, esp_err_to_name(err));
            }
            return;
        }
    }

    if (strncmp(cmd, "MOVE_SYNC", 9) == 0) {
        bool force = false;
        const char *p = cmd + 9;
        if (*p == '_') {
            if (*(p + 1) == 'F' || *(p + 1) == 'f') {
                force = true;
                p += 2;
            }
        }
        long steps_c_sync = 0;
        long steps_a_sync = 0;
        long steps_z_sync = 0;
        float sync_speed_c = -1.0f;
        float sync_speed_a = -1.0f;
        float sync_speed_z = -1.0f;
        float sync_speed_all = -1.0f;
        float sync_accel_val = -1.0f;

        int positional_idx = 0;
        while (*p) {
            while (*p && isspace((unsigned char)*p)) p++;
            if (*p == '\0') break;

            if ((*p == 'S' || *p == 's') && (*(p + 1) == 'C' || *(p + 1) == 'c')) {
                p += 2;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                sync_speed_c = strtof(p, &endp);
                p = endp;
            } else if ((*p == 'S' || *p == 's') && (*(p + 1) == 'A' || *(p + 1) == 'a')) {
                p += 2;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                sync_speed_a = strtof(p, &endp);
                p = endp;
            } else if ((*p == 'S' || *p == 's') && (*(p + 1) == 'Z' || *(p + 1) == 'z')) {
                p += 2;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                sync_speed_z = strtof(p, &endp);
                p = endp;
            } else if (*p == 'C' || *p == 'c' || *p == 'X' || *p == 'x') {
                p++;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                steps_c_sync = strtol(p, &endp, 10);
                p = endp;
            } else if (*p == 'A' || *p == 'a' || *p == 'Y' || *p == 'y') {
                p++;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                steps_a_sync = strtol(p, &endp, 10);
                p = endp;
            } else if (*p == 'Z' || *p == 'z') {
                p++;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                steps_z_sync = strtol(p, &endp, 10);
                p = endp;
            } else if (*p == 'S' || *p == 's') {
                p++;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                sync_speed_all = strtof(p, &endp);
                p = endp;
            } else if (*p == 'F' || *p == 'f') {
                p++;
                while (*p && (isspace((unsigned char)*p) || *p == '=')) p++;
                char *endp = NULL;
                sync_accel_val = strtof(p, &endp);
                p = endp;
            } else if (isdigit((unsigned char)*p) || *p == '-') {
                char *endp = NULL;
                long val = strtol(p, &endp, 10);
                p = endp;
                if (positional_idx == 0) {
                    steps_c_sync = val;
                } else if (positional_idx == 1) {
                    steps_a_sync = val;
                } else if (positional_idx == 2) {
                    steps_z_sync = val;
                }
                positional_idx++;
            } else {
                p++;
            }
        }

        if (sync_speed_all > 0.0f) {
            if (sync_speed_c <= 0.0f) sync_speed_c = sync_speed_all;
            if (sync_speed_a <= 0.0f) sync_speed_a = sync_speed_all;
            if (sync_speed_z <= 0.0f) sync_speed_z = sync_speed_all;
        }

        esp_err_t err = motion_post_move_sync(ctx, (int32_t)steps_c_sync, (int32_t)steps_a_sync, (int32_t)steps_z_sync,
                                              sync_speed_c, sync_speed_a, sync_speed_z, sync_accel_val, force, 0, 0);
        if (err == ESP_OK) {
            printf("MOVE_SYNC%s C=%ld A=%ld Z=%ld enfileirado.\n", force ? "_F" : "", steps_c_sync, steps_a_sync, steps_z_sync);
        } else {
            printf("ERRO no MOVE_SYNC: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (strncmp(cmd, "U ", 2) == 0 || strncmp(cmd, "UNIFIED ", 8) == 0) {
        const char *p = (cmd[0] == 'U' && cmd[1] == ' ') ? (cmd + 2) : (cmd + 8);
        float c_deg = 0.0f, a_deg = 0.0f, z_mm = 0.0f;
        unsigned laser1 = 0, laser2 = 0;
        unsigned node_dummy = 0;
        int parsed = sscanf(p, "%u %f %f %f %u %u", &node_dummy, &c_deg, &a_deg, &z_mm, &laser1, &laser2);
        if (parsed != 6) {
            parsed = sscanf(p, "%f %f %f %u %u", &c_deg, &a_deg, &z_mm, &laser1, &laser2);
        }
        if (parsed == 6 || parsed == 5) {
            uint16_t msteps_c = ctx->settings.tmc_microsteps[AXIS_C_ID] ? ctx->settings.tmc_microsteps[AXIS_C_ID] : 16;
            float spr_c = ctx->settings.steps_per_rev[AXIS_C_ID] ? (float)ctx->settings.steps_per_rev[AXIS_C_ID] : 200.0f;
            float deg_per_step_c = 360.0f / (spr_c * (float)msteps_c);

            uint16_t msteps_a = ctx->settings.tmc_microsteps[AXIS_A_ID] ? ctx->settings.tmc_microsteps[AXIS_A_ID] : 16;
            float spr_a = ctx->settings.steps_per_rev[AXIS_A_ID] ? (float)ctx->settings.steps_per_rev[AXIS_A_ID] : 200.0f;
            float deg_per_step_a = 360.0f / (spr_a * (float)msteps_a);

            uint16_t z_teeth = ctx->settings.z_pulley_teeth ? ctx->settings.z_pulley_teeth : DEFAULT_Z_PULLEY_TEETH;
            uint16_t msteps_z = ctx->settings.tmc_microsteps[AXIS_Z_ID] ? ctx->settings.tmc_microsteps[AXIS_Z_ID] : 16;
            float spr_z = ctx->settings.steps_per_rev[AXIS_Z_ID] ? (float)ctx->settings.steps_per_rev[AXIS_Z_ID] : 200.0f;
            float mm_per_step_z = (float)(z_teeth * Z_BELT_PITCH_MM) / (spr_z * (float)msteps_z);

            int32_t steps_c = (int32_t)lroundf(c_deg / deg_per_step_c);
            int32_t steps_a = (int32_t)lroundf(a_deg / deg_per_step_a);
            int32_t steps_z = (int32_t)lroundf(z_mm / mm_per_step_z);

            esp_err_t err = motion_post_move_sync(ctx, steps_c, steps_a, steps_z,
                                                  -1.0f, -1.0f, -1.0f, -1.0f, false, 0, 0);
            if (err == ESP_OK) {
                if (laser1 > 4095U) laser1 = 4095U;
                if (laser2 > 4095U) laser2 = 4095U;
                (void)hardware_set_laser_level(ctx, 0, (uint16_t)laser1);
                (void)hardware_set_laser_level(ctx, 1, (uint16_t)laser2);
                printf("UNIFIED C=%.1f A=%.1f Z=%.2f mm L1=%u L2=%u enfileirado.\n",
                       c_deg, a_deg, z_mm, laser1, laser2);
            } else {
                printf("ERRO no comando UNIFIED: %s\n", esp_err_to_name(err));
            }
            return;
        } else {
            puts("Uso: U <c_deg> <a_deg> <z_mm> <laser1 0..4095> <laser2 0..4095>");
            return;
        }
    }

    puts("Comando desconhecido. Use HELP.");
}

static esp_err_t persist_settings(app_context_t *ctx)
{
    // Gravacao adiada e so se algo mudou: uma rajada de comandos vira uma escrita na flash.
    // Use SAVE para forcar e confirmar a gravacao.
    storage_request_save(&ctx->settings);
    return ESP_OK;
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

static bool parse_laser_level_token(const char *text, uint16_t *level)
{
    if (text == NULL || level == NULL) {
        return false;
    }

    if (strcmp(text, "ON") == 0) {
        *level = (uint16_t)LASER_PWM_MAX_LEVEL;
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
    if (!parse_u32_token(text, &numeric) || numeric > (uint32_t)LASER_PWM_MAX_LEVEL) {
        return false;
    }

    *level = (uint16_t)numeric;
    return true;
}

static uint16_t percent_to_laser_level(uint32_t percent)
{
    if (percent == 0U) {
        return 0U;
    }
    if (percent == 1U) {
        return (uint16_t)LASER_MIN_USEFUL_DUTY;
    }
    if (percent >= 100U) {
        return (uint16_t)LASER_MAX_USEFUL_DUTY;
    }
    return (uint16_t)(LASER_MIN_USEFUL_DUTY + ((percent - 1U) * (LASER_MAX_USEFUL_DUTY - LASER_MIN_USEFUL_DUTY) + 49U) / 99U);
}

static uint32_t laser_level_to_percent(uint16_t level)
{
    if (level == 0U) {
        return 0U;
    }
    if (level <= LASER_MIN_USEFUL_DUTY) {
        return 1U;
    }
    if (level >= LASER_MAX_USEFUL_DUTY) {
        return 100U;
    }
    return 1U + ((uint32_t)(level - LASER_MIN_USEFUL_DUTY) * 99U + (LASER_MAX_USEFUL_DUTY - LASER_MIN_USEFUL_DUTY) / 2U) / (LASER_MAX_USEFUL_DUTY - LASER_MIN_USEFUL_DUTY);
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
