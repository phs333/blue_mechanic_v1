#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#define APP_TAG "blue_mechanic"
#define AXIS_COUNT 3U

// =================================================================
// =================================================================
// --- MAPEAMENTO DE PINOS ---
// =================================================================
// Eixo C (Base Rotativa)
#define DIR_C GPIO_NUM_4
#define STEP_C GPIO_NUM_5
#define DIR_X DIR_C
#define STEP_X STEP_C

// Eixo A (Pivot com Lasers Colineares Opostos)
#define DIR_A GPIO_NUM_6
#define STEP_A GPIO_NUM_7
#define DIR_Y DIR_A
#define STEP_Y STEP_A

// Eixo Z (Atuador Linear)
#define DIR_Z GPIO_NUM_15
#define STEP_Z GPIO_NUM_16
#define EN_PIN GPIO_NUM_17
#define TEMP_PIN GPIO_NUM_18
#define FAN_PIN GPIO_NUM_14

#define SDA_0 GPIO_NUM_21
#define SCL_0 GPIO_NUM_20
#define SDA_1 GPIO_NUM_42
#define SCL_1 GPIO_NUM_41

#define SWITCH_Z GPIO_NUM_19

#define LASER_1_PIN GPIO_NUM_1
#define LASER_2_PIN GPIO_NUM_2

#define CAN_TX_PIN GPIO_NUM_39
#define CAN_RX_PIN GPIO_NUM_40

#define TMC_UART_TX_PIN GPIO_NUM_8
#define TMC_UART_RX_PIN GPIO_NUM_8

#define BOARD_RGB_LED_PIN GPIO_NUM_48
#define DEFAULT_STATUS_LED_BRIGHTNESS 20U // 20% de brilho padrao para WS2812 integrado

// =================================================================
// --- CONFIGURACAO ---
// =================================================================
#define BM_ENABLE_CAN 1
#define BM_ENABLE_TMC_UART 0

#define LASER_PWM_FREQ_HZ 5000
#define LASER_PWM_MAX_DUTY ((1U << 12) - 1U)
#define LASER_PWM_MAX_LEVEL 4095U
#define LASER_MIN_USEFUL_DUTY 46U
#define LASER_MAX_USEFUL_DUTY 300U

#define Z_DIR_UP 0
#define Z_DIR_DOWN 1

#define PASSOS_POR_VOLTA_MOTOR 3200.0f
#define GRAUS_POR_PASSO_CA (360.0f / PASSOS_POR_VOLTA_MOTOR)
#define GRAUS_POR_PASSO_XY GRAUS_POR_PASSO_CA

#define REDUCAO_C 1.0f
#define REDUCAO_A 1.0f
#define REDUCAO_X REDUCAO_C
#define REDUCAO_Y REDUCAO_A

#define LIMITE_GRAUS_CA 90.0f
#define LIMITE_GRAUS_XY LIMITE_GRAUS_CA
#define PASSOS_ALIVIO_EXTRA_Z 400

#define Z_BELT_PITCH_MM 2.0f
#define Z_BELT_PULLEY_TEETH 20U
#define DEFAULT_Z_PULLEY_TEETH 20U

#define INVERTER_C true
#define INVERTER_A true
#define INVERTER_X INVERTER_C
#define INVERTER_Y INVERTER_A

#define ENCODER_ADDR 0x36
#define ENCODER_REG_RAW_ANGLE 0x0C
#define ENCODER_REG_ANGLE 0x0E

#define SETTINGS_NAMESPACE "cinetica"
#define SETTINGS_KEY "cfg"
#define SETTINGS_VERSION 15U

#define DEFAULT_HOME_X_DEG 0.0f
#define DEFAULT_HOME_Y_DEG 0.0f
#define DEFAULT_HOME_C_DEG 0.0f
#define DEFAULT_HOME_A_DEG 0.0f
#define DEFAULT_LIMIT_MIN_C_DEG -540.0f
#define DEFAULT_LIMIT_MAX_C_DEG 540.0f
#define DEFAULT_LIMIT_MIN_A_DEG -540.0f
#define DEFAULT_LIMIT_MAX_A_DEG 540.0f
#define DEFAULT_MAX_Z_STEPS 38400
#define DEFAULT_Z_PULLEY_TEETH 20U // Polia padrão GT2 20 dentes no motor Z

#define DEFAULT_NODE_ID 1
#define DEFAULT_CAN_ENABLED 0
#define DEFAULT_CAN_COMMAND_BASE_ID 0x200
#define DEFAULT_CAN_STATUS_BASE_ID 0x280
#define DEFAULT_CAN_EVENT_BASE_ID 0x300
#define DEFAULT_CAN_BITRATE 500000

#define Z_RESCUE_TIMEOUT_STEPS 4000
#define Z_HOME_SEARCH_LIMIT_STEPS 30000
#define Z_HOME_RELEASE_LIMIT_STEPS 5000
#define Z_LENGTH_SEARCH_LIMIT_STEPS 40000

#define ENCODER_I2C_TIMEOUT_MS 20
#define DS18B20_CONVERSION_TIMEOUT_MS 800
#define FAN_AUTO_ON_TEMP_C 45.0f
#define FAN_AUTO_OFF_TEMP_C 40.0f

// --- Limites seguros para motor NEMA 17 17HS4401-22B com TMC2209 ---
// Velocidade máxima em deg/s para C/A (1..2 rev/s), mm/s para Z (GT2 16T = 32mm/rev)
#define DEFAULT_SPEED_MAX_DEG_S_CA 720.0f
#define DEFAULT_SPEED_MAX_DEG_S_XY DEFAULT_SPEED_MAX_DEG_S_CA
#define DEFAULT_SPEED_MAX_MM_S_Z   300.0f
// Aceleração máxima em deg/s² para C/A, mm/s² para Z
#define DEFAULT_ACCEL_MAX_DEG_S2_CA 3600.0f
#define DEFAULT_ACCEL_MAX_DEG_S2_XY DEFAULT_ACCEL_MAX_DEG_S2_CA
#define DEFAULT_ACCEL_MAX_MM_S2_Z    2000.0f

// --- Limites mínimos/máximos para validação ---
#define SPEED_MIN_DEG_S_CA 0.1f
#define SPEED_MIN_DEG_S_XY SPEED_MIN_DEG_S_CA
#define SPEED_MAX_DEG_S_CA 10000.0f
#define SPEED_MAX_DEG_S_XY SPEED_MAX_DEG_S_CA
#define SPEED_MIN_MM_S_Z   0.01f
#define SPEED_MAX_MM_S_Z   500.0f

#define ACCEL_MIN_DEG_S2_CA 1.0f
#define ACCEL_MIN_DEG_S2_XY ACCEL_MIN_DEG_S2_CA
#define ACCEL_MAX_DEG_S2_CA 50000.0f
#define ACCEL_MAX_DEG_S2_XY ACCEL_MAX_DEG_S2_CA
#define ACCEL_MIN_MM_S2_Z   0.1f
#define ACCEL_MAX_MM_S2_Z   5000.0f

typedef enum {
    FAN_MODE_MANUAL_OFF = 0,
    FAN_MODE_MANUAL_ON,
    FAN_MODE_AUTO,
} fan_mode_t;

typedef enum {
    AXIS_C_ID = 0,
    AXIS_A_ID = 1,
    AXIS_Z_ID = 2,
    AXIS_X_ID = 0,
    AXIS_Y_ID = 1,
} axis_id_t;

typedef enum {
    DRIVER_BUS_MODE_STEP_DIR_ONLY = 0,
    DRIVER_BUS_MODE_UART_OPTIONAL = 1,
} driver_bus_mode_t;

typedef struct {
    uint32_t version;
    union {
        struct {
            float home_c_deg;
            float home_a_deg;
        };
        struct {
            float home_x_deg;
            float home_y_deg;
        };
    };
    int32_t max_passos_z;
    uint8_t node_id;
    uint8_t can_enabled;
    uint16_t can_command_base_id;
    uint16_t can_status_base_id;
    uint16_t can_event_base_id;
    uint32_t can_bitrate;
    uint8_t driver_bus_mode;
    uint8_t tmc_slave_addr[AXIS_COUNT];
    uint8_t tmc_ihold[AXIS_COUNT];
    uint8_t tmc_irun[AXIS_COUNT];
    uint8_t tmc_ihold_delay[AXIS_COUNT];
    uint16_t tmc_microsteps[AXIS_COUNT];
    uint8_t tmc_spreadcycle[AXIS_COUNT];
    uint16_t steps_per_rev[AXIS_COUNT];
    uint32_t speed_delay_us[AXIS_COUNT];
    float accel[AXIS_COUNT];
    float speed_max[AXIS_COUNT];   // velocidade máxima: deg/s para X/Y, mm/s para Z
    float accel_max[AXIS_COUNT];   // aceleração máxima: deg/s² para X/Y, mm/s² para Z
    uint16_t z_pulley_teeth;       // número de dentes da polia GT2 no motor Z (ex: 16, 20)
    float c_start_speed_deg;       // velocidade inicial de partida da rampa S-Curve C (deg/s)
    float a_start_speed_deg;       // velocidade inicial de partida da rampa S-Curve A (deg/s)
    float z_start_speed_mm;        // velocidade inicial de partida da rampa S-Curve Z (mm/s)
    uint8_t inverter[AXIS_COUNT];  // inversão de direção persistida por eixo (C, A, Z)
    union {
        struct {
            float limit_min_c_deg;
            float limit_max_c_deg;
            float limit_min_a_deg;
            float limit_max_a_deg;
        };
        struct {
            float limit_min_x_deg;
            float limit_max_x_deg;
            float limit_min_y_deg;
            float limit_max_y_deg;
        };
    };
    float speed[AXIS_COUNT];       // velocidade nominal salva por eixo (deg/s para C/A, mm/s para Z)
    uint16_t home_raw[2];          // leitura bruta de encoder (0..4095) quando SETHOME foi gravado (0=C, 1=A)
} persisted_settings_t;

typedef struct {
    volatile bool alarme_z_ativo;
    volatile bool z_bloqueado;
    volatile bool em_homing_z;
    volatile bool drivers_enabled;
    volatile bool fan_output_on;
    volatile fan_mode_t fan_mode;
    volatile uint16_t laser_level[2];
    volatile uint32_t speed_delay_us[AXIS_COUNT];
    volatile float speed[AXIS_COUNT];       // velocidade nominal em uso (fonte unica para todos os tipos de MOVE)
    volatile float speed_max[AXIS_COUNT];   // velocidade máxima: deg/s para X/Y, mm/s para Z
    volatile float accel_max[AXIS_COUNT];   // aceleração máxima: deg/s² para X/Y, mm/s² para Z
    volatile int32_t atual_z;
    volatile float last_temp_c;
    volatile bool temp_valid;
    volatile driver_bus_mode_t driver_mode_requested;
    volatile driver_bus_mode_t driver_mode_active;
    volatile bool tmc_uart_ready;
    volatile bool tmc_axis_online[AXIS_COUNT];
    volatile bool can_online;
    volatile uint32_t can_rx_count;
    volatile uint32_t can_tx_count;
    volatile uint32_t can_last_error_flags;
    volatile uint32_t last_cmd_seq;
    volatile bool inverter[AXIS_COUNT];
    volatile bool homed[AXIS_COUNT]; // Indicador de home realizado com sucesso por eixo (C, A, Z)
    volatile bool in_motion;         // Indicador de movimento ativo
    volatile bool in_homing;         // Indicador de busca de home ativa
    volatile bool ota_in_progress;   // Modo de atualizacao OTA ativo (bloqueia movimentos e lasers)
} runtime_state_t;

typedef enum {
    MOTION_CMD_MOVE_REL,
    MOTION_CMD_HOME,
    MOTION_CMD_MOVE_FORCE,
    MOTION_CMD_MOVE_SYNC,
    MOTION_CMD_JOG,         // jog continuo: segue o alvo de motion_jog_add() ate ficar ocioso
} motion_cmd_type_t;

typedef struct {
    motion_cmd_type_t type;
    char axis;
    int32_t steps;
    int32_t steps_c;
    int32_t steps_a;
    int32_t steps_z;
    uint8_t sender_node_id;
    uint8_t opcode;
    float speed_override;   // velocidade geral em deg/s ou mm/s, -1.0 = usar padrão
    float accel_override;   // aceleração em deg/s² ou mm/s², -1.0 = usar padrão
    float speed_c;          // velocidade individual C (deg/s), -1.0 = usar padrão
    float speed_a;          // velocidade individual A (deg/s), -1.0 = usar padrão
    float speed_z;          // velocidade individual Z (mm/s), -1.0 = usar padrão
    bool force_no_encoder;  // true = ignora limites e encoder (sem correção)
    uint32_t stop_gen;      // geracao de STOP no enfileiramento; divergencia = comando abortado
} motion_cmd_t;

/*
 * Configuracoes estendidas: blob NVS proprio ("cfg_ext"), versionado pelo tamanho.
 * Campos novos entram SEMPRE no fim; um blob antigo (menor) carrega o prefixo e o
 * restante fica com o padrao — sem resetar persisted_settings_t (SETTINGS_VERSION).
 */
typedef struct {
    uint32_t size;                       // sizeof() gravado
    float stealth_max_speed[AXIS_COUNT]; // acima disso o TMC troca stealthChop->spreadCycle (TPWMTHRS); 0 = sempre stealthChop
    uint8_t motion_engine;               // MOTION_ENGINE_STREAM (padrao) ou MOTION_ENGINE_LEGACY
    uint8_t lookahead;                   // 1 = encadeia movimentos consecutivos sem parar entre eles
    float jerk[AXIS_COUNT];              // salto maximo de velocidade por eixo numa juncao (deg/s, deg/s, mm/s)
} ext_settings_t;

#define MOTION_ENGINE_LEGACY 0U
#define MOTION_ENGINE_STREAM 1U

#define APP_EXT_SETTINGS_DEFAULT_INIT                 \
    {                                                 \
        .size = sizeof(ext_settings_t),               \
        .stealth_max_speed = {180.0f, 180.0f, 60.0f}, \
        .motion_engine = MOTION_ENGINE_STREAM,        \
        .lookahead = 1U,                              \
        .jerk = {15.0f, 15.0f, 10.0f},                \
    }

typedef struct {
    persisted_settings_t settings;
    ext_settings_t ext;
    runtime_state_t state;
    SemaphoreHandle_t motion_mutex;
    SemaphoreHandle_t state_mutex;
    QueueHandle_t motion_queue;
} app_context_t;

#define APP_SETTINGS_DEFAULT_INIT            \
    {                                       \
        .version = SETTINGS_VERSION,        \
        .home_x_deg = DEFAULT_HOME_X_DEG,   \
        .home_y_deg = DEFAULT_HOME_Y_DEG,   \
        .max_passos_z = DEFAULT_MAX_Z_STEPS, \
        .node_id = DEFAULT_NODE_ID,         \
        .can_enabled = DEFAULT_CAN_ENABLED, \
        .can_command_base_id = DEFAULT_CAN_COMMAND_BASE_ID, \
        .can_status_base_id = DEFAULT_CAN_STATUS_BASE_ID, \
        .can_event_base_id = DEFAULT_CAN_EVENT_BASE_ID, \
        .can_bitrate = DEFAULT_CAN_BITRATE, \
        .driver_bus_mode = DRIVER_BUS_MODE_UART_OPTIONAL, \
        .tmc_slave_addr = {0, 1, 2},        \
        .tmc_ihold = {8, 8, 6},             \
        .tmc_irun = {14, 14, 15},           \
        .tmc_ihold_delay = {6, 6, 6},       \
        .tmc_microsteps = {16, 16, 16},      \
        .tmc_spreadcycle = {0, 0, 0},        \
        .steps_per_rev = {200, 200, 200},    \
        .speed_delay_us = {400, 400, 400},   \
        .accel = {1800.0f, 1800.0f, 1000.0f}, \
        .speed_max = {DEFAULT_SPEED_MAX_DEG_S_CA, DEFAULT_SPEED_MAX_DEG_S_CA, DEFAULT_SPEED_MAX_MM_S_Z}, \
        .accel_max = {DEFAULT_ACCEL_MAX_DEG_S2_CA, DEFAULT_ACCEL_MAX_DEG_S2_CA, DEFAULT_ACCEL_MAX_MM_S2_Z}, \
        .z_pulley_teeth = DEFAULT_Z_PULLEY_TEETH, \
        .c_start_speed_deg = 10.0f, \
        .a_start_speed_deg = 10.0f, \
        .z_start_speed_mm = 15.0f, \
        .inverter = {INVERTER_C, INVERTER_A, false}, \
        .limit_min_c_deg = DEFAULT_LIMIT_MIN_C_DEG, \
        .limit_max_c_deg = DEFAULT_LIMIT_MAX_C_DEG, \
        .limit_min_a_deg = DEFAULT_LIMIT_MIN_A_DEG, \
        .limit_max_a_deg = DEFAULT_LIMIT_MAX_A_DEG, \
        .speed = {140.0f, 140.0f, 250.0f}, \
        .home_raw = {0, 0} \
    }

#define APP_RUNTIME_DEFAULT_INIT          \
    {                                                       \
        .alarme_z_ativo = true,           \
        .z_bloqueado = false,             \
        .em_homing_z = false,             \
        .drivers_enabled = false,         \
        .fan_output_on = false,           \
        .fan_mode = FAN_MODE_MANUAL_OFF,  \
        .laser_level = {0, 0},            \
        .speed_delay_us = {400, 400, 400}, \
        .speed = {140.0f, 140.0f, 250.0f}, \
        .speed_max = {DEFAULT_SPEED_MAX_DEG_S_XY, DEFAULT_SPEED_MAX_DEG_S_XY, DEFAULT_SPEED_MAX_MM_S_Z}, \
        .accel_max = {DEFAULT_ACCEL_MAX_DEG_S2_XY, DEFAULT_ACCEL_MAX_DEG_S2_XY, DEFAULT_ACCEL_MAX_MM_S2_Z}, \
        .atual_z = 0,                     \
        .last_temp_c = 0.0f,              \
        .temp_valid = false,              \
        .driver_mode_requested = DRIVER_BUS_MODE_STEP_DIR_ONLY, \
        .driver_mode_active = DRIVER_BUS_MODE_STEP_DIR_ONLY, \
        .tmc_uart_ready = false,          \
        .tmc_axis_online = {false, false, false}, \
        .can_online = false,              \
        .can_rx_count = 0,                \
        .can_tx_count = 0,                \
        .can_last_error_flags = 0,        \
        .inverter = {false, false, false}, \
        .homed = {false, false, false},   \
        .in_motion = false,               \
        .in_homing = false,               \
        .ota_in_progress = false          \
    }
