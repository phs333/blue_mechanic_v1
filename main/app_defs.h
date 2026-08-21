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
#define Z_BELT_PULLEY_TEETH 16U
#define DEFAULT_Z_PULLEY_TEETH 16U

#define INVERTER_C false
#define INVERTER_A false
#define INVERTER_X INVERTER_C
#define INVERTER_Y INVERTER_A

#define ENCODER_ADDR 0x36
#define ENCODER_REG_RAW_ANGLE 0x0C
#define ENCODER_REG_ANGLE 0x0E

#define SETTINGS_NAMESPACE "cinetica"
#define SETTINGS_KEY "cfg"
#define SETTINGS_VERSION 7U

#define DEFAULT_HOME_X_DEG 0.0f
#define DEFAULT_HOME_Y_DEG 0.0f
#define DEFAULT_MAX_Z_STEPS 20000
#define DEFAULT_Z_PULLEY_TEETH 16U // Polia padrão GT2 16 dentes no motor Z

#define DEFAULT_NODE_ID 1
#define DEFAULT_CAN_ENABLED 1
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
#define DEFAULT_SPEED_MAX_MM_S_Z   60.0f
// Aceleração máxima em deg/s² para C/A, mm/s² para Z
#define DEFAULT_ACCEL_MAX_DEG_S2_CA 3600.0f
#define DEFAULT_ACCEL_MAX_DEG_S2_XY DEFAULT_ACCEL_MAX_DEG_S2_CA
#define DEFAULT_ACCEL_MAX_MM_S2_Z    800.0f

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
} runtime_state_t;

typedef enum {
    MOTION_CMD_MOVE_REL,
    MOTION_CMD_HOME,
    MOTION_CMD_MOVE_FORCE,
    MOTION_CMD_MOVE_SYNC,
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
    float speed_override;   // velocidade em deg/s (X/Y) ou mm/s (Z), -1.0 = usar padrão
    float accel_override;   // aceleração em deg/s² (X/Y) ou mm/s² (Z), -1.0 = usar padrão
} motion_cmd_t;

typedef struct {
    persisted_settings_t settings;
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
        .tmc_ihold = {5, 5, 6},             \
        .tmc_irun = {14, 14, 15},           \
        .tmc_ihold_delay = {6, 6, 6},       \
        .tmc_microsteps = {16, 16, 16},      \
        .tmc_spreadcycle = {0, 0, 0},        \
        .steps_per_rev = {200, 200, 200},    \
        .speed_delay_us = {400, 400, 400},   \
        .accel = {1800.0f, 1800.0f, 300.0f}, \
        .speed_max = {DEFAULT_SPEED_MAX_DEG_S_CA, DEFAULT_SPEED_MAX_DEG_S_CA, DEFAULT_SPEED_MAX_MM_S_Z}, \
        .accel_max = {DEFAULT_ACCEL_MAX_DEG_S2_CA, DEFAULT_ACCEL_MAX_DEG_S2_CA, DEFAULT_ACCEL_MAX_MM_S2_Z}, \
        .z_pulley_teeth = DEFAULT_Z_PULLEY_TEETH \
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
        .inverter = {false, false, false} \
    }
