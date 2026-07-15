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
// --- MAPEAMENTO DE PINOS ---
// =================================================================
#define DIR_X GPIO_NUM_4
#define STEP_X GPIO_NUM_5
#define DIR_Y GPIO_NUM_6
#define STEP_Y GPIO_NUM_7
#define DIR_Z GPIO_NUM_15
#define STEP_Z GPIO_NUM_16
#define EN_PIN GPIO_NUM_17
#define TEMP_PIN GPIO_NUM_18
#define FAN_PIN GPIO_NUM_1

#define SDA_0 GPIO_NUM_21
#define SCL_0 GPIO_NUM_20
#define SDA_1 GPIO_NUM_42
#define SCL_1 GPIO_NUM_41

#define SWITCH_Z GPIO_NUM_19

#define LASER_1_PIN GPIO_NUM_12
#define LASER_2_PIN GPIO_NUM_14

#define CAN_TX_PIN GPIO_NUM_48
#define CAN_RX_PIN GPIO_NUM_47

#define TMC_UART_TX_PIN GPIO_NUM_8
#define TMC_UART_RX_PIN GPIO_NUM_9

// =================================================================
// --- CONFIGURACAO ---
// =================================================================
#define BM_ENABLE_CAN 0
#define BM_ENABLE_TMC_UART 0

#define LASER_PWM_FREQ_HZ 5000
#define LASER_PWM_MAX_DUTY ((1U << 13) - 1U)

#define Z_DIR_UP 0
#define Z_DIR_DOWN 1

#define PASSOS_POR_VOLTA_MOTOR 3200.0f
#define GRAUS_POR_PASSO_XY (360.0f / PASSOS_POR_VOLTA_MOTOR)

#define REDUCAO_X 1.0f
#define REDUCAO_Y 1.0f

#define LIMITE_GRAUS_XY 90.0f
#define PASSOS_ALIVIO_EXTRA_Z 400

#define INVERTER_X false
#define INVERTER_Y false

#define ENCODER_ADDR 0x36
#define ENCODER_REG_ANGLE 0x0E

#define SETTINGS_NAMESPACE "cinetica"
#define SETTINGS_KEY "cfg"
#define SETTINGS_VERSION 3U

#define DEFAULT_HOME_X_DEG 0.0f
#define DEFAULT_HOME_Y_DEG 0.0f
#define DEFAULT_MAX_Z_STEPS 20000

#define DEFAULT_NODE_ID 1U
#define DEFAULT_CAN_ENABLED 0U
#define DEFAULT_CAN_COMMAND_BASE_ID 0x200U
#define DEFAULT_CAN_STATUS_BASE_ID 0x280U
#define DEFAULT_CAN_EVENT_BASE_ID 0x300U
#define DEFAULT_CAN_BITRATE 500000U

#define Z_RESCUE_TIMEOUT_STEPS 4000
#define Z_HOME_SEARCH_LIMIT_STEPS 30000
#define Z_HOME_RELEASE_LIMIT_STEPS 5000
#define Z_LENGTH_SEARCH_LIMIT_STEPS 40000

#define ENCODER_I2C_TIMEOUT_MS 20
#define DS18B20_CONVERSION_TIMEOUT_MS 800
#define FAN_AUTO_ON_TEMP_C 45.0f
#define FAN_AUTO_OFF_TEMP_C 40.0f

typedef enum {
    FAN_MODE_MANUAL_OFF = 0,
    FAN_MODE_MANUAL_ON,
    FAN_MODE_AUTO,
} fan_mode_t;

typedef enum {
    AXIS_X_ID = 0,
    AXIS_Y_ID = 1,
    AXIS_Z_ID = 2,
} axis_id_t;

typedef enum {
    DRIVER_BUS_MODE_STEP_DIR_ONLY = 0,
    DRIVER_BUS_MODE_UART_OPTIONAL = 1,
} driver_bus_mode_t;

typedef struct {
    uint32_t version;
    float home_x_deg;
    float home_y_deg;
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
} persisted_settings_t;

typedef struct {
    volatile bool alarme_z_ativo;
    volatile bool z_bloqueado;
    volatile bool em_homing_z;
    volatile bool drivers_enabled;
    volatile bool fan_output_on;
    volatile fan_mode_t fan_mode;
    volatile uint8_t laser_level[2];
    volatile uint32_t move_delay_us;
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
} runtime_state_t;

typedef enum {
    MOTION_CMD_MOVE_REL,
    MOTION_CMD_HOME,
    MOTION_CMD_RECOVER_Z,
    MOTION_CMD_MAP_Z_LENGTH
} motion_cmd_type_t;

typedef struct {
    motion_cmd_type_t type;
    char axis;
    int32_t steps;
    uint8_t sender_node_id;
    uint8_t opcode;
} motion_cmd_t;

typedef struct {
    persisted_settings_t settings;
    runtime_state_t state;
    SemaphoreHandle_t motion_mutex;
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
        .driver_bus_mode = DRIVER_BUS_MODE_STEP_DIR_ONLY, \
        .tmc_slave_addr = {0, 1, 2},        \
        .tmc_ihold = {8, 8, 8},             \
        .tmc_irun = {20, 20, 20},           \
        .tmc_ihold_delay = {6, 6, 6}        \
    }

#define APP_RUNTIME_DEFAULT_INIT          \
    {                                     \
        .alarme_z_ativo = true,           \
        .z_bloqueado = false,             \
        .em_homing_z = false,             \
        .drivers_enabled = false,         \
        .fan_output_on = false,           \
        .fan_mode = FAN_MODE_MANUAL_OFF,  \
        .laser_level = {0, 0},            \
        .move_delay_us = 400,             \
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
        .can_last_error_flags = 0         \
    }
