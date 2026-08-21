#pragma once

#include "esp_err.h"

#include "app_defs.h"

typedef enum {
    CAN_OP_PING = 0x01,
    CAN_OP_STATUS_REQUEST = 0x02,
    CAN_OP_ENABLE = 0x10,
    CAN_OP_SPEED = 0x11,
    CAN_OP_AXIS_SPEED = 0x12,
    CAN_OP_AXIS_ACCEL = 0x13,
    CAN_OP_MOVE_PROFILE = 0x14,
    CAN_OP_MOVE = 0x20,
    CAN_OP_HOME = 0x21,
    CAN_OP_MOVE_FORCE = 0x22,
    CAN_OP_LASER = 0x30,
    CAN_OP_FAN = 0x31,
} can_opcode_t;

typedef enum {
    CAN_EVT_HEARTBEAT = 0x80,
    CAN_EVT_PONG = 0x81,
    CAN_EVT_STATUS = 0x82,
    CAN_EVT_ACK = 0x83,
    CAN_EVT_DONE = 0x84,
    CAN_EVT_ERROR = 0xE0,
} can_event_t;

esp_err_t can_bus_init(app_context_t *ctx);
esp_err_t can_bus_apply_settings(app_context_t *ctx);
esp_err_t can_bus_send_status(app_context_t *ctx);
void can_bus_print_status(const app_context_t *ctx);
esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1);
