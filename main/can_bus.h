#pragma once

#include "esp_err.h"

#include "app_defs.h"

#define CAN_NODE_ID_MIN 1U
#define CAN_NODE_ID_MAX 10U

/* Byte 2 do CAN_EVT_STATUS: bit 6 indica que o frame de posicao (status_base+0x10+node)
 * usa o formato v2 (C/A int16 em decimos de grau com sinal). Sem o bit: v1 (uint16 centesimos, 0..360). */
#define CAN_STATUS_FLAG_POS_V2 0x40U

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
    CAN_OP_MOVE_SYNC = 0x23,
    CAN_OP_STOP = 0x24,      // Parada imediata (payload: [0x24, flags]; bit0 = apagar lasers / E-STOP)
    CAN_OP_MOVE_UNIFIED = 0x25, // Movimento unificado 3 eixos absolutos
    CAN_OP_LASER = 0x30,
    CAN_OP_FAN = 0x31,
    CAN_OP_LASER_DUAL = 0x32, // Controle simultâneo dos 2 lasers: [0x32, L1_lo, L1_hi, L2_lo, L2_hi]
    // Comandos de Atualizacao OTA (Teensy -> Nodes)
    CAN_OP_OTA_START = 0x40, // Inicia sessao OTA (payload: [0x40, target_node (0=todos 10 nos, 1..10=especifico), size_b0..b3, flags])
    CAN_OP_OTA_DATA  = 0x41, // Bloco de dados binarios de firmware (payload: [0x41, seq_num, d0..d5])
    CAN_OP_OTA_END   = 0x42, // Finaliza e valida particao flash (payload: [0x42, target_node, crc_lo, crc_hi])
    CAN_OP_OTA_ABORT = 0x43, // Aborta sessao OTA e cancela gravacao
} can_opcode_t;

typedef enum {
    CAN_EVT_HEARTBEAT = 0x80,
    CAN_EVT_PONG = 0x81,
    CAN_EVT_STATUS = 0x82,
    CAN_EVT_ACK = 0x83,
    CAN_EVT_DONE = 0x84,
    // Eventos de Resposta OTA (Nodes -> Teensy)
    CAN_EVT_OTA_READY    = 0x90, // No pronto para receber dados (motores parados, flash aberta)
    CAN_EVT_OTA_PROGRESS = 0x91, // Progresso de gravacao (arg0: progresso %, arg1: status)
    CAN_EVT_OTA_DONE     = 0x92, // Gravacao e verificacao concluidas com sucesso, reiniciando
    CAN_EVT_OTA_ERROR    = 0x93, // Erro na gravacao ou verificacao da imagem flash
    CAN_EVT_ERROR = 0xE0,
} can_event_t;

esp_err_t can_bus_init(app_context_t *ctx);
esp_err_t can_bus_apply_settings(app_context_t *ctx);
esp_err_t can_bus_send_status(app_context_t *ctx);
void can_bus_print_status(const app_context_t *ctx);
/* Autoteste: loopback interno (controlador) e pelo transceiver (sem ACK); restaura o CAN normal. */
esp_err_t can_bus_self_test(app_context_t *ctx);
esp_err_t can_send_event(app_context_t *ctx, can_event_t event_id, uint8_t arg0, uint8_t arg1);
