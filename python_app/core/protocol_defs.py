"""
Protocol Definitions for Blue Mechanic V1
Matches firmware in ESP32-S3 (can_bus.h, app_defs.h, commands.c)

Axis Definitions:
- Eixo C: Base Rotativa
- Eixo A: Pivot acoplado na base rotativa (com lasers colineares opostos)
- Eixo Z: Atuador Linear (correia GT2 e polia dentada)
"""

from enum import IntEnum

# --- Default Constants ---
DEFAULT_BAUDRATE = 115200
DEFAULT_NODE_ID = 1
DEFAULT_CAN_COMMAND_BASE_ID = 0x200
DEFAULT_CAN_STATUS_BASE_ID = 0x280
DEFAULT_CAN_POS_BASE_ID = 0x290
DEFAULT_CAN_EVENT_BASE_ID = 0x300
DEFAULT_CAN_BITRATE = 500000

# Physical & Kinematics Constants
PASSOS_POR_VOLTA_MOTOR = 3200.0
GRAUS_POR_PASSO_CA = 360.0 / PASSOS_POR_VOLTA_MOTOR  # 0.1125 deg/step (Eixos C e A: 800 passos = 90 deg, 400 passos = 45 deg, 133 passos = 15 deg)
GRAUS_POR_PASSO_XY = GRAUS_POR_PASSO_CA  # Alias retrocompatibilidade

Z_BELT_PITCH_MM = 2.0  # Correia GT2 = 2.0 mm passo
DEFAULT_Z_PULLEY_TEETH = 16
PASSOS_POR_MM_Z = PASSOS_POR_VOLTA_MOTOR / (Z_BELT_PITCH_MM * DEFAULT_Z_PULLEY_TEETH) # 100 steps/mm para 16T

def calc_z_mm_per_rev(pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH) -> float:
    teeth = pulley_teeth if pulley_teeth > 0 else DEFAULT_Z_PULLEY_TEETH
    return teeth * Z_BELT_PITCH_MM

def calc_z_mm_per_step(pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH, steps_per_rev: int = 200, microsteps: int = 16) -> float:
    spr = steps_per_rev if steps_per_rev > 0 else 200
    usteps = microsteps if microsteps > 0 else 16
    return calc_z_mm_per_rev(pulley_teeth) / (spr * usteps)

def calc_z_steps_per_mm(pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH, steps_per_rev: int = 200, microsteps: int = 16) -> float:
    mm_step = calc_z_mm_per_step(pulley_teeth, steps_per_rev, microsteps)
    return (1.0 / mm_step) if mm_step > 0 else 100.0

def calc_ca_steps_for_degrees(degrees: float, steps_per_rev: int = 200, microsteps: int = 16) -> int:
    spr = steps_per_rev if steps_per_rev > 0 else 200
    usteps = microsteps if microsteps > 0 else 16
    return int(round(float(degrees) * spr * usteps / 360.0))

def calc_ca_degrees_per_step(steps_per_rev: int = 200, microsteps: int = 16) -> float:
    spr = steps_per_rev if steps_per_rev > 0 else 200
    usteps = microsteps if microsteps > 0 else 16
    return 360.0 / (spr * usteps)

def calc_z_steps_for_mm(mm: float, pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH,
                        steps_per_rev: int = 200, microsteps: int = 16) -> int:
    return int(round(float(mm) * calc_z_steps_per_mm(pulley_teeth, steps_per_rev, microsteps)))

# --- CAN Protocol OpCodes (Master -> Slave) ---
class CanOpcode(IntEnum):
    PING = 0x01
    STATUS_REQUEST = 0x02
    ENABLE = 0x10
    SPEED = 0x11
    AXIS_SPEED = 0x12
    AXIS_ACCEL = 0x13
    MOVE_PROFILE = 0x14
    MOVE = 0x20
    HOME = 0x21
    MOVE_FORCE = 0x22
    LASER = 0x30
    FAN = 0x31

# --- CAN Protocol Events / Responses (Slave -> Master) ---
class CanEvent(IntEnum):
    HEARTBEAT = 0x80
    PONG = 0x81
    STATUS = 0x82
    ACK = 0x83
    DONE = 0x84
    ERROR = 0xE0

# --- Status Byte 2 Bitmask Flags ---
STATUS_FLAG_DRIVERS_ENABLED = 0x01
STATUS_FLAG_Z_BLOQUEADO     = 0x02
STATUS_FLAG_ALARME_Z_ATIVO  = 0x04
STATUS_FLAG_TEMP_VALID      = 0x08
STATUS_FLAG_TMC_UART_READY  = 0x10
STATUS_FLAG_CAN_ONLINE      = 0x20

# --- Fan Modes ---
class FanMode(IntEnum):
    MANUAL_OFF = 0
    MANUAL_ON = 1
    AUTO = 2

# --- Speed Levels (CAN) ---
SPEED_LEVEL_DELAYS = {
    1: 2000, # us (Mais lenta)
    2: 800,  # us (Lenta)
    3: 400,  # us (Média)
    4: 150,  # us (Rápida)
    5: 50,   # us (Mais rápida)
}

# --- ESP-IDF Common Error Codes ---
ESP_ERRORS = {
    0x00: "ESP_OK",
    0x01: "ESP_ERR_NO_MEM",
    0x02: "ESP_ERR_INVALID_ARG",
    0x03: "ESP_ERR_INVALID_STATE",
    0x04: "ESP_ERR_INVALID_SIZE",
    0x05: "ESP_ERR_NOT_FOUND",
    0x06: "ESP_ERR_NOT_SUPPORTED",
    0x07: "ESP_ERR_TIMEOUT",
    0xFF: "ESP_FAIL",
}

def delay_to_speed_level(delay_us: int) -> int:
    if delay_us >= 2000:
        return 1
    elif delay_us >= 800:
        return 2
    elif delay_us >= 400:
        return 3
    elif delay_us >= 150:
        return 4
    return 5

def speed_level_to_delay(level: int) -> int:
    return SPEED_LEVEL_DELAYS.get(level, 400)

LASER_MIN_USEFUL_DUTY = 46
LASER_MAX_USEFUL_DUTY = 300

def laser_level_to_percent(level: int) -> int:
    level = max(0, min(4095, int(level)))
    if level <= 0:
        return 0
    if level <= LASER_MIN_USEFUL_DUTY:
        return 1
    if level >= LASER_MAX_USEFUL_DUTY:
        return 100
    return int(round(1.0 + ((level - LASER_MIN_USEFUL_DUTY) / float(LASER_MAX_USEFUL_DUTY - LASER_MIN_USEFUL_DUTY)) * 99.0))

def percent_to_laser_level(percent: int) -> int:
    percent = max(0, min(100, int(percent)))
    if percent <= 0:
        return 0
    if percent == 1:
        return LASER_MIN_USEFUL_DUTY
    if percent >= 100:
        return LASER_MAX_USEFUL_DUTY
    return int(round(LASER_MIN_USEFUL_DUTY + ((percent - 1) / 99.0) * (LASER_MAX_USEFUL_DUTY - LASER_MIN_USEFUL_DUTY)))
