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
DEFAULT_Z_PULLEY_TEETH = 20
PASSOS_POR_MM_Z = PASSOS_POR_VOLTA_MOTOR / (Z_BELT_PITCH_MM * DEFAULT_Z_PULLEY_TEETH) # 80 steps/mm para 20T

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

def calc_z_mm_for_steps(steps: int, pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH,
                        steps_per_rev: int = 200, microsteps: int = 16) -> float:
    return float(steps) * calc_z_mm_per_step(pulley_teeth, steps_per_rev, microsteps)

# --- CAN Protocol OpCodes (Teensy/host -> ESP32) ---
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
    MOVE_SYNC = 0x23
    STOP = 0x24          # [0x24, flags] bit0 = apagar lasers (E-STOP)
    LASER = 0x30
    FAN = 0x31
    OTA_START = 0x40
    OTA_DATA = 0x41
    OTA_END = 0x42
    OTA_ABORT = 0x43

# --- CAN Protocol Events / Responses (ESP32 -> Teensy/host) ---
class CanEvent(IntEnum):
    HEARTBEAT = 0x80
    PONG = 0x81
    STATUS = 0x82
    ACK = 0x83
    DONE = 0x84
    OTA_READY = 0x90
    OTA_PROGRESS = 0x91
    OTA_DONE = 0x92
    OTA_ERROR = 0x93
    ERROR = 0xE0

# --- Status Byte 2 Bitmask Flags ---
STATUS_FLAG_DRIVERS_ENABLED = 0x01
STATUS_FLAG_Z_BLOQUEADO     = 0x02
STATUS_FLAG_ALARME_Z_ATIVO  = 0x04
STATUS_FLAG_TEMP_VALID      = 0x08
STATUS_FLAG_TMC_UART_READY  = 0x10
STATUS_FLAG_CAN_ONLINE      = 0x20
STATUS_FLAG_POS_V2          = 0x40  # frame de posição do nó usa C/A int16 em décimos de grau com sinal

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
    0x08: "ESP_ERR_INVALID_RESPONSE",  # ex.: HOME terminou fora da tolerancia
    0x09: "ESP_ERR_INVALID_CRC",       # ex.: OTA com frame perdido (lacuna de sequencia)
    0x0C: "ESP_ERR_NOT_FINISHED",      # movimento interrompido por STOP ou removido da fila
    0xFF: "ESP_FAIL",
}

STOP_FLAG_LASERS_OFF = 0x01

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

# --- TMC2209: corrente quantizada em 32 degraus (CS 0..31) ---
# Mesma fórmula do firmware (tmc2209_ma_to_cs / tmc2209_cs_to_ma, Rsense 0,1 Ω).
TMC2209_MA_PER_CS = 59.846


def tmc_ma_to_cs(ma: float) -> int:
    return max(0, min(31, int(float(ma) / TMC2209_MA_PER_CS - 1.0 + 0.5)))


def tmc_cs_to_ma(cs: int) -> int:
    return int((max(0, min(31, int(cs))) + 1) * TMC2209_MA_PER_CS + 0.5)


def tmc_quantize_ma(ma: float) -> int:
    """Corrente que o driver realmente usará para o valor pedido (ex.: 559 -> 539 mA)."""
    return tmc_cs_to_ma(tmc_ma_to_cs(ma))


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


def translate_teensy_to_serial(cmd_str: str, state=None) -> list:
    """
    Traduz comandos ASCII do formato Teensy 4.1 / automação para comandos seriais nativos do ESP32-S3.
    Permite que o sequenciador de testes, automação, presets e comandos diretos funcionem perfeitamente
    em conexão Serial, suportando comandos com nó (ex: MSM 0 Z 400) ou sem nó (ex: MSM Z 400).
    """
    trimmed = cmd_str.strip()
    if not trimmed:
        return []
    parts = trimmed.split()
    if not parts:
        return []

    op = parts[0].upper()

    # Pass-through para comandos nativos do ESP32
    if op in ("STATUS", "TEMP", "LIMITS", "HELP"):
        return [trimmed]
    if op == "DRIVER" and len(parts) >= 3 and parts[1].upper() == "ENABLED":
        return [trimmed]

    params = state.parameters if state and hasattr(state, "parameters") else None
    spr_c = params.steps_per_rev[0] if params else 200
    usteps_c = params.tmc_microsteps[0] if params else 16
    spr_a = params.steps_per_rev[1] if params else 200
    usteps_a = params.tmc_microsteps[1] if params else 16
    spr_z = params.steps_per_rev[2] if params else 200
    usteps_z = params.tmc_microsteps[2] if params else 16
    teeth = params.z_pulley_teeth if (params and params.z_pulley_teeth > 0) else DEFAULT_Z_PULLEY_TEETH

    # 1. Enable / Disable: E [<target>] <0|1|ON|OFF>
    if op == "E":
        val = ""
        if len(parts) >= 3 and parts[1].isdigit():
            val = parts[2].strip().upper()
        elif len(parts) >= 2:
            val = parts[1].strip().upper()
        if val in ("1", "ON", "TRUE"):
            return ["DRIVER ENABLED ON"]
        elif val in ("0", "OFF", "FALSE"):
            return ["DRIVER ENABLED OFF"]
        return [trimmed]

    # 2. Homing: H [<target>] <axis>
    if op == "H":
        ax = ""
        if len(parts) >= 3 and parts[1].isdigit():
            ax = parts[2].upper()
        elif len(parts) >= 2:
            ax = parts[1].upper()
        if ax == "ALL":
            return ["HOME C", "HOME A", "HOME Z"]
        elif ax == "CA":
            return ["HOME C", "HOME A"]
        elif ax in ("C", "X", "A", "Y", "Z"):
            canonical = "C" if ax in ("C", "X") else ("A" if ax in ("A", "Y") else "Z")
            return [f"HOME {canonical}"]
        return [trimmed]

    # 3. Lasers: L [<target>] <index> <level>
    if op == "L":
        try:
            if len(parts) >= 4 and parts[1].isdigit():
                l_idx = int(parts[2])
                l_val = max(0, min(4095, int(parts[3])))
                return [f"LASER {l_idx} {l_val}"]
            elif len(parts) >= 3 and not parts[1].isdigit():
                l_idx = int(parts[1])
                l_val = max(0, min(4095, int(parts[2])))
                return [f"LASER {l_idx} {l_val}"]
        except ValueError:
            pass
        return [trimmed]

    # 4. Fan: F [<target>] <mode>
    if op == "F":
        m = ""
        if len(parts) >= 3 and parts[1].isdigit():
            m = parts[2].upper()
        elif len(parts) >= 2:
            m = parts[1].upper()
        if m in ("2", "AUTO"):
            return ["FAN AUTO"]
        elif m in ("1", "ON"):
            return ["FAN 1"]
        elif m in ("0", "OFF"):
            return ["FAN 0"]
        return [trimmed]

    # 5. Speed level: S [<target>] <level>
    if op == "S":
        try:
            if len(parts) >= 3 and parts[1].isdigit():
                lvl = max(1, min(5, int(parts[2])))
                return [f"VELOCIDADE {lvl}"]
            elif len(parts) >= 2 and not parts[1].isdigit():
                lvl = max(1, min(5, int(parts[1])))
                return [f"VELOCIDADE {lvl}"]
        except ValueError:
            pass
        return [trimmed]

    # 6. Single Axis Move: M / MF / MSM / MSMF / MOVE / MOVE_F [<target>] <axis> <steps> [suffix...]
    if op in ("M", "MF", "MSM", "MSMF", "MOVE", "MOVE_F"):
        is_force = op in ("MF", "MSMF", "MOVE_F")
        cmd_name = "MOVE_F" if is_force else "MOVE"
        ax = ""
        steps_str = ""
        suffix_parts = []
        if len(parts) >= 4 and parts[1].isdigit():
            ax = parts[2].upper()
            steps_str = parts[3]
            suffix_parts = parts[4:]
        elif len(parts) >= 3 and not parts[1].isdigit():
            ax = parts[1].upper()
            steps_str = parts[2]
            suffix_parts = parts[3:]

        if ax in ("C", "X", "A", "Y", "Z"):
            try:
                steps = int(steps_str)
                canonical = "C" if ax in ("C", "X") else ("A" if ax in ("A", "Y") else "Z")
                res = f"{cmd_name} {canonical} {steps}"
                if suffix_parts:
                    res += " " + " ".join(suffix_parts)
                return [res]
            except ValueError:
                pass
        return [trimmed]

    # 7. Synchronized Move: MS / MSF [<target>] <deg_c> <deg_a> <mm_z>
    if op in ("MS", "MSF"):
        is_force = (op == "MSF")
        cmd_name = "MOVE_SYNC_F" if is_force else "MOVE_SYNC"
        c_str, a_str, z_str = "", "", ""
        suffix_parts = []
        if len(parts) >= 5 and parts[1].isdigit():
            c_str, a_str, z_str = parts[2], parts[3], parts[4]
            suffix_parts = parts[5:]
        elif len(parts) >= 4 and not parts[1].isdigit():
            c_str, a_str, z_str = parts[1], parts[2], parts[3]
            suffix_parts = parts[4:]

        try:
            deg_c = float(c_str)
            deg_a = float(a_str)
            mm_z = float(z_str)
            steps_c = calc_ca_steps_for_degrees(deg_c, spr_c, usteps_c)
            steps_a = calc_ca_steps_for_degrees(deg_a, spr_a, usteps_a)
            steps_z = calc_z_steps_for_mm(mm_z, teeth, spr_z, usteps_z)
            res = f"{cmd_name} C {steps_c} A {steps_a} Z {steps_z}"
            if suffix_parts:
                res += " " + " ".join(suffix_parts)
            return [res]
        except ValueError:
            pass
        return [trimmed]

    # 8. Status request: R [<target>]
    if op == "R":
        return ["STATUS"]

    # 9. Ping: P [<target>] ...
    if op == "P":
        return ["STATUS"]

    return [trimmed]
