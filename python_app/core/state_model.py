"""
Data model and reactive state representation for Blue Mechanic V1.
Uses PyQt6 QObject and pyqtSignal for thread-safe UI updates.

Eixo C: Base Rotativa
Eixo A: Pivot dos Lasers
Eixo Z: Atuador Linear
"""

import time
from dataclasses import dataclass, field
from typing import List, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal
from .protocol_defs import (
    FanMode, DEFAULT_Z_PULLEY_TEETH, calc_z_mm_per_step, calc_z_mm_per_rev, calc_z_steps_per_mm
)

@dataclass
class HardwareTelemetry:
    # Kinematics
    pos_c_deg: float = 0.0  # Base Rotativa
    pos_a_deg: float = 0.0  # Pivot dos Lasers
    pos_c_valid: bool = False
    pos_a_valid: bool = False
    pos_z_steps: int = 0    # Atuador Linear
    homed: List[bool] = field(default_factory=lambda: [False, False, False])  # C, A, Z referenciados
    in_motion: bool = False
    max_z_steps: int = 38400
    
    # State flags
    drivers_enabled: bool = False
    alarme_z_ativo: bool = True
    z_bloqueado: bool = False
    em_homing_z: bool = False
    
    # Thermal & Fan
    temperature_c: float = 0.0
    temp_valid: bool = False
    fan_output_on: bool = False
    fan_mode: FanMode = FanMode.MANUAL_OFF
    
    # Lasers (Colineares e opostos no Pivot A)
    laser1_level: int = 0  # 0..4095
    laser2_level: int = 0  # 0..4095
    
    # Drivers & CAN State
    tmc_uart_ready: bool = False
    can_online: bool = False
    speed_delay_us: List[int] = field(default_factory=lambda: [400, 400, 400])
    speed_level: int = 3
    
    # Comm Stats
    rx_frames: int = 0
    tx_frames: int = 0
    error_count: int = 0
    last_heartbeat_timestamp: float = 0.0
    last_seen_timestamp: float = 0.0
    online: bool = False
    
    # Backward compatibility properties
    @property
    def pos_x_deg(self) -> float:
        return self.pos_c_deg

    @pos_x_deg.setter
    def pos_x_deg(self, val: float):
        self.pos_c_deg = val

    @property
    def pos_y_deg(self) -> float:
        return self.pos_a_deg

    @pos_y_deg.setter
    def pos_y_deg(self, val: float):
        self.pos_a_deg = val
    
    def get_pos_z_mm(self, pulley_teeth: int = 16, steps_per_rev: int = 200, microsteps: int = 16) -> float:
        return self.pos_z_steps * calc_z_mm_per_step(pulley_teeth, steps_per_rev, microsteps)

    def get_max_z_mm(self, pulley_teeth: int = 16, steps_per_rev: int = 200, microsteps: int = 16) -> float:
        return self.max_z_steps * calc_z_mm_per_step(pulley_teeth, steps_per_rev, microsteps)

    @property
    def pos_z_mm(self) -> float:
        return self.get_pos_z_mm(DEFAULT_Z_PULLEY_TEETH, 200, 16)

    @property
    def max_z_mm(self) -> float:
        return self.get_max_z_mm(DEFAULT_Z_PULLEY_TEETH, 200, 16)

    @property
    def z_progress_pct(self) -> float:
        if self.max_z_steps <= 0:
            return 0.0
        return max(0.0, min(100.0, (self.pos_z_steps / self.max_z_steps) * 100.0))

@dataclass
class HardwareParameters:
    # Home & Limits
    home_c_deg: float = 0.0  # Base Rotativa
    home_a_deg: float = 0.0  # Pivot dos Lasers
    limit_min_deg_c: float = -360.0  # Limite angular mínimo Eixo C
    limit_max_deg_c: float = 360.0   # Limite angular máximo Eixo C
    limit_min_deg_a: float = -360.0  # Limite angular mínimo Eixo A
    limit_max_deg_a: float = 360.0   # Limite angular máximo Eixo A
    max_passos_z: int = 38400        # 480 mm @ 80 passos/mm (20T GT2)
    z_pulley_teeth: int = 20         # Dentes da polia GT2 motor Z (20T padrão)
    c_start_speed_deg: float = 10.0  # Velocidade inicial da rampa S-Curve C (deg/s)
    a_start_speed_deg: float = 10.0  # Velocidade inicial da rampa S-Curve A (deg/s)
    z_start_speed_mm: float = 15.0   # Velocidade inicial da rampa S-Curve Z (mm/s)
    
    # Backward compatibility
    @property
    def home_x_deg(self) -> float:
        return self.home_c_deg

    @home_x_deg.setter
    def home_x_deg(self, val: float):
        self.home_c_deg = val

    @property
    def home_y_deg(self) -> float:
        return self.home_a_deg

    @home_y_deg.setter
    def home_y_deg(self, val: float):
        self.home_a_deg = val
    
    # Steps per revolution
    steps_per_rev: List[int] = field(default_factory=lambda: [200, 200, 200])
    
    # Speeds & Accelerations
    # Keep startup values aligned with APP_SETTINGS_DEFAULT_INIT in firmware.
    speed: List[float] = field(default_factory=lambda: [360.0, 360.0, 250.0])
    speed_max: List[float] = field(default_factory=lambda: [720.0, 720.0, 500.0])
    accel_max: List[float] = field(default_factory=lambda: [7200.0, 7200.0, 4000.0])
    accel: List[float] = field(default_factory=lambda: [3600.0, 3600.0, 2000.0])
    inverter: List[bool] = field(default_factory=lambda: [True, True, False])
    
    # TMC2209 Settings
    driver_bus_mode: int = 1  # 0=StepDir, 1=UART Digital
    tmc_slave_addr: List[int] = field(default_factory=lambda: [0, 1, 2])
    tmc_ihold_ma: List[int] = field(default_factory=lambda: [559, 559, 418])
    tmc_irun_ma: List[int] = field(default_factory=lambda: [897, 897, 957])
    tmc_ihold_delay: List[int] = field(default_factory=lambda: [6, 6, 6])
    tmc_microsteps: List[int] = field(default_factory=lambda: [16, 16, 16])
    tmc_spreadcycle: List[bool] = field(default_factory=lambda: [False, False, False])
    # Velocidade até a qual o driver usa stealthChop (acima: spreadCycle). 0 = sempre stealthChop
    tmc_stealth_max: List[float] = field(default_factory=lambda: [180.0, 180.0, 60.0])

    # Motor de movimento (firmware): STREAM = S-curve no ISR + encadeamento; LEGACY = anterior
    motion_engine: str = "STREAM"
    lookahead: bool = True
    jerk: List[float] = field(default_factory=lambda: [30.0, 30.0, 15.0])
    
    # CAN Settings
    node_id: int = 1
    can_enabled: bool = True
    can_bitrate: int = 1000000
    can_command_base_id: int = 0x200
    can_status_base_id: int = 0x280
    can_event_base_id: int = 0x300

class DeviceState(QObject):
    """
    Thread-safe reactive store for hardware status and parameters.
    Supports single-node monitoring as well as 10-node fleet tracking.
    """
    telemetry_updated = pyqtSignal(object)  # Emits HardwareTelemetry (active/focused node)
    parameters_updated = pyqtSignal(object) # Emits HardwareParameters
    connection_changed = pyqtSignal(bool, str) # connected, backend_name
    heartbeat_received = pyqtSignal(int)    # node_id
    node_telemetry_updated = pyqtSignal(int, object) # node_id, HardwareTelemetry
    nodes_summary_updated = pyqtSignal(int) # total_online_nodes
    raw_message_received = pyqtSignal(str, str) # direction ('TX'/'RX'), text
    can_frame_received = pyqtSignal(dict)   # raw CAN frame info dictionary
    error_occurred = pyqtSignal(str)        # error message
    config_dump_completed = pyqtSignal()    # fim de um CONFIG DUMP (parâmetros confirmados pelo nó)
    ota_ready = pyqtSignal(int, int)        # node_id, error_code (0=OK)
    ota_progress = pyqtSignal(int, int)     # node_id, pct (0..100)
    ota_done = pyqtSignal(int)              # node_id
    ota_error = pyqtSignal(int, int)        # node_id, error_code
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.telemetry = HardwareTelemetry()
        self.parameters = HardwareParameters()
        self.nodes_telemetry: Dict[int, HardwareTelemetry] = {
            i: HardwareTelemetry() for i in range(1, 11)
        }
        self.is_connected = False
        self.backend_name = "None"
        
    def update_telemetry(self, **kwargs):
        changed = False
        for key, value in kwargs.items():
            if hasattr(self.telemetry, key) and getattr(self.telemetry, key) != value:
                setattr(self.telemetry, key, value)
                changed = True
        if changed:
            self.telemetry_updated.emit(self.telemetry)
            if not getattr(self, "_in_node_telemetry_update", False):
                node_id = getattr(self.parameters, "node_id", 1)
                if 1 <= node_id <= 10:
                    self.update_node_telemetry(node_id, **kwargs)

    def update_node_telemetry(self, node_id: int, **kwargs):
        """Update telemetry for a specific node (1..10) in the fleet."""
        if not (1 <= node_id <= 10):
            return
        node_t = self.nodes_telemetry.get(node_id)
        if not node_t:
            node_t = HardwareTelemetry()
            self.nodes_telemetry[node_id] = node_t

        now = time.time()
        if kwargs.get("online", True):
            if "last_seen_timestamp" not in kwargs:
                kwargs["last_seen_timestamp"] = now
            kwargs["online"] = True

        self._in_node_telemetry_update = True
        try:
            changed = False
            for key, value in kwargs.items():
                if hasattr(node_t, key) and getattr(node_t, key) != value:
                    setattr(node_t, key, value)
                    changed = True

            if changed:
                self.node_telemetry_updated.emit(node_id, node_t)
                self.nodes_summary_updated.emit(self.get_online_nodes_count())
        finally:
            self._in_node_telemetry_update = False

    def mark_node_offline(self, node_id: int):
        """Explicitly mark a node as offline in the fleet."""
        if not (1 <= node_id <= 10):
            return
        node_t = self.nodes_telemetry.get(node_id)
        if not node_t:
            return
        node_t.online = False
        node_t.can_online = False
        node_t.last_heartbeat_timestamp = 0.0
        node_t.last_seen_timestamp = 0.0
        self.node_telemetry_updated.emit(node_id, node_t)
        self.nodes_summary_updated.emit(self.get_online_nodes_count())

    def get_node_telemetry(self, node_id: int) -> HardwareTelemetry:
        if node_id not in self.nodes_telemetry:
            self.nodes_telemetry[node_id] = HardwareTelemetry()
        return self.nodes_telemetry[node_id]

    def is_node_online(self, node_id: int, timeout_sec: float = 3.5) -> bool:
        node_t = self.nodes_telemetry.get(node_id)
        if not node_t:
            return False
        last_time = max(node_t.last_heartbeat_timestamp, node_t.last_seen_timestamp)
        return (time.time() - last_time) <= timeout_sec if last_time > 0 else False

    def get_online_nodes_count(self, timeout_sec: float = 3.5) -> int:
        return sum(1 for i in range(1, 11) if self.is_node_online(i, timeout_sec))
            
    def update_parameters(self, **kwargs):
        changed = False
        for key, value in kwargs.items():
            if hasattr(self.parameters, key) and getattr(self.parameters, key) != value:
                setattr(self.parameters, key, value)
                changed = True
        if changed:
            self.parameters_updated.emit(self.parameters)

    def set_connection_status(self, connected: bool, backend: str = ""):
        self.is_connected = connected
        if backend:
            self.backend_name = backend
        self.connection_changed.emit(connected, self.backend_name)
