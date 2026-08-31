"""
Data model and reactive state representation for Blue Mechanic V1.
Uses PyQt6 QObject and pyqtSignal for thread-safe UI updates.

Eixo C: Base Rotativa
Eixo A: Pivot dos Lasers
Eixo Z: Atuador Linear
"""

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
    max_z_steps: int = 20000
    
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
    limit_min_deg_c: float = 10.0  # Limite angular mínimo Eixo C
    limit_max_deg_c: float = 190.0 # Limite angular máximo Eixo C
    limit_min_deg_a: float = 10.0  # Limite angular mínimo Eixo A
    limit_max_deg_a: float = 190.0 # Limite angular máximo Eixo A
    max_passos_z: int = 20000
    z_pulley_teeth: int = DEFAULT_Z_PULLEY_TEETH  # Dentes da polia GT2 motor Z (ex: 16, 20)
    
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
    speed: List[float] = field(default_factory=lambda: [140.625, 140.625, 12.5])
    speed_max: List[float] = field(default_factory=lambda: [720.0, 720.0, 60.0])
    accel_max: List[float] = field(default_factory=lambda: [3600.0, 3600.0, 800.0])
    accel: List[float] = field(default_factory=lambda: [1800.0, 1800.0, 300.0])
    inverter: List[bool] = field(default_factory=lambda: [False, False, False])
    
    # TMC2209 Settings
    driver_bus_mode: int = 0  # 0=StepDir, 1=UART
    tmc_slave_addr: List[int] = field(default_factory=lambda: [0, 1, 2])
    tmc_ihold_ma: List[int] = field(default_factory=lambda: [300, 300, 300])
    tmc_irun_ma: List[int] = field(default_factory=lambda: [800, 800, 800])
    tmc_ihold_delay: List[int] = field(default_factory=lambda: [6, 6, 6])
    tmc_microsteps: List[int] = field(default_factory=lambda: [16, 16, 16])
    tmc_spreadcycle: List[bool] = field(default_factory=lambda: [False, False, False])
    
    # CAN Settings
    node_id: int = 1
    can_enabled: bool = False
    can_bitrate: int = 500000
    can_command_base_id: int = 0x200
    can_status_base_id: int = 0x280
    can_event_base_id: int = 0x300

class DeviceState(QObject):
    """
    Thread-safe reactive store for hardware status and parameters.
    """
    telemetry_updated = pyqtSignal(object)  # Emits HardwareTelemetry
    parameters_updated = pyqtSignal(object) # Emits HardwareParameters
    connection_changed = pyqtSignal(bool, str) # connected, backend_name
    heartbeat_received = pyqtSignal(int)    # node_id
    raw_message_received = pyqtSignal(str, str) # direction ('TX'/'RX'), text
    can_frame_received = pyqtSignal(dict)   # raw CAN frame info dictionary
    error_occurred = pyqtSignal(str)        # error message
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.telemetry = HardwareTelemetry()
        self.parameters = HardwareParameters()
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
