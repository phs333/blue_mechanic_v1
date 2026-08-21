"""
Communication Manager for Blue Mechanic V1.
Seamlessly switches between Serial (COM), PeakCAN, and Simulator backends.
"""

from typing import Optional, List, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal

from .state_model import DeviceState
from .base_client import BaseClient
from .serial_client import SerialClient
from .can_client import CanClient
from .simulator import SimulatorClient

class CommManager(QObject):
    def __init__(self, state: Optional[DeviceState] = None, parent=None):
        super().__init__(parent)
        self.state = state or DeviceState()
        
        self.serial_client = SerialClient(self.state)
        self.can_client = CanClient(self.state)
        self.sim_client = SimulatorClient(self.state)
        
        self.active_client: Optional[BaseClient] = None
        self.backend_type: str = "None"

    def get_available_com_ports(self) -> List[Dict[str, str]]:
        return SerialClient.list_available_ports()

    def get_available_can_channels(self) -> List[str]:
        return CanClient.list_available_channels()

    def connect_serial(self, port: str, baudrate: int = 115200) -> bool:
        self.disconnect_all()
        success = self.serial_client.connect(port=port, baudrate=baudrate)
        if success:
            self.active_client = self.serial_client
            self.backend_type = "COM"
        return success

    def connect_can(self, channel: str = "PCAN_USBBUS1", bitrate: int = 500000, 
                    node_id: int = 1, cmd_base: int = 0x200, status_base: int = 0x280, 
                    event_base: int = 0x300, interface: str = "pcan") -> bool:
        self.disconnect_all()
        success = self.can_client.connect(
            channel=channel, bitrate=bitrate, node_id=node_id,
            cmd_base=cmd_base, status_base=status_base, event_base=event_base,
            interface=interface
        )
        if success:
            self.active_client = self.can_client
            self.backend_type = "CAN"
        return success

    def connect_simulator(self) -> bool:
        self.disconnect_all()
        success = self.sim_client.connect()
        if success:
            self.active_client = self.sim_client
            self.backend_type = "SIMULATOR"
        return success

    def disconnect_all(self):
        if self.active_client:
            self.active_client.disconnect()
            self.active_client = None
        self.serial_client.disconnect()
        self.can_client.disconnect()
        self.sim_client.disconnect()
        self.backend_type = "None"
        self.state.set_connection_status(False, "None")

    @property
    def is_connected(self) -> bool:
        return self.active_client is not None and self.active_client.is_connected

    @property
    def backend(self) -> str:
        return self.backend_type

    # Forwarding high-level commands
    def send_raw(self, cmd: str) -> bool:
        if self.active_client:
            return self.active_client.send_raw(cmd)
        return False

    def request_status(self) -> bool:
        if self.active_client:
            return self.active_client.request_status()
        return False

    def set_driver_enabled(self, enable: bool) -> bool:
        if self.active_client:
            return self.active_client.set_driver_enabled(enable)
        return False

    def set_alarm_z(self, enable: bool) -> bool:
        if self.active_client:
            return self.active_client.set_alarm_z(enable)
        return False

    def home_axis(self, axis: str) -> bool:
        if self.active_client:
            return self.active_client.home_axis(axis)
        return False

    def set_home(self, axis: str) -> bool:
        if self.active_client:
            return self.active_client.set_home(axis)
        return False

    def set_axis_limits(self, axis: str, min_deg: float, max_deg: float) -> bool:
        if self.active_client:
            return self.active_client.set_axis_limits(axis, min_deg, max_deg)
        return False

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        if self.active_client:
            return self.active_client.move_axis(axis, steps, speed, accel, force_no_encoder)
        return False

    def move_sync(self, steps_c: int = 0, steps_a: int = 0, steps_z: int = 0, speed: Optional[float] = None, accel: Optional[float] = None) -> bool:
        if self.active_client:
            return self.active_client.move_sync(steps_c, steps_a, steps_z, speed, accel)
        return False

    def set_laser(self, laser_index: int, level: int) -> bool:
        if self.active_client:
            return self.active_client.set_laser(laser_index, level)
        return False

    def set_fan(self, mode: int) -> bool:
        if self.active_client:
            return self.active_client.set_fan(mode)
        return False

    def set_speed_level(self, level: int) -> bool:
        if self.active_client:
            return self.active_client.set_speed_level(level)
        return False

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        if self.active_client:
            return self.active_client.set_axis_speed(axis, speed)
        return False

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        if self.active_client:
            return self.active_client.set_axis_accel(axis, accel)
        return False

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        if self.active_client:
            return self.active_client.set_z_pulley_teeth(teeth)
        return False

    def set_driver_mode(self, mode: str) -> bool:
        if self.active_client:
            return self.active_client.set_driver_mode(mode)
        return False

    def set_driver_invert(self, axis: str, invert: bool) -> bool:
        if self.active_client:
            return self.active_client.set_driver_invert(axis, invert)
        return False

    def read_tmc_reg(self, axis: str, reg: int) -> bool:
        if self.active_client:
            return self.active_client.read_tmc_reg(axis, reg)
        return False

    def write_tmc_reg(self, axis: str, reg: int, val: int) -> bool:
        if self.active_client:
            return self.active_client.write_tmc_reg(axis, reg, val)
        return False
