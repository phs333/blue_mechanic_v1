"""
Communication Manager for Blue Mechanic V1.
Switches between direct ESP32 serial, Teensy USB/CAN, PeakCAN, and Simulator.
"""

from typing import Optional, List, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal

from .state_model import DeviceState
from .base_client import BaseClient
from .serial_client import SerialClient
from .teensy_serial_client import TeensySerialClient
from .can_client import CanClient
from .simulator import SimulatorClient

class CommManager(QObject):
    broadcast_changed = pyqtSignal(bool)

    def __init__(self, state: Optional[DeviceState] = None, parent=None):
        super().__init__(parent)
        self.state = state or DeviceState()
        
        self.serial_client = SerialClient(self.state)
        self.teensy_serial_client = TeensySerialClient(self.state)
        self.can_client = CanClient(self.state)
        self.sim_client = SimulatorClient(self.state)
        
        self.active_client: Optional[BaseClient] = None
        self.backend_type: str = "None"

    def set_broadcast_mode(self, enable: bool) -> None:
        if hasattr(self.active_client, "set_broadcast_mode"):
            self.active_client.set_broadcast_mode(enable)
        elif hasattr(self.teensy_serial_client, "set_broadcast_mode"):
            self.teensy_serial_client.set_broadcast_mode(enable)
        self.broadcast_changed.emit(bool(enable))

    def is_broadcast_mode(self) -> bool:
        if self.active_client and hasattr(self.active_client, "broadcast_mode"):
            return bool(self.active_client.broadcast_mode)
        return bool(getattr(self.teensy_serial_client, "broadcast_mode", False))

    def set_target_node(self, node_id: int) -> None:
        if hasattr(self.active_client, "node_id"):
            self.active_client.node_id = int(node_id)
        self.teensy_serial_client.node_id = int(node_id)
        self.can_client.node_id = int(node_id)
        self.state.parameters.node_id = int(node_id)

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

    def connect_teensy(
        self,
        port: str,
        baudrate: int = 115200,
        node_id: int = 1,
    ) -> bool:
        self.disconnect_all()
        success = self.teensy_serial_client.connect(
            port=port,
            baudrate=baudrate,
            node_id=node_id,
        )
        if success:
            self.active_client = self.teensy_serial_client
            self.backend_type = "TEENSY"
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
        self.teensy_serial_client.disconnect()
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

    def request_config_dump(self) -> bool:
        if self.active_client and hasattr(self.active_client, 'request_config_dump'):
            return self.active_client.request_config_dump()
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

    def move_sync(
        self,
        steps_c: int = 0,
        steps_a: int = 0,
        steps_z: int = 0,
        speed_c: Optional[float] = None,
        speed_a: Optional[float] = None,
        speed_z: Optional[float] = None,
        accel: Optional[float] = None,
        force_no_encoder: bool = False,
    ) -> bool:
        if self.active_client:
            return self.active_client.move_sync(
                steps_c, steps_a, steps_z, speed_c, speed_a, speed_z, accel, force_no_encoder
            )
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

    def set_can_enabled(self, enabled: bool) -> bool:
        if self.active_client and hasattr(self.active_client, 'set_can_enabled'):
            return self.active_client.set_can_enabled(enabled)
        return False

    def configure_can(self, node_id: int, bitrate: int, cmd_base: int,
                      status_base: int, event_base: int, enabled: bool = True) -> bool:
        if self.active_client and hasattr(self.active_client, 'configure_can'):
            return self.active_client.configure_can(node_id, bitrate, cmd_base, status_base, event_base, enabled=enabled)
        return False

    # --- OTA Update Delegation ---
    def ota_start(self, target_node: int, image_size: int) -> bool:
        if self.active_client and hasattr(self.active_client, 'ota_start'):
            return self.active_client.ota_start(target_node, image_size)
        return False

    def ota_send_chunk(self, seq_num: int, chunk: bytes, target_node: int = 0) -> bool:
        if self.active_client and hasattr(self.active_client, 'ota_send_chunk'):
            return self.active_client.ota_send_chunk(seq_num, chunk, target_node)
        return False

    def ota_end(self, target_node: int, checksum: int = 0) -> bool:
        if self.active_client and hasattr(self.active_client, 'ota_end'):
            return self.active_client.ota_end(target_node, checksum)
        return False

    def ota_abort(self, target_node: int = 0) -> bool:
        if self.active_client and hasattr(self.active_client, 'ota_abort'):
            return self.active_client.ota_abort(target_node)
        return False
