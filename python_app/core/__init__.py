"""
Core modules for Blue Mechanic V1.
"""
from .protocol_defs import CanOpcode, CanEvent, FanMode, delay_to_speed_level, speed_level_to_delay
from .state_model import DeviceState, HardwareTelemetry, HardwareParameters
from .comm_manager import CommManager
