"""
Base Abstract Client for Hardware Communication Backends.
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, List
from .state_model import DeviceState

class BaseClient(ABC):
    def __init__(self, state: DeviceState):
        self.state = state
        self.is_connected = False
        
    @abstractmethod
    def connect(self, **kwargs) -> bool:
        """Connect to hardware device with provided parameters."""
        pass
        
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from hardware device."""
        pass
        
    @abstractmethod
    def send_raw(self, data: str) -> bool:
        """Send raw string/command."""
        pass

    # Standard high-level hardware commands
    @abstractmethod
    def request_status(self) -> bool:
        """Poll status."""
        pass
        
    @abstractmethod
    def set_driver_enabled(self, enable: bool) -> bool:
        """Enable or disable stepper drivers."""
        pass

    @abstractmethod
    def set_alarm_z(self, enable: bool) -> bool:
        """Enable/disable Z axis limit switch vigilance."""
        pass

    @abstractmethod
    def home_axis(self, axis: str) -> bool:
        """Home C, A or Z axis."""
        pass

    @abstractmethod
    def set_home(self, axis: str) -> bool:
        """Store current position as home for C or A."""
        pass

    @abstractmethod
    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        """Move specified axis by relative steps with optional speed/accel."""
        pass

    @abstractmethod
    def set_laser(self, laser_index: int, level: int) -> bool:
        """Set laser 1 or 2 level (0..255)."""
        pass

    @abstractmethod
    def set_fan(self, mode: int) -> bool:
        """Set fan mode (0=Off, 1=On, 2=Auto)."""
        pass

    @abstractmethod
    def set_speed_level(self, level: int) -> bool:
        """Set global speed level (1..5)."""
        pass
        
    @abstractmethod
    def set_axis_speed(self, axis: str, speed: float) -> bool:
        """Set axis speed (deg/s or mm/s)."""
        pass

    @abstractmethod
    def set_axis_accel(self, axis: str, accel: float) -> bool:
        """Set axis accel (deg/s^2 or mm/s^2)."""
        pass

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        """Set Z motor pulley teeth count."""
        return True
