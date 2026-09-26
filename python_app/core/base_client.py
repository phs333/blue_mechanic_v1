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

    def _unsupported(self, operation: str) -> bool:
        self.state.error_occurred.emit(
            f"{operation} não é suportado pela interface {self.__class__.__name__}."
        )
        return False
        
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

    def set_axis_limits(self, axis: str, min_deg: float, max_deg: float) -> bool:
        """Store min/max angular limits for axis C or A in NVS."""
        return self._unsupported("SET_LIMITS")

    @abstractmethod
    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        """Move specified axis by relative steps with optional speed/accel."""
        pass

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
        """Move C, A, and Z axes simultaneously in hardware with optional speeds and limit control."""
        return self._unsupported("MOVE_SYNC")

    @abstractmethod
    def set_laser(self, laser_index: int, level: int) -> bool:
        """Set laser 1 or 2 level (0..4095)."""
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
        return self._unsupported("Configuração da polia Z")

    def set_steps_per_rev(self, axis: str, steps: int) -> bool:
        return self._unsupported("Passos por volta")

    def set_length_z(self, steps: int) -> bool:
        return self._unsupported("Limite do eixo Z")

    def set_axis_ramp_speed(self, axis: str, speed: float) -> bool:
        """Set axis S-Curve start/ramp speed (deg/s for C/A, mm/s for Z)."""
        return self._unsupported(f"Rampa do eixo {axis}")

    def set_z_ramp_speed(self, speed_mm_s: float) -> bool:
        """Legacy helper for setting Z ramp speed."""
        return self.set_axis_ramp_speed("Z", speed_mm_s)

    def set_driver_mode(self, mode: str) -> bool:
        """Set driver bus mode (STEPDIR or UART)."""
        return self._unsupported("Modo do driver")

    def set_driver_invert(self, axis: str, invert: bool) -> bool:
        """Set hardware driver DIR inversion (C, A, Z)."""
        return self._unsupported("Inversão de direção")

    def set_tmc_uart_current(self, axis: str, ihold: int, irun: int, delay: int) -> bool:
        return self._unsupported("Corrente TMC2209")

    def set_tmc_spreadcycle(self, axis: str, enable: bool) -> bool:
        return self._unsupported("SpreadCycle TMC2209")

    def set_tmc_microsteps(self, axis: str, microsteps: int) -> bool:
        return self._unsupported("Microsteps TMC2209")

    def apply_driver_settings(self) -> bool:
        return self._unsupported("Aplicação das configurações TMC2209")

    def set_can_enabled(self, enabled: bool) -> bool:
        """Enable or disable CAN bus on the node."""
        return self._unsupported("Habilitar/Desabilitar barramento CAN")

    def configure_can(self, node_id: int, bitrate: int, cmd_base: int,
                      status_base: int, event_base: int, enabled: bool = True) -> bool:
        return self._unsupported("Configuração da rede CAN")

    def read_tmc_reg(self, axis: str, reg: int) -> bool:
        """Read TMC register directly."""
        return self._unsupported("Leitura de registrador TMC2209")

    def write_tmc_reg(self, axis: str, reg: int, val: int) -> bool:
        """Write TMC register directly."""
        return self._unsupported("Escrita de registrador TMC2209")
