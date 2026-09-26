"""
Virtual Simulator for Blue Mechanic V1.
Emulates motion physics, limit switches, temperature fluctuations,
laser PWM, fan response, and CAN frames for offline testing.

Axes:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (Lasers Colineares Opostos)
- Eixo Z: Atuador Linear
"""

import threading
import time
import math
import random
from typing import Optional
from .base_client import BaseClient
from .state_model import DeviceState
from .protocol_defs import FanMode, calc_ca_degrees_per_step, calc_z_mm_per_step

class SimulatorClient(BaseClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.sim_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        
        # Simulated physical state
        self.sim_c_deg = 0.0  # Base Rotativa
        self.sim_a_deg = 0.0  # Pivot dos Lasers
        self.sim_z_steps = 0  # Atuador Linear
        self.max_z_steps = 20000
        self.target_c_deg = 0.0
        self.target_a_deg = 0.0
        self.target_z_steps = 0
        
        self.temp_base = 28.5
        self.temp_ambient = 28.5
        
        self.drivers_en = False
        self.alarm_z = True
        self.z_locked = False
        self.laser1 = 0
        self.laser2 = 0
        self.fan_on = False
        self.fan_mode = FanMode.MANUAL_OFF
        self.speed_level = 3

    def connect(self, **kwargs) -> bool:
        self.disconnect()
        self.is_connected = True
        self.stop_event.clear()
        
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()
        
        self.state.set_connection_status(True, "Simulador Virtual")
        self.state.update_telemetry(
            pos_c_deg=self.sim_c_deg,
            pos_a_deg=self.sim_a_deg,
            pos_z_steps=self.sim_z_steps,
            max_z_steps=self.max_z_steps,
            drivers_enabled=self.drivers_en,
            alarme_z_ativo=self.alarm_z,
            z_bloqueado=self.z_locked,
            laser1_level=self.laser1,
            laser2_level=self.laser2,
            fan_output_on=self.fan_on,
            fan_mode=self.fan_mode,
            temperature_c=self.temp_base,
            temp_valid=True,
            can_online=True,
            tmc_uart_ready=True
        )
        return True

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.sim_thread and self.sim_thread.is_alive():
            self.sim_thread.join(timeout=0.5)
        self.sim_thread = None
        self.is_connected = False
        self.state.set_connection_status(False, "Simulador")

    def _sim_loop(self):
        last_hb = time.time()
        last_temp_update = time.time()
        
        while not self.stop_event.is_set():
            now = time.time()
            dt = 0.05
            
            # Simulated smooth movement towards targets
            if self.drivers_en:
                # C axis (Base Rotativa)
                if abs(self.target_c_deg - self.sim_c_deg) > 0.05:
                    step = (self.target_c_deg - self.sim_c_deg) * 0.2
                    self.sim_c_deg += step
                else:
                    self.sim_c_deg = self.target_c_deg
                    
                # A axis (Pivot)
                if abs(self.target_a_deg - self.sim_a_deg) > 0.05:
                    step = (self.target_a_deg - self.sim_a_deg) * 0.2
                    self.sim_a_deg += step
                else:
                    self.sim_a_deg = self.target_a_deg

                # Z axis (Linear)
                if not self.z_locked:
                    if abs(self.target_z_steps - self.sim_z_steps) > 5:
                        step = (self.target_z_steps - self.sim_z_steps) * 0.25
                        self.sim_z_steps += int(step)
                    else:
                        self.sim_z_steps = self.target_z_steps

            # Thermal simulation (laser heats up, fan cools down)
            if now - last_temp_update > 0.5:
                laser_heat = (self.laser1 + self.laser2) / 8190.0 * 15.0 # up to +15C
                fan_cooling = -10.0 if self.fan_on else 0.0
                target_temp = self.temp_ambient + laser_heat + fan_cooling
                
                # Auto fan logic
                if self.fan_mode == FanMode.AUTO:
                    if self.temp_base >= 45.0:
                        self.fan_on = True
                    elif self.temp_base <= 40.0:
                        self.fan_on = False
                        
                self.temp_base += (target_temp - self.temp_base) * 0.1 + (random.random() - 0.5) * 0.1
                last_temp_update = now

            # Heartbeat generation
            if now - last_hb >= 1.0:
                self.state.heartbeat_received.emit(1)
                self.state.telemetry.last_heartbeat_timestamp = now
                self.state.can_frame_received.emit({
                    "timestamp": now,
                    "dir": "RX",
                    "id": "0x301",
                    "dlc": 8,
                    "data": "80 01 00 00 00 00 00 00",
                    "desc": "HEARTBEAT (Node 1) [Simulado]"
                })
                last_hb = now

            # Publish updated telemetry
            self.state.update_telemetry(
                pos_c_deg=round(self.sim_c_deg, 2),
                pos_a_deg=round(self.sim_a_deg, 2),
                pos_z_steps=int(self.sim_z_steps),
                temperature_c=round(self.temp_base, 2),
                fan_output_on=self.fan_on
            )
            time.sleep(dt)

    def send_raw(self, cmd: str) -> bool:
        self.state.raw_message_received.emit("TX", cmd)
        time.sleep(0.02)
        
        cmd_u = cmd.strip().upper()
        if cmd_u == "STATUS":
            self.request_status()
        elif "PULLEY Z" in cmd_u or "SET_PULLEY Z" in cmd_u:
            parts = cmd_u.split()
            if len(parts) >= 3:
                try:
                    self.set_z_pulley_teeth(int(parts[2]))
                except ValueError:
                    pass
        elif "DRIVER ENABLED ON" in cmd_u:
            self.set_driver_enabled(True)
        elif "DRIVER ENABLED OFF" in cmd_u:
            self.set_driver_enabled(False)
        elif "ALARM ON" in cmd_u:
            self.set_alarm_z(True)
        elif "ALARM OFF" in cmd_u:
            self.set_alarm_z(False)
        elif "HOME" in cmd_u:
            parts = cmd_u.split()
            if len(parts) > 1:
                self.home_axis(parts[1])
        else:
            self.state.raw_message_received.emit("RX", f"OK: {cmd}")
        return True

    def request_status(self) -> bool:
        params = self.state.parameters
        teeth = params.z_pulley_teeth or 16
        mm_step = calc_z_mm_per_step(
            teeth,
            params.steps_per_rev[2],
            params.tmc_microsteps[2],
        )
        pos_mm = self.sim_z_steps * mm_step
        max_mm = self.max_z_steps * mm_step
        self.state.raw_message_received.emit("RX", 
            f"=== STATUS ===\n"
            f"Eixo C (Base): {self.sim_c_deg:.2f} deg\n"
            f"Eixo A (Pivot): {self.sim_a_deg:.2f} deg\n"
            f"Eixo Z: {self.sim_z_steps} / {self.max_z_steps} passos ({pos_mm:.2f} / {max_mm:.2f} mm | Polia: {teeth}T GT2)\n"
            f"Alarme Z: {'ON' if self.alarm_z else 'OFF'}\n"
            f"Estado Z: {'BLOQUEADO' if self.z_locked else 'LIVRE'}\n"
            f"Drivers: {'ENERGIZADOS' if self.drivers_en else 'DESLIGADOS'}\n"
            f"Laser 1: {int(self.laser1*100/4095)}% ({self.laser1}/4095)\n"
            f"Laser 2: {int(self.laser2*100/4095)}% ({self.laser2}/4095)\n"
            f"Fan: {'ON' if self.fan_on else 'OFF'} ({self.fan_mode.name})\n"
            f"Temperatura: {self.temp_base:.2f} C"
        )
        return True

    def set_driver_enabled(self, enable: bool) -> bool:
        self.drivers_en = enable
        self.state.update_telemetry(drivers_enabled=enable)
        self.state.raw_message_received.emit("RX", f"Drivers {'energizados' if enable else 'desligados'}.")
        return True

    def set_alarm_z(self, enable: bool) -> bool:
        self.alarm_z = enable
        if not enable:
            self.z_locked = False
        self.state.update_telemetry(alarme_z_ativo=enable, z_bloqueado=self.z_locked)
        self.state.raw_message_received.emit("RX", f"ALARME Z {'ATIVADO' if enable else 'DESATIVADO'}.")
        return True

    def home_axis(self, axis: str) -> bool:
        axis = axis.upper()
        if axis in ["C", "X", "ALL", "CA"]:
            self.target_c_deg = 0.0
            self.sim_c_deg = 0.0
        if axis in ["A", "Y", "ALL", "CA"]:
            self.target_a_deg = 0.0
            self.sim_a_deg = 0.0
        if axis in ["Z", "ALL"]:
            self.target_z_steps = 0
            self.sim_z_steps = 0
            self.z_locked = False
        self.state.raw_message_received.emit("RX", f"Home {axis} finalizado.")
        return True

    def set_home(self, axis: str) -> bool:
        axis = axis.upper()
        if axis in ["C", "X"]:
            self.sim_c_deg = 0.0
            self.target_c_deg = 0.0
            self.state.update_telemetry(pos_c_deg=0.0, pos_c_valid=True)
            self.state.update_parameters(home_c_deg=0.0)
        elif axis in ["A", "Y"]:
            self.sim_a_deg = 0.0
            self.target_a_deg = 0.0
            self.state.update_telemetry(pos_a_deg=0.0, pos_a_valid=True)
            self.state.update_parameters(home_a_deg=0.0)
        self.state.raw_message_received.emit("RX", f"Home {axis} gravado em 0.00 deg (posicao atual zerada, voltas resetadas na NVS).")
        return True

    def set_axis_limits(self, axis: str, min_deg: float, max_deg: float) -> bool:
        axis = axis.upper()
        if axis in ["C", "X"]:
            self.state.update_parameters(limit_min_deg_c=min_deg, limit_max_deg_c=max_deg)
        elif axis in ["A", "Y"]:
            self.state.update_parameters(limit_min_deg_a=min_deg, limit_max_deg_a=max_deg)
        self.state.raw_message_received.emit("RX", f"LIMIT {axis} gravado: min={min_deg:.2f} max={max_deg:.2f} deg.")
        return True

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        axis = axis.upper()
        if not self.drivers_en:
            self.set_driver_enabled(True)

        params = self.state.parameters
        if axis in ["C", "X"]:
            deg_delta = steps * calc_ca_degrees_per_step(
                params.steps_per_rev[0], params.tmc_microsteps[0]
            )
            self.target_c_deg = max(-90.0, min(90.0, self.target_c_deg + deg_delta))
        elif axis in ["A", "Y"]:
            deg_delta = steps * calc_ca_degrees_per_step(
                params.steps_per_rev[1], params.tmc_microsteps[1]
            )
            self.target_a_deg = max(-90.0, min(90.0, self.target_a_deg + deg_delta))
        elif axis == "Z":
            if self.z_locked:
                self.state.error_occurred.emit("Eixo Z bloqueado por segurança")
                return False
            self.target_z_steps = max(0, min(self.max_z_steps, self.target_z_steps + steps))
            
        self.state.raw_message_received.emit("RX", f"MOVE {axis} {steps} enfileirado.")
        return True

    def set_laser(self, laser_index: int, level: int) -> bool:
        level = max(0, min(4095, level))
        if laser_index == 1:
            self.laser1 = level
            self.state.update_telemetry(laser1_level=level)
        elif laser_index == 2:
            self.laser2 = level
            self.state.update_telemetry(laser2_level=level)
        self.state.raw_message_received.emit("RX", f"Laser {laser_index} ajustado para {level}/4095.")
        return True

    def set_fan(self, mode: int) -> bool:
        mode = max(0, min(2, mode))
        self.fan_mode = FanMode(mode)
        if mode == 1:
            self.fan_on = True
            self.state.update_telemetry(fan_output_on=True)
        elif mode == 0:
            self.fan_on = False
            self.state.update_telemetry(fan_output_on=False)
        self.state.update_telemetry(fan_mode=self.fan_mode)
        self.state.raw_message_received.emit("RX", f"Fan definido para {self.fan_mode.name}.")
        return True

    def set_speed_level(self, level: int) -> bool:
        self.speed_level = max(1, min(5, level))
        self.state.update_telemetry(speed_level=self.speed_level)
        self.state.raw_message_received.emit("RX", f"Velocidade definida para nível {self.speed_level}.")
        return True

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        axis = axis.upper()
        index = 0 if axis in ('C', 'X') else (1 if axis in ('A', 'Y') else 2)
        values = list(self.state.parameters.speed)
        values[index] = float(speed)
        self.state.update_parameters(speed=values)
        return True

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        axis = axis.upper()
        index = 0 if axis in ('C', 'X') else (1 if axis in ('A', 'Y') else 2)
        values = list(self.state.parameters.accel)
        values[index] = float(accel)
        self.state.update_parameters(accel=values)
        return True

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        teeth = max(6, min(200, int(teeth)))
        self.state.update_parameters(z_pulley_teeth=teeth)
        mm_rev = teeth * 2.0
        self.state.raw_message_received.emit("RX", f"Polia do motor Z configurada para {teeth} dentes GT2 (passo 2.0mm -> {mm_rev:.2f} mm/volta).")
        return True

    def set_steps_per_rev(self, axis: str, steps: int) -> bool:
        return True

    def set_length_z(self, steps: int) -> bool:
        self.state.update_parameters(max_passos_z=steps)
        self.state.update_telemetry(max_z_steps=steps)
        return True

    def set_driver_mode(self, mode: str) -> bool:
        return True

    def set_driver_invert(self, axis: str, invert: bool) -> bool:
        return True

    def set_tmc_uart_current(self, axis: str, ihold: int, irun: int, delay: int) -> bool:
        return True

    def set_tmc_spreadcycle(self, axis: str, sc: bool) -> bool:
        return True

    def set_tmc_microsteps(self, axis: str, usteps: int) -> bool:
        return True

    def apply_driver_settings(self) -> bool:
        return True

    def set_can_enabled(self, enabled: bool) -> bool:
        self.state.update_parameters(can_enabled=enabled)
        self.state.raw_message_received.emit("RX", "Estado do CAN salvo.")
        return True

    def configure_can(self, node_id: int, bitrate: int, cmd_base: int, status_base: int, event_base: int, enabled: bool = True) -> bool:
        self.state.update_parameters(
            node_id=node_id,
            can_bitrate=bitrate,
            can_command_base_id=cmd_base,
            can_status_base_id=status_base,
            can_event_base_id=event_base,
            can_enabled=enabled
        )
        self.state.raw_message_received.emit("RX", "Configuracao CAN reaplicada.")
        return True

    # --- Simulated OTA Methods ---
    def ota_start(self, target_node: int, image_size: int) -> bool:
        self.state.raw_message_received.emit("TX", f"SIM CAN OTA_START (Node {target_node}, {image_size}B)")
        self.state.telemetry.tx_frames += 1
        nodes = [target_node] if target_node != 0 else list(range(1, 11))
        for nid in nodes:
            t = threading.Timer(0.05, lambda n=nid: self.state.ota_ready.emit(n, 0))
            t.daemon = True
            t.start()
        return True

    def ota_send_chunk(self, seq_num: int, chunk: bytes, target_node: int = 0) -> bool:
        self.state.telemetry.tx_frames += 1
        return True

    def ota_end(self, target_node: int, checksum: int = 0) -> bool:
        self.state.raw_message_received.emit("TX", f"SIM CAN OTA_END (Node {target_node})")
        self.state.telemetry.tx_frames += 1
        nodes = [target_node] if target_node != 0 else list(range(1, 11))
        for nid in nodes:
            t = threading.Timer(0.1, lambda n=nid: self.state.ota_done.emit(n))
            t.daemon = True
            t.start()
        return True

    def ota_abort(self, target_node: int = 0) -> bool:
        self.state.raw_message_received.emit("TX", f"SIM CAN OTA_ABORT (Node {target_node})")
        self.state.telemetry.tx_frames += 1
        nodes = [target_node] if target_node != 0 else list(range(1, 11))
        for nid in nodes:
            t = threading.Timer(0.05, lambda n=nid: self.state.ota_error.emit(n, 0xFF))
            t.daemon = True
            t.start()
        return True
