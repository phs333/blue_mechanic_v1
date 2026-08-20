"""
Serial (COM) Port Client for Blue Mechanic V1.
Implements asynchronous serial communication, auto-parsing of firmware responses,
and rich command sending.

Axes:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers
- Eixo Z: Atuador Linear
"""

import threading
import time
import re
from typing import Optional, List, Dict, Any
import serial
import serial.tools.list_ports

from .base_client import BaseClient
from .state_model import DeviceState
from .protocol_defs import FanMode, delay_to_speed_level

class SerialClient(BaseClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.serial_port: Optional[serial.Serial] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.port_name = ""
        self.baudrate = 115200
        
    @staticmethod
    def list_available_ports() -> List[Dict[str, str]]:
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid
            })
        return ports

    def connect(self, port: str = "COM3", baudrate: int = 115200, **kwargs) -> bool:
        self.disconnect()
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=0.5
            )
            self.port_name = port
            self.baudrate = baudrate
            self.is_connected = True
            self.stop_event.clear()
            
            self.reader_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.reader_thread.start()
            
            self.state.set_connection_status(True, f"COM ({port} @ {baudrate})")
            
            # Initial poll
            time.sleep(0.1)
            self.request_status()
            return True
        except Exception as e:
            self.state.error_occurred.emit(f"Falha ao abrir porta serial {port}: {e}")
            self.is_connected = False
            self.state.set_connection_status(False, "COM")
            return False

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None
        
        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = None
            
        self.is_connected = False
        self.state.set_connection_status(False, "COM")

    def send_raw(self, cmd: str) -> bool:
        if not self.is_connected or not self.serial_port:
            self.state.error_occurred.emit("Não conectado à porta serial")
            return False
            
        cmd_str = cmd.strip() + "\r\n"
        try:
            with self.lock:
                self.serial_port.write(cmd_str.encode('utf-8'))
                self.serial_port.flush()
                time.sleep(0.040)  # Pacing delay between serial commands
            self.state.raw_message_received.emit("TX", cmd.strip())
            self.state.telemetry.tx_frames += 1
            return True
        except Exception as e:
            self.state.error_occurred.emit(f"Erro ao transmitir comando serial: {e}")
            return False

    def _rx_loop(self):
        line_buffer = bytearray()
        while not self.stop_event.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    line_buffer.extend(data)
                    while b'\n' in line_buffer:
                        line, _, line_buffer = line_buffer.partition(b'\n')
                        text = line.decode('utf-8', errors='ignore').strip()
                        if text:
                            self.state.raw_message_received.emit("RX", text)
                            self.state.telemetry.rx_frames += 1
                            self._parse_response_line(text)
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                self.state.error_occurred.emit(f"Conexão serial perdida: {e}")
                break
            except Exception as e:
                time.sleep(0.01)
                
        self.is_connected = False
        self.state.set_connection_status(False, "COM")

    def _parse_response_line(self, line: str):
        """Parse status lines printed by Blue Mechanic V1 ESP32 firmware."""
        # Eixo C (Base): 45.20 deg / Eixo X: 45.20 deg
        m_c = re.search(r'Eixo\s+[CX](?:\s*\(.*?\))?:\s*([\d\.\-]+)\s*(?:deg|°)', line, re.IGNORECASE)
        if m_c:
            try:
                self.state.update_telemetry(pos_c_deg=float(m_c.group(1)), pos_c_valid=True)
            except ValueError:
                pass
        elif re.search(r'Eixo\s+[CX](?:\s*\(.*?\))?:\s*erro', line, re.IGNORECASE):
            self.state.update_telemetry(pos_c_valid=False)

        # Eixo A (Pivot): 12.30 deg / Eixo Y: 12.30 deg
        m_a = re.search(r'Eixo\s+[AY](?:\s*\(.*?\))?:\s*([\d\.\-]+)\s*(?:deg|°)', line, re.IGNORECASE)
        if m_a:
            try:
                self.state.update_telemetry(pos_a_deg=float(m_a.group(1)), pos_a_valid=True)
            except ValueError:
                pass
        elif re.search(r'Eixo\s+[AY](?:\s*\(.*?\))?:\s*erro', line, re.IGNORECASE):
            self.state.update_telemetry(pos_a_valid=False)

        # Eixo Z: 400 / 20000 passos
        m_z = re.search(r'Eixo Z:\s*([\d\-]+)(?:\s*/\s*(\d+))?\s*passos', line, re.IGNORECASE)
        if m_z:
            try:
                pos_z = int(m_z.group(1))
                updates = {"pos_z_steps": pos_z}
                if m_z.group(2):
                    updates["max_z_steps"] = int(m_z.group(2))
                self.state.update_telemetry(**updates)
            except ValueError:
                pass

        # Alarme Z: ON / OFF
        m = re.search(r'Alarme Z:\s*(ON|OFF)', line, re.IGNORECASE)
        if m:
            self.state.update_telemetry(alarme_z_ativo=(m.group(1).upper() == "ON"))

        # Estado Z: BLOQUEADO / LIVRE
        m = re.search(r'Estado Z:\s*(BLOQUEADO|LIVRE)', line, re.IGNORECASE)
        if m:
            self.state.update_telemetry(z_bloqueado=(m.group(1).upper() == "BLOQUEADO"))

        # Drivers: ENERGIZADOS / DESLIGADOS ou Drivers energizados / desligados
        m = re.search(r'Drivers:?\s*(ENERGIZADOS|DESLIGADOS)', line, re.IGNORECASE)
        if m:
            self.state.update_telemetry(drivers_enabled=(m.group(1).upper() == "ENERGIZADOS"))

        # Laser 1 / Laser Esquerdo: 50% (128/255)
        m = re.search(r'Laser\s*(?:1|Esquerdo)?:.*\((\d+)/255\)', line, re.IGNORECASE)
        if m:
            try:
                self.state.update_telemetry(laser1_level=int(m.group(1)))
            except ValueError:
                pass

        # Laser 2 / Laser Direito: 50% (128/255)
        m = re.search(r'Laser\s*(?:2|Direito)?:.*\((\d+)/255\)', line, re.IGNORECASE)
        if m:
            try:
                self.state.update_telemetry(laser2_level=int(m.group(1)))
            except ValueError:
                pass

        # Fan: ON (MANUAL ON) / Fan: OFF (AUTO)
        m = re.search(r'Fan:\s*(ON|OFF)\s*\((.+)\)', line, re.IGNORECASE)
        if m:
            fan_on = (m.group(1).upper() == "ON")
            mode_str = m.group(2).upper()
            mode = FanMode.MANUAL_OFF
            if "AUTO" in mode_str:
                mode = FanMode.AUTO
            elif "ON" in mode_str:
                mode = FanMode.MANUAL_ON
            self.state.update_telemetry(fan_output_on=fan_on, fan_mode=mode)

        # Temperatura: 32.50 C / Temperatura atual: 32.50 C
        m = re.search(r'Temperatura(?:\satual)?:\s*([\d\.\-]+)\s*C', line, re.IGNORECASE)
        if m:
            try:
                self.state.update_telemetry(temperature_c=float(m.group(1)), temp_valid=True)
            except ValueError:
                pass
        elif "Temperatura: indisponivel" in line:
            self.state.update_telemetry(temp_valid=False)

        # CAN: ONLINE node=1 bitrate=500000
        m = re.search(r'CAN:\s*(ONLINE|OFF|CONFIGURADO/OFFLINE)\s+node=(\d+)\s+bitrate=(\d+)', line, re.IGNORECASE)
        if m:
            can_online = (m.group(1).upper() == "ONLINE")
            node_id = int(m.group(2))
            bitrate = int(m.group(3))
            self.state.update_telemetry(can_online=can_online)
            self.state.update_parameters(node_id=node_id, can_bitrate=bitrate, can_enabled=can_online)

        # Home C/X/A/Y gravado em ... deg
        m_home = re.search(r'Home\s+([CAXY])\s+gravado\s+em\s+([\d\.\-]+)\s*deg', line, re.IGNORECASE)
        if m_home:
            axis = m_home.group(1).upper()
            val = float(m_home.group(2))
            if axis in ['C', 'X']:
                self.state.update_parameters(home_c_deg=val)
            else:
                self.state.update_parameters(home_a_deg=val)

        # Polia Z: 16T GT2 ou Polia do motor Z configurada para 16 dentes
        m_pulley = re.search(r'Polia(?:.*Z)?[^\d]*(\d+)\s*(?:T|dentes)', line, re.IGNORECASE)
        if m_pulley:
            try:
                self.state.update_parameters(z_pulley_teeth=int(m_pulley.group(1)))
            except ValueError:
                pass

        # --- NVS CONFIG DUMP PARSERS ---
        m_cfg_steps = re.search(r'CONFIG STEPS C=(\d+)\s+A=(\d+)\s+Z=(\d+)', line, re.IGNORECASE)
        if m_cfg_steps:
            c, a, z = int(m_cfg_steps.group(1)), int(m_cfg_steps.group(2)), int(m_cfg_steps.group(3))
            self.state.parameters.steps_per_rev = [c, a, z]
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_spd = re.search(r'CONFIG SPEED C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_spd:
            c, a, z = float(m_cfg_spd.group(1)), float(m_cfg_spd.group(2)), float(m_cfg_spd.group(3))
            self.state.parameters.speed_max = [c, a, z]
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_acc = re.search(r'CONFIG ACCEL C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_acc:
            c, a, z = float(m_cfg_acc.group(1)), float(m_cfg_acc.group(2)), float(m_cfg_acc.group(3))
            self.state.parameters.accel_max = [c, a, z]
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_inv = re.search(r'CONFIG INVERT C=([01])\s+A=([01])\s+Z=([01])', line, re.IGNORECASE)
        if m_cfg_inv:
            c, a, z = (m_cfg_inv.group(1) == "1"), (m_cfg_inv.group(2) == "1"), (m_cfg_inv.group(3) == "1")
            self.state.parameters.inverter = [c, a, z]
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_z = re.search(r'CONFIG PULLEY_Z=(\d+)\s+MAX_PASSOS_Z=(\d+)', line, re.IGNORECASE)
        if m_cfg_z:
            teeth = int(m_cfg_z.group(1))
            max_z = int(m_cfg_z.group(2))
            self.state.parameters.z_pulley_teeth = teeth
            self.state.parameters.max_passos_z = max_z
            self.state.telemetry.max_z_steps = max_z
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_mode = re.search(r'CONFIG DRIVER_BUS_MODE=(\d+)', line, re.IGNORECASE)
        if m_cfg_mode:
            self.state.parameters.driver_bus_mode = int(m_cfg_mode.group(1))
            self.state.parameters_updated.emit(self.state.parameters)

        m_cfg_tmc = re.search(r'CONFIG TMC ([CAZ]) addr=(\d+) ihold=(\d+) irun=(\d+) delay=(\d+) usteps=(\d+) spread=([01])', line, re.IGNORECASE)
        if m_cfg_tmc:
            axis_char = m_cfg_tmc.group(1).upper()
            idx = 0 if axis_char == 'C' else (1 if axis_char == 'A' else 2)
            self.state.parameters.tmc_slave_addr[idx] = int(m_cfg_tmc.group(2))
            self.state.parameters.tmc_ihold_ma[idx] = int(m_cfg_tmc.group(3))
            self.state.parameters.tmc_irun_ma[idx] = int(m_cfg_tmc.group(4))
            self.state.parameters.tmc_ihold_delay[idx] = int(m_cfg_tmc.group(5))
            self.state.parameters.tmc_microsteps[idx] = int(m_cfg_tmc.group(6))
            self.state.parameters.tmc_spreadcycle[idx] = (m_cfg_tmc.group(7) == "1")
            self.state.parameters_updated.emit(self.state.parameters)

        # TMC line from STATUS (TMC C: addr=0 ihold=59mA irun=59mA delay=6 status=UART OK)
        m_tmc_stat = re.search(r'TMC ([CAZ]):\s+addr=(\d+)\s+ihold=(\d+)mA\s+irun=(\d+)mA\s+delay=(\d+)', line, re.IGNORECASE)
        if m_tmc_stat:
            axis_char = m_tmc_stat.group(1).upper()
            idx = 0 if axis_char == 'C' else (1 if axis_char == 'A' else 2)
            self.state.parameters.tmc_slave_addr[idx] = int(m_tmc_stat.group(2))
            self.state.parameters.tmc_ihold_ma[idx] = int(m_tmc_stat.group(3))
            self.state.parameters.tmc_irun_ma[idx] = int(m_tmc_stat.group(4))
            self.state.parameters.tmc_ihold_delay[idx] = int(m_tmc_stat.group(5))
            self.state.parameters_updated.emit(self.state.parameters)

        # CAN bases from STATUS (CAN bases: cmd=0x200 status=0x280 event=0x300)
        m_can_bases = re.search(r'CAN bases:\s+cmd=(0x[0-9A-Fa-f]+)\s+status=(0x[0-9A-Fa-f]+)\s+event=(0x[0-9A-Fa-f]+)', line, re.IGNORECASE)
        if m_can_bases:
            try:
                self.state.parameters.can_command_base_id = int(m_can_bases.group(1), 16)
                self.state.parameters.can_status_base_id = int(m_can_bases.group(2), 16)
                self.state.parameters.can_event_base_id = int(m_can_bases.group(3), 16)
                self.state.parameters_updated.emit(self.state.parameters)
            except ValueError:
                pass

        m_cfg_can = re.search(r'CONFIG CAN node=(\d+)\s+bitrate=(\d+)\s+cmd=(0x[0-9A-Fa-f]+)\s+status=(0x[0-9A-Fa-f]+)\s+event=(0x[0-9A-Fa-f]+)', line, re.IGNORECASE)
        if m_cfg_can:
            try:
                self.state.parameters.node_id = int(m_cfg_can.group(1))
                self.state.parameters.can_bitrate = int(m_cfg_can.group(2))
                self.state.parameters.can_command_base_id = int(m_cfg_can.group(3), 16)
                self.state.parameters.can_status_base_id = int(m_cfg_can.group(4), 16)
                self.state.parameters.can_event_base_id = int(m_cfg_can.group(5), 16)
                self.state.parameters_updated.emit(self.state.parameters)
            except ValueError:
                pass

    # --- High Level Implementation ---
    def request_status(self) -> bool:
        self.send_raw("CONFIG DUMP")
        return self.send_raw("STATUS")

    def request_config_dump(self) -> bool:
        return self.send_raw("CONFIG DUMP")

    def set_driver_enabled(self, enable: bool) -> bool:
        self.state.update_telemetry(drivers_enabled=enable)
        return self.send_raw("DRIVER ENABLED ON" if enable else "DRIVER ENABLED OFF")

    def set_alarm_z(self, enable: bool) -> bool:
        self.state.update_telemetry(alarme_z_ativo=enable)
        return self.send_raw("ALARM ON" if enable else "ALARM OFF")

    def set_driver_mode(self, mode: str) -> bool:
        """mode: 'STEPDIR' ou 'UART'"""
        mode_u = mode.upper()
        if "UART" in mode_u:
            self.send_raw("DRIVER MODE UART")
        else:
            self.send_raw("DRIVER MODE STEPDIR")
        return self.send_raw("DRIVER APPLY")

    def set_driver_invert(self, axis: str, invert: bool) -> bool:
        return self.send_raw(f"DRIVER INVERT {axis.upper()} {'ON' if invert else 'OFF'}")

    def read_tmc_reg(self, axis: str, reg: int) -> bool:
        return self.send_raw(f"DRIVER REG READ {axis.upper()} {reg}")

    def write_tmc_reg(self, axis: str, reg: int, val: int) -> bool:
        return self.send_raw(f"DRIVER REG WRITE {axis.upper()} {reg} {val}")

    def home_axis(self, axis: str) -> bool:
        axis = axis.upper()
        if axis == "ALL" or axis == "CA":
            self.send_raw("HOME C")
            self.send_raw("HOME A")
            return self.send_raw("HOME Z")
        return self.send_raw(f"HOME {axis}")

    def set_home(self, axis: str) -> bool:
        return self.send_raw(f"SETHOME {axis.upper()}")

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        cmd_name = "MOVE_F" if force_no_encoder else "MOVE"
        cmd = f"{cmd_name} {axis.upper()} {steps}"
        if speed is not None and speed > 0:
            cmd += f" S={speed:.1f}"
        if accel is not None and accel > 0:
            cmd += f" F={accel:.1f}"
        return self.send_raw(cmd)

    def set_laser(self, laser_index: int, level: int) -> bool:
        level = max(0, min(255, level))
        return self.send_raw(f"LASER {laser_index} {level}")

    def set_fan(self, mode: int) -> bool:
        if mode == 2:
            return self.send_raw("FAN AUTO")
        elif mode == 1:
            return self.send_raw("FAN 1")
        else:
            return self.send_raw("FAN 0")

    def set_speed_level(self, level: int) -> bool:
        level = max(1, min(5, level))
        return self.send_raw(f"VELOCIDADE {level}")

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        return self.send_raw(f"SPEED {axis.upper()} {speed:.2f}")

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        return self.send_raw(f"ACCEL {axis.upper()} {accel:.2f}")

    def set_max_speed(self, axis: str, speed: float) -> bool:
        return self.send_raw(f"SPEED_MAX {axis.upper()} {speed:.2f}")

    def set_max_accel(self, axis: str, accel: float) -> bool:
        return self.send_raw(f"ACCEL_MAX {axis.upper()} {accel:.2f}")

    def set_steps_per_rev(self, axis: str, steps: int) -> bool:
        return self.send_raw(f"STEPS {axis.upper()} {steps}")

    def set_length_z(self, steps: int) -> bool:
        return self.send_raw(f"SETLENGTH Z {steps}")

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        teeth = max(6, min(200, int(teeth)))
        self.state.update_parameters(z_pulley_teeth=teeth)
        return self.send_raw(f"PULLEY Z {teeth}")

    def set_driver_invert(self, axis: str, inverted: bool) -> bool:
        return self.send_raw(f"DRIVER INVERT {axis.upper()} {'ON' if inverted else 'OFF'}")

    def set_tmc_uart_current(self, axis: str, ihold_ma: int, irun_ma: int, delay: int) -> bool:
        return self.send_raw(f"DRIVER UART CURRENT {axis.upper()} {ihold_ma} {irun_ma} {delay}")

    def set_tmc_spreadcycle(self, axis: str, enable: bool) -> bool:
        return self.send_raw(f"DRIVER UART SPREADCYCLE {axis.upper()} {'ON' if enable else 'OFF'}")

    def set_tmc_microsteps(self, axis: str, microsteps: int) -> bool:
        return self.send_raw(f"DRIVER UART MICROSTEPS {axis.upper()} {microsteps}")

    def read_tmc_register(self, axis: str, reg_addr: int) -> bool:
        return self.send_raw(f"DRIVER REG READ {axis.upper()} 0x{reg_addr:02X}")

    def write_tmc_register(self, axis: str, reg_addr: int, value: int) -> bool:
        return self.send_raw(f"DRIVER REG WRITE {axis.upper()} 0x{reg_addr:02X} 0x{value:08X}")

    def apply_driver_settings(self) -> bool:
        return self.send_raw("DRIVER APPLY")

    def configure_can(self, node_id: int, bitrate: int, cmd_base: int, status_base: int, event_base: int) -> bool:
        self.send_raw(f"CAN NODE {node_id}")
        self.send_raw(f"CAN BITRATE {bitrate}")
        self.send_raw(f"CAN BASE CMD 0x{cmd_base:03X}")
        self.send_raw(f"CAN BASE STATUS 0x{status_base:03X}")
        self.send_raw(f"CAN BASE EVENT 0x{event_base:03X}")
        return self.send_raw("CAN APPLY")
