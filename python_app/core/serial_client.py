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
from .main_thread import MainThreadRelay
from .state_model import DeviceState
from .protocol_defs import FanMode, delay_to_speed_level, translate_teensy_to_serial

class SerialClient(BaseClient):
    RESET_PULSE_S = 0.1          # EN em nível baixo durante o reset
    BOOT_TIMEOUT_S = 6.0         # boot do firmware: ~1,5 s de estabilização + drivers/CAN
    BOOT_READY_MARKER = "SISTEMA PRONTO PARA COMANDOS"

    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.serial_port: Optional[serial.Serial] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.port_name = ""
        self.baudrate = 115200
        self._in_config_dump = False
        self._relay = MainThreadRelay()
        self._initial_pending = False
        self._boot_timer: Optional[threading.Timer] = None
        
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

    def connect(self, port: str = "COM3", baudrate: int = 115200, reset_on_connect: bool = True, **kwargs) -> bool:
        self.disconnect()
        self.supports_jog = True  # reavaliado a cada conexão (o firmware pode ter sido regravado)
        try:
            port_obj = serial.Serial()
            port_obj.port = port
            port_obj.baudrate = baudrate
            port_obj.timeout = 0.1
            port_obj.write_timeout = 0.5
            # DTR/RTS inativos ANTES de abrir. No circuito de auto-reset do ESP32 (RTS->EN,
            # DTR->IO0) a abertura padrão do pyserial pode pulsar as linhas e deixar o chip
            # no bootloader de gravação — e aí ele não responde a comandos até apertar o botão.
            port_obj.dtr = False
            port_obj.rts = False
            port_obj.open()
            self.serial_port = port_obj
            self.port_name = port
            self.baudrate = baudrate
            self.is_connected = True
            self.stop_event.clear()
            
            self.reader_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.reader_thread.start()
            
            self.state.set_connection_status(True, f"COM ({port} @ {baudrate})")

            self._initial_pending = True
            if reset_on_connect:
                # Leitura inicial só depois do boot (linha "SISTEMA PRONTO" ou timeout):
                # enviada durante o boot, seria perdida.
                self.reset_device()
                self._boot_timer = threading.Timer(self.BOOT_TIMEOUT_S, self._request_initial_state)
                self._boot_timer.daemon = True
                self._boot_timer.start()
            else:
                time.sleep(0.1)
                self._request_initial_state()
            return True
        except Exception as e:
            self.state.error_occurred.emit(f"Falha ao abrir porta serial {port}: {e}")
            self.is_connected = False
            self.state.set_connection_status(False, "COM")
            return False

    def reset_device(self) -> bool:
        """Reset por hardware do ESP32 via RTS->EN, com DTR (IO0) inativo: boot normal,
        nunca o modo de gravação. Equivale a apertar o botão EN/RST da placa."""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            with self.lock:
                self.serial_port.dtr = False   # IO0 alto
                self.serial_port.rts = True    # EN baixo: chip em reset
                time.sleep(self.RESET_PULSE_S)
                self.serial_port.rts = False   # EN alto: boot do firmware
            self.state.raw_message_received.emit("TX", "[reset do ESP32 via RTS/EN]")
            return True
        except (serial.SerialException, OSError) as exc:
            self.state.error_occurred.emit(f"Falha ao resetar o ESP32 pela serial: {exc}")
            return False

    def _request_initial_state(self) -> None:
        if not self._initial_pending or not self.is_connected:
            return
        self._initial_pending = False
        if self._boot_timer is not None:
            self._boot_timer.cancel()
            self._boot_timer = None
        self.request_config_dump()
        self.request_status()

    def disconnect(self) -> None:
        if self._boot_timer is not None:
            self._boot_timer.cancel()
            self._boot_timer = None
        self._initial_pending = False
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
            
        translated_cmds = translate_teensy_to_serial(cmd, self.state)
        if not translated_cmds:
            return True

        success = True
        for single_cmd in translated_cmds:
            cmd_str = single_cmd.strip() + "\r\n"
            try:
                with self.lock:
                    self.serial_port.write(cmd_str.encode('utf-8'))
                    self.serial_port.flush()
                self.state.raw_message_received.emit("TX", single_cmd.strip())
                self.state.telemetry.tx_frames += 1
            except Exception as e:
                self.state.error_occurred.emit(f"Erro ao transmitir comando serial: {e}")
                success = False
        return success

    supports_jog = True  # firmware com JOG contínuo (controle por mouse)

    def jog(self, d_c: float = 0.0, d_a: float = 0.0, d_z: float = 0.0) -> bool:
        """Incrementa o alvo do jog contínuo do nó (graus C/A, mm Z).

        Não passa pelo log de TX do terminal: no controle por mouse chega a ~50 linhas/s.
        """
        if not self.is_connected or not self.serial_port:
            return False
        parts = [f"{name} {val:.5f}" for name, val in (("C", d_c), ("A", d_a), ("Z", d_z)) if val]
        if not parts:
            return True
        try:
            with self.lock:
                self.serial_port.write(("JOG " + " ".join(parts) + "\r\n").encode("ascii"))
            self.state.telemetry.tx_frames += 1
            self._last_jog_time = time.monotonic()
            return True
        except Exception as e:
            self.state.error_occurred.emit(f"Erro ao transmitir JOG: {e}")
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
                            # Parse e alterações de estado na thread da interface
                            self._relay.post(self._handle_rx_line, text)
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                self.state.error_occurred.emit(f"Conexão serial perdida: {e}")
                break
            except Exception as e:
                time.sleep(0.01)
                
        self.is_connected = False
        self.state.set_connection_status(False, "COM")

    def _handle_rx_line(self, text: str) -> None:
        self.state.raw_message_received.emit("RX", text)
        self.state.telemetry.rx_frames += 1
        try:
            self._parse_response_line(text)
        except (ValueError, IndexError) as exc:
            self.state.error_occurred.emit(f"Linha serial não reconhecida ({exc}): {text}")

    def _parse_response_line(self, line: str):
        """Parse status lines printed by Blue Mechanic V1 ESP32 firmware."""
        # Firmware terminou o boot (após conectar, reset manual ou reinício inesperado):
        # relê configuração e status para a tela refletir o nó
        if self.BOOT_READY_MARKER in line:
            self._initial_pending = True
            self._request_initial_state()
            return

        # Firmware antigo sem JOG: volta ao modo compatível (MOVE_SYNC) nesta conexão
        if ("Comando desconhecido" in line and self.supports_jog
                and time.monotonic() - getattr(self, "_last_jog_time", -10.0) < 1.0):
            self.supports_jog = False
            self.state.error_occurred.emit(
                "Firmware do nó sem JOG contínuo: controle por mouse em modo compatível. "
                "Grave o firmware atualizado para o movimento suave.")
            return

        # --- 0. LINHA ESTRUTURADA (@POS chave=valor) ---
        if line.startswith("@POS "):
            fields = dict(item.split("=", 1) for item in line[5:].split() if "=" in item)
            updates = {}
            for key, attr in (("C", "pos_c"), ("A", "pos_a")):
                value = float(fields.get(key, "nan"))
                valid = value == value  # nan != nan
                updates[f"{attr}_valid"] = valid
                if valid:
                    updates[f"{attr}_deg"] = value
            if "Z" in fields:
                updates["pos_z_steps"] = int(fields["Z"])
            if "ZMAX" in fields:
                updates["max_z_steps"] = int(fields["ZMAX"])
            homed = fields.get("HOMED", "")
            if len(homed) == 3:
                updates["homed"] = [ch == "1" for ch in homed]
            if "MOVING" in fields:
                updates["in_motion"] = fields["MOVING"] == "1"
            if "ZLOCK" in fields:
                updates["z_bloqueado"] = fields["ZLOCK"] == "1"
            if "ALARM" in fields:
                updates["alarme_z_ativo"] = fields["ALARM"] == "1"
            self.state.update_telemetry(**updates)
            return

        # Mensagens que mudam o estado do Z (firmwares sem ZLOCK/ALARM no @POS também as emitem)
        if re.search(r'Home\s+Z\s+finalizado', line, re.IGNORECASE):
            self.state.update_telemetry(z_bloqueado=False)
        elif re.search(r'Home\s+Z\s+falhou|fim de curso Z acionado inesperadamente', line, re.IGNORECASE):
            self.state.update_telemetry(z_bloqueado=True)
        elif "ALARME Z DESATIVADO" in line.upper():
            self.state.update_telemetry(alarme_z_ativo=False, z_bloqueado=False)
        elif "ALARME Z ATIVADO" in line.upper():
            self.state.update_telemetry(alarme_z_ativo=True)

        # --- 1. LIVE TELEMETRY PARSERS (Pure Dynamic State) ---
        # Eixo C (Base): 45.20 deg (Limites: [10.00, 190.00] deg)
        m_c = re.search(r'Eixo\s+[CX](?:\s*\(.*?\))?:\s*([\d\.\-]+)\s*(?:deg|°)', line, re.IGNORECASE)
        if m_c:
            try:
                self.state.update_telemetry(pos_c_deg=float(m_c.group(1)), pos_c_valid=True)
            except ValueError:
                pass
        elif re.search(r'Eixo\s+[CX](?:\s*\(.*?\))?:\s*erro', line, re.IGNORECASE):
            self.state.update_telemetry(pos_c_valid=False)

        # Eixo A (Pivot): 12.30 deg (Limites: [10.00, 190.00] deg)
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

        # Drivers: ENERGIZADOS / DESLIGADOS
        m = re.search(r'Drivers:?\s*(ENERGIZADOS|DESLIGADOS)', line, re.IGNORECASE)
        if m:
            self.state.update_telemetry(drivers_enabled=(m.group(1).upper() == "ENERGIZADOS"))

        # Laser 1 / Laser Esquerdo: 50% (2048/4095)
        m1 = re.search(r'Laser\s+(?:1|Esquerdo)[:\s].*\((\d+)/(?:4095|255)\)', line, re.IGNORECASE)
        if m1:
            try:
                self.state.update_telemetry(laser1_level=int(m1.group(1)))
            except ValueError:
                pass

        # Laser 2 / Laser Direito: 50% (2048/4095)
        m2 = re.search(r'Laser\s+(?:2|Direito)[:\s].*\((\d+)/(?:4095|255)\)', line, re.IGNORECASE)
        if m2:
            try:
                self.state.update_telemetry(laser2_level=int(m2.group(1)))
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

        # Temperatura: 32.50 C
        m = re.search(r'Temperatura(?:\satual)?:\s*([\d\.\-]+)\s*C', line, re.IGNORECASE)
        if m:
            try:
                self.state.update_telemetry(temperature_c=float(m.group(1)), temp_valid=True)
            except ValueError:
                pass
        elif "Temperatura: indisponivel" in line:
            self.state.update_telemetry(temp_valid=False)

        # TMC UART status line
        if "TMC UART: ATIVA" in line:
            self.state.update_telemetry(tmc_uart_ready=True)

        # CAN status line
        m_can_stat = re.search(r'CAN:\s*(ONLINE|OFF|CONFIGURADO/OFFLINE)', line, re.IGNORECASE)
        if m_can_stat:
            self.state.update_telemetry(can_online=(m_can_stat.group(1).upper() == "ONLINE"))

        # --- 2. CONFIG DUMP BATCH PARSER (Persistent Parameters) ---
        if "=== CONFIG DUMP ===" in line:
            self._in_config_dump = True
            return

        if self._in_config_dump and "===================" in line:
            self._in_config_dump = False
            self.state.parameters_updated.emit(self.state.parameters)
            self.state.config_dump_completed.emit()
            return

        m_cfg_motion = re.search(r'CONFIG MOTION ENGINE=(\w+)\s+LOOKAHEAD=([01])\s+JERK C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_motion:
            self.state.parameters.motion_engine = m_cfg_motion.group(1).upper()
            self.state.parameters.lookahead = m_cfg_motion.group(2) == "1"
            self.state.parameters.jerk = [float(m_cfg_motion.group(i)) for i in (3, 4, 5)]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_stealth = re.search(r'CONFIG STEALTH_MAX C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_stealth:
            self.state.parameters.tmc_stealth_max = [float(m_cfg_stealth.group(i)) for i in (1, 2, 3)]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_steps = re.search(r'CONFIG STEPS C=(\d+)\s+A=(\d+)\s+Z=(\d+)', line, re.IGNORECASE)
        if m_cfg_steps:
            c, a, z = int(m_cfg_steps.group(1)), int(m_cfg_steps.group(2)), int(m_cfg_steps.group(3))
            self.state.parameters.steps_per_rev = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_spd = re.search(r'CONFIG SPEED C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_spd:
            c, a, z = float(m_cfg_spd.group(1)), float(m_cfg_spd.group(2)), float(m_cfg_spd.group(3))
            self.state.parameters.speed = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_spd_max = re.search(r'CONFIG SPEED_MAX C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_spd_max:
            c, a, z = float(m_cfg_spd_max.group(1)), float(m_cfg_spd_max.group(2)), float(m_cfg_spd_max.group(3))
            self.state.parameters.speed_max = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_acc = re.search(r'CONFIG ACCEL C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_acc:
            c, a, z = float(m_cfg_acc.group(1)), float(m_cfg_acc.group(2)), float(m_cfg_acc.group(3))
            self.state.parameters.accel = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_acc_max = re.search(r'CONFIG ACCEL_MAX C=([\d\.\-]+)\s+A=([\d\.\-]+)\s+Z=([\d\.\-]+)', line, re.IGNORECASE)
        if m_cfg_acc_max:
            c, a, z = float(m_cfg_acc_max.group(1)), float(m_cfg_acc_max.group(2)), float(m_cfg_acc_max.group(3))
            self.state.parameters.accel_max = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_inv = re.search(r'CONFIG INVERT C=([01])\s+A=([01])\s+Z=([01])', line, re.IGNORECASE)
        if m_cfg_inv:
            c, a, z = (m_cfg_inv.group(1) == "1"), (m_cfg_inv.group(2) == "1"), (m_cfg_inv.group(3) == "1")
            self.state.parameters.inverter = [c, a, z]
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_z = re.search(r'CONFIG PULLEY_Z=(\d+)(?:\s+MAX_Z_MM=[\d\.\-]+)?\s+MAX_PASSOS_Z=(\d+)', line, re.IGNORECASE)
        if m_cfg_z:
            teeth = int(m_cfg_z.group(1))
            max_z = int(m_cfg_z.group(2))
            self.state.parameters.z_pulley_teeth = teeth
            self.state.parameters.max_passos_z = max_z
            self.state.telemetry.max_z_steps = max_z
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        if "CONFIG RAMP" in line.upper():
            updated = False
            m_c = re.search(r'RAMP_C=([\d\.\-]+)', line, re.IGNORECASE)
            if m_c:
                self.state.parameters.c_start_speed_deg = float(m_c.group(1))
                updated = True
            m_a = re.search(r'RAMP_A=([\d\.\-]+)', line, re.IGNORECASE)
            if m_a:
                self.state.parameters.a_start_speed_deg = float(m_a.group(1))
                updated = True
            m_z = re.search(r'RAMP_Z=([\d\.\-]+)', line, re.IGNORECASE)
            if m_z:
                self.state.parameters.z_start_speed_mm = float(m_z.group(1))
                updated = True
            if updated and not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_lim = re.search(r'CONFIG LIMITS C=(-?[\d\.]+)\.\.(-?[\d\.]+)\s+A=(-?[\d\.]+)\.\.(-?[\d\.]+)', line, re.IGNORECASE)
        if m_cfg_lim:
            self.state.parameters.limit_min_deg_c = float(m_cfg_lim.group(1))
            self.state.parameters.limit_max_deg_c = float(m_cfg_lim.group(2))
            self.state.parameters.limit_min_deg_a = float(m_cfg_lim.group(3))
            self.state.parameters.limit_max_deg_a = float(m_cfg_lim.group(4))
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_mode = re.search(r'CONFIG DRIVER_BUS_MODE=(\d+)', line, re.IGNORECASE)
        if m_cfg_mode:
            self.state.parameters.driver_bus_mode = int(m_cfg_mode.group(1))
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

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
            if not self._in_config_dump:
                self.state.parameters_updated.emit(self.state.parameters)
            return

        m_cfg_can = re.search(r'CONFIG CAN (?:enabled=(\d+)\s+)?node=(\d+)\s+bitrate=(\d+)\s+cmd=(0x[0-9A-Fa-f]+)\s+status=(0x[0-9A-Fa-f]+)\s+event=(0x[0-9A-Fa-f]+)', line, re.IGNORECASE)
        if m_cfg_can:
            try:
                if m_cfg_can.group(1) is not None:
                    self.state.parameters.can_enabled = (m_cfg_can.group(1) == "1")
                self.state.parameters.node_id = int(m_cfg_can.group(2))
                self.state.parameters.can_bitrate = int(m_cfg_can.group(3))
                self.state.parameters.can_command_base_id = int(m_cfg_can.group(4), 16)
                self.state.parameters.can_status_base_id = int(m_cfg_can.group(5), 16)
                self.state.parameters.can_event_base_id = int(m_cfg_can.group(6), 16)
                if not self._in_config_dump:
                    self.state.parameters_updated.emit(self.state.parameters)
            except ValueError:
                pass
            return

        # --- 3. INDIVIDUAL PARAMETER CHANGE CONFIRMATIONS ---
        m_home = re.search(r'Home\s+([CAXY])\s+gravado\s+em\s+([\d\.\-]+)\s*deg', line, re.IGNORECASE)
        if m_home:
            axis = m_home.group(1).upper()
            val = float(m_home.group(2))
            if axis in ['C', 'X']:
                self.state.parameters.home_c_deg = val
                self.state.update_telemetry(pos_c_deg=val, pos_c_valid=True)
            else:
                self.state.parameters.home_a_deg = val
                self.state.update_telemetry(pos_a_deg=val, pos_a_valid=True)
            self.state.parameters_updated.emit(self.state.parameters)
            return

        m_enc_zero = re.search(r'Encoder\s+([CAXY]):\s*Zero\s*\(Home\)\s*gravado', line, re.IGNORECASE)
        if m_enc_zero:
            ax = m_enc_zero.group(1).upper()
            if ax in ['C', 'X']:
                self.state.update_telemetry(pos_c_deg=0.0, pos_c_valid=True)
            else:
                self.state.update_telemetry(pos_a_deg=0.0, pos_a_valid=True)
            return

        m_lim_resp = re.search(r'LIMIT\s+([CA])\s+gravado:\s+min=([\d\.\-]+)\s+max=([\d\.\-]+)', line, re.IGNORECASE)
        if m_lim_resp:
            ax = m_lim_resp.group(1).upper()
            mn = float(m_lim_resp.group(2))
            mx = float(m_lim_resp.group(3))
            if ax == 'C':
                self.state.parameters.limit_min_deg_c = mn
                self.state.parameters.limit_max_deg_c = mx
            else:
                self.state.parameters.limit_min_deg_a = mn
                self.state.parameters.limit_max_deg_a = mx
            self.state.parameters_updated.emit(self.state.parameters)
            return

        m_pulley = re.search(r'Polia(?:.*Z)?[^\d]*(\d+)\s*(?:T|dentes)', line, re.IGNORECASE)
        if m_pulley:
            try:
                self.state.parameters.z_pulley_teeth = int(m_pulley.group(1))
                self.state.parameters_updated.emit(self.state.parameters)
            except ValueError:
                pass
            return

    # --- High Level Implementation ---
    def request_status(self) -> bool:
        return self.send_raw("STATUS")

    def request_config_dump(self) -> bool:
        return self.send_raw("CONFIG DUMP")

    def stop_all(self, lasers_off: bool = True) -> bool:
        return self.send_raw("ESTOP" if lasers_off else "STOP")

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

    def read_tmc_reg(self, axis: str, reg: int) -> bool:
        return self.send_raw(f"DRIVER REG READ {axis.upper()} {reg}")

    def write_tmc_reg(self, axis: str, reg: int, val: int) -> bool:
        return self.send_raw(f"DRIVER REG WRITE {axis.upper()} {reg} {val}")

    def home_axis(self, axis: str) -> bool:
        axis = axis.upper()
        if axis == "ALL":
            self.send_raw("HOME C")
            self.send_raw("HOME A")
            return self.send_raw("HOME Z")
        elif axis == "CA":
            self.send_raw("HOME C")
            return self.send_raw("HOME A")
        return self.send_raw(f"HOME {axis}")

    def set_home(self, axis: str) -> bool:
        ax = axis.upper()
        if ax in ['C', 'X']:
            self.state.update_telemetry(pos_c_deg=0.0, pos_c_valid=True)
        elif ax in ['A', 'Y']:
            self.state.update_telemetry(pos_a_deg=0.0, pos_a_valid=True)
        return self.send_raw(f"SETHOME {ax}")

    def set_axis_limits(self, axis: str, min_deg: float, max_deg: float) -> bool:
        return self.send_raw(f"LIMIT {axis.upper()} {min_deg:.2f} {max_deg:.2f}")

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        cmd_name = "MOVE_F" if force_no_encoder else "MOVE"
        cmd = f"{cmd_name} {axis.upper()} {steps}"
        if speed is not None and speed > 0:
            cmd += f" S={speed:.1f}"
        if accel is not None and accel > 0:
            cmd += f" F={accel:.1f}"
        return self.send_raw(cmd)

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
        cmd_name = "MOVE_SYNC_F" if force_no_encoder else "MOVE_SYNC"
        cmd = f"{cmd_name} C {steps_c} A {steps_a} Z {steps_z}"
        # 3 casas: em segmentos curtos (controle por mouse) a velocidade define a duração
        if speed_c is not None and speed_c > 0:
            cmd += f" SC={speed_c:.3f}"
        if speed_a is not None and speed_a > 0:
            cmd += f" SA={speed_a:.3f}"
        if speed_z is not None and speed_z > 0:
            cmd += f" SZ={speed_z:.3f}"
        if accel is not None and accel > 0:
            cmd += f" F={accel:.1f}"
        return self.send_raw(cmd)

    def set_laser(self, laser_index: int, level: int) -> bool:
        level = max(0, min(4095, level))
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

    def set_axis_ramp_speed(self, axis: str, speed: float) -> bool:
        axis_up = axis.upper()
        if axis_up == "Z":
            speed = max(0.5, min(150.0, float(speed)))
            self.state.update_parameters(z_start_speed_mm=speed)
        elif axis_up == "C":
            speed = max(0.5, min(100.0, float(speed)))
            self.state.update_parameters(c_start_speed_deg=speed)
        elif axis_up == "A":
            speed = max(0.5, min(100.0, float(speed)))
            self.state.update_parameters(a_start_speed_deg=speed)
        return self.send_raw(f"RAMP {axis_up} {speed:.2f}")

    def set_z_ramp_speed(self, speed_mm_s: float) -> bool:
        return self.set_axis_ramp_speed("Z", speed_mm_s)

    def set_driver_invert(self, axis: str, inverted: bool) -> bool:
        return self.send_raw(f"DRIVER INVERT {axis.upper()} {'ON' if inverted else 'OFF'}")

    def set_tmc_uart_current(self, axis: str, ihold_ma: int, irun_ma: int, delay: int) -> bool:
        return self.send_raw(f"DRIVER UART CURRENT {axis.upper()} {ihold_ma} {irun_ma} {delay}")

    def set_tmc_spreadcycle(self, axis: str, enable: bool) -> bool:
        return self.send_raw(f"DRIVER UART SPREADCYCLE {axis.upper()} {'ON' if enable else 'OFF'}")

    def set_tmc_microsteps(self, axis: str, microsteps: int) -> bool:
        return self.send_raw(f"DRIVER UART MICROSTEPS {axis.upper()} {microsteps}")

    def set_tmc_address(self, axis: str, address: int) -> bool:
        return self.send_raw(f"DRIVER UART ADDR {axis.upper()} {int(address)}")

    def set_tmc_stealth_max(self, axis: str, speed: float) -> bool:
        return self.send_raw(f"DRIVER UART STEALTH_MAX {axis.upper()} {float(speed):.1f}")

    def set_motion_engine(self, engine: str) -> bool:
        return self.send_raw(f"MOTION ENGINE {'LEGACY' if engine.upper() == 'LEGACY' else 'STREAM'}")

    def set_lookahead(self, enabled: bool) -> bool:
        return self.send_raw(f"MOTION LOOKAHEAD {'ON' if enabled else 'OFF'}")

    def set_jerk(self, axis: str, value: float) -> bool:
        return self.send_raw(f"MOTION JERK {axis.upper()} {float(value):.2f}")

    def save_nvs(self) -> bool:
        """Força a gravação pendente na NVS (o firmware agrupa alterações por 300 ms)."""
        return self.send_raw("SAVE")

    def read_tmc_register(self, axis: str, reg_addr: int) -> bool:
        return self.send_raw(f"DRIVER REG READ {axis.upper()} 0x{reg_addr:02X}")

    def write_tmc_register(self, axis: str, reg_addr: int, value: int) -> bool:
        return self.send_raw(f"DRIVER REG WRITE {axis.upper()} 0x{reg_addr:02X} 0x{value:08X}")

    def apply_driver_settings(self) -> bool:
        return self.send_raw("DRIVER APPLY")

    def set_can_enabled(self, enabled: bool) -> bool:
        cmd = "CAN ON" if enabled else "CAN OFF"
        self.state.parameters.can_enabled = enabled
        return self.send_raw(cmd)

    def configure_can(self, node_id: int, bitrate: int, cmd_base: int, status_base: int, event_base: int, enabled: bool = True) -> bool:
        if enabled is not None:
            self.send_raw("CAN ON" if enabled else "CAN OFF")
            time.sleep(0.04)
        self.send_raw(f"CAN NODE {node_id}")
        time.sleep(0.04)
        self.send_raw(f"CAN BITRATE {bitrate}")
        time.sleep(0.04)
        self.send_raw(f"CAN BASE CMD 0x{cmd_base:03X}")
        time.sleep(0.04)
        self.send_raw(f"CAN BASE STATUS 0x{status_base:03X}")
        time.sleep(0.04)
        self.send_raw(f"CAN BASE EVENT 0x{event_base:03X}")
        time.sleep(0.04)
        return self.send_raw("CAN APPLY")
