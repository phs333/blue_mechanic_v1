"""
PeakCAN (PCAN-Basic) Client for Blue Mechanic V1 using python-can.
Implements 11-bit standard frames, opcode encoding,
background reception, status polling, and event parsing.
"""

import threading
import time
import struct
from typing import Optional, List, Union
import can

from .base_client import BaseClient
from .state_model import DeviceState
from .protocol_defs import (
    CanOpcode, CanEvent, FanMode, ESP_ERRORS,
    STATUS_FLAG_DRIVERS_ENABLED, STATUS_FLAG_Z_BLOQUEADO,
    STATUS_FLAG_ALARME_Z_ATIVO, STATUS_FLAG_TEMP_VALID,
    STATUS_FLAG_TMC_UART_READY, STATUS_FLAG_CAN_ONLINE,
    calc_ca_degrees_per_step, calc_z_mm_per_step,
)

class CanClient(BaseClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.bus: Optional[can.BusABC] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.poll_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        
        self.interface = "pcan"
        self.channel = "PCAN_USBBUS1"
        self.bitrate = 500000
        self.node_id = 1
        self.cmd_base_id = 0x200
        self.status_base_id = 0x280
        self.pos_base_id = 0x290
        self.event_base_id = 0x300

    @staticmethod
    def list_available_channels() -> List[str]:
        channels = ["PCAN_USBBUS1", "PCAN_USBBUS2", "PCAN_USBBUS3", "PCAN_USBBUS4", "PCAN_PCIBUS1"]
        return channels

    def connect(self, channel: str = "PCAN_USBBUS1", bitrate: int = 500000, 
                node_id: int = 1, cmd_base: int = 0x200, status_base: int = 0x280, 
                pos_base: Optional[int] = None, event_base: int = 0x300, interface: str = "pcan", **kwargs) -> bool:
        self.disconnect()
        self.channel = channel
        self.bitrate = bitrate
        self.node_id = node_id
        self.cmd_base_id = cmd_base
        self.status_base_id = status_base
        self.pos_base_id = status_base + 0x10 if pos_base is None else pos_base
        self.event_base_id = event_base
        self.interface = interface

        try:
            # Try to connect to PCAN hardware
            self.bus = can.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate
            )
            self.is_connected = True
            self.stop_event.clear()
            
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            self.poll_thread = threading.Thread(target=self._auto_poll_loop, daemon=True)
            self.poll_thread.start()

            self.state.set_connection_status(True, f"PeakCAN ({self.channel} @ {self.bitrate//1000}k)")
            self.state.update_telemetry(can_online=True)
            
            # Initial status ping
            time.sleep(0.05)
            self.request_status()
            return True
        except Exception as e:
            self.state.error_occurred.emit(f"Falha ao conectar ao PeakCAN ({channel}): {e}")
            self.is_connected = False
            self.state.set_connection_status(False, "PeakCAN")
            return False

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.poll_thread and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=0.5)
            
        self.rx_thread = None
        self.poll_thread = None
        
        with self.lock:
            if self.bus:
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
            self.bus = None
            
        self.is_connected = False
        self.state.set_connection_status(False, "PeakCAN")
        self.state.update_telemetry(
            can_online=False,
            pos_c_valid=False,
            pos_a_valid=False,
            temp_valid=False,
        )

    def send_frame(self, arbitration_id: int, data: Union[bytearray, bytes], desc: str = "") -> bool:
        if not self.is_connected or not self.bus:
            return False
            
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
            is_fd=False
        )
        try:
            with self.lock:
                if self.bus:
                    self.bus.send(msg, timeout=0.05)
            self.state.telemetry.tx_frames += 1
            
            # Emit for sniffer
            self.state.can_frame_received.emit({
                "timestamp": time.time(),
                "dir": "TX",
                "id": f"0x{arbitration_id:03X}",
                "dlc": len(data),
                "data": " ".join(f"{b:02X}" for b in data),
                "desc": desc
            })
            return True
        except can.CanOperationError:
            self.state.telemetry.error_count += 1
            return False
        except can.CanError:
            self.state.telemetry.error_count += 1
            return False
        except Exception:
            self.state.telemetry.error_count += 1
            return False

    def send_raw(self, cmd: str) -> bool:
        # For CAN, parse human commands or raw hex: "ID#DATA"
        if "#" in cmd:
            parts = cmd.split("#")
            try:
                frame_id = int(parts[0], 16)
                data = bytes.fromhex(parts[1])
                return self.send_frame(frame_id, data, "Raw Frame")
            except Exception as e:
                self.state.error_occurred.emit(f"Formato de frame CAN inválido: {e}")
                return False
        self.state.error_occurred.emit("Modo CAN ativo. Envie comandos via botões ou use formato ID#HEXDATA")
        return False

    def _auto_poll_loop(self):
        while not self.stop_event.is_set():
            if self.is_connected:
                self.request_status()
            self.stop_event.wait(0.5)  # Poll status every 500ms

    def _rx_loop(self):
        while not self.stop_event.is_set():
            if not self.bus:
                break
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is not None:
                    self.state.telemetry.rx_frames += 1
                    self._process_rx_frame(msg)
            except can.CanOperationError:
                time.sleep(0.05)
            except can.CanError:
                time.sleep(0.05)
            except Exception:
                time.sleep(0.05)
                
        self.is_connected = False
        self.state.set_connection_status(False, "PeakCAN")

    def _process_rx_frame(self, msg: can.Message):
        frame_id = msg.arbitration_id
        data = list(msg.data)
        dlc = len(data)
        
        status_id = self.status_base_id + self.node_id
        pos_id = self.pos_base_id + self.node_id
        event_id = self.event_base_id + self.node_id
        
        desc = "Frame Desconhecido"
        
        if frame_id == status_id:
            # CAN_EVT_STATUS (0x82)
            if dlc >= 8 and data[0] == CanEvent.STATUS:
                node = data[1]
                flags = data[2]
                laser1 = (data[3] & 0xFF) | ((data[4] & 0xFF) << 8)
                laser2 = (data[5] & 0xFF) | ((data[6] & 0xFF) << 8)
                fan_byte = data[7]
                
                drivers_en = bool(flags & STATUS_FLAG_DRIVERS_ENABLED)
                z_locked = bool(flags & STATUS_FLAG_Z_BLOQUEADO)
                alarm_on = bool(flags & STATUS_FLAG_ALARME_Z_ATIVO)
                temp_ok = bool(flags & STATUS_FLAG_TEMP_VALID)
                tmc_ready = bool(flags & STATUS_FLAG_TMC_UART_READY)
                can_on = bool(flags & STATUS_FLAG_CAN_ONLINE)
                
                fan_on = bool(fan_byte & 0x01)
                fan_mode_val = (fan_byte >> 1) & 0x07
                fan_mode = FanMode(fan_mode_val) if fan_mode_val in [0, 1, 2] else FanMode.MANUAL_OFF
                speed_lvl = (fan_byte >> 4) & 0x0F
                
                self.state.update_telemetry(
                    drivers_enabled=drivers_en,
                    z_bloqueado=z_locked,
                    alarme_z_ativo=alarm_on,
                    temp_valid=self.state.telemetry.temp_valid if temp_ok else False,
                    tmc_uart_ready=tmc_ready,
                    can_online=can_on,
                    laser1_level=laser1,
                    laser2_level=laser2,
                    fan_output_on=fan_on,
                    fan_mode=fan_mode,
                    speed_level=speed_lvl
                )
                desc = f"STATUS (Node {node}): Drivers={'ON' if drivers_en else 'OFF'} L1={laser1} L2={laser2}"
                
        elif frame_id == pos_id:
            # POS_TELEMETRY: unsigned centidegrees C/A, unsigned Z steps,
            # signed decicelsius. UINT16_MAX / INT16_MIN are invalid sentinels.
            if dlc >= 8:
                pos_c_raw = struct.unpack('<H', bytes(data[0:2]))[0]
                pos_a_raw = struct.unpack('<H', bytes(data[2:4]))[0]
                pos_z = struct.unpack('<H', bytes(data[4:6]))[0]
                temp_deci = struct.unpack('<h', bytes(data[6:8]))[0]
                pos_c_valid = pos_c_raw != 0xFFFF and pos_c_raw <= 36000
                pos_a_valid = pos_a_raw != 0xFFFF and pos_a_raw <= 36000
                temp_valid = temp_deci != -32768
                temp_c = temp_deci / 10.0 if temp_valid else self.state.telemetry.temperature_c
                temp_valid = temp_valid and -55.0 <= temp_c <= 125.0

                updates = dict(
                    pos_z_steps=pos_z,
                    pos_c_valid=pos_c_valid,
                    pos_a_valid=pos_a_valid,
                    temperature_c=temp_c,
                    temp_valid=temp_valid,
                )
                if pos_c_valid:
                    updates["pos_c_deg"] = pos_c_raw / 100.0
                if pos_a_valid:
                    updates["pos_a_deg"] = pos_a_raw / 100.0
                self.state.update_telemetry(**updates)

                c_text = f"{pos_c_raw / 100.0:.2f}°" if pos_c_valid else "N/A"
                a_text = f"{pos_a_raw / 100.0:.2f}°" if pos_a_valid else "N/A"
                temp_text = f"{temp_c:.1f}°C" if temp_valid else "N/A"
                desc = f"POS TELEMETRY: C={c_text} A={a_text} Z={pos_z} Temp={temp_text}"
                
        elif frame_id == event_id:
            if dlc > 0:
                evt_type = data[0]
                node = data[1] if dlc > 1 else self.node_id
                
                if evt_type == CanEvent.HEARTBEAT:
                    self.state.telemetry.last_heartbeat_timestamp = time.time()
                    self.state.heartbeat_received.emit(node)
                    desc = f"HEARTBEAT (Node {node})"
                    
                elif evt_type == CanEvent.PONG:
                    arg0 = data[2] if dlc > 2 else 0
                    arg1 = data[3] if dlc > 3 else 0
                    desc = f"PONG (Arg0={arg0}, Arg1={arg1})"
                    self.state.raw_message_received.emit("RX", f"CAN PONG: {arg0}, {arg1}")
                    
                elif evt_type == CanEvent.ACK:
                    op = data[2] if dlc > 2 else 0
                    try:
                        op_name = CanOpcode(op).name
                    except ValueError:
                        op_name = f"0x{op:02X}"
                    desc = f"ACK: Opcode {op_name} executado"
                    self.state.raw_message_received.emit("RX", f"CAN ACK: {op_name}")
                    
                elif evt_type == CanEvent.DONE:
                    op = data[2] if dlc > 2 else 0
                    try:
                        op_name = CanOpcode(op).name
                    except ValueError:
                        op_name = f"0x{op:02X}"
                    desc = f"DONE: Movimento ({op_name}) concluído!"
                    self.state.raw_message_received.emit("RX", f"CAN DONE: {op_name}")
                    
                elif evt_type == CanEvent.ERROR:
                    op = data[2] if dlc > 2 else 0
                    err = data[3] if dlc > 3 else 0
                    err_name = ESP_ERRORS.get(err, f"0x{err:02X}")
                    try:
                        op_name = CanOpcode(op).name
                    except ValueError:
                        op_name = f"0x{op:02X}"
                    desc = f"ERROR: Opcode {op_name} falhou: {err_name}"
                    self.state.telemetry.error_count += 1
                    self.state.error_occurred.emit(f"CAN Erro no comando {op_name}: {err_name}")

        # Emit to sniffer table
        self.state.can_frame_received.emit({
            "timestamp": time.time(),
            "dir": "RX",
            "id": f"0x{frame_id:03X}",
            "dlc": dlc,
            "data": " ".join(f"{b:02X}" for b in data),
            "desc": desc
        })

    # --- High-level command implementations ---
    def request_status(self) -> bool:
        target_id = self.cmd_base_id + self.node_id
        payload = bytes([CanOpcode.STATUS_REQUEST])
        return self.send_frame(target_id, payload, "STATUS_REQUEST")

    def ping(self, arg0: int = 0xAA, arg1: int = 0x55) -> bool:
        target_id = self.cmd_base_id + self.node_id
        payload = bytes([CanOpcode.PING, arg0 & 0xFF, arg1 & 0xFF])
        return self.send_frame(target_id, payload, f"PING ({arg0:02X}, {arg1:02X})")

    def set_driver_enabled(self, enable: bool) -> bool:
        target_id = self.cmd_base_id + self.node_id
        payload = bytes([CanOpcode.ENABLE, 1 if enable else 0])
        return self.send_frame(target_id, payload, f"ENABLE {'ON' if enable else 'OFF'}")

    def set_alarm_z(self, enable: bool) -> bool:
        return self._unsupported("Configuração do alarme Z via CAN")

    def home_axis(self, axis: str) -> bool:
        axis = axis.upper()
        target_id = self.cmd_base_id + self.node_id
        if axis == "ALL" or axis == "CA":
            self.send_frame(target_id, bytes([CanOpcode.HOME, ord('C')]), "HOME C")
            self.send_frame(target_id, bytes([CanOpcode.HOME, ord('A')]), "HOME A")
            return self.send_frame(target_id, bytes([CanOpcode.HOME, ord('Z')]), "HOME Z")
        
        canonical_char = 'C' if axis[0] in ['C', 'X'] else ('A' if axis[0] in ['A', 'Y'] else 'Z')
        payload = bytes([CanOpcode.HOME, ord(canonical_char)])
        return self.send_frame(target_id, payload, f"HOME {axis}")

    def set_home(self, axis: str) -> bool:
        return self._unsupported("SETHOME via CAN")

    def set_axis_limits(self, axis: str, min_deg: float, max_deg: float) -> bool:
        ax = axis.lower()
        if ax in ['c', 'x']:
            self.state.parameters.limit_min_deg_c = min_deg
            self.state.parameters.limit_max_deg_c = max_deg
        elif ax in ['a', 'y']:
            self.state.parameters.limit_min_deg_a = min_deg
            self.state.parameters.limit_max_deg_a = max_deg
        self.state.parameters_updated.emit(self.state.parameters)
        return True

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        target_id = self.cmd_base_id + self.node_id
        axis_char = self._canonical_axis(axis)

        if (speed is not None and speed > 0) or (accel is not None and accel > 0):
            speed_value = float(speed) if speed is not None and speed > 0 else 0.0
            accel_value = int(round(accel)) if accel is not None and accel > 0 else 0
            accel_value = max(0, min(65535, accel_value))
            profile = bytearray([CanOpcode.MOVE_PROFILE, ord(axis_char)])
            profile.extend(struct.pack('<fH', speed_value, accel_value))
            if not self.send_frame(
                target_id,
                bytes(profile),
                f"MOVE_PROFILE {axis_char} S={speed_value:g} F={accel_value or 'default'}",
            ):
                return False

        # 32-bit signed int, little endian
        steps_bytes = struct.pack('<i', int(steps))
        opcode = CanOpcode.MOVE_FORCE if force_no_encoder else CanOpcode.MOVE
        payload = bytearray([opcode, ord(axis_char)])
        payload.extend(steps_bytes)
        desc = f"{'MOVE_F' if force_no_encoder else 'MOVE'} {axis_char} {steps} steps"
        return self.send_frame(target_id, bytes(payload), desc)

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
        target_id = self.cmd_base_id + self.node_id
        params = self.state.parameters

        angle_c_deg = int(steps_c) * calc_ca_degrees_per_step(
            params.steps_per_rev[0], params.tmc_microsteps[0]
        )
        angle_a_deg = int(steps_a) * calc_ca_degrees_per_step(
            params.steps_per_rev[1], params.tmc_microsteps[1]
        )
        distance_z_mm = int(steps_z) * calc_z_mm_per_step(
            params.z_pulley_teeth,
            params.steps_per_rev[2],
            params.tmc_microsteps[2],
        )

        angle_c_deci = round(angle_c_deg * 10.0)
        angle_a_deci = round(angle_a_deg * 10.0)
        distance_z_centi = round(distance_z_mm * 100.0)
        fixed_values = (angle_c_deci, angle_a_deci, distance_z_centi)
        if any(value < -32768 or value > 32767 for value in fixed_values):
            self.state.error_occurred.emit(
                "MOVE_SYNC fora da faixa CAN: C/A devem caber em int16 de 0,1° "
                "e Z em int16 de 0,01 mm."
            )
            return False

        speeds = (speed_c, speed_a, speed_z)
        accel_value = int(round(accel)) if accel is not None and accel > 0 else 0
        accel_value = max(0, min(65535, accel_value))
        for axis, speed in zip("CAZ", speeds):
            if (speed is not None and speed > 0) or accel_value > 0:
                speed_value = float(speed) if speed is not None and speed > 0 else 0.0
                profile = bytearray([CanOpcode.MOVE_PROFILE, ord(axis)])
                profile.extend(struct.pack("<fH", speed_value, accel_value))
                if not self.send_frame(
                    target_id,
                    bytes(profile),
                    f"MOVE_SYNC PROFILE {axis} S={speed_value:g} F={accel_value or 'default'}",
                ):
                    return False

        flags = 0x01 if force_no_encoder else 0x00
        payload = struct.pack(
            "<BhhhB",
            CanOpcode.MOVE_SYNC,
            angle_c_deci,
            angle_a_deci,
            distance_z_centi,
            flags,
        )
        desc = (
            f"MOVE_SYNC C={angle_c_deci / 10.0:.1f}° "
            f"A={angle_a_deci / 10.0:.1f}° "
            f"Z={distance_z_centi / 100.0:.2f}mm force={bool(flags)}"
        )
        return self.send_frame(target_id, payload, desc)

    def set_laser(self, laser_index: int, level: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        level = max(0, min(4095, int(level)))
        lvl_low = level & 0xFF
        lvl_high = (level >> 8) & 0xFF
        payload = bytes([CanOpcode.LASER, laser_index, lvl_low, lvl_high])
        return self.send_frame(target_id, payload, f"LASER {laser_index} -> {level}")

    def set_fan(self, mode: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        mode = max(0, min(2, mode))
        payload = bytes([CanOpcode.FAN, mode])
        mode_names = ["OFF", "ON", "AUTO"]
        return self.send_frame(target_id, payload, f"FAN -> {mode_names[mode]}")

    def set_speed_level(self, level: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        level = max(1, min(5, level))
        payload = bytes([CanOpcode.SPEED, level])
        return self.send_frame(target_id, payload, f"SPEED -> Level {level}")

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        axis_char = self._canonical_axis(axis)
        target_id = self.cmd_base_id + self.node_id
        payload = bytearray([CanOpcode.AXIS_SPEED, ord(axis_char)])
        payload.extend(struct.pack('<f', float(speed)))
        sent = self.send_frame(target_id, bytes(payload), f"SPEED {axis_char} {speed:g}")
        if sent:
            index = 'CAZ'.index(axis_char)
            values = list(self.state.parameters.speed)
            values[index] = float(speed)
            self.state.update_parameters(speed=values)
        return sent

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        axis_char = self._canonical_axis(axis)
        target_id = self.cmd_base_id + self.node_id
        payload = bytearray([CanOpcode.AXIS_ACCEL, ord(axis_char)])
        payload.extend(struct.pack('<f', float(accel)))
        sent = self.send_frame(target_id, bytes(payload), f"ACCEL {axis_char} {accel:g}")
        if sent:
            index = 'CAZ'.index(axis_char)
            values = list(self.state.parameters.accel)
            values[index] = float(accel)
            self.state.update_parameters(accel=values)
        return sent

    @staticmethod
    def _canonical_axis(axis: str) -> str:
        token = axis.strip().upper()[:1]
        if token in ('C', 'X'):
            return 'C'
        if token in ('A', 'Y'):
            return 'A'
        if token == 'Z':
            return 'Z'
        raise ValueError(f"Eixo inválido: {axis!r}")

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        return self._unsupported("Configuração da polia Z via CAN")
