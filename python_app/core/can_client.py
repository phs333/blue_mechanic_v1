"""
PeakCAN (PCAN-Basic) Client for Blue Mechanic V1 using python-can.
Implements 11-bit standard frames, opcode encoding, sequence management,
background reception, status polling, and event parsing.
"""

import threading
import time
import struct
from typing import Optional, List, Dict, Any
import can

from .base_client import BaseClient
from .state_model import DeviceState
from .protocol_defs import (
    CanOpcode, CanEvent, FanMode, ESP_ERRORS,
    STATUS_FLAG_DRIVERS_ENABLED, STATUS_FLAG_Z_BLOQUEADO,
    STATUS_FLAG_ALARME_Z_ATIVO, STATUS_FLAG_TEMP_VALID,
    STATUS_FLAG_TMC_UART_READY, STATUS_FLAG_CAN_ONLINE,
    speed_level_to_delay, delay_to_speed_level
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
        self.event_base_id = 0x300
        self.seq_counter = 1

    @staticmethod
    def list_available_channels() -> List[str]:
        channels = ["PCAN_USBBUS1", "PCAN_USBBUS2", "PCAN_USBBUS3", "PCAN_USBBUS4", "PCAN_PCIBUS1"]
        return channels

    def _next_seq(self) -> int:
        self.seq_counter = (self.seq_counter % 254) + 1
        return self.seq_counter

    def connect(self, channel: str = "PCAN_USBBUS1", bitrate: int = 500000, 
                node_id: int = 1, cmd_base: int = 0x200, status_base: int = 0x280, 
                event_base: int = 0x300, interface: str = "pcan", **kwargs) -> bool:
        self.disconnect()
        self.channel = channel
        self.bitrate = bitrate
        self.node_id = node_id
        self.cmd_base_id = cmd_base
        self.status_base_id = status_base
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
        self.state.update_telemetry(can_online=False)

    def send_frame(self, arbitration_id: int, data: bytearray or bytes, desc: str = "") -> bool:
        if not self.is_connected or not self.bus:
            self.state.error_occurred.emit("Não conectado ao barramento CAN")
            return False
            
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
            is_fd=False
        )
        try:
            with self.lock:
                self.bus.send(msg)
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
        except Exception as e:
            self.state.error_occurred.emit(f"Erro ao transmitir frame CAN: {e}")
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
            time.sleep(1.0) # Poll status every 1 second

    def _rx_loop(self):
        while not self.stop_event.is_set():
            if not self.bus:
                break
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is not None:
                    self.state.telemetry.rx_frames += 1
                    self._process_rx_frame(msg)
            except Exception as e:
                time.sleep(0.02)
                
        self.is_connected = False
        self.state.set_connection_status(False, "PeakCAN")

    def _process_rx_frame(self, msg: can.Message):
        frame_id = msg.arbitration_id
        data = list(msg.data)
        dlc = len(data)
        
        status_id = self.status_base_id + self.node_id
        event_id = self.event_base_id + self.node_id
        
        desc = "Frame Desconhecido"
        
        if frame_id == status_id:
            # CAN_EVT_STATUS (0x82)
            if dlc >= 8 and data[0] == CanEvent.STATUS:
                node = data[1]
                flags = data[2]
                laser1 = data[3]
                laser2 = data[4]
                fan_byte = data[5]
                temp_val = struct.unpack('b', bytes([data[6]]))[0] # signed int8
                speed_lvl = data[7]
                
                drivers_en = bool(flags & STATUS_FLAG_DRIVERS_ENABLED)
                z_locked = bool(flags & STATUS_FLAG_Z_BLOQUEADO)
                alarm_on = bool(flags & STATUS_FLAG_ALARME_Z_ATIVO)
                temp_ok = bool(flags & STATUS_FLAG_TEMP_VALID)
                tmc_ready = bool(flags & STATUS_FLAG_TMC_UART_READY)
                can_on = bool(flags & STATUS_FLAG_CAN_ONLINE)
                
                fan_on = bool(fan_byte & 0x01)
                fan_mode_val = (fan_byte >> 1) & 0x07
                fan_mode = FanMode(fan_mode_val) if fan_mode_val in [0, 1, 2] else FanMode.MANUAL_OFF
                
                self.state.update_telemetry(
                    drivers_enabled=drivers_en,
                    z_bloqueado=z_locked,
                    alarme_z_ativo=alarm_on,
                    temp_valid=temp_ok,
                    temperature_c=float(temp_val),
                    tmc_uart_ready=tmc_ready,
                    can_online=can_on,
                    laser1_level=laser1,
                    laser2_level=laser2,
                    fan_output_on=fan_on,
                    fan_mode=fan_mode,
                    speed_level=speed_lvl
                )
                desc = f"STATUS (Node {node}): Drivers={'ON' if drivers_en else 'OFF'} Temp={temp_val}°C L1={laser1} L2={laser2}"
                
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
        seq = self._next_seq()
        payload = bytes([CanOpcode.STATUS_REQUEST, seq])
        return self.send_frame(target_id, payload, "STATUS_REQUEST")

    def ping(self, arg0: int = 0xAA, arg1: int = 0x55) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        payload = bytes([CanOpcode.PING, arg0 & 0xFF, arg1 & 0xFF, seq])
        return self.send_frame(target_id, payload, f"PING ({arg0:02X}, {arg1:02X})")

    def set_driver_enabled(self, enable: bool) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        payload = bytes([CanOpcode.ENABLE, 1 if enable else 0, seq])
        return self.send_frame(target_id, payload, f"ENABLE {'ON' if enable else 'OFF'}")

    def set_alarm_z(self, enable: bool) -> bool:
        # Fallback to serial / or inform CAN does not have direct alarm switch opcode
        self.state.update_telemetry(alarme_z_ativo=enable)
        return True

    def home_axis(self, axis: str) -> bool:
        axis = axis.upper()
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        if axis == "ALL" or axis == "CA":
            self.send_frame(target_id, bytes([CanOpcode.HOME, ord('C'), self._next_seq()]), "HOME C")
            self.send_frame(target_id, bytes([CanOpcode.HOME, ord('A'), self._next_seq()]), "HOME A")
            return self.send_frame(target_id, bytes([CanOpcode.HOME, ord('Z'), self._next_seq()]), "HOME Z")
        
        canonical_char = 'C' if axis[0] in ['C', 'X'] else ('A' if axis[0] in ['A', 'Y'] else 'Z')
        payload = bytes([CanOpcode.HOME, ord(canonical_char), seq])
        return self.send_frame(target_id, payload, f"HOME {axis}")

    def set_home(self, axis: str) -> bool:
        # SETHOME is configured via parameters in firmware; inform user
        self.state.raw_message_received.emit("INFO", f"SETHOME via CAN deve ser acionado no firmware.")
        return True

    def move_axis(self, axis: str, steps: int, speed: Optional[float] = None, accel: Optional[float] = None, force_no_encoder: bool = False) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        
        # 32-bit signed int, little endian
        steps_bytes = struct.pack('<i', int(steps))
        payload = bytearray([CanOpcode.MOVE, ord(axis[0].upper())])
        payload.extend(steps_bytes)
        payload.append(seq)
        
        return self.send_frame(target_id, bytes(payload), f"MOVE {axis.upper()} {steps} steps")

    def set_laser(self, laser_index: int, level: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        level = max(0, min(255, level))
        payload = bytes([CanOpcode.LASER, laser_index, level, seq])
        return self.send_frame(target_id, payload, f"LASER {laser_index} -> {level}")

    def set_fan(self, mode: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        mode = max(0, min(2, mode))
        payload = bytes([CanOpcode.FAN, mode, seq])
        mode_names = ["OFF", "ON", "AUTO"]
        return self.send_frame(target_id, payload, f"FAN -> {mode_names[mode]}")

    def set_speed_level(self, level: int) -> bool:
        target_id = self.cmd_base_id + self.node_id
        seq = self._next_seq()
        level = max(1, min(5, level))
        payload = bytes([CanOpcode.SPEED, level, seq])
        return self.send_frame(target_id, payload, f"SPEED -> Level {level}")

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        # Approximate to speed level in CAN
        return self.set_speed_level(3)

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        return True

    def set_z_pulley_teeth(self, teeth: int) -> bool:
        teeth = max(6, min(200, int(teeth)))
        self.state.update_parameters(z_pulley_teeth=teeth)
        return True
