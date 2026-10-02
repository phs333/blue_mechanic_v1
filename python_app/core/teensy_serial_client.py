"""
Teensy 4.1 USB/CAN bridge client for Blue Mechanic V1.

This backend intentionally implements the Teensy compact ASCII protocol instead
of reusing the direct ESP32-S3 serial commands.
"""

import math
import threading
import time
from typing import Dict, List, Optional

import serial
import serial.tools.list_ports

from .base_client import BaseClient
from .main_thread import MainThreadRelay
from .protocol_defs import (
    CanOpcode,
    ESP_ERRORS,
    FanMode,
    STATUS_FLAG_ALARME_Z_ATIVO,
    STATUS_FLAG_CAN_ONLINE,
    STATUS_FLAG_DRIVERS_ENABLED,
    STATUS_FLAG_TEMP_VALID,
    STATUS_FLAG_TMC_UART_READY,
    STATUS_FLAG_Z_BLOQUEADO,
    calc_ca_degrees_per_step,
    calc_z_mm_per_step,
)
from .state_model import DeviceState


class TeensySerialClient(BaseClient):
    """Serial client for the Teensy USB CDC-ACM to Classic CAN bridge."""

    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.serial_port: Optional[serial.Serial] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.poll_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.port_name = ""
        self.baudrate = 115200
        self.node_id = 1
        self.broadcast_mode = False
        self.node_offline_until: Dict[int, float] = {}
        self._last_actuation_time: float = 0.0
        self._last_tx_time: float = 0.0
        self._ota_active: bool = False
        self._relay = MainThreadRelay()

    @property
    def target_node(self) -> int:
        """Retorna 0 se o modo broadcast estiver ativo; caso contrário, o node_id específico."""
        return 0 if self.broadcast_mode else self.node_id

    @target_node.setter
    def target_node(self, value: int) -> None:
        val = int(value)
        if val == 0:
            self.broadcast_mode = True
        else:
            self.broadcast_mode = False
            self.node_id = max(1, min(10, val))

    def set_broadcast_mode(self, enable: bool) -> None:
        self.broadcast_mode = bool(enable)

    @staticmethod
    def list_available_ports() -> List[Dict[str, str]]:
        return [
            {
                "device": port.device,
                "description": port.description,
                "hwid": port.hwid,
            }
            for port in serial.tools.list_ports.comports()
        ]

    def connect(
        self,
        port: str = "COM3",
        baudrate: int = 115200,
        node_id: int = 1,
        **kwargs,
    ) -> bool:
        self.disconnect()
        try:
            selected_node = int(node_id)
        except (TypeError, ValueError):
            selected_node = 0
        if selected_node == 0:
            self.broadcast_mode = True
            selected_node = 1
        elif not 1 <= selected_node <= 10:
            self.state.error_occurred.emit("Node CAN do Teensy deve estar entre 1 e 10 (ou 0 para broadcast).")
            return False

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=0.5,
            )
            # Assert DTR e RTS para que o driver USB CDC ACM do Teensy/Zephyr
            # reconheça ativamente a conexão do host no Windows
            self.serial_port.dtr = True
            self.serial_port.rts = True

            self.port_name = port
            self.baudrate = baudrate
            self.node_id = selected_node
            self.is_connected = True
            self.stop_event.clear()

            self.reader_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.reader_thread.start()
            self.poll_thread = threading.Thread(target=self._auto_poll_loop, daemon=True)
            self.poll_thread.start()

            label = f"Teensy USB/CAN ({port}, Node {self.node_id})"
            if self.broadcast_mode:
                label += " [Broadcast]"
            self.state.set_connection_status(True, label)
            # A porta USB aberta não garante que o barramento CAN ou o node
            # estejam online; o bit CAN_ONLINE da resposta STATUS é a fonte.
            self.state.update_telemetry(can_online=False)

            # USB CDC may reset its line state when the host opens the port.
            time.sleep(0.1)
            # Ressincroniza o parser de linhas do Teensy: uma linha parcial/estourada de uma
            # sessão anterior faria o primeiro comando ser descartado como LINE_TOO_LONG.
            try:
                with self.lock:
                    self.serial_port.write(b"\r\n")
            except Exception:
                pass
            # Solicita status inicial de todos os nós para popular o painel imediatamente
            self.request_status_all()
            return True
        except Exception as exc:
            self.stop_event.set()
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = None
            self.state.error_occurred.emit(
                f"Falha ao abrir Teensy USB/CAN em {port}: {exc}"
            )
            self.is_connected = False
            self.state.set_connection_status(False, "Teensy USB/CAN")
            return False

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        if self.poll_thread and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=0.5)
        self.reader_thread = None
        self.poll_thread = None

        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = None

        self.is_connected = False
        self._ota_active = False
        self.state.set_connection_status(False, "Teensy USB/CAN")
        self.state.update_telemetry(
            can_online=False,
            pos_c_valid=False,
            pos_a_valid=False,
            temp_valid=False,
        )

    def send_raw(self, data: str) -> bool:
        if not self.is_connected or not self.serial_port:
            self.state.error_occurred.emit("Teensy USB/CAN não está conectado.")
            return False

        command = data.strip()
        if not command:
            return False

        tokens = command.split()
        op = tokens[0].upper() if tokens else ""
        if op in ("M", "MF", "MSM", "MSMF", "MS", "MSF", "U", "UF", "TD", "SYNC", "L", "E", "H", "S", "CFG", "OTA_START", "OTA_END", "OTA_ABORT"):
            self._last_actuation_time = time.time()

        try:
            with self.lock:
                now = time.time()
                elapsed = now - self._last_tx_time
                if elapsed < 0.012:
                    time.sleep(0.012 - elapsed)
                self.serial_port.write((command + "\r\n").encode("ascii"))
                self.serial_port.flush()
                self._last_tx_time = time.time()
            self.state.raw_message_received.emit("TX", command)
            self.state.telemetry.tx_frames += 1
            return True
        except Exception as exc:
            self.state.telemetry.error_count += 1
            self.state.error_occurred.emit(
                f"Erro ao transmitir para o Teensy USB/CAN: {exc}"
            )
            return False

    def _rx_loop(self) -> None:
        line_buffer = bytearray()
        while not self.stop_event.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not data:
                    time.sleep(0.01)
                    continue

                line_buffer.extend(data)
                while b"\n" in line_buffer:
                    line, _, line_buffer = line_buffer.partition(b"\n")
                    text = line.decode("ascii", errors="ignore").strip()
                    if text:
                        # Parse e alterações de estado na thread da interface
                        self._relay.post(self._handle_rx_line, text)
            except serial.SerialException as exc:
                self.state.error_occurred.emit(
                    f"Conexão com o Teensy USB/CAN perdida: {exc}"
                )
                break
            except Exception:
                time.sleep(0.01)

        self.is_connected = False
        self.state.set_connection_status(False, "Teensy USB/CAN")
        self.state.update_telemetry(can_online=False)

    def _auto_poll_loop(self) -> None:
        """
        Poll nodes continuously.
        - Actively polls confirmed online nodes round-robin (~350ms) to maintain fresh
          kinematics, temperature, and status.
        - Continuously probes offline nodes (round-robin across 1..10) to discover newly
          connected nodes and recover disconnected/restarted nodes without delay.
        - Background polling automatically yields when actuation commands (moves, lasers,
          enables, homing) or OTA are active, guaranteeing zero bus collisions.
        """
        online_idx = 0
        offline_idx = 0
        last_discovery_probe = 0.0

        while not self.stop_event.is_set():
            if self.is_connected:
                now = time.time()
                # Yield auto-poll if OTA is active or actuation command was sent recently (within 100ms)
                if self._ota_active or (now - self._last_actuation_time) < 0.10:
                    self.stop_event.wait(0.05)
                    continue

                online_nodes = [
                    node for node in range(1, 11)
                    if self.state.is_node_online(node, timeout_sec=3.5)
                ]
                offline_nodes = [
                    node for node in range(1, 11)
                    if node not in online_nodes
                ]

                if online_nodes:
                    # Interleave live online telemetry and periodic offline recovery
                    if offline_nodes and (now - last_discovery_probe) >= 1.5:
                        last_discovery_probe = now
                        target = offline_nodes[offline_idx % len(offline_nodes)]
                        offline_idx = (offline_idx + 1) % len(offline_nodes)
                    else:
                        target = online_nodes[online_idx % len(online_nodes)]
                        online_idx = (online_idx + 1) % len(online_nodes)
                    self.send_raw(f"R {target}")
                else:
                    # No nodes confirmed online yet: cycle through all nodes 1..10 in round-robin
                    # so any connected node is discovered within ~3.5 seconds
                    target = 1 + (offline_idx % 10)
                    offline_idx = (offline_idx + 1) % 10
                    self.send_raw(f"R {target}")

            self.stop_event.wait(0.35)

    def _handle_rx_line(self, text: str) -> None:
        self.state.raw_message_received.emit("RX", text)
        self.state.telemetry.rx_frames += 1
        self._parse_response_line(text)

    def _parse_response_line(self, line: str) -> None:
        """Parse the ASCII response contract emitted by the Teensy bridge."""
        fields = line.strip().split()
        if not fields:
            return

        message_type = fields[0].upper()
        try:
            if message_type == "STATUS" and len(fields) == 8:
                node = int(fields[1])
                if not (1 <= node <= 10):
                    return
                self.node_offline_until.pop(node, None)
                flags = int(fields[2], 0)
                laser1 = max(0, min(4095, int(fields[3], 0)))
                laser2 = max(0, min(4095, int(fields[4], 0)))
                fan_on = bool(int(fields[5], 0))
                fan_mode_value = int(fields[6], 0)
                speed_level = int(fields[7], 0)
                fan_mode = (
                    FanMode(fan_mode_value)
                    if fan_mode_value in (0, 1, 2)
                    else FanMode.MANUAL_OFF
                )
                temp_flag = bool(flags & STATUS_FLAG_TEMP_VALID)
                status_updates = dict(
                    drivers_enabled=bool(flags & STATUS_FLAG_DRIVERS_ENABLED),
                    z_bloqueado=bool(flags & STATUS_FLAG_Z_BLOQUEADO),
                    alarme_z_ativo=bool(flags & STATUS_FLAG_ALARME_Z_ATIVO),
                    tmc_uart_ready=bool(flags & STATUS_FLAG_TMC_UART_READY),
                    can_online=bool(flags & STATUS_FLAG_CAN_ONLINE),
                    laser1_level=laser1,
                    laser2_level=laser2,
                    fan_output_on=fan_on,
                    fan_mode=fan_mode,
                    speed_level=max(1, min(5, speed_level)),
                )
                if temp_flag:
                    status_updates["temp_valid"] = True
                self.state.update_node_telemetry(node, **status_updates)
                if node == self.node_id:
                    self.state.update_telemetry(**status_updates)
                return

            if message_type == "POS" and len(fields) == 6:
                node = int(fields[1])
                if not (1 <= node <= 10):
                    return
                self.node_offline_until.pop(node, None)
                try:
                    pos_c = float(fields[2])
                except (ValueError, TypeError):
                    pos_c = float("nan")
                try:
                    pos_a = float(fields[3])
                except (ValueError, TypeError):
                    pos_a = float("nan")
                try:
                    pos_z = int(fields[4], 0)
                except (ValueError, TypeError):
                    pos_z = 0
                try:
                    temperature = float(fields[5])
                except (ValueError, TypeError):
                    temperature = -99.9

                # Bridge atual: ângulos com sinal e "nan" para inválido. Bridge antigo usava
                # 0..360 e "-1.00" como inválido — continua reconhecido.
                pos_c_valid = math.isfinite(pos_c) and fields[2] != "-1.00"
                pos_a_valid = math.isfinite(pos_a) and fields[3] != "-1.00"
                temp_valid = math.isfinite(temperature) and temperature != -99.9 and -55.0 <= temperature <= 125.0

                updates = {
                    "pos_c_valid": pos_c_valid,
                    "pos_a_valid": pos_a_valid,
                    "pos_z_steps": pos_z,
                    "temp_valid": temp_valid,
                }
                if pos_c_valid:
                    updates["pos_c_deg"] = pos_c
                if pos_a_valid:
                    updates["pos_a_deg"] = pos_a
                if temp_valid:
                    updates["temperature_c"] = temperature
                self.state.update_node_telemetry(node, **updates)
                if node == self.node_id:
                    self.state.update_telemetry(**updates)
                return

            if message_type == "HEARTBEAT" and len(fields) == 2:
                node = int(fields[1])
                if 1 <= node <= 10:
                    self.node_offline_until.pop(node, None)
                    self.state.update_node_telemetry(
                        node,
                        last_heartbeat_timestamp=time.time(),
                        can_online=True,
                    )
                    if node == self.node_id:
                        self.state.update_telemetry(can_online=True)
                    self.state.heartbeat_received.emit(node)
                return

            if message_type == "PONG" and len(fields) == 4:
                return

            if message_type in ("ACK", "DONE") and len(fields) == 3:
                node = int(fields[1])
                if 1 <= node <= 10:
                    self.state.update_node_telemetry(
                        node,
                        last_seen_timestamp=time.time(),
                    )
                # Intercepta eventos OTA refletidos via ACK / DONE
                try:
                    opcode_val = int(fields[2], 0)
                    if message_type == "ACK" and opcode_val == CanOpcode.OTA_START:
                        self.state.ota_ready.emit(node, 0)
                    elif message_type == "DONE" and opcode_val == CanOpcode.OTA_END:
                        self.state.ota_done.emit(node)
                except ValueError:
                    pass
                return

            if message_type == "OTA_READY" and len(fields) >= 3:
                node = int(fields[1])
                status = int(fields[2], 0)
                self.state.ota_ready.emit(node, status)
                self.state.raw_message_received.emit("RX", f"TEENSY OTA_READY: Node {node} (status={status})")
                return

            if message_type == "OTA_PROGRESS" and len(fields) >= 3:
                node = int(fields[1])
                pct = int(fields[2], 0)
                self.state.ota_progress.emit(node, pct)
                self.state.raw_message_received.emit("RX", f"TEENSY OTA_PROGRESS: Node {node} {pct}%")
                return

            if message_type == "OTA_DONE" and len(fields) >= 2:
                node = int(fields[1])
                self.state.ota_done.emit(node)
                self.state.raw_message_received.emit("RX", f"TEENSY OTA_DONE: Node {node}")
                return

            if message_type == "OTA_ERROR" and len(fields) >= 3:
                node = int(fields[1])
                err = int(fields[2], 0)
                self.state.ota_error.emit(node, err)
                self.state.raw_message_received.emit("RX", f"TEENSY OTA_ERROR: Node {node} (err={err})")
                return

            if message_type == "ERROR" and len(fields) == 4:
                node = int(fields[1])
                opcode = int(fields[2], 16)
                error_code = int(fields[3], 16)
                try:
                    opcode_name = CanOpcode(opcode).name
                except ValueError:
                    opcode_name = f"0x{opcode:02X}"
                error_name = ESP_ERRORS.get(error_code, f"0x{error_code:02X}")
                self.state.telemetry.error_count += 1
                if 1 <= node <= 10:
                    node_t = self.state.get_node_telemetry(node)
                    node_t.error_count += 1
                self.state.error_occurred.emit(
                    f"Teensy/CAN Node {node}: comando {opcode_name} falhou: {error_name}"
                )
                return

            if message_type == "TEENSY_ERROR":
                # Handle CAN_TX_FAILED gracefully without wiping node state
                if len(fields) >= 3 and fields[1] == "CAN_TX_FAILED":
                    try:
                        failed_node = int(fields[2])
                        if 1 <= failed_node <= 10:
                            self.node_offline_until[failed_node] = time.time() + 10.0
                    except ValueError:
                        pass
                    return

                if len(fields) >= 3 and fields[1] == "CAN_TX_BUSY":
                    # Mailbox temporarily busy during transmit, ignore
                    return

                self.state.telemetry.error_count += 1
                detail = " ".join(fields[1:]) or "erro não especificado"
                self.state.error_occurred.emit(f"Teensy USB/CAN: {detail}")
        except (TypeError, ValueError):
            self.state.telemetry.error_count += 1
            self.state.error_occurred.emit(
                f"Resposta inválida do Teensy USB/CAN: {line}"
            )

    def request_status(self, node_id: Optional[int] = None) -> bool:
        target = self.node_id if node_id is None else int(node_id)
        if 1 <= target <= 10:
            return self.send_raw(f"R {target}")
        return False

    def request_status_all(self) -> bool:
        """Envia solicitação de status para todos os 10 nós."""
        return all(self.send_raw(f"R {i}") for i in range(1, 11))

    def ping(self, arg0: int = 0, arg1: int = 0) -> bool:
        return self.send_raw(
            f"P {self.node_id} {max(0, min(255, int(arg0)))} "
            f"{max(0, min(255, int(arg1)))}"
        )

    def set_driver_enabled(self, enable: bool) -> bool:
        return self.send_raw(f"E {self.target_node} {1 if enable else 0}")

    def stop_all(self, lasers_off: bool = True) -> bool:
        """STOP/E-STOP em broadcast (opcode CAN 0x24 via bridge): interrompe o movimento e
        esvazia a fila de todos os nós; ESTOP também apaga os lasers."""
        self._last_actuation_time = time.time()
        return self.send_raw("ESTOP 0" if lasers_off else "STOP 0")

    def set_alarm_z(self, enable: bool) -> bool:
        return self._unsupported("Configuração do alarme Z via Teensy")

    def home_axis(self, axis: str) -> bool:
        token = axis.strip().upper()
        if token in ("ALL", "CA"):
            axes = ("C", "A", "Z") if token == "ALL" else ("C", "A")
            return all(self.send_raw(f"H {self.target_node} {item}") for item in axes)
        try:
            canonical = self._canonical_axis(token)
        except ValueError as exc:
            self.state.error_occurred.emit(str(exc))
            return False
        return self.send_raw(f"H {self.target_node} {canonical}")

    def set_home(self, axis: str) -> bool:
        return self._unsupported("SETHOME via Teensy")

    def move_axis(
        self,
        axis: str,
        steps: int,
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        force_no_encoder: bool = False,
    ) -> bool:
        if (speed is not None and speed > 0) or (accel is not None and accel > 0):
            return self._unsupported(
                "Override de velocidade/aceleração por movimento via Teensy"
            )
        try:
            canonical = self._canonical_axis(axis)
        except ValueError as exc:
            self.state.error_occurred.emit(str(exc))
            return False
        command = "MF" if force_no_encoder else "M"
        return self.send_raw(f"{command} {self.target_node} {canonical} {int(steps)}")

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
        """Envia movimento sincronizado em coordenadas absolutas a partir de passos absolutos."""
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
        if any(
            value < -32768 or value > 32767
            for value in (angle_c_deci, angle_a_deci, distance_z_centi)
        ):
            self.state.error_occurred.emit(
                "MOVE_SYNC fora da faixa do protocolo Teensy/CAN."
            )
            return False

        command = "MSF" if force_no_encoder else "MS"
        return self.send_raw(
            f"{command} {self.target_node} {angle_c_deci / 10.0:.1f} "
            f"{angle_a_deci / 10.0:.1f} {distance_z_centi / 100.0:.2f}"
        )

    def move_sync_deg(
        self,
        c_deg: float,
        a_deg: float,
        z_mm: float,
        force_no_encoder: bool = False,
        node_id: Optional[int] = None,
    ) -> bool:
        """Envia movimento sincronizado em coordenadas absolutas físicas (graus para C/A, mm para Z)."""
        target = self.target_node if node_id is None else int(node_id)
        target = max(0, min(10, target))
        command = "MSF" if force_no_encoder else "MS"
        return self.send_raw(
            f"{command} {target} {float(c_deg):.1f} {float(a_deg):.1f} {float(z_mm):.2f}"
        )

    def send_unified(
        self,
        c_deg: float,
        a_deg: float,
        z_mm: float,
        laser1: int = 0,
        laser2: int = 0,
        force_no_encoder: bool = False,
        node_id: Optional[int] = None,
    ) -> bool:
        """Envia comando unificado com coordenadas absolutas dos 3 eixos e 2 lasers (TouchDesigner).

        :param c_deg: Coordenada angular absoluta do eixo C em graus (-3276.7 a +3276.7).
        :param a_deg: Coordenada angular absoluta do eixo A em graus (-3276.7 a +3276.7).
        :param z_mm: Coordenada linear absoluta do eixo Z em mm (0.00 a 327.67).
        :param laser1: Nível PWM 12-bit do Laser 1 (0 a 4095).
        :param laser2: Nível PWM 12-bit do Laser 2 (0 a 4095).
        :param force_no_encoder: Se True, comanda em malha aberta (UF).
        :param node_id: Nó alvo (1..10) ou 0 para broadcast. None usa self.target_node.
        """
        target = self.target_node if node_id is None else int(node_id)
        target = max(0, min(10, target))
        l1 = max(0, min(4095, int(laser1)))
        l2 = max(0, min(4095, int(laser2)))
        command = "UF" if force_no_encoder else "U"
        return self.send_raw(
            f"{command} {target} {float(c_deg):.1f} {float(a_deg):.1f} {float(z_mm):.2f} {l1} {l2}"
        )

    def set_laser(self, laser_index: int, level: int) -> bool:
        laser_index = int(laser_index)
        if laser_index not in (1, 2):
            self.state.error_occurred.emit("Canal de laser deve ser 1 ou 2.")
            return False
        level = max(0, min(4095, int(level)))
        return self.send_raw(f"L {self.target_node} {laser_index} {level}")

    def set_fan(self, mode: int) -> bool:
        mode = max(0, min(2, int(mode)))
        return self.send_raw(f"F {self.target_node} {mode}")

    def set_speed_level(self, level: int) -> bool:
        level = max(1, min(5, int(level)))
        return self.send_raw(f"S {self.target_node} {level}")

    def set_axis_speed(self, axis: str, speed: float) -> bool:
        return self._unsupported("Velocidade individual por eixo via Teensy")

    def set_axis_accel(self, axis: str, accel: float) -> bool:
        return self._unsupported("Aceleração individual por eixo via Teensy")

    @staticmethod
    def _canonical_axis(axis: str) -> str:
        token = axis.strip().upper()[:1]
        if token in ("C", "X"):
            return "C"
        if token in ("A", "Y"):
            return "A"
        if token == "Z":
            return "Z"
        raise ValueError(f"Eixo inválido: {axis!r}; use C, A ou Z.")

    # --- OTA Update Methods (via Teensy USB CDC-ACM) ---
    def ota_start(self, target_node: int, image_size: int) -> bool:
        """Inicia sessão OTA no nó via Teensy (CAN ID 0x200 ou 0x200+node)."""
        self._ota_active = True
        self._last_actuation_time = time.time()
        cmd = f"OTA_START {int(target_node)} {int(image_size)}"
        return self.send_raw(cmd)

    def ota_send_chunk(self, seq_num: int, chunk: bytes, target_node: int = 0) -> bool:
        """Transmite bloco de firmware binário sem throttling artificial de jog."""
        if not self.is_connected or not self.serial_port:
            return False
        self._last_actuation_time = time.time()
        hex_data = chunk.hex().upper()
        cmd = f"OTA_DATA {int(target_node)} {int(seq_num)} {hex_data}\r\n"
        try:
            with self.lock:
                self.serial_port.write(cmd.encode("ascii"))
                self.serial_port.flush()
            self.state.telemetry.tx_frames += 1
            return True
        except Exception as exc:
            self.state.telemetry.error_count += 1
            self.state.error_occurred.emit(f"Erro ao transmitir bloco OTA via Teensy: {exc}")
            return False

    def ota_end(self, target_node: int, checksum: int = 0) -> bool:
        """Finaliza gravação OTA e comanda validação e reboot dos nós via Teensy."""
        self._ota_active = False
        self._last_actuation_time = time.time()
        cmd = f"OTA_END {int(target_node)} {int(checksum)}"
        return self.send_raw(cmd)

    def ota_abort(self, target_node: int = 0) -> bool:
        """Cancela sessão OTA em andamento via Teensy."""
        self._ota_active = False
        self._last_actuation_time = time.time()
        cmd = f"OTA_ABORT {int(target_node)}"
        return self.send_raw(cmd)
