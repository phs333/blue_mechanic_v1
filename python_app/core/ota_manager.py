"""
OTA Update Manager for Blue Mechanic V1.
Handles background binary firmware transmission over CAN bus (Classic CAN 2.0A).
Chunks the .bin file into 6-byte slices with rolling sequence counter,
monitoring per-node acknowledgments and progress.
"""

import os
import time
import hashlib
import threading
from typing import Dict, List, Optional, Set
from PyQt6.QtCore import QThread, pyqtSignal, QObject, Qt

from .comm_manager import CommManager
from .state_model import DeviceState
from .protocol_defs import ESP_ERRORS


class OtaWorker(QThread):
    """
    Background worker thread for transmitting firmware updates without blocking GUI.

    Fluxo com confirmação dos nós:
      1. OTA_START e espera OTA_READY (o nó só responde depois de apagar a partição —
         frames enviados antes disso seriam perdidos);
      2. streaming dos blocos, abortando se algum nó reportar OTA_ERROR;
      3. OTA_END e espera OTA_DONE/OTA_ERROR de cada nó antes de declarar sucesso.
    """
    progress_changed = pyqtSignal(int, int, float, float) # bytes_sent, total_bytes, speed_kb_s, eta_sec
    step_changed = pyqtSignal(str)                        # Current step description
    log_message = pyqtSignal(str)                         # Log message for console
    node_status_changed = pyqtSignal(int, str, int)       # node_id, status_text, pct
    finished = pyqtSignal(bool, str)                      # success, final message

    READY_TIMEOUT_S = 30.0     # erase da partição de destino pode levar vários segundos
    READY_GRACE_S = 1.5        # broadcast sem lista de nós conhecida: espera outros READY
    DONE_TIMEOUT_S = 20.0      # esp_ota_end() valida o SHA-256 da imagem inteira
    MAX_SEND_FAILURES = 20     # falhas consecutivas de envio antes de abortar

    def __init__(
        self,
        comm: CommManager,
        state: DeviceState,
        binary_path: str,
        target_node: int = 0, # 0 = Broadcast (todos os 10 nós), 1..10 = nó específico
        chunk_delay_s: float = 0.0008,
        parent: Optional[QObject] = None
    ):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.binary_path = binary_path
        self.target_node = int(target_node)
        self.chunk_delay_s = chunk_delay_s
        self.is_aborted = False

        # State tracking
        self.total_bytes = 0
        self.bytes_sent = 0
        self.start_time = 0.0

        # Eventos dos nós chegam das threads de RX: coletados sob lock (DirectConnection)
        self._evt_lock = threading.Lock()
        self._ready: Dict[int, int] = {}
        self._done: Set[int] = set()
        self._errors: Dict[int, int] = {}

    def abort(self):
        """Signals worker to abort the OTA transfer safely."""
        self.is_aborted = True

    # --- Coleta de eventos dos nós (executa na thread que emitiu o sinal) ---
    def _on_node_ready(self, node_id: int, err: int):
        with self._evt_lock:
            self._ready[int(node_id)] = int(err)

    def _on_node_done(self, node_id: int):
        with self._evt_lock:
            self._done.add(int(node_id))

    def _on_node_error(self, node_id: int, err: int):
        with self._evt_lock:
            self._errors[int(node_id)] = int(err)

    def _connect_node_events(self) -> None:
        direct = Qt.ConnectionType.DirectConnection
        self.state.ota_ready.connect(self._on_node_ready, direct)
        self.state.ota_done.connect(self._on_node_done, direct)
        self.state.ota_error.connect(self._on_node_error, direct)

    def _disconnect_node_events(self) -> None:
        for signal, slot in ((self.state.ota_ready, self._on_node_ready),
                             (self.state.ota_done, self._on_node_done),
                             (self.state.ota_error, self._on_node_error)):
            try:
                signal.disconnect(slot)
            except (TypeError, RuntimeError):
                pass

    @staticmethod
    def _err_name(code: int) -> str:
        return ESP_ERRORS.get(code, f"0x{code:02X}")

    def _expected_nodes(self) -> List[int]:
        """Nós que devem responder: o alvo, ou no broadcast os nós vistos online (pode ser vazio)."""
        if self.target_node != 0:
            return [self.target_node]
        return [n for n in range(1, 11) if self.state.is_node_online(n)]

    def _wait_for_ready(self, expected: List[int]) -> List[int]:
        """Espera OTA_READY; retorna os nós prontos (lista vazia = nenhuma confirmação)."""
        deadline = time.time() + self.READY_TIMEOUT_S
        first_ready_at = 0.0
        while time.time() < deadline and not self.is_aborted:
            with self._evt_lock:
                ready = dict(self._ready)
            if expected and all(n in ready for n in expected):
                break
            if not expected and ready:
                first_ready_at = first_ready_at or time.time()
                if time.time() - first_ready_at >= self.READY_GRACE_S:
                    break
            time.sleep(0.02)
        with self._evt_lock:
            ready = dict(self._ready)
        ok_nodes = []
        for nid, err in sorted(ready.items()):
            if err == 0:
                ok_nodes.append(nid)
                self.node_status_changed.emit(nid, "Pronto (flash apagada)", 0)
            else:
                self.log_message.emit(f"❌ Node {nid} recusou o OTA: {self._err_name(err)}")
                self.node_status_changed.emit(nid, f"Erro Init ({self._err_name(err)})", 0)
        return ok_nodes

    def run(self):
        self._connect_node_events()
        try:
            self._run_transfer()
        finally:
            self._disconnect_node_events()

    def _run_transfer(self):
        self.is_aborted = False
        with self._evt_lock:
            self._ready.clear()
            self._done.clear()
            self._errors.clear()
        target_desc = "Todos os 10 Nós (Broadcast)" if self.target_node == 0 else f"Nó {self.target_node}"

        # 1. Validar e ler o arquivo binário
        if not os.path.isfile(self.binary_path):
            self.log_message.emit(f"❌ Erro: Arquivo de firmware não encontrado: {self.binary_path}")
            self.finished.emit(False, "Arquivo binário não encontrado.")
            return

        try:
            with open(self.binary_path, "rb") as f:
                bin_data = f.read()
        except Exception as e:
            self.log_message.emit(f"❌ Erro ao ler arquivo binário: {e}")
            self.finished.emit(False, f"Falha ao ler arquivo: {e}")
            return

        self.total_bytes = len(bin_data)
        if self.total_bytes == 0:
            self.log_message.emit("❌ Erro: Arquivo de firmware vazio (0 bytes).")
            self.finished.emit(False, "Arquivo de firmware vazio.")
            return

        # Calcular hashes para log e segurança
        sha256_hash = hashlib.sha256(bin_data).hexdigest()
        checksum16 = sum(bin_data) & 0xFFFF

        self.log_message.emit(f"📄 Firmware: {os.path.basename(self.binary_path)}")
        self.log_message.emit(f"📦 Tamanho: {self.total_bytes:,} bytes ({self.total_bytes / 1024:.1f} KB)")
        self.log_message.emit(f"🔒 SHA-256: {sha256_hash[:16]}... | Checksum: 0x{checksum16:04X}")
        self.log_message.emit(f"🎯 Alvo: {target_desc}")

        # 2. Inicializar status dos nós
        expected = self._expected_nodes()
        for nid in (expected or range(1, 11)):
            self.node_status_changed.emit(nid, "Preparando...", 0)

        # 3. Enviar CAN_OP_OTA_START (0x40) e aguardar OTA_READY
        self.step_changed.emit("Iniciando sessão OTA e preparando nós...")
        self.log_message.emit("⚡ Enviando CAN_OP_OTA_START (0x40)...")

        if not self.comm.ota_start(self.target_node, self.total_bytes):
            self.log_message.emit("❌ Falha ao emitir comando OTA_START no barramento CAN.")
            self.finished.emit(False, "Falha ao enviar OTA_START.")
            return

        self.step_changed.emit("Aguardando nós apagarem a partição flash...")
        active = self._wait_for_ready(expected)
        if self.is_aborted:
            self._handle_abort()
            return
        if not active:
            if expected or self.target_node != 0:
                self.log_message.emit("❌ Nenhum nó confirmou OTA_READY — transmissão cancelada.")
                self.comm.ota_abort(self.target_node)
                self.finished.emit(False, "Nenhum nó confirmou OTA_READY.")
                return
            # Broadcast sem nós conhecidos e sem READY (bridge que não repassa o evento):
            # prossegue; o firmware detecta frames perdidos pela sequência e reporta erro.
            self.log_message.emit("⚠️ Nenhum OTA_READY recebido; prosseguindo em broadcast sem confirmação.")
            active = list(range(1, 11))
        missing = [n for n in expected if n not in active]
        if missing:
            self.log_message.emit(f"⚠️ Sem confirmação dos nós {missing}; seguem fora desta atualização.")
        self.log_message.emit(f"✅ Nós prontos: {active}")

        for nid in active:
            self.node_status_changed.emit(nid, "Gravando...", 0)

        # 4. Transmissão dos blocos CAN_OP_OTA_DATA (0x41)
        self.step_changed.emit("Transmitindo blocos de firmware via CAN...")
        self.log_message.emit("🚀 Iniciando streaming de frames CAN (chunks de 6 bytes)...")

        self.start_time = time.time()
        self.bytes_sent = 0
        seq_num = 0
        chunk_size = 6
        last_progress_emit = 0.0
        send_failures = 0

        for offset in range(0, self.total_bytes, chunk_size):
            if self.is_aborted:
                self._handle_abort()
                return

            chunk = bin_data[offset:offset + chunk_size]
            if self.comm.ota_send_chunk(seq_num, chunk, self.target_node):
                send_failures = 0
            else:
                # O nó exige sequência contínua: repete o mesmo bloco em vez de pular
                send_failures += 1
                if send_failures > self.MAX_SEND_FAILURES:
                    self.log_message.emit(f"❌ {send_failures} falhas seguidas de envio no offset {offset}.")
                    self.comm.ota_abort(self.target_node)
                    self.finished.emit(False, "Falha persistente ao transmitir blocos OTA.")
                    return
                time.sleep(0.01)
                continue
            seq_num = (seq_num + 1) & 0xFF
            self.bytes_sent += len(chunk)

            # Pacing de barramento para evitar estouro da fila TWAI/FlexCAN
            if self.chunk_delay_s > 0:
                time.sleep(self.chunk_delay_s)

            # Atualização de métricas (a cada 50ms ou término) e erros reportados pelos nós
            now = time.time()
            if (now - last_progress_emit) >= 0.05 or self.bytes_sent >= self.total_bytes:
                last_progress_emit = now
                with self._evt_lock:
                    errors = {n: e for n, e in self._errors.items() if n in active}
                if errors:
                    for nid, err in errors.items():
                        self.log_message.emit(f"❌ Node {nid} reportou erro durante a gravação: {self._err_name(err)}")
                        self.node_status_changed.emit(nid, f"Erro ({self._err_name(err)})", 0)
                    active = [n for n in active if n not in errors]
                    if not active:
                        self.comm.ota_abort(self.target_node)
                        self.finished.emit(False, "Todos os nós reportaram erro durante a gravação.")
                        return

                elapsed = max(0.001, now - self.start_time)
                speed_kb_s = (self.bytes_sent / 1024.0) / elapsed
                remaining_bytes = self.total_bytes - self.bytes_sent
                eta_sec = (remaining_bytes / (speed_kb_s * 1024.0)) if speed_kb_s > 0 else 0.0

                pct = int((self.bytes_sent * 100) / self.total_bytes)
                self.progress_changed.emit(self.bytes_sent, self.total_bytes, speed_kb_s, eta_sec)

                # Atualiza pct nos nós
                if offset % 16384 < chunk_size:
                    for nid in active:
                        self.node_status_changed.emit(nid, f"Gravando {pct}%", pct)

        # 5. Enviar CAN_OP_OTA_END (0x42) e aguardar validação de cada nó
        self.step_changed.emit("Finalizando gravação e validando integridade flash...")
        self.log_message.emit("🔒 Todos os bytes transmitidos. Enviando CAN_OP_OTA_END (0x42)...")

        time.sleep(0.05)
        self.comm.ota_end(self.target_node, checksum16)

        total_elapsed = max(0.001, time.time() - self.start_time)
        avg_speed = (self.total_bytes / 1024.0) / total_elapsed
        self.progress_changed.emit(self.total_bytes, self.total_bytes, avg_speed, 0.0)

        deadline = time.time() + self.DONE_TIMEOUT_S
        while time.time() < deadline and not self.is_aborted:
            with self._evt_lock:
                pending = [n for n in active if n not in self._done and n not in self._errors]
            if not pending:
                break
            time.sleep(0.05)

        with self._evt_lock:
            done = [n for n in active if n in self._done]
            failed = {n: self._errors[n] for n in active if n in self._errors and n not in self._done}
        silent = [n for n in active if n not in done and n not in failed]

        for nid in done:
            self.node_status_changed.emit(nid, "Validado / Reboot", 100)
        for nid, err in failed.items():
            self.node_status_changed.emit(nid, f"Falhou ({self._err_name(err)})", 0)
            self.log_message.emit(f"❌ Node {nid} rejeitou a imagem: {self._err_name(err)}")
        for nid in silent:
            self.node_status_changed.emit(nid, "Sem confirmação", 0)

        if done and not failed and not silent:
            self.log_message.emit(f"✅ Transmissão concluída com sucesso em {total_elapsed:.1f}s ({avg_speed:.1f} KB/s)!")
            self.log_message.emit("🔄 Os nós validaram o hash da imagem e estão reiniciando com o novo firmware.")
            self.step_changed.emit("Atualização concluída! Nós reiniciando...")
            self.finished.emit(True, f"Atualização concluída em {total_elapsed:.1f}s ({len(done)} nó(s) confirmado(s)).")
        else:
            summary = f"Confirmados: {done or 'nenhum'} | Falharam: {sorted(failed) or 'nenhum'} | Sem resposta: {silent or 'nenhum'}"
            self.log_message.emit(f"⚠️ {summary}")
            self.step_changed.emit("Atualização incompleta")
            self.finished.emit(False, summary)

    def _handle_abort(self):
        self.step_changed.emit("Atualização abortada pelo usuário!")
        self.log_message.emit("🛑 Sessão OTA abortada pelo usuário. Enviando CAN_OP_OTA_ABORT...")
        self.comm.ota_abort(self.target_node)
        nodes = list(range(1, 11)) if self.target_node == 0 else [self.target_node]
        for nid in nodes:
            self.node_status_changed.emit(nid, "Abortado", 0)
        self.finished.emit(False, "Atualização abortada pelo usuário.")
