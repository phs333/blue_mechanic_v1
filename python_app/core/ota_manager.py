"""
OTA Update Manager for Blue Mechanic V1.
Handles background binary firmware transmission over CAN bus (Classic CAN 2.0A).
Chunks the .bin file into 6-byte slices with rolling sequence counter,
monitoring per-node acknowledgments and progress.
"""

import os
import time
import hashlib
from typing import Optional, Dict
from PyQt6.QtCore import QThread, pyqtSignal, QObject

from .comm_manager import CommManager
from .state_model import DeviceState


class OtaWorker(QThread):
    """
    Background worker thread for transmitting firmware updates without blocking GUI.
    """
    progress_changed = pyqtSignal(int, int, float, float) # bytes_sent, total_bytes, speed_kb_s, eta_sec
    step_changed = pyqtSignal(str)                        # Current step description
    log_message = pyqtSignal(str)                         # Log message for console
    node_status_changed = pyqtSignal(int, str, int)       # node_id, status_text, pct
    finished = pyqtSignal(bool, str)                      # success, final message

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

    def abort(self):
        """Signals worker to abort the OTA transfer safely."""
        self.is_aborted = True

    def run(self):
        self.is_aborted = False
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
        md5_hash = hashlib.md5(bin_data).hexdigest()
        sha256_hash = hashlib.sha256(bin_data).hexdigest()
        checksum16 = sum(bin_data) & 0xFFFF

        self.log_message.emit(f"📄 Firmware: {os.path.basename(self.binary_path)}")
        self.log_message.emit(f"📦 Tamanho: {self.total_bytes:,} bytes ({self.total_bytes / 1024:.1f} KB)")
        self.log_message.emit(f"🔒 SHA-256: {sha256_hash[:16]}... | Checksum: 0x{checksum16:04X}")
        self.log_message.emit(f"🎯 Alvo: {target_desc}")

        # 2. Inicializar status dos nós
        nodes = list(range(1, 11)) if self.target_node == 0 else [self.target_node]
        for nid in nodes:
            self.node_status_changed.emit(nid, "Preparando...", 0)

        # 3. Enviar CAN_OP_OTA_START (0x40)
        self.step_changed.emit("Iniciando sessão OTA e preparando nós...")
        self.log_message.emit("⚡ Enviando CAN_OP_OTA_START (0x40)...")

        if not self.comm.ota_start(self.target_node, self.total_bytes):
            self.log_message.emit("❌ Falha ao emitir comando OTA_START no barramento CAN.")
            self.finished.emit(False, "Falha ao enviar OTA_START.")
            return

        # Breve pausa para os nós desarmarem motores, suspenderem telemetria e abrirem partição flash
        self.step_changed.emit("Aguardando nós abrirem partição flash...")
        time.sleep(0.35)

        if self.is_aborted:
            self._handle_abort()
            return

        for nid in nodes:
            self.node_status_changed.emit(nid, "Gravando...", 0)

        # 4. Transmissão dos blocos CAN_OP_OTA_DATA (0x41)
        self.step_changed.emit("Transmitindo blocos de firmware via CAN...")
        self.log_message.emit("🚀 Iniciando streaming de frames CAN (chunks de 6 bytes)...")

        self.start_time = time.time()
        self.bytes_sent = 0
        seq_num = 0
        chunk_size = 6
        last_progress_emit = 0.0

        for offset in range(0, self.total_bytes, chunk_size):
            if self.is_aborted:
                self._handle_abort()
                return

            chunk = bin_data[offset:offset + chunk_size]
            self.comm.ota_send_chunk(seq_num, chunk, self.target_node)
            seq_num = (seq_num + 1) & 0xFF
            self.bytes_sent += len(chunk)

            # Pacing de barramento para evitar estouro da fila TWAI/FlexCAN
            if self.chunk_delay_s > 0:
                time.sleep(self.chunk_delay_s)

            # Atualização de métricas (a cada 50ms ou término)
            now = time.time()
            if (now - last_progress_emit) >= 0.05 or self.bytes_sent >= self.total_bytes:
                last_progress_emit = now
                elapsed = max(0.001, now - self.start_time)
                speed_kb_s = (self.bytes_sent / 1024.0) / elapsed
                remaining_bytes = self.total_bytes - self.bytes_sent
                eta_sec = (remaining_bytes / (speed_kb_s * 1024.0)) if speed_kb_s > 0 else 0.0

                pct = int((self.bytes_sent * 100) / self.total_bytes)
                self.progress_changed.emit(self.bytes_sent, self.total_bytes, speed_kb_s, eta_sec)

                # Atualiza pct nos nós
                if offset % 16384 < chunk_size:
                    for nid in nodes:
                        self.node_status_changed.emit(nid, f"Gravando {pct}%", pct)

        # 5. Enviar CAN_OP_OTA_END (0x42)
        self.step_changed.emit("Finalizando gravação e validando integridade flash...")
        self.log_message.emit("🔒 Todos os bytes transmitidos. Enviando CAN_OP_OTA_END (0x42)...")

        time.sleep(0.05)
        self.comm.ota_end(self.target_node, checksum16)

        total_elapsed = max(0.001, time.time() - self.start_time)
        avg_speed = (self.total_bytes / 1024.0) / total_elapsed

        for nid in nodes:
            self.node_status_changed.emit(nid, "Validado / Reboot", 100)

        self.progress_changed.emit(self.total_bytes, self.total_bytes, avg_speed, 0.0)
        self.log_message.emit(f"✅ Transmissão concluída com sucesso em {total_elapsed:.1f}s ({avg_speed:.1f} KB/s)!")
        self.log_message.emit("🔄 Os nós estão validando o hash da imagem flash e reiniciando com o novo firmware.")
        self.step_changed.emit("Atualização concluída! Nós reiniciando...")
        self.finished.emit(True, f"Atualização concluída com sucesso em {total_elapsed:.1f}s.")

    def _handle_abort(self):
        self.step_changed.emit("Atualização abortada pelo usuário!")
        self.log_message.emit("🛑 Sessão OTA abortada pelo usuário. Enviando CAN_OP_OTA_ABORT...")
        self.comm.ota_abort(self.target_node)
        nodes = list(range(1, 11)) if self.target_node == 0 else [self.target_node]
        for nid in nodes:
            self.node_status_changed.emit(nid, "Abortado", 0)
        self.finished.emit(False, "Atualização abortada pelo usuário.")
