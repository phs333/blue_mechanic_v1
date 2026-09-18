"""
OTA Firmware Update View for Blue Mechanic V1.
Provides file selection, node targeting (Single Node or 10-Node Broadcast),
progress tracking, transmission speed, and CAN event logs.
"""

import os
import time
import hashlib
from typing import Optional, Dict

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QFileDialog, QProgressBar, QLineEdit, QComboBox, QRadioButton,
    QButtonGroup, QScrollArea, QPlainTextEdit, QMessageBox, QDoubleSpinBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QTextCursor

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.ota_manager import OtaWorker


class NodeOtaCard(QFrame):
    """Compact card representing OTA progress and state for a single node."""
    def __init__(self, node_id: int, parent=None):
        super().__init__(parent)
        self.node_id = node_id
        self.setObjectName(f"otaCard_{node_id}")
        self.setStyleSheet("""
            QFrame {
                background-color: #0b1322;
                border: 1px solid #1e293b;
                border-radius: 8px;
                padding: 6px;
            }
        """)
        self.setMinimumWidth(140)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        hdr = QHBoxLayout()
        self.lbl_title = QLabel(f"🤖 Nó {node_id:02d}")
        self.lbl_title.setStyleSheet("font-weight: 700; font-size: 11px; color: #94a3b8;")
        hdr.addWidget(self.lbl_title)
        hdr.addStretch()

        self.lbl_status = QLabel("Ocioso")
        self.lbl_status.setStyleSheet("font-size: 10px; color: #64748b;")
        hdr.addWidget(self.lbl_status)
        layout.addLayout(hdr)

        self.pbar = QProgressBar()
        self.pbar.setRange(0, 100)
        self.pbar.setValue(0)
        self.pbar.setFixedHeight(6)
        self.pbar.setTextVisible(False)
        self.pbar.setStyleSheet("""
            QProgressBar {
                background-color: #1e293b;
                border: none;
                border-radius: 3px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.pbar)

    def set_status(self, status: str, pct: int):
        self.lbl_status.setText(status)
        self.pbar.setValue(max(0, min(100, pct)))

        if "Erro" in status or "Abortado" in status:
            self.lbl_status.setStyleSheet("font-size: 10px; color: #f87171; font-weight: 600;")
            self.lbl_title.setStyleSheet("font-weight: 700; font-size: 11px; color: #f87171;")
        elif "Validado" in status or "Reboot" in status or pct == 100:
            self.lbl_status.setStyleSheet("font-size: 10px; color: #34d399; font-weight: 600;")
            self.lbl_title.setStyleSheet("font-weight: 700; font-size: 11px; color: #34d399;")
        elif pct > 0:
            self.lbl_status.setStyleSheet("font-size: 10px; color: #38bdf8; font-weight: 600;")
            self.lbl_title.setStyleSheet("font-weight: 700; font-size: 11px; color: #38bdf8;")
        else:
            self.lbl_status.setStyleSheet("font-size: 10px; color: #64748b;")
            self.lbl_title.setStyleSheet("font-weight: 700; font-size: 11px; color: #94a3b8;")


class OtaView(QWidget):
    """Comprehensive OTA Update View."""

    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.worker: Optional[OtaWorker] = None
        self.selected_file: str = ""

        self._init_ui()
        self._setup_state_signals()

    def _init_ui(self):
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(14)

        # ==========================================
        # 1. HEADER CARD
        # ==========================================
        hdr_card = QFrame()
        hdr_card.setProperty("class", "card")
        hdr_layout = QHBoxLayout(hdr_card)
        hdr_layout.setContentsMargins(16, 12, 16, 12)

        title_vbox = QVBoxLayout()
        lbl_title = QLabel("🚀 Atualização de Firmware OTA (Nós ESP32-S3)")
        lbl_title.setStyleSheet("color: #38bdf8; font-size: 17px; font-weight: 800;")
        lbl_sub = QLabel("Transmissão segura de firmware via Teensy 4.1 USB/CAN ou PeakCAN para os nós 1 a 10 com verificação flash")
        lbl_sub.setStyleSheet("color: #64748b; font-size: 12px;")
        title_vbox.addWidget(lbl_title)
        title_vbox.addWidget(lbl_sub)
        hdr_layout.addLayout(title_vbox)

        hdr_layout.addStretch()

        self.lbl_backend_badge = QLabel("Teensy USB/CAN / PeakCAN / Simulador")
        self.lbl_backend_badge.setProperty("class", "badge badge-blue")
        hdr_layout.addWidget(self.lbl_backend_badge)

        main_layout.addWidget(hdr_card)

        # ==========================================
        # 2. FILE SELECTION CARD
        # ==========================================
        file_card = QFrame()
        file_card.setProperty("class", "card")
        file_vbox = QVBoxLayout(file_card)
        file_vbox.setContentsMargins(16, 14, 16, 14)
        file_vbox.setSpacing(10)

        lbl_f_title = QLabel("1. Arquivo de Firmware Binário (.bin)")
        lbl_f_title.setStyleSheet("color: #e2e8f0; font-weight: 700; font-size: 13px;")
        file_vbox.addWidget(lbl_f_title)

        file_row = QHBoxLayout()
        file_row.setSpacing(8)

        self.txt_file_path = QLineEdit()
        self.txt_file_path.setPlaceholderText("Selecione o binário de firmware compilado pelo ESP-IDF / PlatformIO (ex: blue_mechanic.bin)...")
        self.txt_file_path.setReadOnly(True)
        file_row.addWidget(self.txt_file_path, 1)

        self.btn_browse = QPushButton("📁 Selecionar Arquivo...")
        self.btn_browse.setProperty("class", "btn-primary")
        self.btn_browse.clicked.connect(self._on_browse_file)
        file_row.addWidget(self.btn_browse)
        file_vbox.addLayout(file_row)

        # File metadata labels
        meta_row = QHBoxLayout()
        meta_row.setSpacing(16)

        self.lbl_file_size = QLabel("Tamanho: --")
        self.lbl_file_size.setStyleSheet("color: #94a3b8; font-size: 11px;")
        meta_row.addWidget(self.lbl_file_size)

        self.lbl_file_sha = QLabel("SHA-256: --")
        self.lbl_file_sha.setStyleSheet("color: #94a3b8; font-size: 11px;")
        meta_row.addWidget(self.lbl_file_sha)

        self.lbl_file_date = QLabel("Modificado: --")
        self.lbl_file_date.setStyleSheet("color: #94a3b8; font-size: 11px;")
        meta_row.addWidget(self.lbl_file_date)

        meta_row.addStretch()
        file_vbox.addLayout(meta_row)

        main_layout.addWidget(file_card)

        # ==========================================
        # 3. TARGET NODE CONFIGURATION CARD
        # ==========================================
        cfg_card = QFrame()
        cfg_card.setProperty("class", "card")
        cfg_vbox = QVBoxLayout(cfg_card)
        cfg_vbox.setContentsMargins(16, 14, 16, 14)
        cfg_vbox.setSpacing(12)

        lbl_c_title = QLabel("2. Destino da Atualização e Parâmetros")
        lbl_c_title.setStyleSheet("color: #e2e8f0; font-weight: 700; font-size: 13px;")
        cfg_vbox.addWidget(lbl_c_title)

        target_row = QHBoxLayout()
        target_row.setSpacing(18)

        self.rb_broadcast = QRadioButton("📢 Todos os 10 Nós (Broadcast — CAN ID 0x200)")
        self.rb_broadcast.setChecked(True)
        self.rb_broadcast.setToolTip("Envia blocos de firmware em broadcast simultâneo para todos os nós conectados ao barramento")
        target_row.addWidget(self.rb_broadcast)

        self.rb_single = QRadioButton("🤖 Nó Específico:")
        target_row.addWidget(self.rb_single)

        self.combo_target_node = QComboBox()
        for i in range(1, 11):
            self.combo_target_node.addItem(f"Nó {i:02d}", i)
        self.combo_target_node.setEnabled(False)
        self.rb_single.toggled.connect(lambda checked: self.combo_target_node.setEnabled(checked))
        target_row.addWidget(self.combo_target_node)

        target_row.addSpacing(20)

        lbl_delay = QLabel("Pacing CAN (ms):")
        lbl_delay.setStyleSheet("color: #94a3b8; font-size: 11px;")
        target_row.addWidget(lbl_delay)

        self.spin_delay = QDoubleSpinBox()
        self.spin_delay.setRange(0.1, 5.0)
        self.spin_delay.setSingleStep(0.1)
        self.spin_delay.setValue(0.8)
        self.spin_delay.setToolTip("Tempo de espera entre frames CAN de 6 bytes para evitar saturação do barramento")
        target_row.addWidget(self.spin_delay)

        target_row.addStretch()
        cfg_vbox.addLayout(target_row)

        # Safety Notice Box
        advisory_box = QFrame()
        advisory_box.setStyleSheet("""
            QFrame {
                background-color: rgba(245, 158, 11, 0.08);
                border: 1px solid rgba(245, 158, 11, 0.3);
                border-radius: 6px;
                padding: 8px;
            }
        """)
        adv_layout = QHBoxLayout(advisory_box)
        adv_layout.setContentsMargins(10, 6, 10, 6)
        lbl_adv_icon = QLabel("⚠️")
        lbl_adv_icon.setStyleSheet("font-size: 16px;")
        adv_layout.addWidget(lbl_adv_icon)
        lbl_adv_text = QLabel(
            "<b>Protocolo de Segurança Ativo:</b> Durante a sessão OTA, os motores serão automaticamente desarmados, "
            "os lasers desligados e a telemetria suspensa. Ao término da transmissão, cada nó valida o hash de integridade "
            "da imagem na flash e reinicia autonomamente na nova partição."
        )
        lbl_adv_text.setStyleSheet("color: #fcd34d; font-size: 11px;")
        lbl_adv_text.setWordWrap(True)
        adv_layout.addWidget(lbl_adv_text, 1)
        cfg_vbox.addWidget(advisory_box)

        main_layout.addWidget(cfg_card)

        # ==========================================
        # 4. PROGRESS & TRANSMISSION CONTROLS CARD
        # ==========================================
        prog_card = QFrame()
        prog_card.setProperty("class", "card")
        prog_vbox = QVBoxLayout(prog_card)
        prog_vbox.setContentsMargins(16, 14, 16, 14)
        prog_vbox.setSpacing(12)

        prog_hdr = QHBoxLayout()
        lbl_p_title = QLabel("3. Progresso da Transmissão")
        lbl_p_title.setStyleSheet("color: #e2e8f0; font-weight: 700; font-size: 13px;")
        prog_hdr.addWidget(lbl_p_title)

        prog_hdr.addStretch()

        self.lbl_state_badge = QLabel("AGUARDANDO INÍCIO")
        self.lbl_state_badge.setProperty("class", "badge badge-gray")
        prog_hdr.addWidget(self.lbl_state_badge)
        prog_vbox.addLayout(prog_hdr)

        # Large Main Progress Bar
        self.main_pbar = QProgressBar()
        self.main_pbar.setRange(0, 100)
        self.main_pbar.setValue(0)
        self.main_pbar.setFixedHeight(22)
        self.main_pbar.setStyleSheet("""
            QProgressBar {
                background-color: #0b111e;
                border: 1px solid #1e293b;
                border-radius: 6px;
                text-align: center;
                color: #ffffff;
                font-weight: 700;
                font-size: 11px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
                border-radius: 5px;
            }
        """)
        prog_vbox.addWidget(self.main_pbar)

        # Live Metrics Grid
        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(16)

        self.lbl_metric_bytes = QLabel("Dados: 0 / 0 KB (0%)")
        self.lbl_metric_bytes.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        metrics_row.addWidget(self.lbl_metric_bytes)

        self.lbl_metric_speed = QLabel("Velocidade: 0.0 KB/s")
        self.lbl_metric_speed.setStyleSheet("color: #38bdf8; font-size: 12px; font-weight: 600;")
        metrics_row.addWidget(self.lbl_metric_speed)

        self.lbl_metric_eta = QLabel("Estimativa Restante: --")
        self.lbl_metric_eta.setStyleSheet("color: #94a3b8; font-size: 12px;")
        metrics_row.addWidget(self.lbl_metric_eta)

        metrics_row.addStretch()

        # Action Buttons
        self.btn_start = QPushButton("⚡ Iniciar Gravação OTA")
        self.btn_start.setProperty("class", "btn-success")
        self.btn_start.setFixedHeight(34)
        self.btn_start.clicked.connect(self._on_start_ota)
        metrics_row.addWidget(self.btn_start)

        self.btn_abort = QPushButton("🛑 Abortar OTA")
        self.btn_abort.setProperty("class", "btn-danger")
        self.btn_abort.setFixedHeight(34)
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self._on_abort_ota)
        metrics_row.addWidget(self.btn_abort)

        prog_vbox.addLayout(metrics_row)
        main_layout.addWidget(prog_card)

        # ==========================================
        # 5. 10-NODE OTA STATUS GRID
        # ==========================================
        grid_card = QFrame()
        grid_card.setProperty("class", "card")
        grid_vbox = QVBoxLayout(grid_card)
        grid_vbox.setContentsMargins(14, 12, 14, 12)
        grid_vbox.setSpacing(10)

        lbl_grid_hdr = QLabel("4. Status dos 10 Nós na Rede CAN:")
        lbl_grid_hdr.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
        grid_vbox.addWidget(lbl_grid_hdr)

        self.node_cards: Dict[int, NodeOtaCard] = {}
        nodes_grid = QGridLayout()
        nodes_grid.setSpacing(8)

        for i in range(1, 11):
            row = (i - 1) // 5
            col = (i - 1) % 5
            card = NodeOtaCard(i)
            self.node_cards[i] = card
            nodes_grid.addWidget(card, row, col)

        grid_vbox.addLayout(nodes_grid)
        main_layout.addWidget(grid_card)

        # ==========================================
        # 6. OTA EVENT LOG CONSOLE
        # ==========================================
        log_card = QFrame()
        log_card.setProperty("class", "card")
        log_vbox = QVBoxLayout(log_card)
        log_vbox.setContentsMargins(14, 12, 14, 12)
        log_vbox.setSpacing(8)

        log_hdr = QHBoxLayout()
        lbl_l_title = QLabel("5. Registro de Eventos e Auditoria OTA")
        lbl_l_title.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        log_hdr.addWidget(lbl_l_title)

        log_hdr.addStretch()

        btn_clear_log = QPushButton("Limpar Log")
        btn_clear_log.setStyleSheet("padding: 2px 8px; font-size: 11px;")
        btn_clear_log.clicked.connect(lambda: self.txt_log.clear())
        log_hdr.addWidget(btn_clear_log)
        log_vbox.addLayout(log_hdr)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(140)
        self.txt_log.setStyleSheet("""
            QPlainTextEdit {
                background-color: #060910;
                color: #38bdf8;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
                border: 1px solid #1e293b;
                border-radius: 6px;
            }
        """)
        log_vbox.addWidget(self.txt_log)
        main_layout.addWidget(log_card)

        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)

    def _setup_state_signals(self):
        self.state.ota_ready.connect(self._on_ota_ready)
        self.state.ota_progress.connect(self._on_ota_node_progress)
        self.state.ota_done.connect(self._on_ota_node_done)
        self.state.ota_error.connect(self._on_ota_node_error)
        self.state.connection_changed.connect(self._on_connection_changed)
        # Sincroniza estado de conexão inicial
        if hasattr(self.comm, "backend"):
            self._on_connection_changed(self.comm.is_connected, self.comm.backend)

    def _on_connection_changed(self, connected: bool, backend: str):
        if connected:
            self.lbl_backend_badge.setText(f"Conectado: {backend}")
            self.lbl_backend_badge.setProperty("class", "badge badge-green")
        else:
            self.lbl_backend_badge.setText("DESCONECTADO")
            self.lbl_backend_badge.setProperty("class", "badge badge-gray")
        self.lbl_backend_badge.style().unpolish(self.lbl_backend_badge)
        self.lbl_backend_badge.style().polish(self.lbl_backend_badge)

    def _append_log(self, text: str):
        now_str = time.strftime("%H:%M:%S")
        self.txt_log.appendPlainText(f"[{now_str}] {text}")
        self.txt_log.moveCursor(QTextCursor.MoveOperation.End)

    def _on_browse_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selecionar Firmware Binário",
            "",
            "Binários ESP32 (*.bin);;Todos os Arquivos (*.*)"
        )
        if not file_path:
            return

        self.selected_file = file_path
        self.txt_file_path.setText(file_path)

        try:
            stat = os.stat(file_path)
            size_kb = stat.st_size / 1024.0
            self.lbl_file_size.setText(f"Tamanho: {stat.st_size:,} B ({size_kb:.1f} KB)")

            mtime_str = time.strftime("%d/%m/%Y %H:%M:%S", time.localtime(stat.st_mtime))
            self.lbl_file_date.setText(f"Modificado: {mtime_str}")

            with open(file_path, "rb") as f:
                sha = hashlib.sha256(f.read()).hexdigest()
            self.lbl_file_sha.setText(f"SHA-256: {sha[:12]}...")

            self._append_log(f"Arquivo selecionado: {os.path.basename(file_path)} ({size_kb:.1f} KB)")
        except Exception as e:
            self._append_log(f"Erro ao analisar arquivo: {e}")

    def _on_start_ota(self):
        if not self.comm.is_connected:
            QMessageBox.warning(self, "Não Conectado", "Conecte-se primeiro à interface (Teensy USB/CAN, PeakCAN ou Simulador) antes de iniciar a gravação OTA.")
            return

        if not self.selected_file or not os.path.isfile(self.selected_file):
            QMessageBox.warning(self, "Aviso", "Por favor, selecione um arquivo de firmware (.bin) válido.")
            return

        target_node = 0 if self.rb_broadcast.isChecked() else self.combo_target_node.currentData()
        target_str = "TODOS OS 10 NÓS (Broadcast)" if target_node == 0 else f"Nó {target_node}"

        resp = QMessageBox.question(
            self,
            "Confirmar Atualização OTA",
            f"Deseja iniciar a gravação de firmware OTA para:\n\n"
            f"• Destino: {target_str}\n"
            f"• Arquivo: {os.path.basename(self.selected_file)}\n\n"
            f"Os motores e lasers dos nós serão desligados durante a atualização.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        if resp != QMessageBox.StandardButton.Yes:
            return

        # UI State update
        self.btn_start.setEnabled(False)
        self.btn_abort.setEnabled(True)
        self.btn_browse.setEnabled(False)
        self.main_pbar.setValue(0)
        self.lbl_state_badge.setText("TRANSMITINDO")
        self.lbl_state_badge.setProperty("class", "badge badge-blue")
        self.lbl_state_badge.style().unpolish(self.lbl_state_badge)
        self.lbl_state_badge.style().polish(self.lbl_state_badge)

        delay_s = self.spin_delay.value() / 1000.0

        # Start background worker
        self.worker = OtaWorker(
            comm=self.comm,
            state=self.state,
            binary_path=self.selected_file,
            target_node=target_node,
            chunk_delay_s=delay_s,
            parent=self
        )
        self.worker.progress_changed.connect(self._on_worker_progress)
        self.worker.step_changed.connect(self._on_worker_step)
        self.worker.log_message.connect(self._append_log)
        self.worker.node_status_changed.connect(self._on_worker_node_status)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.start()

    def _on_abort_ota(self):
        if self.worker and self.worker.isRunning():
            self._append_log("Enviando solicitação de cancelamento OTA...")
            self.worker.abort()
            self.btn_abort.setEnabled(False)

    def _on_worker_progress(self, sent: int, total: int, speed: float, eta: float):
        pct = int((sent * 100) / total) if total > 0 else 0
        self.main_pbar.setValue(pct)
        self.lbl_metric_bytes.setText(f"Dados: {sent / 1024:.1f} / {total / 1024:.1f} KB ({pct}%)")
        self.lbl_metric_speed.setText(f"Velocidade: {speed:.1f} KB/s")
        eta_str = f"{int(eta)}s" if eta < 60 else f"{int(eta // 60)}m {int(eta % 60)}s"
        self.lbl_metric_eta.setText(f"Estimativa Restante: {eta_str}")

    def _on_worker_step(self, desc: str):
        self.lbl_state_badge.setText(desc.upper()[:24])

    def _on_worker_node_status(self, node_id: int, status: str, pct: int):
        if node_id in self.node_cards:
            self.node_cards[node_id].set_status(status, pct)

    def _on_worker_finished(self, success: bool, msg: str):
        self.btn_start.setEnabled(True)
        self.btn_abort.setEnabled(False)
        self.btn_browse.setEnabled(True)

        if success:
            self.lbl_state_badge.setText("CONCLUÍDO")
            self.lbl_state_badge.setProperty("class", "badge badge-green")
            self.main_pbar.setValue(100)
            QMessageBox.information(self, "OTA Concluída", f"Atualização de firmware finalizada com sucesso!\n\n{msg}")
        else:
            self.lbl_state_badge.setText("FALHA / ABORTADO")
            self.lbl_state_badge.setProperty("class", "badge badge-red")
            QMessageBox.warning(self, "Aviso OTA", f"A atualização foi interrompida:\n\n{msg}")

        self.lbl_state_badge.style().unpolish(self.lbl_state_badge)
        self.lbl_state_badge.style().polish(self.lbl_state_badge)

    def _on_ota_ready(self, node_id: int, err: int):
        status_text = "Pronto (Flash OK)" if err == 0 else f"Erro Init ({err})"
        self._append_log(f"📡 Node {node_id} respondeu CAN_EVT_OTA_READY: {status_text}")
        if node_id in self.node_cards:
            self.node_cards[node_id].set_status(status_text, 0)

    def _on_ota_node_progress(self, node_id: int, pct: int):
        self._append_log(f"📈 Node {node_id} progresso flash: {pct}%")
        if node_id in self.node_cards:
            self.node_cards[node_id].set_status(f"Gravando {pct}%", pct)

    def _on_ota_node_done(self, node_id: int):
        self._append_log(f"🎉 Node {node_id} concluiu e reiniciou (CAN_EVT_OTA_DONE)")
        if node_id in self.node_cards:
            self.node_cards[node_id].set_status("Reiniciado", 100)

    def _on_ota_node_error(self, node_id: int, err: int):
        self._append_log(f"❌ Node {node_id} reportou CAN_EVT_OTA_ERROR: código {err}")
        if node_id in self.node_cards:
            self.node_cards[node_id].set_status(f"Erro ({err})", 0)
