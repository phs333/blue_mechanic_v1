"""
Main Application Window for Blue Mechanic V1.
Features a sleek sidebar navigation, header connection manager,
stacked views, and real-time status bar.
"""

import time
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QComboBox, QFrame, QStatusBar, QMessageBox, QSpinBox, QCheckBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QIcon

from python_app.ui.style import DARK_THEME_QSS
from python_app.ui.views.dashboard_view import DashboardView
from python_app.ui.views.teensy_dashboard_view import TeensyDashboardView
from python_app.ui.views.parameters_view import ParametersView
from python_app.ui.views.terminal_view import TerminalView
from python_app.ui.views.automation_view import AutomationView
from python_app.ui.views.ota_view import OtaView
from python_app.ui.widgets.kinematic_3d_view import Kinematic3DView
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState

class MainWindow(QMainWindow):
    def __init__(self, comm: CommManager, state: DeviceState):
        super().__init__()
        self.comm = comm
        self.state = state
        self.last_heartbeat_time = 0
        
        self.setWindowTitle("Blue Mechanic V1 — Control & Telemetry Suite")
        self.resize(1280, 850)
        self.setMinimumSize(1024, 700)
        
        # Apply dark theme
        self.setStyleSheet(DARK_THEME_QSS)
        
        self._init_ui()
        self._setup_signals()
        
        # Periodic UI timer for heartbeat and status check
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(500)
        
        # Auto refresh COM ports
        self._refresh_ports()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        app_layout = QHBoxLayout(central_widget)
        app_layout.setContentsMargins(0, 0, 0, 0)
        app_layout.setSpacing(0)
        
        # ==========================================
        # --- LEFT SIDEBAR NAVIGATION ---
        # ==========================================
        self.sidebar = QFrame()
        self.sidebar.setObjectName("sidebar")
        sidebar_layout = QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 16)
        sidebar_layout.setSpacing(4)
        
        lbl_brand = QLabel("⚙️ BLUE MECHANIC")
        lbl_brand.setObjectName("sidebarTitle")
        sidebar_layout.addWidget(lbl_brand)
        
        lbl_subbrand = QLabel("Hardware Suite v1.0")
        lbl_subbrand.setObjectName("sidebarSubTitle")
        sidebar_layout.addWidget(lbl_subbrand)
        
        self.nav_buttons = []
        
        self.btn_nav_dash = QPushButton("📊  Dashboard Geral")
        self.btn_nav_dash.setProperty("class", "nav-btn")
        self.btn_nav_dash.clicked.connect(lambda: self._set_page(0))
        sidebar_layout.addWidget(self.btn_nav_dash)
        self.nav_buttons.append(self.btn_nav_dash)
        
        self.btn_nav_params = QPushButton("⚙️  Parâmetros & NVS")
        self.btn_nav_params.setProperty("class", "nav-btn")
        self.btn_nav_params.clicked.connect(lambda: self._set_page(1))
        sidebar_layout.addWidget(self.btn_nav_params)
        self.nav_buttons.append(self.btn_nav_params)
        
        self.btn_nav_term = QPushButton("📟  Terminal & Sniffer")
        self.btn_nav_term.setProperty("class", "nav-btn")
        self.btn_nav_term.clicked.connect(lambda: self._set_page(2))
        sidebar_layout.addWidget(self.btn_nav_term)
        self.nav_buttons.append(self.btn_nav_term)
        
        self.btn_nav_auto = QPushButton("🔁  Automação & Testes")
        self.btn_nav_auto.setProperty("class", "nav-btn")
        self.btn_nav_auto.clicked.connect(lambda: self._set_page(3))
        sidebar_layout.addWidget(self.btn_nav_auto)
        self.nav_buttons.append(self.btn_nav_auto)

        self.btn_nav_ota = QPushButton("🚀  Atualização OTA")
        self.btn_nav_ota.setProperty("class", "nav-btn")
        self.btn_nav_ota.clicked.connect(lambda: self._set_page(4))
        sidebar_layout.addWidget(self.btn_nav_ota)
        self.nav_buttons.append(self.btn_nav_ota)
        
        # 3D Kinematics Widget below Navigation buttons
        sidebar_layout.addSpacing(8)
        self.kinematic_3d = Kinematic3DView(self.state)
        sidebar_layout.addWidget(self.kinematic_3d)
        
        sidebar_layout.addStretch()
        
        # Simulation Mode Banner in sidebar
        self.lbl_sim_notice = QLabel("Modo Simulação Disponível")
        self.lbl_sim_notice.setStyleSheet("color: #64748b; font-size: 11px; text-align: center; margin: 10px;")
        self.lbl_sim_notice.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sidebar_layout.addWidget(self.lbl_sim_notice)
        
        app_layout.addWidget(self.sidebar)
        
        # ==========================================
        # --- RIGHT CONTENT AREA ---
        # ==========================================
        content_area = QWidget()
        content_layout = QVBoxLayout(content_area)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        
        # --- Top Connection Header Bar ---
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #0f172a; border-bottom: 1px solid #1e293b;")
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(18, 10, 18, 10)
        top_bar_layout.setSpacing(12)
        
        top_bar_layout.addWidget(QLabel("Interface:"))
        self.combo_backend = QComboBox()
        self.combo_backend.addItems([
            "ESP32-S3 Serial Direta",
            "Teensy USB/CAN",
            "PeakCAN (PCAN-Basic)",
            "Simulador Virtual",
        ])
        self.combo_backend.currentIndexChanged.connect(self._on_backend_change)
        top_bar_layout.addWidget(self.combo_backend)
        
        # COM config widgets
        self.lbl_port = QLabel("Porta:")
        self.combo_ports = QComboBox()
        self.btn_refresh_ports = QPushButton("🔄")
        self.btn_refresh_ports.setToolTip("Atualizar portas")
        self.btn_refresh_ports.clicked.connect(self._refresh_ports)
        
        self.lbl_baud = QLabel("Baud:")
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "9600", "57600", "230400", "460800", "921600"])
        
        top_bar_layout.addWidget(self.lbl_port)
        top_bar_layout.addWidget(self.combo_ports)
        top_bar_layout.addWidget(self.btn_refresh_ports)
        top_bar_layout.addWidget(self.lbl_baud)
        top_bar_layout.addWidget(self.combo_baud)
        
        # CAN config widgets
        self.lbl_can_chan = QLabel("Canal CAN:", top_bar)
        self.combo_can_chan = QComboBox(top_bar)
        self.combo_can_chan.addItems(self.comm.get_available_can_channels())
        
        self.lbl_can_bit = QLabel("Bitrate:", top_bar)
        self.combo_can_bit = QComboBox(top_bar)
        self.combo_can_bit.addItems(["500000", "250000", "125000", "1000000"])

        self.lbl_can_node = QLabel("Node:", top_bar)
        self.spin_can_node = QSpinBox(top_bar)
        self.spin_can_node.setRange(1, 10)
        self.spin_can_node.setValue(self.state.parameters.node_id)
        self.spin_can_node.setToolTip(
            "Node ESP32-S3 selecionado (1..10) para PeakCAN ou Teensy USB/CAN"
        )
        
        top_bar_layout.addWidget(self.lbl_can_chan)
        top_bar_layout.addWidget(self.combo_can_chan)
        top_bar_layout.addWidget(self.lbl_can_bit)
        top_bar_layout.addWidget(self.combo_can_bit)
        top_bar_layout.addWidget(self.lbl_can_node)
        top_bar_layout.addWidget(self.spin_can_node)
        
        self.chk_broadcast = QCheckBox("📢 Broadcast (Node 0)")
        self.chk_broadcast.setToolTip(
            "Modo Broadcast Teensy: transmite comandos de atuação (M, MF, MS, H, E, S, L, F) para todos os nós (Node 0 / CAN 0x200)"
        )
        self.chk_broadcast.toggled.connect(self._on_broadcast_toggled)
        top_bar_layout.addWidget(self.chk_broadcast)
        
        self.lbl_can_chan.setVisible(False)
        self.combo_can_chan.setVisible(False)
        self.lbl_can_bit.setVisible(False)
        self.combo_can_bit.setVisible(False)
        self.lbl_can_node.setVisible(False)
        self.spin_can_node.setVisible(False)
        self.chk_broadcast.setVisible(False)
        
        top_bar_layout.addStretch()
        
        # Connect / Disconnect button
        self.btn_connect = QPushButton("🔌 Conectar")
        self.btn_connect.setProperty("class", "btn-primary")
        self.btn_connect.clicked.connect(self._toggle_connection)
        top_bar_layout.addWidget(self.btn_connect)
        
        content_layout.addWidget(top_bar)
        
        # --- Stacked Pages ---
        self.dash_stack = QStackedWidget()
        self.page_dash = DashboardView(self.comm, self.state)
        self.page_teensy_dash = TeensyDashboardView(self.comm, self.state)
        self.dash_stack.addWidget(self.page_dash)
        self.dash_stack.addWidget(self.page_teensy_dash)

        self.page_params = ParametersView(self.comm, self.state)
        self.page_term = TerminalView(self.comm, self.state)
        self.page_auto = AutomationView(self.comm, self.state)
        self.page_ota = OtaView(self.comm, self.state)
        
        self.stack = QStackedWidget()
        self.stack.addWidget(self.dash_stack)
        self.stack.addWidget(self.page_params)
        self.stack.addWidget(self.page_term)
        self.stack.addWidget(self.page_auto)
        self.stack.addWidget(self.page_ota)
        
        content_layout.addWidget(self.stack, 1)
        app_layout.addWidget(content_area, 1)
        
        # ==========================================
        # --- BOTTOM STATUS BAR ---
        # ==========================================
        status_bar = self.statusBar()
        status_bar.setStyleSheet("background-color: #0b0f19; border-top: 1px solid #1e293b; color: #94a3b8;")
        
        self.lbl_status_conn = QLabel("DESCONECTADO")
        self.lbl_status_conn.setProperty("class", "badge badge-gray")
        status_bar.addPermanentWidget(self.lbl_status_conn)
        
        self.lbl_status_nodes = QLabel("🌐 Nós: --/10")
        self.lbl_status_nodes.setStyleSheet("color: #64748b; font-size: 11px; margin-right: 15px;")
        self.lbl_status_nodes.setVisible(False)
        status_bar.addPermanentWidget(self.lbl_status_nodes)
        
        self.lbl_status_hb = QLabel("💓 Heartbeat: --")
        self.lbl_status_hb.setStyleSheet("color: #64748b; font-size: 11px; margin-right: 15px;")
        status_bar.addPermanentWidget(self.lbl_status_hb)
        
        self.lbl_status_frames = QLabel("TX: 0 | RX: 0")
        self.lbl_status_frames.setStyleSheet("color: #64748b; font-size: 11px; margin-right: 15px;")
        status_bar.addPermanentWidget(self.lbl_status_frames)
        
        self._set_page(0)

    def _setup_signals(self):
        self.state.connection_changed.connect(self._on_connection_changed)
        self.state.heartbeat_received.connect(self._on_heartbeat)
        self.state.error_occurred.connect(self._on_error)
        self.state.parameters_updated.connect(
            lambda params: self.spin_can_node.setValue(params.node_id)
        )
        self.state.nodes_summary_updated.connect(self._on_nodes_summary_updated)
        self.comm.broadcast_changed.connect(self._on_comm_broadcast_changed)
        self.spin_can_node.valueChanged.connect(self._on_spin_node_changed)
        self.page_teensy_dash.selected_node_changed.connect(self._on_node_selected_from_teensy_dash)

    def _set_page(self, index: int):
        if not (0 <= index < len(self.nav_buttons)):
            return
        if not self.nav_buttons[index].isEnabled():
            # Bloqueia navegação para telas sem suporte no modo atual
            return
        self.stack.setCurrentIndex(index)
        for i, btn in enumerate(self.nav_buttons):
            if i == index:
                btn.setProperty("class", "nav-btn active")
            else:
                btn.setProperty("class", "nav-btn")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _on_spin_node_changed(self, val: int):
        if self.combo_backend.currentIndex() == 1:
            if self.state.get_online_nodes_count() > 0 and not self.state.is_node_online(val):
                # Bloqueia foco em nó offline
                self.spin_can_node.blockSignals(True)
                self.spin_can_node.setValue(self.page_teensy_dash.selected_node)
                self.spin_can_node.blockSignals(False)
                self.statusBar().showMessage(f"⚠️ Nó {val} está offline. Foco bloqueado.", 3000)
                return
        self.comm.set_target_node(val)
        self.page_teensy_dash.select_node(val)

    def _on_node_selected_from_teensy_dash(self, node_id: int):
        self.spin_can_node.blockSignals(True)
        self.spin_can_node.setValue(node_id)
        self.spin_can_node.blockSignals(False)
        self.comm.set_target_node(node_id)

    def _on_nodes_summary_updated(self, online_count: int):
        if self.combo_backend.currentIndex() == 1:
            self.lbl_status_nodes.setText(f"🌐 Nós Online: {online_count}/10")
            if online_count > 0:
                self.lbl_status_nodes.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px; margin-right: 15px;")
            else:
                self.lbl_status_nodes.setStyleSheet("color: #64748b; font-size: 11px; margin-right: 15px;")

    def _on_backend_change(self, index: int):
        # 0 = ESP32 serial, 1 = Teensy USB/CAN, 2 = PeakCAN, 3 = Simulator
        is_serial = index in (0, 1)
        is_teensy = index == 1
        is_can = index == 2
        uses_node = index in (1, 2)
        
        self.lbl_port.setVisible(is_serial)
        self.combo_ports.setVisible(is_serial)
        self.btn_refresh_ports.setVisible(is_serial)
        self.lbl_baud.setVisible(is_serial)
        self.combo_baud.setVisible(is_serial)
        
        self.lbl_can_chan.setVisible(is_can)
        self.combo_can_chan.setVisible(is_can)
        self.lbl_can_bit.setVisible(is_can)
        self.combo_can_bit.setVisible(is_can)
        self.lbl_can_node.setVisible(uses_node)
        self.spin_can_node.setVisible(uses_node)
        self.chk_broadcast.setVisible(is_teensy)
        self.lbl_status_nodes.setVisible(is_teensy)

        # Bloqueio de telas sem suporte no modo Teensy
        if is_teensy:
            self.dash_stack.setCurrentWidget(self.page_teensy_dash)
            self.btn_nav_dash.setText("🌐  Dashboard (10 Nós)")
            
            # Telas não suportadas no modo Teensy (Params)
            self.btn_nav_params.setEnabled(False)
            self.btn_nav_params.setToolTip("Indisponível no modo Teensy (Bridge não suporta NVS/TMC)")

            # Atualização OTA dos Nós é plenamente suportada via Teensy!
            self.btn_nav_ota.setEnabled(True)
            self.btn_nav_ota.setToolTip("")

            # Se a página atual for a bloqueada (1=Params), retorna ao Dashboard
            if self.stack.currentIndex() == 1:
                self._set_page(0)
        else:
            self.dash_stack.setCurrentWidget(self.page_dash)
            self.btn_nav_dash.setText("📊  Dashboard Geral")

            # Reabilita telas
            self.btn_nav_params.setEnabled(True)
            self.btn_nav_params.setToolTip("")
            
            # OTA é suportado em Teensy (1), PeakCAN (2) e Simulador (3)
            ota_supported = index in (1, 2, 3)
            self.btn_nav_ota.setEnabled(ota_supported)
            self.btn_nav_ota.setToolTip("" if ota_supported else "OTA via CAN requer Teensy, PeakCAN ou Simulador")
            if not ota_supported and self.stack.currentIndex() == 4:
                self._set_page(0)

    def _on_broadcast_toggled(self, checked: bool):
        self.comm.set_broadcast_mode(checked)
        self.lbl_can_node.setText("Node Monit.:" if checked else "Node:")

    def _on_comm_broadcast_changed(self, enabled: bool):
        if self.chk_broadcast.isChecked() != enabled:
            self.chk_broadcast.blockSignals(True)
            self.chk_broadcast.setChecked(enabled)
            self.chk_broadcast.blockSignals(False)
            self.lbl_can_node.setText("Node Monit.:" if enabled else "Node:")

    def _refresh_ports(self):
        self.combo_ports.clear()
        ports = self.comm.get_available_com_ports()
        for p in ports:
            self.combo_ports.addItem(f"{p['device']} ({p['description']})", p['device'])
        if not ports:
            self.combo_ports.addItem("Nenhuma COM encontrada", "")

    def _toggle_connection(self):
        if self.comm.is_connected:
            self.comm.disconnect_all()
        else:
            backend_idx = self.combo_backend.currentIndex()
            if backend_idx in (0, 1):  # ESP32 serial or Teensy USB/CAN
                port = self.combo_ports.currentData() or self.combo_ports.currentText().split()[0]
                if not port or port == "Nenhuma":
                    QMessageBox.warning(self, "Aviso", "Selecione uma porta COM válida.")
                    return
                baud = int(self.combo_baud.currentText())
                if backend_idx == 0:
                    self.comm.connect_serial(port=port, baudrate=baud)
                else:
                    self.comm.connect_teensy(
                        port=port,
                        baudrate=baud,
                        node_id=self.spin_can_node.value(),
                    )
                    if self.chk_broadcast.isChecked():
                        self.comm.set_broadcast_mode(True)
            elif backend_idx == 2:  # PeakCAN
                chan = self.combo_can_chan.currentText()
                bitrate = int(self.combo_can_bit.currentText())
                params = self.state.parameters
                self.comm.connect_can(
                    channel=chan,
                    bitrate=bitrate,
                    node_id=self.spin_can_node.value(),
                    cmd_base=params.can_command_base_id,
                    status_base=params.can_status_base_id,
                    event_base=params.can_event_base_id,
                )
            else: # Simulator
                self.comm.connect_simulator()

    def _on_connection_changed(self, connected: bool, backend: str):
        if connected:
            self.btn_connect.setText("🔌 Desconectar")
            self.btn_connect.setProperty("class", "btn-danger")
            self.lbl_status_conn.setText(f"CONECTADO: {backend}")
            self.lbl_status_conn.setProperty("class", "badge badge-green")
        else:
            self.btn_connect.setText("🔌 Conectar")
            self.btn_connect.setProperty("class", "btn-primary")
            self.lbl_status_conn.setText("DESCONECTADO")
            self.lbl_status_conn.setProperty("class", "badge badge-gray")
            
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)
        self.lbl_status_conn.style().unpolish(self.lbl_status_conn)
        self.lbl_status_conn.style().polish(self.lbl_status_conn)

    def _on_heartbeat(self, node_id: int):
        self.last_heartbeat_time = time.time()
        self.lbl_status_hb.setText(f"💓 Heartbeat: Node {node_id} (Agora)")
        self.lbl_status_hb.setStyleSheet("color: #34d399; font-size: 11px; margin-right: 15px;")

    def _on_error(self, err_msg: str):
        self.statusBar().showMessage(f"⚠️ {err_msg}", 5000)

    def _on_tick(self):
        # Update frame counts
        t = self.state.telemetry
        self.lbl_status_frames.setText(f"TX: {t.tx_frames} | RX: {t.rx_frames} | Erros: {t.error_count}")
        
        # Check heartbeat timeout
        if self.last_heartbeat_time > 0 and (time.time() - self.last_heartbeat_time > 2.5):
            self.lbl_status_hb.setText(f"💓 Heartbeat: Sem sinal ({int(time.time() - self.last_heartbeat_time)}s)")
            self.lbl_status_hb.setStyleSheet("color: #f87171; font-size: 11px; margin-right: 15px;")

        # Telemetry is event-driven (ESP32 pushes updates on move finish / parameter changes)
        pass

    def closeEvent(self, event):
        try:
            if hasattr(self, "page_auto") and hasattr(self.page_auto, "worker"):
                self.page_auto.worker.stop()
            if hasattr(self, "comm"):
                self.comm.disconnect()
        except Exception:
            pass
        super().closeEvent(event)
