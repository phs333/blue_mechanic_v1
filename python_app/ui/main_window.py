"""
Main Application Window for Blue Mechanic V1.
Features a sleek sidebar navigation, header connection manager,
stacked views, and real-time status bar.
"""

import time
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QComboBox, QFrame, QStatusBar, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QIcon

from python_app.ui.style import DARK_THEME_QSS
from python_app.ui.views.dashboard_view import DashboardView
from python_app.ui.views.motion_view import MotionView
from python_app.ui.views.peripherals_view import PeripheralsView
from python_app.ui.views.parameters_view import ParametersView
from python_app.ui.views.terminal_view import TerminalView
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
        
        self.btn_nav_motion = QPushButton("🕹️  Movimentação & Jog")
        self.btn_nav_motion.setProperty("class", "nav-btn")
        self.btn_nav_motion.clicked.connect(lambda: self._set_page(1))
        sidebar_layout.addWidget(self.btn_nav_motion)
        self.nav_buttons.append(self.btn_nav_motion)
        
        self.btn_nav_periph = QPushButton("💡  Lasers & Ventoinha")
        self.btn_nav_periph.setProperty("class", "nav-btn")
        self.btn_nav_periph.clicked.connect(lambda: self._set_page(2))
        sidebar_layout.addWidget(self.btn_nav_periph)
        self.nav_buttons.append(self.btn_nav_periph)
        
        self.btn_nav_params = QPushButton("⚙️  Parâmetros & NVS")
        self.btn_nav_params.setProperty("class", "nav-btn")
        self.btn_nav_params.clicked.connect(lambda: self._set_page(3))
        sidebar_layout.addWidget(self.btn_nav_params)
        self.nav_buttons.append(self.btn_nav_params)
        
        self.btn_nav_term = QPushButton("📟  Terminal & Sniffer")
        self.btn_nav_term.setProperty("class", "nav-btn")
        self.btn_nav_term.clicked.connect(lambda: self._set_page(4))
        sidebar_layout.addWidget(self.btn_nav_term)
        self.nav_buttons.append(self.btn_nav_term)
        
        # 3D Kinematics Widget below Terminal & Sniffer button
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
        self.combo_backend.addItems(["Porta Serial (COM)", "PeakCAN (PCAN-Basic)", "Simulador Virtual"])
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
        
        top_bar_layout.addWidget(self.lbl_can_chan)
        top_bar_layout.addWidget(self.combo_can_chan)
        top_bar_layout.addWidget(self.lbl_can_bit)
        top_bar_layout.addWidget(self.combo_can_bit)
        
        self.lbl_can_chan.setVisible(False)
        self.combo_can_chan.setVisible(False)
        self.lbl_can_bit.setVisible(False)
        self.combo_can_bit.setVisible(False)
        
        top_bar_layout.addStretch()
        
        # Connect / Disconnect button
        self.btn_connect = QPushButton("🔌 Conectar")
        self.btn_connect.setProperty("class", "btn-primary")
        self.btn_connect.clicked.connect(self._toggle_connection)
        top_bar_layout.addWidget(self.btn_connect)
        
        content_layout.addWidget(top_bar)
        
        # --- Stacked Pages ---
        self.stack = QStackedWidget()
        self.page_dash = DashboardView(self.comm, self.state)
        self.page_motion = MotionView(self.comm, self.state)
        self.page_periph = PeripheralsView(self.comm, self.state)
        self.page_params = ParametersView(self.comm, self.state)
        self.page_term = TerminalView(self.comm, self.state)
        
        self.stack.addWidget(self.page_dash)
        self.stack.addWidget(self.page_motion)
        self.stack.addWidget(self.page_periph)
        self.stack.addWidget(self.page_params)
        self.stack.addWidget(self.page_term)
        
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

    def _set_page(self, index: int):
        self.stack.setCurrentIndex(index)
        for i, btn in enumerate(self.nav_buttons):
            if i == index:
                btn.setProperty("class", "nav-btn active")
            else:
                btn.setProperty("class", "nav-btn")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _on_backend_change(self, index: int):
        # 0 = COM, 1 = PeakCAN, 2 = Simulator
        is_com = (index == 0)
        is_can = (index == 1)
        
        self.lbl_port.setVisible(is_com)
        self.combo_ports.setVisible(is_com)
        self.btn_refresh_ports.setVisible(is_com)
        self.lbl_baud.setVisible(is_com)
        self.combo_baud.setVisible(is_com)
        
        self.lbl_can_chan.setVisible(is_can)
        self.combo_can_chan.setVisible(is_can)
        self.lbl_can_bit.setVisible(is_can)
        self.combo_can_bit.setVisible(is_can)

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
            if backend_idx == 0: # COM
                port = self.combo_ports.currentData() or self.combo_ports.currentText().split()[0]
                if not port or port == "Nenhuma":
                    QMessageBox.warning(self, "Aviso", "Selecione uma porta COM válida.")
                    return
                baud = int(self.combo_baud.currentText())
                self.comm.connect_serial(port=port, baudrate=baud)
            elif backend_idx == 1: # PeakCAN
                chan = self.combo_can_chan.currentText()
                bitrate = int(self.combo_can_bit.currentText())
                self.comm.connect_can(channel=chan, bitrate=bitrate)
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

        # Periodically refresh live hardware position and telemetry for Serial/COM (throttled to 1.5s)
        now = time.time()
        if self.comm.is_connected and self.comm.backend == "COM":
            if not hasattr(self, '_last_status_poll'):
                self._last_status_poll = 0.0
            if now - self._last_status_poll >= 1.5:
                self._last_status_poll = now
                self.comm.request_status()
