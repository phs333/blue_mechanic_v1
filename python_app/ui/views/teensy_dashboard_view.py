"""
Teensy Multi-Node Fleet Dashboard View for Blue Mechanic V1.
Displays real-time telemetry, kinematics, thermal status, drivers,
and lasers across all 10 ESP32-S3 nodes connected via Teensy 4.1 CAN bridge.
"""

import time
from typing import Dict, Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QScrollArea, QProgressBar, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState, HardwareTelemetry
from python_app.ui.widgets.jog_pad import JogPad
from python_app.ui.widgets.laser_slider import LaserSlider
from python_app.ui.theme import add_class


class NodeCardWidget(QFrame):
    """Visual card displaying live telemetry and health of a single ESP32-S3 node."""

    def __init__(self, node_id: int, on_select_callback, parent=None):
        super().__init__(parent)
        self.node_id = node_id
        self.on_select = on_select_callback
        self.is_selected = False
        self.is_online = False

        self.setObjectName(f"nodeCard_{node_id}")
        self.setProperty("class", "node-card-offline")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setMinimumWidth(210)
        self.setMinimumHeight(180)

        card_layout = QVBoxLayout(self)
        card_layout.setContentsMargins(10, 9, 10, 9)
        card_layout.setSpacing(6)

        # Header Row: Node Title + Driver Pill + Status Badge
        hdr_layout = QHBoxLayout()
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        hdr_layout.setSpacing(4)

        self.lbl_node_title = QLabel(f"🤖 NÓ {node_id:02d}")
        self.lbl_node_title.setStyleSheet("color: #64748b; font-weight: 800; font-size: 13px;")
        hdr_layout.addWidget(self.lbl_node_title)

        hdr_layout.addStretch()

        self.lbl_drivers = QLabel("⚡ OFF")
        self.lbl_drivers.setStyleSheet(
            "color: #64748b; font-size: 10px; font-weight: 600; "
            "background: rgba(100, 116, 139, 0.12); border: 1px solid #334155; "
            "border-radius: 4px; padding: 1px 4px;"
        )
        hdr_layout.addWidget(self.lbl_drivers)

        self.lbl_badge = QLabel("○ OFFLINE")
        self.lbl_badge.setProperty("class", "badge badge-gray")
        self.lbl_badge.setStyleSheet("font-size: 10px; padding: 2px 6px;")
        hdr_layout.addWidget(self.lbl_badge)

        card_layout.addLayout(hdr_layout)

        # 3-Column Kinematic Tiles Row
        kin_row = QHBoxLayout()
        kin_row.setContentsMargins(0, 0, 0, 0)
        kin_row.setSpacing(4)

        # Tile C (Base)
        self.tile_c = QFrame()
        self.tile_c.setProperty("class", "node-tile")
        tile_c_layout = QVBoxLayout(self.tile_c)
        tile_c_layout.setContentsMargins(2, 3, 2, 3)
        tile_c_layout.setSpacing(1)
        lbl_c_tag = QLabel("BASE C")
        lbl_c_tag.setStyleSheet("color: #64748b; font-size: 9px; font-weight: 700;")
        lbl_c_tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_pos_c = QLabel("--")
        self.lbl_pos_c.setStyleSheet("color: #64748b; font-weight: 800; font-size: 12px; font-family: 'Consolas', monospace;")
        self.lbl_pos_c.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tile_c_layout.addWidget(lbl_c_tag)
        tile_c_layout.addWidget(self.lbl_pos_c)
        kin_row.addWidget(self.tile_c, 1)

        # Tile A (Pivot)
        self.tile_a = QFrame()
        self.tile_a.setProperty("class", "node-tile")
        tile_a_layout = QVBoxLayout(self.tile_a)
        tile_a_layout.setContentsMargins(2, 3, 2, 3)
        tile_a_layout.setSpacing(1)
        lbl_a_tag = QLabel("PIVOT A")
        lbl_a_tag.setStyleSheet("color: #64748b; font-size: 9px; font-weight: 700;")
        lbl_a_tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_pos_a = QLabel("--")
        self.lbl_pos_a.setStyleSheet("color: #64748b; font-weight: 800; font-size: 12px; font-family: 'Consolas', monospace;")
        self.lbl_pos_a.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tile_a_layout.addWidget(lbl_a_tag)
        tile_a_layout.addWidget(self.lbl_pos_a)
        kin_row.addWidget(self.tile_a, 1)

        # Tile Z (Linear)
        self.tile_z = QFrame()
        self.tile_z.setProperty("class", "node-tile")
        tile_z_layout = QVBoxLayout(self.tile_z)
        tile_z_layout.setContentsMargins(2, 3, 2, 3)
        tile_z_layout.setSpacing(1)
        lbl_z_tag = QLabel("ALTURA Z")
        lbl_z_tag.setStyleSheet("color: #64748b; font-size: 9px; font-weight: 700;")
        lbl_z_tag.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_pos_z = QLabel("0.0 mm\n(0p)")
        self.lbl_pos_z.setStyleSheet("color: #64748b; font-weight: 700; font-size: 10px; font-family: 'Consolas', monospace;")
        self.lbl_pos_z.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tile_z_layout.addWidget(lbl_z_tag)
        tile_z_layout.addWidget(self.lbl_pos_z)
        kin_row.addWidget(self.tile_z, 1)

        card_layout.addLayout(kin_row)

        # Footer Row: Sensors and Lasers
        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(0, 0, 0, 0)
        footer_layout.setSpacing(6)

        self.lbl_temp = QLabel("🌡️ -- °C")
        add_class(self.lbl_temp, "hint")
        footer_layout.addWidget(self.lbl_temp)

        footer_layout.addStretch()

        self.lbl_lasers = QLabel("💡 L1: 0% · L2: 0%")
        self.lbl_lasers.setStyleSheet("color: #64748b; font-size: 10px;")
        footer_layout.addWidget(self.lbl_lasers)

        card_layout.addLayout(footer_layout)

        # Heartbeat & Activity line
        self.lbl_heartbeat = QLabel("💓 Sem resposta")
        self.lbl_heartbeat.setStyleSheet("color: #475569; font-size: 10px;")
        card_layout.addWidget(self.lbl_heartbeat)
        
        self.setCursor(Qt.CursorShape.ForbiddenCursor)
        self.setToolTip(f"Nó {node_id} Offline — foco bloqueado")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if not self.is_online:
                # Bloqueia foco em nós offline
                return
            self.on_select(self.node_id)
        super().mousePressEvent(event)

    def set_selected(self, selected: bool):
        self.is_selected = selected
        nid = self.node_id
        if selected:
            # Scoped strictly to outer frame id to prevent leaking to child labels/frames
            self.setStyleSheet(
                f"QFrame#nodeCard_{nid} {{ "
                f"background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #102644, stop:1 #0b182d); "
                f"border: 2px solid #38bdf8; "
                f"border-radius: 12px; "
                f"}}"
            )
            self.lbl_node_title.setText(f"🤖 NÓ {nid:02d} ★")
            self.lbl_node_title.setStyleSheet("color: #38bdf8; font-weight: 800; font-size: 13px;")
        else:
            self.setStyleSheet("")
            self.lbl_node_title.setText(f"🤖 NÓ {nid:02d}")
            if self.is_online:
                self.lbl_node_title.setStyleSheet("color: #e2e8f0; font-weight: 800; font-size: 13px;")
            else:
                self.lbl_node_title.setStyleSheet("color: #64748b; font-weight: 700; font-size: 13px;")

    def update_data(self, t: HardwareTelemetry, is_online: bool):
        self.is_online = is_online

        # Card style class: online (vibrant) vs offline (dimmed)
        target_class = "node-card-online" if is_online else "node-card-offline"
        if self.property("class") != target_class:
            self.setProperty("class", target_class)
            self.style().unpolish(self)
            self.style().polish(self)

        if is_online:
            self.setCursor(Qt.CursorShape.PointingHandCursor)
            self.setToolTip(f"Nó {self.node_id} Online — clique para focar")
        else:
            self.setCursor(Qt.CursorShape.ForbiddenCursor)
            self.setToolTip(f"Nó {self.node_id} Offline — foco bloqueado")

        # Title color when not focused
        if not self.is_selected:
            self.lbl_node_title.setStyleSheet(
                "color: #e2e8f0; font-weight: 800; font-size: 13px;" if is_online
                else "color: #64748b; font-weight: 700; font-size: 13px;"
            )

        # Status badge logic (compact text to avoid truncation)
        if not is_online:
            self.lbl_badge.setText("○ OFFLINE")
            self.lbl_badge.setProperty("class", "badge badge-gray")
            self.lbl_badge.setToolTip("Nó desconectado do barramento CAN")
        elif t.z_bloqueado:
            self.lbl_badge.setText("● Z-BLOQ")
            self.lbl_badge.setProperty("class", "badge badge-amber")
            self.lbl_badge.setToolTip("Nó online. Homing do eixo Z pendente (H <node> Z)")
        else:
            self.lbl_badge.setText("● ONLINE")
            self.lbl_badge.setProperty("class", "badge badge-green")
            self.lbl_badge.setToolTip("Nó online e pronto para movimentação")
        self.lbl_badge.style().unpolish(self.lbl_badge)
        self.lbl_badge.style().polish(self.lbl_badge)

        # Drivers pill
        if t.drivers_enabled:
            self.lbl_drivers.setText("⚡ ON")
            self.lbl_drivers.setStyleSheet(
                "color: #34d399; font-size: 10px; font-weight: 700; "
                "background: rgba(16, 185, 129, 0.15); border: 1px solid #059669; "
                "border-radius: 4px; padding: 1px 5px;"
            )
        else:
            self.lbl_drivers.setText("⚡ OFF")
            self.lbl_drivers.setStyleSheet(
                "color: #64748b; font-size: 10px; font-weight: 600; "
                "background: rgba(100, 116, 139, 0.12); border: 1px solid #334155; "
                "border-radius: 4px; padding: 1px 5px;"
            )

        # Kinematics readouts
        if t.pos_c_valid:
            self.lbl_pos_c.setText(f"{t.pos_c_deg:.2f}°")
            self.lbl_pos_c.setStyleSheet("color: #38bdf8; font-weight: 800; font-size: 12px; font-family: 'Consolas', monospace;")
        else:
            self.lbl_pos_c.setText("--")
            self.lbl_pos_c.setStyleSheet("color: #64748b; font-weight: 700; font-size: 12px; font-family: 'Consolas', monospace;")

        if t.pos_a_valid:
            self.lbl_pos_a.setText(f"{t.pos_a_deg:.2f}°")
            self.lbl_pos_a.setStyleSheet("color: #c084fc; font-weight: 800; font-size: 12px; font-family: 'Consolas', monospace;")
        else:
            self.lbl_pos_a.setText("--")
            self.lbl_pos_a.setStyleSheet("color: #64748b; font-weight: 700; font-size: 12px; font-family: 'Consolas', monospace;")

        pos_z_mm = t.pos_z_mm
        if is_online:
            self.lbl_pos_z.setText(f"{pos_z_mm:.1f}mm\n({t.pos_z_steps}p)")
            self.lbl_pos_z.setStyleSheet("color: #34d399; font-weight: 800; font-size: 10px; font-family: 'Consolas', monospace;")
        else:
            self.lbl_pos_z.setText("0.0mm\n(0p)")
            self.lbl_pos_z.setStyleSheet("color: #64748b; font-weight: 700; font-size: 10px; font-family: 'Consolas', monospace;")

        # Temperature
        if t.temp_valid:
            self.lbl_temp.setText(f"🌡️ {t.temperature_c:.1f}°C")
            if t.temperature_c > 45.0:
                self.lbl_temp.setStyleSheet("color: #f87171; font-weight: 700; font-size: 11px;")
            else:
                self.lbl_temp.setStyleSheet("color: #34d399; font-weight: 700; font-size: 11px;")
        else:
            self.lbl_temp.setText("🌡️ -- °C")
            add_class(self.lbl_temp, "hint")

        # Lasers
        pct1 = int((t.laser1_level / 4095.0) * 100)
        pct2 = int((t.laser2_level / 4095.0) * 100)
        if pct1 > 0 or pct2 > 0:
            self.lbl_lasers.setText(f"💡 L1: {pct1}% · L2: {pct2}%")
            self.lbl_lasers.setStyleSheet("color: #fbbf24; font-weight: 700; font-size: 10px;")
        else:
            self.lbl_lasers.setText(f"💡 L1: 0% · L2: 0%")
            self.lbl_lasers.setStyleSheet("color: #64748b; font-size: 10px;")

        # Heartbeat timestamp
        now = time.time()
        last = max(t.last_heartbeat_timestamp, t.last_seen_timestamp)
        if last > 0 and is_online:
            diff = now - last
            if diff < 2.5:
                self.lbl_heartbeat.setText(f"💓 Há {diff:.1f}s")
                self.lbl_heartbeat.setStyleSheet("color: #34d399; font-size: 10px; font-weight: 600;")
            elif diff < 10.0:
                self.lbl_heartbeat.setText(f"💓 Há {int(diff)}s")
                self.lbl_heartbeat.setStyleSheet("color: #fbbf24; font-size: 10px;")
            else:
                self.lbl_heartbeat.setText("💓 Sem sinal recente")
                self.lbl_heartbeat.setStyleSheet("color: #f87171; font-size: 10px;")
        else:
            self.lbl_heartbeat.setText("💓 Sem resposta")
            self.lbl_heartbeat.setStyleSheet("color: #475569; font-size: 10px;")


class TeensyDashboardView(QWidget):
    """
    Dedicated Fleet Dashboard View for Teensy USB/CAN connection.
    Polls and displays all 10 nodes simultaneously with fleet quick actions.
    """
    selected_node_changed = pyqtSignal(int)

    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.selected_node = 1
        self.node_cards: Dict[int, NodeCardWidget] = {}

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(14)

        # ==========================================
        # 1. FLEET HEADER & MASTER ACTIONS
        # ==========================================
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QVBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)
        header_layout.setSpacing(10)

        top_row = QHBoxLayout()
        title_box = QVBoxLayout()
        lbl_title = QLabel("🎛️ Painel dos 10 Nós — Teensy / CAN")
        lbl_title.setStyleSheet("color: #38bdf8; font-size: 17px; font-weight: 800;")
        lbl_sub = QLabel("Monitoramento contínuo em tempo real de todos os 10 nós via Teensy 4.1 USB/CAN Bridge")
        add_class(lbl_sub, "hint")
        title_box.addWidget(lbl_title)
        title_box.addWidget(lbl_sub)
        top_row.addLayout(title_box)

        top_row.addStretch()

        self.lbl_fleet_online = QLabel("🟢 Nós Online: 0 / 10")
        self.lbl_fleet_online.setProperty("class", "badge badge-blue")
        self.lbl_fleet_online.setStyleSheet("font-size: 12px; padding: 6px 12px;")
        top_row.addWidget(self.lbl_fleet_online)

        self.lbl_fleet_mode = QLabel("🎯 Foco: Node 1")
        self.lbl_fleet_mode.setProperty("class", "badge badge-purple")
        self.lbl_fleet_mode.setStyleSheet("font-size: 12px; padding: 6px 12px;")
        top_row.addWidget(self.lbl_fleet_mode)

        header_layout.addLayout(top_row)

        # Master Actions Buttons
        actions_row = QHBoxLayout()
        actions_row.setSpacing(8)

        lbl_acts = QLabel("Comandos Globais (Todos os Nós):")
        lbl_acts.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px;")
        actions_row.addWidget(lbl_acts)

        self.btn_fleet_enable = QPushButton("⚡ Ligar Todos Drivers (E 0 1)")
        self.btn_fleet_enable.setProperty("class", "btn-success")
        self.btn_fleet_enable.clicked.connect(lambda: self.comm.send_raw("E 0 1"))
        actions_row.addWidget(self.btn_fleet_enable)

        self.btn_fleet_disable = QPushButton("⚡ Desligar Todos (E 0 0)")
        self.btn_fleet_disable.clicked.connect(lambda: self.comm.send_raw("E 0 0"))
        actions_row.addWidget(self.btn_fleet_disable)

        self.btn_fleet_home = QPushButton("🏠 Home Todos (H 0 ALL)")
        self.btn_fleet_home.setProperty("class", "btn-primary")
        self.btn_fleet_home.clicked.connect(self._home_all_nodes)
        actions_row.addWidget(self.btn_fleet_home)

        self.btn_fleet_lasers_off = QPushButton("💡 Apagar Todos Lasers (L 0)")
        self.btn_fleet_lasers_off.clicked.connect(self._lasers_off_all)
        actions_row.addWidget(self.btn_fleet_lasers_off)

        self.btn_poll_all = QPushButton("🔄 Polling Forçado (1..10)")
        self.btn_poll_all.clicked.connect(self._poll_all_nodes)
        actions_row.addWidget(self.btn_poll_all)

        actions_row.addStretch()
        header_layout.addLayout(actions_row)

        main_layout.addWidget(header_card)

        # ==========================================
        # 2. 10-NODE FLEET GRID
        # ==========================================
        grid_frame = QFrame()
        grid_frame.setProperty("class", "card")
        grid_vbox = QVBoxLayout(grid_frame)
        grid_vbox.setContentsMargins(14, 12, 14, 12)
        grid_vbox.setSpacing(10)

        lbl_grid_title = QLabel("Status dos Nós 1 a 10 no Barramento CAN (Clique em um nó para focar):")
        add_class(lbl_grid_title, "accent")
        grid_vbox.addWidget(lbl_grid_title)

        nodes_grid = QGridLayout()
        nodes_grid.setSpacing(10)

        for i in range(1, 11):
            row = (i - 1) // 5
            col = (i - 1) % 5
            card = NodeCardWidget(i, self.select_node)
            self.node_cards[i] = card
            nodes_grid.addWidget(card, row, col)

        grid_vbox.addLayout(nodes_grid)
        main_layout.addWidget(grid_frame)

        # ==========================================
        # 3. FOCUSED NODE QUICK CONTROL & JOG
        # ==========================================
        self.focus_frame = QFrame()
        self.focus_frame.setProperty("class", "card")
        focus_layout = QHBoxLayout(self.focus_frame)
        focus_layout.setContentsMargins(14, 12, 14, 12)
        focus_layout.setSpacing(16)

        # Left: Jog pad targeting focused node
        self.jog_pad = JogPad(self.state)
        self.jog_pad.jog_requested.connect(self._on_focused_jog)
        self.jog_pad.home_requested.connect(self._on_focused_home)
        focus_layout.addWidget(self.jog_pad, 3)

        # Right: Lasers & Quick commands for focused node
        ctrl_box = QVBoxLayout()
        ctrl_box.setSpacing(10)

        self.lbl_focus_title = QLabel(f"Controle Manual do Node {self.selected_node}:")
        self.lbl_focus_title.setStyleSheet("color: #38bdf8; font-weight: 800; font-size: 14px;")
        ctrl_box.addWidget(self.lbl_focus_title)

        # Node specific quick buttons
        btn_node_row = QHBoxLayout()
        btn_node_row.setSpacing(6)

        self.btn_node_driver = QPushButton("⚡ Alternar Drivers")
        self.btn_node_driver.clicked.connect(self._toggle_focused_driver)
        btn_node_row.addWidget(self.btn_node_driver)

        self.btn_node_ping = QPushButton("📡 Ping (P)")
        self.btn_node_ping.clicked.connect(lambda: self.comm.send_raw(f"P {self.selected_node} 10 20"))
        btn_node_row.addWidget(self.btn_node_ping)

        self.btn_node_status = QPushButton("🔄 Atualizar (R)")
        self.btn_node_status.clicked.connect(lambda: self.comm.send_raw(f"R {self.selected_node}"))
        btn_node_row.addWidget(self.btn_node_status)

        btn_node_row.addStretch()
        ctrl_box.addLayout(btn_node_row)

        # Lasers for focused node
        self.laser1_slider = LaserSlider(1, "Laser 1 (Frontal)")
        self.laser1_slider.laser_level_changed.connect(
            lambda idx, lvl: self.comm.send_raw(f"L {self.selected_node} {idx} {lvl}")
        )
        ctrl_box.addWidget(self.laser1_slider)

        self.laser2_slider = LaserSlider(2, "Laser 2 (Oposto 180°)")
        self.laser2_slider.laser_level_changed.connect(
            lambda idx, lvl: self.comm.send_raw(f"L {self.selected_node} {idx} {lvl}")
        )
        ctrl_box.addWidget(self.laser2_slider)

        focus_layout.addLayout(ctrl_box, 2)
        main_layout.addWidget(self.focus_frame)

        main_layout.addStretch()

        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)

        # Connect signals
        self.state.node_telemetry_updated.connect(self._on_node_telemetry_updated)
        self.comm.broadcast_changed.connect(self._on_broadcast_changed)

        # Periodic UI update timer (checks offline status and refreshes elapsed times)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_all_cards)
        self.refresh_timer.start(500)

        # Set default focus
        self.select_node(1)

    def select_node(self, node_id: int, force: bool = False):
        node_id = max(1, min(10, int(node_id)))
        # Se houver nós online e o requisitado estiver offline, bloqueia seleção de foco
        if not force and self.state.get_online_nodes_count() > 0:
            if not self.state.is_node_online(node_id):
                return

        self.selected_node = node_id
        for nid, card in self.node_cards.items():
            card.set_selected(nid == node_id)

        is_online = self.state.is_node_online(node_id)
        if not is_online and self.state.get_online_nodes_count() > 0:
            self.lbl_focus_title.setText(f"Controle Manual do Node {self.selected_node} (⚠️ OFFLINE — BLOQUEADO):")
            self.lbl_focus_title.setStyleSheet("color: #f87171; font-weight: 800; font-size: 14px;")
            self.focus_frame.setEnabled(False)
        else:
            self.lbl_focus_title.setText(f"Controle Manual do Node {self.selected_node}:")
            self.lbl_focus_title.setStyleSheet("color: #38bdf8; font-weight: 800; font-size: 14px;")
            self.focus_frame.setEnabled(True)

        self._update_fleet_mode_label()
        self.selected_node_changed.emit(node_id)

    def _update_fleet_mode_label(self):
        if self.comm.is_broadcast_mode():
            self.lbl_fleet_mode.setText("📢 BROADCAST (Node 0) ATIVO")
            self.lbl_fleet_mode.setProperty("class", "badge badge-yellow")
        else:
            self.lbl_fleet_mode.setText(f"🎯 Foco: Node {self.selected_node}")
            self.lbl_fleet_mode.setProperty("class", "badge badge-purple")
        self.lbl_fleet_mode.style().unpolish(self.lbl_fleet_mode)
        self.lbl_fleet_mode.style().polish(self.lbl_fleet_mode)

    def _on_broadcast_changed(self, enabled: bool):
        self._update_fleet_mode_label()

    def _on_node_telemetry_updated(self, node_id: int, t: HardwareTelemetry):
        if node_id in self.node_cards:
            is_online = self.state.is_node_online(node_id)
            self.node_cards[node_id].update_data(t, is_online)

    def _refresh_all_cards(self):
        online_count = 0
        for node_id, card in self.node_cards.items():
            t = self.state.get_node_telemetry(node_id)
            is_online = self.state.is_node_online(node_id)
            if is_online:
                online_count += 1
            card.update_data(t, is_online)
        self.lbl_fleet_online.setText(f"🟢 Nós Online: {online_count} / 10")

        # Se o nó atualmente focado estiver offline enquanto há nós online, bloqueia controles de foco
        focused_online = self.state.is_node_online(self.selected_node)
        if not focused_online and online_count > 0:
            self.lbl_focus_title.setText(f"Controle Manual do Node {self.selected_node} (⚠️ OFFLINE — BLOQUEADO):")
            self.lbl_focus_title.setStyleSheet("color: #f87171; font-weight: 800; font-size: 14px;")
            self.focus_frame.setEnabled(False)
        else:
            self.lbl_focus_title.setText(f"Controle Manual do Node {self.selected_node}:")
            self.lbl_focus_title.setStyleSheet("color: #38bdf8; font-weight: 800; font-size: 14px;")
            self.focus_frame.setEnabled(True)

    def _on_focused_jog(self, axis: str, steps: int, force: bool = False):
        target = 0 if self.comm.is_broadcast_mode() else self.selected_node
        cmd = "MF" if force else "M"
        self.comm.send_raw(f"{cmd} {target} {axis} {steps}")

    def _on_focused_home(self, axis: str):
        target = 0 if self.comm.is_broadcast_mode() else self.selected_node
        if axis == "ALL":
            for a in ("Z", "C", "A"):
                self.comm.send_raw(f"H {target} {a}")
        else:
            self.comm.send_raw(f"H {target} {axis}")

    def _toggle_focused_driver(self):
        target = 0 if self.comm.is_broadcast_mode() else self.selected_node
        t = self.state.get_node_telemetry(self.selected_node)
        new_state = 0 if t.drivers_enabled else 1
        self.comm.send_raw(f"E {target} {new_state}")

    def _home_all_nodes(self):
        self.comm.send_raw("H 0 Z")
        self.comm.send_raw("H 0 C")
        self.comm.send_raw("H 0 A")

    def _lasers_off_all(self):
        self.comm.send_raw("L 0 1 0")
        self.comm.send_raw("L 0 2 0")

    def _poll_all_nodes(self):
        online_nodes = [i for i in range(1, 11) if self.state.is_node_online(i, timeout_sec=3.5)]
        if online_nodes:
            for i in online_nodes:
                self.comm.send_raw(f"R {i}")
        else:
            self.comm.send_raw(f"R {self.selected_node}")
