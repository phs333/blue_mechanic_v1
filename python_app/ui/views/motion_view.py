"""
Motion Control and Calibration View.
Provides comprehensive controls for Axis C (Base Rotativa), Axis A (Pivot Lasers),
Axis Z (Atuador Linear), jogging, homing, calibration of zero positions (SETHOME)
and Z travel limits (SETLENGTH).

Kinematic Structure:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (2 lasers colineares opostos)
- Eixo Z: Atuador Linear
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QGroupBox, QScrollArea
)
from PyQt6.QtCore import Qt
from python_app.ui.widgets.jog_pad import JogPad
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareTelemetry, HardwareParameters, DeviceState
from python_app.core.protocol_defs import GRAUS_POR_PASSO_CA, PASSOS_POR_MM_Z

class MotionView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(16)
        
        # --- 1. Header with Drivers & Alarm state toggles ---
        top_bar = QFrame()
        top_bar.setProperty("class", "card")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(16, 12, 16, 12)
        
        lbl_info = QLabel("Controle Cinemático — Eixo C (Base), Eixo A (Pivot) e Eixo Z")
        lbl_info.setStyleSheet("color: #38bdf8; font-size: 16px; font-weight: 700;")
        top_layout.addWidget(lbl_info)
        
        top_layout.addStretch()
        
        self.btn_driver_toggle = QPushButton("⚡ Ligar Drivers")
        self.btn_driver_toggle.setProperty("class", "btn-success")
        self.btn_driver_toggle.clicked.connect(self._toggle_drivers)
        top_layout.addWidget(self.btn_driver_toggle)
        
        self.btn_alarm_toggle = QPushButton("🛡️ Limite Seg. Z: ON")
        self.btn_alarm_toggle.setProperty("class", "btn-warning")
        self.btn_alarm_toggle.clicked.connect(self._toggle_alarm_z)
        top_layout.addWidget(self.btn_alarm_toggle)
        
        self.btn_unlock_z = QPushButton("🔓 Reset Limite Z")
        self.btn_unlock_z.clicked.connect(lambda: self.comm.set_alarm_z(False))
        top_layout.addWidget(self.btn_unlock_z)
        
        main_layout.addWidget(top_bar)
        
        # --- 2. Main Motion Grid (Jog + Direct Move / Homing) ---
        grid_layout = QHBoxLayout()
        grid_layout.setSpacing(16)
        
        # Left: Jog Pad
        self.jog_pad = JogPad()
        self.jog_pad.jog_requested.connect(self._on_jog)
        self.jog_pad.home_requested.connect(self.comm.home_axis)
        grid_layout.addWidget(self.jog_pad, 1)
        
        # Right: Calibration & Direct Move Card
        move_card = QFrame()
        move_card.setProperty("class", "card")
        move_vbox = QVBoxLayout(move_card)
        move_vbox.setContentsMargins(16, 14, 16, 14)
        move_vbox.setSpacing(14)
        
        lbl_move_title = QLabel("Movimentação Precisa & Calibração")
        lbl_move_title.setProperty("class", "section-title")
        move_vbox.addWidget(lbl_move_title)
        
        # Axis Selection & Values
        form_grid = QGridLayout()
        form_grid.setSpacing(10)
        
        form_grid.addWidget(QLabel("Eixo Alvo:"), 0, 0)
        self.combo_axis = QComboBox()
        self.combo_axis.addItems(["Eixo C (Base Rotativa)", "Eixo A (Pivot dos Lasers)", "Eixo Z (Linear)"])
        form_grid.addWidget(self.combo_axis, 0, 1)
        
        form_grid.addWidget(QLabel("Distância / Passos:"), 1, 0)
        self.spin_steps = QSpinBox()
        self.spin_steps.setRange(-100000, 100000)
        self.spin_steps.setValue(1000)
        self.spin_steps.setSingleStep(100)
        form_grid.addWidget(self.spin_steps, 1, 1)
        
        form_grid.addWidget(QLabel("Velocidade Override:"), 2, 0)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.0, 5000.0)
        self.spin_speed.setValue(0.0) # 0 = use default
        self.spin_speed.setSpecialValueText("Padrão da Configuração")
        form_grid.addWidget(self.spin_speed, 2, 1)
        
        form_grid.addWidget(QLabel("Aceleração Override:"), 3, 0)
        self.spin_accel = QDoubleSpinBox()
        self.spin_accel.setRange(0.0, 20000.0)
        self.spin_accel.setValue(0.0) # 0 = use default
        self.spin_accel.setSpecialValueText("Padrão da Configuração")
        form_grid.addWidget(self.spin_accel, 3, 1)
        
        self.chk_force = QCheckBox("Forçar sem correção de encoder (MOVE_F)")
        form_grid.addWidget(self.chk_force, 4, 0, 1, 2)
        
        move_vbox.addLayout(form_grid)
        
        # Action buttons for Move
        btn_row = QHBoxLayout()
        self.btn_execute_move = QPushButton("🚀 Executar Movimento")
        self.btn_execute_move.setProperty("class", "btn-primary")
        self.btn_execute_move.clicked.connect(self._execute_direct_move)
        btn_row.addWidget(self.btn_execute_move)
        move_vbox.addLayout(btn_row)
        
        move_vbox.addSpacing(6)
        
        # Calibration Section
        calib_title = QLabel("Gravação de Home e Limites (NVS):")
        calib_title.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        move_vbox.addWidget(calib_title)
        
        calib_grid = QGridLayout()
        calib_grid.setSpacing(8)
        
        self.btn_sethome_c = QPushButton("📍 Gravar Zero C (SETHOME C)")
        self.btn_sethome_c.clicked.connect(lambda: self.comm.set_home('C'))
        calib_grid.addWidget(self.btn_sethome_c, 0, 0)
        
        self.btn_sethome_a = QPushButton("📍 Gravar Zero A (SETHOME A)")
        self.btn_sethome_a.clicked.connect(lambda: self.comm.set_home('A'))
        calib_grid.addWidget(self.btn_sethome_a, 0, 1)
        
        self.btn_homing_c = QPushButton("🏠 Home C (Base)")
        self.btn_homing_c.clicked.connect(lambda: self.comm.home_axis('C'))
        calib_grid.addWidget(self.btn_homing_c, 1, 0)
        
        self.btn_homing_a = QPushButton("🏠 Home A (Pivot)")
        self.btn_homing_a.clicked.connect(lambda: self.comm.home_axis('A'))
        calib_grid.addWidget(self.btn_homing_a, 1, 1)
        
        self.btn_homing_z = QPushButton("🏠 Home Z (Fim de Curso)")
        self.btn_homing_z.clicked.connect(lambda: self.comm.home_axis('Z'))
        calib_grid.addWidget(self.btn_homing_z, 2, 0)
        
        self.btn_home_all = QPushButton("🏠 Home Todos (C, A, Z)")
        self.btn_home_all.setProperty("class", "btn-primary")
        self.btn_home_all.clicked.connect(lambda: self.comm.home_axis("ALL"))
        calib_grid.addWidget(self.btn_home_all, 2, 1)
        
        move_vbox.addLayout(calib_grid)
        
        move_vbox.addSpacing(6)
        
        # Synchronized Move C+A+Z
        sync_title = QLabel("Movimento Sincronizado Simultâneo (C + A + Z por Hardware RMT):")
        sync_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
        move_vbox.addWidget(sync_title)
        
        sync_grid = QGridLayout()
        sync_grid.setSpacing(8)
        
        sync_grid.addWidget(QLabel("Passos C:"), 0, 0)
        self.spin_sync_c = QSpinBox()
        self.spin_sync_c.setRange(-100000, 100000)
        self.spin_sync_c.setValue(1600)
        self.spin_sync_c.setSingleStep(100)
        sync_grid.addWidget(self.spin_sync_c, 0, 1)
        
        sync_grid.addWidget(QLabel("Passos A:"), 0, 2)
        self.spin_sync_a = QSpinBox()
        self.spin_sync_a.setRange(-100000, 100000)
        self.spin_sync_a.setValue(3200)
        self.spin_sync_a.setSingleStep(100)
        sync_grid.addWidget(self.spin_sync_a, 0, 3)

        sync_grid.addWidget(QLabel("Passos Z:"), 1, 0)
        self.spin_sync_z = QSpinBox()
        self.spin_sync_z.setRange(-100000, 100000)
        self.spin_sync_z.setValue(0)
        self.spin_sync_z.setSingleStep(100)
        sync_grid.addWidget(self.spin_sync_z, 1, 1)
        
        self.btn_sync_move = QPushButton("⚡ Mover C, A e Z Simultaneamente (MOVE_SYNC)")
        self.btn_sync_move.setProperty("class", "btn-warning")
        self.btn_sync_move.clicked.connect(self._execute_sync_move)
        sync_grid.addWidget(self.btn_sync_move, 2, 0, 1, 4)
        
        move_vbox.addLayout(sync_grid)
        
        grid_layout.addWidget(move_card, 1)
        main_layout.addLayout(grid_layout)
        
        # --- 3. Live Position & Status Bar at bottom ---
        pos_bar = QFrame()
        pos_bar.setProperty("class", "metric-card")
        pos_layout = QHBoxLayout(pos_bar)
        pos_layout.setContentsMargins(14, 10, 14, 10)
        
        self.lbl_pos_c = QLabel("Posição C (Base): 0.00°")
        self.lbl_pos_c.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 13px;")
        pos_layout.addWidget(self.lbl_pos_c)
        
        pos_layout.addSpacing(20)
        
        self.lbl_pos_a = QLabel("Posição A (Pivot): 0.00°")
        self.lbl_pos_a.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 13px;")
        pos_layout.addWidget(self.lbl_pos_a)
        
        pos_layout.addSpacing(20)
        
        self.lbl_pos_z = QLabel("Posição Z: 0 passos (0.0 mm)")
        self.lbl_pos_z.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 13px;")
        pos_layout.addWidget(self.lbl_pos_z)
        
        pos_layout.addStretch()
        
        self.lbl_alarm_status = QLabel("Alarme Z: ATIVO")
        self.lbl_alarm_status.setProperty("class", "badge badge-green")
        pos_layout.addWidget(self.lbl_alarm_status)
        
        main_layout.addWidget(pos_bar)
        main_layout.addStretch()
        
        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)
        
        self.state.telemetry_updated.connect(self.update_telemetry)

    def _on_jog(self, axis: str, steps: int):
        self.comm.move_axis(axis, steps)

    def _execute_direct_move(self):
        axis_idx = self.combo_axis.currentIndex()
        axis_map = ["C", "A", "Z"]
        axis = axis_map[axis_idx]
        steps = self.spin_steps.value()
        speed = self.spin_speed.value() if self.spin_speed.value() > 0 else None
        accel = self.spin_accel.value() if self.spin_accel.value() > 0 else None
        force = self.chk_force.isChecked()
        
        self.comm.move_axis(axis, steps, speed=speed, accel=accel, force_no_encoder=force)

    def _execute_sync_move(self):
        steps_c = self.spin_sync_c.value()
        steps_a = self.spin_sync_a.value()
        steps_z = self.spin_sync_z.value()
        speed = self.spin_speed.value() if self.spin_speed.value() > 0 else None
        accel = self.spin_accel.value() if self.spin_accel.value() > 0 else None
        self.comm.move_sync(steps_c, steps_a, steps_z, speed=speed, accel=accel)

    def _toggle_drivers(self):
        new_state = not self.state.telemetry.drivers_enabled
        self.comm.set_driver_enabled(new_state)

    def _toggle_alarm_z(self):
        new_state = not self.state.telemetry.alarme_z_ativo
        self.comm.set_alarm_z(new_state)

    def update_telemetry(self, t: HardwareTelemetry):
        teeth = self.state.parameters.z_pulley_teeth or 16
        pos_z_mm = t.get_pos_z_mm(teeth, 200, 16)
        self.lbl_pos_c.setText(f"Posição C (Base): {t.pos_c_deg:.2f}°")
        self.lbl_pos_a.setText(f"Posição A (Pivot): {t.pos_a_deg:.2f}°")
        self.lbl_pos_z.setText(f"Posição Z: {t.pos_z_steps} passos ({pos_z_mm:.2f} mm | Polia: {teeth}T)")
        
        if t.drivers_enabled:
            self.btn_driver_toggle.setText("⚡ Desligar Drivers")
            self.btn_driver_toggle.setProperty("class", "btn-danger")
        else:
            self.btn_driver_toggle.setText("⚡ Ligar Drivers")
            self.btn_driver_toggle.setProperty("class", "btn-success")
        self.btn_driver_toggle.style().unpolish(self.btn_driver_toggle)
        self.btn_driver_toggle.style().polish(self.btn_driver_toggle)

        if t.z_bloqueado:
            self.lbl_alarm_status.setText("Z BLOQUEADO")
            self.lbl_alarm_status.setProperty("class", "badge badge-red")
        elif t.alarme_z_ativo:
            self.lbl_alarm_status.setText("Limite Seg. Z: ATIVO")
            self.lbl_alarm_status.setProperty("class", "badge badge-green")
        else:
            self.lbl_alarm_status.setText("Limite Seg. Z: OFF")
            self.lbl_alarm_status.setProperty("class", "badge badge-yellow")
        self.lbl_alarm_status.style().unpolish(self.lbl_alarm_status)
        self.lbl_alarm_status.style().polish(self.lbl_alarm_status)
