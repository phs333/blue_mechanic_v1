"""
Dashboard View for Real-time Telemetry, Motion Jog, and Centralized Machine Control.

Kinematics Architecture:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (2 Lasers Colineares Opostos)
- Eixo Z: Atuador Linear
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QScrollArea, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QMessageBox,
    QTabWidget, QSizePolicy
)
from PyQt6.QtCore import Qt
from python_app.ui.widgets.status_card import StatusCard
from python_app.ui.widgets.jog_pad import JogPad
from python_app.ui.widgets.laser_slider import LaserSlider
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareTelemetry, DeviceState
from python_app.core.protocol_defs import FanMode, calc_ca_steps_for_degrees, calc_z_steps_for_mm, DEFAULT_Z_PULLEY_TEETH

class DashboardView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.current_fan_mode = FanMode.AUTO
        
        # Scroll Area for clean responsiveness
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        container = QWidget()
        self.main_layout = QVBoxLayout(container)
        self.main_layout.setContentsMargins(14, 10, 14, 10)
        self.main_layout.setSpacing(10)
        
        # --- 1. Header Banner & Quick Actions ---
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(14, 8, 14, 8)
        
        title_vbox = QVBoxLayout()
        title_lbl = QLabel("Blue Mechanic V1 — Painel Central de Controle")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 15px; font-weight: 700;")
        sub_lbl = QLabel("Controle unificado: Jog, Homing, Zero dos Encoders, Movimento Direto e Telemetria em Tempo Real")
        sub_lbl.setStyleSheet("color: #64748b; font-size: 11px;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        # Quick Action Buttons
        self.btn_status = QPushButton("🔄 Atualizar Status")
        self.btn_status.clicked.connect(self.comm.request_status)
        header_layout.addWidget(self.btn_status)
        
        self.btn_home_all = QPushButton("🏠 Home Geral (C, A, Z)")
        self.btn_home_all.setProperty("class", "btn-primary")
        self.btn_home_all.clicked.connect(lambda: self.comm.home_axis("ALL"))
        header_layout.addWidget(self.btn_home_all)
        
        self.btn_driver_toggle = QPushButton("⚡ Ligar Drivers")
        self.btn_driver_toggle.setProperty("class", "btn-success")
        self.btn_driver_toggle.clicked.connect(self._toggle_drivers)
        header_layout.addWidget(self.btn_driver_toggle)

        self.btn_alarm_toggle = QPushButton("🛡️ Limite Z: ON")
        self.btn_alarm_toggle.setProperty("class", "btn-warning")
        self.btn_alarm_toggle.clicked.connect(self._toggle_alarm_z)
        header_layout.addWidget(self.btn_alarm_toggle)

        self.btn_unlock_z = QPushButton("🔓 Reset Limite Z")
        self.btn_unlock_z.clicked.connect(lambda: self.comm.set_alarm_z(False))
        header_layout.addWidget(self.btn_unlock_z)
        
        self.main_layout.addWidget(header_card)
        
        # --- 2. Telemetry Cards Grid ---
        cards_grid = QGridLayout()
        cards_grid.setSpacing(8)
        
        # Card C (Base Rotativa)
        self.card_c = StatusCard("Eixo C (Base Rotativa)", "--", "deg")
        self.card_c.set_badge("SEM ENCODER", "amber")
        self.card_c.set_progress(0, True)
        cards_grid.addWidget(self.card_c, 0, 0)
        
        # Card A (Pivot dos Lasers)
        self.card_a = StatusCard("Eixo A (Pivot Lasers)", "--", "deg")
        self.card_a.set_badge("SEM ENCODER", "amber")
        self.card_a.set_progress(0, True)
        cards_grid.addWidget(self.card_a, 0, 1)
        
        # Card Z
        self.card_z = StatusCard("Eixo Z (Atuador Linear)", "0", "passos")
        self.card_z.set_badge("LIVRE", "green")
        self.card_z.set_progress(0, True)
        cards_grid.addWidget(self.card_z, 0, 2)
        
        # Card Temp
        self.card_temp = StatusCard("Temperatura DS18B20", "--", "°C")
        self.card_temp.set_badge("N/A", "gray")
        cards_grid.addWidget(self.card_temp, 1, 0)
        
        # Card Drivers
        self.card_drivers = StatusCard("Drivers de Passo", "DESLIGADOS", "")
        self.card_drivers.set_badge("DESENERGIZADO", "red")
        cards_grid.addWidget(self.card_drivers, 1, 1)
        
        # Card Alarme Z
        self.card_alarm = StatusCard("Limite de Segurança Z", "ON", "")
        self.card_alarm.set_badge("ATIVO", "green")
        cards_grid.addWidget(self.card_alarm, 1, 2)
        
        self.main_layout.addLayout(cards_grid)
        
        # --- 3. Interactive Quick Control Row ---
        row_layout = QHBoxLayout()
        row_layout.setSpacing(16)
        row_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        # Left: Jog Pad (Interactive directional control + Quick Homing / Zero)
        self.jog_widget = JogPad(self.state)
        self.jog_widget.jog_requested.connect(self._on_jog)
        self.jog_widget.home_requested.connect(self.comm.home_axis)
        self.jog_widget.set_home_requested.connect(self._on_set_home)
        row_layout.addWidget(self.jog_widget, 3)
        
        # Right Column: Direct Move + Lasers & Fan Controls
        right_vbox = QVBoxLayout()
        right_vbox.setSpacing(8)
        right_vbox.setAlignment(Qt.AlignmentFlag.AlignTop)

        # 3A. Precise Move Card (Compact Individual & Synchronized Move Tabs)
        move_card = QFrame()
        move_card.setProperty("class", "card")
        move_card.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        move_card_layout = QVBoxLayout(move_card)
        move_card_layout.setContentsMargins(10, 6, 10, 6)
        move_card_layout.setSpacing(4)

        move_tabs = QTabWidget()
        move_tabs.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        # --- Tab 1: Individual Axis Move ---
        tab_indiv = QWidget()
        tab_indiv_layout = QVBoxLayout(tab_indiv)
        tab_indiv_layout.setContentsMargins(6, 6, 6, 6)
        tab_indiv_layout.setSpacing(6)
        tab_indiv_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        form_grid = QGridLayout()
        form_grid.setContentsMargins(0, 0, 0, 0)
        form_grid.setSpacing(6)

        lbl_eixo = QLabel("Eixo:")
        lbl_eixo.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        form_grid.addWidget(lbl_eixo, 0, 0)
        self.combo_axis = QComboBox()
        self.combo_axis.addItems(["Eixo C (Base Rotativa)", "Eixo A (Pivot Lasers)", "Eixo Z (Linear)"])
        self.combo_axis.setStyleSheet("max-height: 26px; font-size: 11px;")
        form_grid.addWidget(self.combo_axis, 0, 1)

        lbl_passos = QLabel("Passos:")
        lbl_passos.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        form_grid.addWidget(lbl_passos, 0, 2)
        self.spin_steps = QSpinBox()
        self.spin_steps.setRange(-2000000, 2000000)
        self.spin_steps.setValue(1000)
        self.spin_steps.setSingleStep(100)
        self.spin_steps.setStyleSheet("max-height: 26px; font-size: 11px;")
        form_grid.addWidget(self.spin_steps, 0, 3)

        lbl_vel = QLabel("Velocidade:")
        lbl_vel.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        form_grid.addWidget(lbl_vel, 1, 0)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.0, 5000.0)
        self.spin_speed.setValue(0.0)
        self.spin_speed.setSpecialValueText("Padrão NVS")
        self.spin_speed.setStyleSheet("max-height: 26px; font-size: 11px;")
        form_grid.addWidget(self.spin_speed, 1, 1)

        self.chk_force_direct = QCheckBox("Forçar sem encoder (MOVE_F)")
        self.chk_force_direct.setStyleSheet("color: #f59e0b; font-size: 11px;")
        form_grid.addWidget(self.chk_force_direct, 1, 2, 1, 2)

        tab_indiv_layout.addLayout(form_grid)

        move_btn_row = QHBoxLayout()
        move_btn_row.setContentsMargins(0, 0, 0, 0)
        move_btn_row.setSpacing(6)
        self.btn_execute_move = QPushButton("🚀 Executar Movimento")
        self.btn_execute_move.setProperty("class", "btn-primary")
        self.btn_execute_move.setStyleSheet("max-height: 28px; font-size: 11px; font-weight: 700; padding: 4px 8px;")
        self.btn_execute_move.clicked.connect(self._execute_direct_move)
        move_btn_row.addWidget(self.btn_execute_move)

        self.btn_set_zero_direct = QPushButton("📍 Gravar Zero (SETHOME)")
        self.btn_set_zero_direct.setStyleSheet("max-height: 28px; font-size: 11px; padding: 4px 8px;")
        self.btn_set_zero_direct.clicked.connect(self._execute_direct_sethome)
        move_btn_row.addWidget(self.btn_set_zero_direct)
        tab_indiv_layout.addLayout(move_btn_row)
        tab_indiv_layout.addStretch()

        move_tabs.addTab(tab_indiv, "🎯 Eixo Individual")

        # --- Tab 2: Simultaneous Move C+A+Z ---
        tab_sync = QWidget()
        tab_sync_layout = QVBoxLayout(tab_sync)
        tab_sync_layout.setContentsMargins(6, 6, 6, 6)
        tab_sync_layout.setSpacing(6)
        tab_sync_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        sync_grid = QGridLayout()
        sync_grid.setContentsMargins(0, 0, 0, 0)
        sync_grid.setSpacing(6)

        lbl_sync_c = QLabel("Graus C:")
        lbl_sync_c.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        sync_grid.addWidget(lbl_sync_c, 0, 0)
        self.spin_sync_c = QDoubleSpinBox()
        self.spin_sync_c.setRange(-360.0, 360.0)
        self.spin_sync_c.setValue(0.0)
        self.spin_sync_c.setSingleStep(5.0)
        self.spin_sync_c.setSuffix(" °")
        self.spin_sync_c.setStyleSheet("max-height: 26px; font-size: 11px;")
        sync_grid.addWidget(self.spin_sync_c, 0, 1)

        lbl_sync_a = QLabel("Graus A:")
        lbl_sync_a.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        sync_grid.addWidget(lbl_sync_a, 0, 2)
        self.spin_sync_a = QDoubleSpinBox()
        self.spin_sync_a.setRange(-360.0, 360.0)
        self.spin_sync_a.setValue(0.0)
        self.spin_sync_a.setSingleStep(5.0)
        self.spin_sync_a.setSuffix(" °")
        self.spin_sync_a.setStyleSheet("max-height: 26px; font-size: 11px;")
        sync_grid.addWidget(self.spin_sync_a, 0, 3)

        lbl_sync_z = QLabel("Desloc. Z:")
        lbl_sync_z.setStyleSheet("font-size: 11px; font-weight: 600; color: #94a3b8;")
        sync_grid.addWidget(lbl_sync_z, 1, 0)
        self.spin_sync_z = QDoubleSpinBox()
        self.spin_sync_z.setRange(-500.0, 500.0)
        self.spin_sync_z.setValue(0.0)
        self.spin_sync_z.setSingleStep(5.0)
        self.spin_sync_z.setSuffix(" mm")
        self.spin_sync_z.setStyleSheet("max-height: 26px; font-size: 11px;")
        sync_grid.addWidget(self.spin_sync_z, 1, 1)

        self.chk_sync_force = QCheckBox("Forçar sem encoder (MOVE_SYNC_F)")
        self.chk_sync_force.setStyleSheet("color: #f59e0b; font-size: 11px;")
        sync_grid.addWidget(self.chk_sync_force, 1, 2, 1, 2)

        tab_sync_layout.addLayout(sync_grid)

        self.btn_sync_move = QPushButton("⚡ Mover C, A e Z Simultaneamente (MOVE_SYNC)")
        self.btn_sync_move.setProperty("class", "btn-warning")
        self.btn_sync_move.setStyleSheet("max-height: 28px; font-size: 11px; font-weight: 700; padding: 4px 8px;")
        self.btn_sync_move.clicked.connect(self._execute_sync_move)
        tab_sync_layout.addWidget(self.btn_sync_move)
        tab_sync_layout.addStretch()

        move_tabs.addTab(tab_sync, "⚡ Simultâneo (C+A+Z)")

        move_card_layout.addWidget(move_tabs)
        right_vbox.addWidget(move_card)

        # 3B. Lasers & Fan Controls Card (Stacked layout, full-width comfort)
        periph_card = QFrame()
        periph_card.setProperty("class", "card")
        periph_vbox = QVBoxLayout(periph_card)
        periph_vbox.setContentsMargins(10, 8, 10, 8)
        periph_vbox.setSpacing(6)
        
        periph_title = QLabel("Lasers no Pivot & Ventoinha")
        periph_title.setProperty("class", "section-title")
        periph_title.setStyleSheet("font-size: 13px; font-weight: 700; margin-bottom: 2px;")
        periph_vbox.addWidget(periph_title)
        
        # Stacked layout (um sobre o outro): full horizontal width for each laser slider
        self.laser1_ctrl = LaserSlider(1, "Laser 1 Esquerdo (PWM)")
        self.laser1_ctrl.laser_level_changed.connect(self.comm.set_laser)
        periph_vbox.addWidget(self.laser1_ctrl)
        
        self.laser2_ctrl = LaserSlider(2, "Laser 2 Direito (PWM)")
        self.laser2_ctrl.laser_level_changed.connect(self.comm.set_laser)
        periph_vbox.addWidget(self.laser2_ctrl)

        # Fan Controls Row (Generous and easy to click)
        fan_row = QHBoxLayout()
        fan_row.setContentsMargins(0, 0, 0, 0)
        fan_row.setSpacing(8)
        fan_lbl = QLabel("Ventoinha:")
        fan_lbl.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        fan_row.addWidget(fan_lbl)

        self.btn_fan_off = QPushButton("Desligada")
        self.btn_fan_off.setProperty("class", "btn-fan")
        self.btn_fan_off.setStyleSheet("font-size: 12px; min-height: 26px; max-height: 28px; padding: 3px 12px;")
        self.btn_fan_off.clicked.connect(lambda: self._set_fan_mode(0))
        fan_row.addWidget(self.btn_fan_off)

        self.btn_fan_on = QPushButton("Ligada")
        self.btn_fan_on.setProperty("class", "btn-fan")
        self.btn_fan_on.setStyleSheet("font-size: 12px; min-height: 26px; max-height: 28px; padding: 3px 12px;")
        self.btn_fan_on.clicked.connect(lambda: self._set_fan_mode(1))
        fan_row.addWidget(self.btn_fan_on)

        self.btn_fan_auto = QPushButton("Auto (≥45°C)")
        self.btn_fan_auto.setProperty("class", "btn-fan-active")
        self.btn_fan_auto.setStyleSheet("font-size: 12px; min-height: 26px; max-height: 28px; padding: 3px 12px;")
        self.btn_fan_auto.clicked.connect(lambda: self._set_fan_mode(2))
        fan_row.addWidget(self.btn_fan_auto)
        fan_row.addStretch()

        periph_vbox.addLayout(fan_row)
        
        right_vbox.addWidget(periph_card)
        right_vbox.addStretch()
        row_layout.addLayout(right_vbox, 2)
        self.main_layout.addLayout(row_layout)
        
        self.main_layout.addStretch()
        
        # Setup scroll
        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)
        
        # Connect reactive state updates
        self.state.telemetry_updated.connect(self.update_telemetry)
        
    def _on_jog(self, axis: str, steps: int, force: bool = False):
        self.comm.move_axis(axis, steps, force_no_encoder=force)

    def _on_set_home(self, axis: str):
        self.comm.set_home(axis)

    def _execute_direct_move(self):
        axis_str = self.combo_axis.currentText()
        if "Eixo C" in axis_str:
            axis = 'C'
        elif "Eixo A" in axis_str:
            axis = 'A'
        else:
            axis = 'Z'
        steps = self.spin_steps.value()
        spd = self.spin_speed.value() if self.spin_speed.value() > 0 else None
        force = self.chk_force_direct.isChecked()
        self.comm.move_axis(axis, steps, speed=spd, force_no_encoder=force)

    def _execute_sync_move(self):
        params = self.state.parameters
        deg_c = self.spin_sync_c.value()
        deg_a = self.spin_sync_a.value()
        mm_z = self.spin_sync_z.value()
        steps_c = calc_ca_steps_for_degrees(deg_c, params.steps_per_rev[0], params.tmc_microsteps[0])
        steps_a = calc_ca_steps_for_degrees(deg_a, params.steps_per_rev[1], params.tmc_microsteps[1])
        teeth = params.z_pulley_teeth or DEFAULT_Z_PULLEY_TEETH
        steps_z = calc_z_steps_for_mm(mm_z, teeth, params.steps_per_rev[2], params.tmc_microsteps[2])
        force = self.chk_sync_force.isChecked()

        self.comm.move_sync(
            steps_c, steps_a, steps_z,
            speed_c=params.speed[0] if params.speed[0] > 0 else None,
            speed_a=params.speed[1] if params.speed[1] > 0 else None,
            speed_z=params.speed[2] if params.speed[2] > 0 else None,
            accel=None,
            force_no_encoder=force
        )

    def _execute_direct_sethome(self):
        axis_str = self.combo_axis.currentText()
        if "Eixo C" in axis_str:
            axis = 'C'
        elif "Eixo A" in axis_str:
            axis = 'A'
        else:
            axis = 'Z'
        self.comm.set_home(axis)

    def _toggle_drivers(self):
        new_state = not self.state.telemetry.drivers_enabled
        self.comm.set_driver_enabled(new_state)

    def _toggle_alarm_z(self):
        new_state = not self.state.telemetry.alarme_z_ativo
        self.comm.set_alarm_z(new_state)

    def _set_fan_mode(self, mode_id: int):
        self.comm.set_fan(mode_id)
        mode = FanMode(mode_id)
        self.current_fan_mode = mode
        self.btn_fan_off.setProperty("class", "btn-fan-active" if mode == FanMode.MANUAL_OFF else "")
        self.btn_fan_on.setProperty("class", "btn-fan-active" if mode == FanMode.MANUAL_ON else "")
        self.btn_fan_auto.setProperty("class", "btn-fan-active" if mode == FanMode.AUTO else "")
        for btn in [self.btn_fan_off, self.btn_fan_on, self.btn_fan_auto]:
            btn.style().unpolish(btn)
            btn.style().polish(btn)
        
    def update_telemetry(self, t: HardwareTelemetry):
        # Update C & A with real encoder communication check and angular progress bar
        if t.pos_c_valid:
            self.card_c.set_value(f"{t.pos_c_deg:.2f}")
            self.card_c.set_badge("AS5600 OK", "green")
            min_c = self.state.parameters.limit_min_deg_c
            max_c = self.state.parameters.limit_max_deg_c
            span_c = max_c - min_c
            if span_c > 0:
                pct_c = max(0.0, min(100.0, (t.pos_c_deg - min_c) / span_c * 100.0))
                self.card_c.set_progress(pct_c, visible=True)
            else:
                self.card_c.set_progress(0, visible=False)
        else:
            self.card_c.set_value("--")
            self.card_c.set_badge("SEM ENCODER", "amber")
            self.card_c.set_progress(0, visible=False)

        if t.pos_a_valid:
            self.card_a.set_value(f"{t.pos_a_deg:.2f}")
            self.card_a.set_badge("AS5600 OK", "green")
            min_a = self.state.parameters.limit_min_deg_a
            max_a = self.state.parameters.limit_max_deg_a
            span_a = max_a - min_a
            if span_a > 0:
                pct_a = max(0.0, min(100.0, (t.pos_a_deg - min_a) / span_a * 100.0))
                self.card_a.set_progress(pct_a, visible=True)
            else:
                self.card_a.set_progress(0, visible=False)
        else:
            self.card_a.set_value("--")
            self.card_a.set_badge("SEM ENCODER", "amber")
            self.card_a.set_progress(0, visible=False)
        
        # Update Z
        params = self.state.parameters
        teeth = params.z_pulley_teeth or DEFAULT_Z_PULLEY_TEETH
        pos_z_mm = t.get_pos_z_mm(
            teeth,
            params.steps_per_rev[2],
            params.tmc_microsteps[2],
        )
        self.card_z.set_value(f"{t.pos_z_steps} ({pos_z_mm:.2f} mm)")
        self.card_z.set_progress(t.z_progress_pct)
        if t.z_bloqueado:
            self.card_z.set_badge("BLOQUEADO", "red")
        else:
            self.card_z.set_badge("LIVRE", "green")
            
        # Update Temp
        if t.temp_valid:
            self.card_temp.set_value(f"{t.temperature_c:.1f}")
            if t.temperature_c >= 50.0:
                self.card_temp.set_badge("CRÍTICA", "red")
            elif t.temperature_c >= 40.0:
                self.card_temp.set_badge("AQUECIDO", "yellow")
            else:
                self.card_temp.set_badge("NORMAL", "green")
        else:
            self.card_temp.set_value("--")
            self.card_temp.set_badge("SEM LEITURA", "gray")
            
        # Update Drivers
        if t.drivers_enabled:
            self.card_drivers.set_value("ENERGIZADOS")
            self.card_drivers.set_badge("ON", "green")
            self.btn_driver_toggle.setText("🛑 Desligar Drivers")
            self.btn_driver_toggle.setProperty("class", "btn-danger")
        else:
            self.card_drivers.set_value("DESLIGADOS")
            self.card_drivers.set_badge("OFF", "gray")
            self.btn_driver_toggle.setText("⚡ Ligar Drivers")
            self.btn_driver_toggle.setProperty("class", "btn-success")
        self.btn_driver_toggle.style().unpolish(self.btn_driver_toggle)
        self.btn_driver_toggle.style().polish(self.btn_driver_toggle)
            
        # Update Alarm Z
        if t.z_bloqueado:
            self.card_alarm.set_value("BLOQUEIO Z")
            self.card_alarm.set_badge("FALHA", "red")
        elif t.alarme_z_ativo:
            self.card_alarm.set_value("VIGIA ATIVA")
            self.card_alarm.set_badge("ARMADO", "green")
            self.btn_alarm_toggle.setText("🛡️ Limite Z: ON")
            self.btn_alarm_toggle.setProperty("class", "btn-warning")
        else:
            self.card_alarm.set_value("DESATIVADO")
            self.card_alarm.set_badge("IGNORADO", "yellow")
            self.btn_alarm_toggle.setText("🛡️ Limite Z: OFF")
            self.btn_alarm_toggle.setProperty("class", "")
        self.btn_alarm_toggle.style().unpolish(self.btn_alarm_toggle)
        self.btn_alarm_toggle.style().polish(self.btn_alarm_toggle)
            
        # Update Lasers without feedback loops
        self.laser1_ctrl.update_from_telemetry(t.laser1_level)
        self.laser2_ctrl.update_from_telemetry(t.laser2_level)
