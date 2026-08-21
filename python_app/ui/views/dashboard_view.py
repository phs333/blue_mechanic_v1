"""
Dashboard View for Real-time Telemetry and Quick Actions.

Kinematics Architecture:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (2 Lasers Colineares Opostos)
- Eixo Z: Atuador Linear
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton, QFrame, QScrollArea
)
from PyQt6.QtCore import Qt
from python_app.ui.widgets.status_card import StatusCard
from python_app.ui.widgets.jog_pad import JogPad
from python_app.ui.widgets.laser_slider import LaserSlider
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareTelemetry, DeviceState
from python_app.core.protocol_defs import FanMode

class DashboardView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        
        # Scroll Area for clean responsiveness
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        container = QWidget()
        self.main_layout = QVBoxLayout(container)
        self.main_layout.setContentsMargins(18, 16, 18, 16)
        self.main_layout.setSpacing(16)
        
        # --- 1. Header Banner & Quick Actions ---
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)
        
        title_vbox = QVBoxLayout()
        title_lbl = QLabel("Blue Mechanic V1 — Visão Geral")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 18px; font-weight: 700;")
        sub_lbl = QLabel("Monitoramento em tempo real: Eixo C (Base), Eixo A (Pivot Lasers), Eixo Z e Sensores")
        sub_lbl.setStyleSheet("color: #64748b; font-size: 12px;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        # Quick Action Buttons
        self.btn_status = QPushButton("🔄 Atualizar Status")
        self.btn_status.clicked.connect(self.comm.request_status)
        header_layout.addWidget(self.btn_status)
        
        self.btn_home_all = QPushButton("🏠 Homing Geral (C, A, Z)")
        self.btn_home_all.setProperty("class", "btn-primary")
        self.btn_home_all.clicked.connect(lambda: self.comm.home_axis("ALL"))
        header_layout.addWidget(self.btn_home_all)
        
        self.btn_driver_toggle = QPushButton("⚡ Ligar Drivers")
        self.btn_driver_toggle.setProperty("class", "btn-success")
        self.btn_driver_toggle.clicked.connect(self._toggle_drivers)
        header_layout.addWidget(self.btn_driver_toggle)
        
        self.main_layout.addWidget(header_card)
        
        # --- 2. Telemetry Cards Grid ---
        cards_grid = QGridLayout()
        cards_grid.setSpacing(12)
        
        # Card C (Base Rotativa)
        self.card_c = StatusCard("Eixo C (Base Rotativa)", "--", "deg")
        self.card_c.set_badge("SEM ENCODER", "amber")
        cards_grid.addWidget(self.card_c, 0, 0)
        
        # Card A (Pivot dos Lasers)
        self.card_a = StatusCard("Eixo A (Pivot Lasers)", "--", "deg")
        self.card_a.set_badge("SEM ENCODER", "amber")
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
        
        # --- 3. Interactive Quick Control Row (Jog Pad + Laser 1/2 preview) ---
        row_layout = QHBoxLayout()
        row_layout.setSpacing(16)
        
        # Jog Pad
        self.jog_widget = JogPad(self.state)
        self.jog_widget.jog_requested.connect(self._on_jog)
        self.jog_widget.home_requested.connect(self.comm.home_axis)
        row_layout.addWidget(self.jog_widget, 3)
        
        # Lasers preview (Pivot A - Colineares opostos)
        lasers_card = QFrame()
        lasers_card.setProperty("class", "card")
        lasers_vbox = QVBoxLayout(lasers_card)
        lasers_vbox.setContentsMargins(14, 12, 14, 12)
        lasers_vbox.setSpacing(10)
        
        lasers_title = QLabel("Lasers no Pivot (Eixo A)")
        lasers_title.setProperty("class", "section-title")
        lasers_vbox.addWidget(lasers_title)
        
        laser_note = QLabel("Orientação: Lasers 1 e 2 montados 180° colineares opostos")
        laser_note.setStyleSheet("color: #64748b; font-size: 11px;")
        lasers_vbox.addWidget(laser_note)
        
        self.laser1_ctrl = LaserSlider(1, "Laser Esquerdo (PWM)")
        self.laser1_ctrl.laser_level_changed.connect(self.comm.set_laser)
        lasers_vbox.addWidget(self.laser1_ctrl)
        
        self.laser2_ctrl = LaserSlider(2, "Laser Direito (PWM)")
        self.laser2_ctrl.laser_level_changed.connect(self.comm.set_laser)
        lasers_vbox.addWidget(self.laser2_ctrl)
        
        row_layout.addWidget(lasers_card, 2)
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

    def _toggle_drivers(self):
        new_state = not self.state.telemetry.drivers_enabled
        self.comm.set_driver_enabled(new_state)
        
    def update_telemetry(self, t: HardwareTelemetry):
        # Update C & A with real encoder communication check
        if t.pos_c_valid:
            self.card_c.set_value(f"{t.pos_c_deg:.2f}")
            self.card_c.set_badge("AS5600 OK", "green")
        else:
            self.card_c.set_value("--")
            self.card_c.set_badge("SEM ENCODER", "amber")

        if t.pos_a_valid:
            self.card_a.set_value(f"{t.pos_a_deg:.2f}")
            self.card_a.set_badge("AS5600 OK", "green")
        else:
            self.card_a.set_value("--")
            self.card_a.set_badge("SEM ENCODER", "amber")
        
        # Update Z
        params = self.state.parameters
        teeth = params.z_pulley_teeth or 16
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
        else:
            self.card_alarm.set_value("DESATIVADO")
            self.card_alarm.set_badge("IGNORADO", "yellow")
            
        # Update Lasers without feedback loops
        self.laser1_ctrl.update_from_telemetry(t.laser1_level)
        self.laser2_ctrl.update_from_telemetry(t.laser2_level)
