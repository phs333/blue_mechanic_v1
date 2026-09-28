"""
Peripherals View (Lasers, Fan and Thermal Management).

Kinematic Details:
- Eixo C: Base Rotativa
- Eixo A: Pivot acoplado na base rotativa (onde os lasers ficam montados um oposto ao outro colinearmente a 180°)
- Laser Esquerdo (Laser 1 / GPIO 1)
- Laser Direito (Laser 2 / GPIO 2)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QScrollArea
)
from PyQt6.QtCore import Qt
from python_app.ui.widgets.laser_slider import LaserSlider
from python_app.ui.widgets.status_card import StatusCard
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareTelemetry, DeviceState
from python_app.core.protocol_defs import FanMode
from python_app.ui.theme import add_class

class PeripheralsView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.current_fan_mode = FanMode.MANUAL_OFF
        
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(16)
        
        # --- 1. Header Banner ---
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)
        
        title_vbox = QVBoxLayout()
        title_lbl = QLabel("Controle de Periféricos & Gestão Térmica")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 16px; font-weight: 700;")
        sub_lbl = QLabel("Gerenciamento dos Lasers Esquerdo e Direito (Pivot A) e Ventoinha 12V")
        add_class(sub_lbl, "muted")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        self.btn_kill_lasers = QPushButton("⚠️ Desligar Todos os Lasers")
        self.btn_kill_lasers.setProperty("class", "btn-danger")
        self.btn_kill_lasers.clicked.connect(self._kill_all_lasers)
        header_layout.addWidget(self.btn_kill_lasers)
        
        main_layout.addWidget(header_card)
        
        # --- 2. Lasers Section (Side by Side Cards) ---
        lasers_box = QFrame()
        lasers_box.setProperty("class", "card")
        lasers_layout = QVBoxLayout(lasers_box)
        lasers_layout.setContentsMargins(16, 14, 16, 14)
        lasers_layout.setSpacing(14)
        
        lbl_lasers_sec = QLabel("Lasers no Pivot (Eixo A) — Montagem Colinear Oposta (180°)")
        lbl_lasers_sec.setProperty("class", "section-title")
        lasers_layout.addWidget(lbl_lasers_sec)
        
        desc_laser = QLabel("Os lasers ficam posicionados colinearmente em direções opostas (180°) no Pivot (Eixo A), rotacionando sobre a Base (Eixo C).")
        desc_laser.setStyleSheet("color: #94a3b8; font-size: 12px;")
        lasers_layout.addWidget(desc_laser)
        
        lasers_row = QHBoxLayout()
        lasers_row.setSpacing(16)
        
        self.laser1_ctrl = LaserSlider(1, "Laser Esquerdo (GPIO 1)")
        self.laser1_ctrl.laser_level_changed.connect(self.comm.set_laser)
        lasers_row.addWidget(self.laser1_ctrl)
        
        self.laser2_ctrl = LaserSlider(2, "Laser Direito (GPIO 2)")
        self.laser2_ctrl.laser_level_changed.connect(self.comm.set_laser)
        lasers_row.addWidget(self.laser2_ctrl)
        
        lasers_layout.addLayout(lasers_row)
        main_layout.addWidget(lasers_box)
        
        # --- 3. Thermal & Fan Management Section ---
        thermal_box = QFrame()
        thermal_box.setProperty("class", "card")
        thermal_layout = QVBoxLayout(thermal_box)
        thermal_layout.setContentsMargins(16, 14, 16, 14)
        thermal_layout.setSpacing(14)
        
        lbl_thermal_sec = QLabel("Monitoramento Térmico & Ventoinha (Fan)")
        lbl_thermal_sec.setProperty("class", "section-title")
        thermal_layout.addWidget(lbl_thermal_sec)
        
        therm_row = QHBoxLayout()
        therm_row.setSpacing(16)
        
        # Temp card
        self.card_temp = StatusCard("Sensor DS18B20 (1-Wire)", "--", "°C")
        self.card_temp.set_badge("VALIDANDO...", "gray")
        therm_row.addWidget(self.card_temp, 1)
        
        # Fan Control Card
        fan_card = QFrame()
        fan_card.setProperty("class", "metric-card")
        fan_card_layout = QVBoxLayout(fan_card)
        fan_card_layout.setContentsMargins(14, 12, 14, 12)
        fan_card_layout.setSpacing(10)
        
        fan_header = QHBoxLayout()
        fan_title = QLabel("Modo da Ventoinha (Fan MOSFET)")
        fan_title.setProperty("class", "card-title")
        fan_header.addWidget(fan_title)
        fan_header.addStretch()
        
        self.lbl_fan_state = QLabel("OFF")
        self.lbl_fan_state.setProperty("class", "badge badge-gray")
        fan_header.addWidget(self.lbl_fan_state)
        fan_card_layout.addLayout(fan_header)
        
        # Buttons for Fan Modes
        fan_btn_row = QHBoxLayout()
        fan_btn_row.setSpacing(8)
        
        self.btn_fan_off = QPushButton("⏸️ Desligado (0)")
        self.btn_fan_off.setProperty("class", "btn-fan-active")
        self.btn_fan_off.setToolTip("Desligar ventoinha manualmente")
        self.btn_fan_off.clicked.connect(lambda: self._set_fan_mode(0))
        fan_btn_row.addWidget(self.btn_fan_off)
        
        self.btn_fan_on = QPushButton("⚡ Ligado (1)")
        self.btn_fan_on.setProperty("class", "btn-fan")
        self.btn_fan_on.setToolTip("Ligar ventoinha em potência contínua")
        self.btn_fan_on.clicked.connect(lambda: self._set_fan_mode(1))
        fan_btn_row.addWidget(self.btn_fan_on)
        
        self.btn_fan_auto = QPushButton("🔄 Automático (≥45°C)")
        self.btn_fan_auto.setProperty("class", "btn-fan")
        self.btn_fan_auto.setToolTip("Controle automático: liga ≥45°C e desliga ≤40°C")
        self.btn_fan_auto.clicked.connect(lambda: self._set_fan_mode(2))
        fan_btn_row.addWidget(self.btn_fan_auto)
        
        fan_card_layout.addLayout(fan_btn_row)
        
        lbl_fan_desc = QLabel("Em modo AUTO: acionamento automático por histerese térmica (40°C a 45°C).")
        add_class(lbl_fan_desc, "hint")
        fan_card_layout.addWidget(lbl_fan_desc)
        
        therm_row.addWidget(fan_card, 2)
        thermal_layout.addLayout(therm_row)
        main_layout.addWidget(thermal_box)
        
        main_layout.addStretch()
        
        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)
        
        self.state.telemetry_updated.connect(self.update_telemetry)

    def _kill_all_lasers(self):
        self.laser1_ctrl.set_level(0)
        self.laser2_ctrl.set_level(0)

    def _set_fan_mode(self, mode_id: int):
        self.comm.set_fan(mode_id)
        self._update_fan_buttons(FanMode(mode_id))

    def _update_fan_buttons(self, mode: FanMode):
        self.current_fan_mode = mode
        self.btn_fan_off.setProperty("class", "btn-fan-active" if mode == FanMode.MANUAL_OFF else "btn-fan")
        self.btn_fan_on.setProperty("class", "btn-fan-active" if mode == FanMode.MANUAL_ON else "btn-fan")
        self.btn_fan_auto.setProperty("class", "btn-fan-active" if mode == FanMode.AUTO else "btn-fan")
        
        for btn in [self.btn_fan_off, self.btn_fan_on, self.btn_fan_auto]:
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def update_telemetry(self, t: HardwareTelemetry):
        self.laser1_ctrl.update_from_telemetry(t.laser1_level)
        self.laser2_ctrl.update_from_telemetry(t.laser2_level)
        
        if t.temp_valid:
            self.card_temp.set_value(f"{t.temperature_c:.2f}")
            if t.temperature_c >= 50.0:
                self.card_temp.set_badge("ALTA TEMPERATURA", "red")
            elif t.temperature_c >= 40.0:
                self.card_temp.set_badge("AQUECIDO", "yellow")
            else:
                self.card_temp.set_badge("NORMAL", "green")
        else:
            self.card_temp.set_value("--")
            self.card_temp.set_badge("INDISPONÍVEL", "gray")

        self._update_fan_buttons(t.fan_mode)

        if t.fan_output_on:
            self.lbl_fan_state.setText(f"LIGADO ({t.fan_mode.name})")
            self.lbl_fan_state.setProperty("class", "badge badge-green")
        else:
            self.lbl_fan_state.setText(f"DESLIGADO ({t.fan_mode.name})")
            self.lbl_fan_state.setProperty("class", "badge badge-gray")
        self.lbl_fan_state.style().unpolish(self.lbl_fan_state)
        self.lbl_fan_state.style().polish(self.lbl_fan_state)
