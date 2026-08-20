"""
Laser Power Controller Widget.
Interactive percentage slider (0% to 100% mapped to hardware PWM 3..17 de 255),
with direct raw level readout, ON/OFF toggle, and quick presets.
"""

from PyQt6.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QSpinBox
)
from PyQt6.QtCore import pyqtSignal, Qt
from python_app.core.protocol_defs import (
    laser_level_to_percent, percent_to_laser_level,
    LASER_PWM_MIN_ACTIVE_LEVEL_8BIT, LASER_PWM_MAX_USEFUL_LEVEL_8BIT
)

class LaserSlider(QFrame):
    # Emits (laser_index: 1|2, level_8bit: 0..255)
    laser_level_changed = pyqtSignal(int, int)

    def __init__(self, laser_index: int, title: str = "", parent=None):
        super().__init__(parent)
        self.laser_index = laser_index
        self.current_level = 0
        self.setProperty("class", "metric-card")
        self._block_signals = False
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 12, 14, 12)
        main_layout.setSpacing(10)
        
        # Header (Title & Badges)
        header_layout = QHBoxLayout()
        default_title = "Laser Esquerdo" if laser_index == 1 else "Laser Direito"
        self.lbl_title = QLabel(title or default_title)
        self.lbl_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 14px;")
        header_layout.addWidget(self.lbl_title)
        
        header_layout.addStretch()
        
        self.lbl_status = QLabel("0% (0/255)")
        self.lbl_status.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 13px;")
        header_layout.addWidget(self.lbl_status)
        
        main_layout.addLayout(header_layout)
        
        # Percentage Slider & Spinbox Row
        slider_layout = QHBoxLayout()
        slider_layout.setSpacing(12)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(0)
        self.slider.setToolTip("Ajuste de potência do laser (0% = desligado, 100% = 17/255)")
        self.slider.valueChanged.connect(self._on_slider_change)
        slider_layout.addWidget(self.slider, 4)
        
        self.spin_pct = QSpinBox()
        self.spin_pct.setRange(0, 100)
        self.spin_pct.setValue(0)
        self.spin_pct.setSuffix(" %")
        self.spin_pct.valueChanged.connect(self._on_spin_pct_change)
        slider_layout.addWidget(self.spin_pct, 1)
        
        main_layout.addLayout(slider_layout)
        
        # Quick Presets Buttons Row
        preset_layout = QHBoxLayout()
        preset_layout.setSpacing(6)
        
        self.btn_off = QPushButton("DESLIGAR")
        self.btn_off.setProperty("class", "btn-danger")
        self.btn_off.setToolTip("Desligar laser (0/255)")
        self.btn_off.clicked.connect(lambda: self.set_level(0))
        preset_layout.addWidget(self.btn_off)
        
        for pct in [25, 50, 75, 100]:
            btn = QPushButton(f"{pct}%")
            btn.setToolTip(f"Definir {pct}% de potência")
            btn.clicked.connect(lambda checked, p=pct: self.set_percent(p))
            preset_layout.addWidget(btn)
            
        main_layout.addLayout(preset_layout)

    def _update_ui_state(self, level: int, pct: int):
        self.current_level = level
        self.lbl_status.setText(f"{pct}% ({level}/255)")
        if level > 0:
            self.lbl_status.setStyleSheet("color: #f87171; font-weight: 700; font-size: 13px;")
        else:
            self.lbl_status.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 13px;")

    def _on_slider_change(self, pct: int):
        if self._block_signals:
            return
        self._block_signals = True
        self.spin_pct.setValue(pct)
        level = percent_to_laser_level(pct)
        self._update_ui_state(level, pct)
        self._block_signals = False
        self.laser_level_changed.emit(self.laser_index, level)

    def _on_spin_pct_change(self, pct: int):
        if self._block_signals:
            return
        self._block_signals = True
        self.slider.setValue(pct)
        level = percent_to_laser_level(pct)
        self._update_ui_state(level, pct)
        self._block_signals = False
        self.laser_level_changed.emit(self.laser_index, level)

    def set_level(self, level: int, notify: bool = True):
        level = max(0, min(255, level))
        pct = laser_level_to_percent(level)
        self._block_signals = True
        self.slider.setValue(pct)
        self.spin_pct.setValue(pct)
        self._update_ui_state(level, pct)
        self._block_signals = False
        if notify:
            self.laser_level_changed.emit(self.laser_index, level)

    def set_percent(self, pct: int):
        pct = max(0, min(100, pct))
        level = percent_to_laser_level(pct)
        self._block_signals = True
        self.slider.setValue(pct)
        self.spin_pct.setValue(pct)
        self._update_ui_state(level, pct)
        self._block_signals = False
        self.laser_level_changed.emit(self.laser_index, level)

    def update_from_telemetry(self, level: int):
        """Update without re-emitting change signal back to hardware."""
        level = max(0, min(255, level))
        pct = laser_level_to_percent(level)
        self._block_signals = True
        self.slider.setValue(pct)
        self.spin_pct.setValue(pct)
        self._update_ui_state(level, pct)
        self._block_signals = False
