"""
Laser Power Controller Widget.
0% to 100% control mapped to physical PWM duty (0/4095, 46..300/4095).
Slider and SpinBox increment 1 in 1 (0% to 100%).
Features command throttling and telemetry grace periods to prevent desynchronization.
"""

import time
from PyQt6.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QSpinBox
)
from PyQt6.QtCore import pyqtSignal, Qt, QTimer
from python_app.core.protocol_defs import laser_level_to_percent, percent_to_laser_level
from python_app.ui.theme import add_class

class LaserSlider(QFrame):
    # Emits (laser_index: 1|2, raw_level_12bit: 0..4095)
    laser_level_changed = pyqtSignal(int, int)

    def __init__(self, laser_index: int, title: str = "", parent=None):
        super().__init__(parent)
        self.laser_index = laser_index
        self.current_percent = 0
        self.current_raw_level = 0
        self.setProperty("class", "metric-card")
        self._block_signals = False

        # Throttling & Telemetry synchronization protection
        self._last_user_time = 0.0
        self._pending_raw_level = None
        self._throttle_timer = QTimer(self)
        self._throttle_timer.setSingleShot(True)
        self._throttle_timer.timeout.connect(self._flush_throttled_change)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 6, 10, 6)
        main_layout.setSpacing(5)
        
        # Header (Title & Status)
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        default_title = "Laser Esquerdo" if laser_index == 1 else "Laser Direito"
        self.lbl_title = QLabel(title or default_title)
        add_class(self.lbl_title, "accent")
        header_layout.addWidget(self.lbl_title)
        
        header_layout.addStretch()
        
        self.lbl_status = QLabel("0% (0/4095)")
        add_class(self.lbl_status, "table-header")
        header_layout.addWidget(self.lbl_status)
        
        main_layout.addLayout(header_layout)
        
        # Slider & Spinbox Row (0% .. 100%, passo de 1 em 1)
        slider_layout = QHBoxLayout()
        slider_layout.setContentsMargins(0, 0, 0, 0)
        slider_layout.setSpacing(10)
        
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(0)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setStyleSheet("min-height: 22px;")
        self.slider.setToolTip("Ajuste de potência do laser de 0% a 100% (Passo de 1%)")
        self.slider.valueChanged.connect(self._on_slider_change)
        self.slider.sliderReleased.connect(self._on_slider_released)
        slider_layout.addWidget(self.slider, 1)
        
        self.spin_val = QSpinBox()
        self.spin_val.setRange(0, 100)
        self.spin_val.setValue(0)
        self.spin_val.setSingleStep(1)
        self.spin_val.setSuffix(" %")
        self.spin_val.setStyleSheet("font-size: 12px; font-weight: 600; min-height: 24px; max-height: 26px; min-width: 65px; padding: 1px 4px;")
        self.spin_val.setToolTip("Digite a porcentagem desejada (0% a 100%)")
        self.spin_val.valueChanged.connect(self._on_spin_change)
        slider_layout.addWidget(self.spin_val)
        
        main_layout.addLayout(slider_layout)
        
        # Quick Presets Buttons Row
        preset_layout = QHBoxLayout()
        preset_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.setSpacing(6)
        
        self.btn_off = QPushButton("DESL.")
        self.btn_off.setProperty("class", "btn-danger")
        self.btn_off.setStyleSheet("font-size: 11px; font-weight: 700; min-height: 24px; max-height: 26px; padding: 2px 10px;")
        self.btn_off.setToolTip("Desligar laser (0%)")
        self.btn_off.clicked.connect(lambda: self.set_percent(0))
        preset_layout.addWidget(self.btn_off)
        
        for pct in [10, 25, 50, 75, 100]:
            btn = QPushButton(f"{pct}%")
            btn.setStyleSheet("font-size: 11px; font-weight: 600; min-height: 24px; max-height: 26px; padding: 2px 8px;")
            raw_lvl = percent_to_laser_level(pct)
            btn.setToolTip(f"Definir {pct}% ({raw_lvl}/4095)")
            btn.clicked.connect(lambda checked, p=pct: self.set_percent(p))
            preset_layout.addWidget(btn)
            
        main_layout.addLayout(preset_layout)

    def _update_display(self, percent: int, raw_level: int):
        self.current_percent = percent
        self.current_raw_level = raw_level
        self.lbl_status.setText(f"{percent}% ({raw_level}/4095)")
        if percent > 0:
            self.lbl_status.setStyleSheet("color: #f87171; font-weight: 700; font-size: 12px;")
        else:
            add_class(self.lbl_status, "table-header")

    def _flush_throttled_change(self):
        if self._pending_raw_level is not None:
            val = self._pending_raw_level
            self._pending_raw_level = None
            self.laser_level_changed.emit(self.laser_index, val)

    def _on_slider_change(self, percent: int):
        if self._block_signals:
            return
        self._last_user_time = time.time()
        raw_level = percent_to_laser_level(percent)
        
        self._block_signals = True
        self.spin_val.setValue(percent)
        self._update_display(percent, raw_level)
        self._block_signals = False

        self._pending_raw_level = raw_level
        if not self._throttle_timer.isActive():
            self._throttle_timer.start(60)

    def _on_slider_released(self):
        self._last_user_time = time.time()
        self._throttle_timer.stop()
        raw_level = percent_to_laser_level(self.slider.value())
        self._pending_raw_level = None
        self.laser_level_changed.emit(self.laser_index, raw_level)

    def _on_spin_change(self, percent: int):
        if self._block_signals:
            return
        self._last_user_time = time.time()
        self._throttle_timer.stop()
        raw_level = percent_to_laser_level(percent)
        self._pending_raw_level = None
        
        self._block_signals = True
        self.slider.setValue(percent)
        self._update_display(percent, raw_level)
        self._block_signals = False
        
        self.laser_level_changed.emit(self.laser_index, raw_level)

    def set_percent(self, pct: int, notify: bool = True):
        pct = max(0, min(100, int(pct)))
        raw_level = percent_to_laser_level(pct)
        self._last_user_time = time.time()
        self._throttle_timer.stop()
        self._pending_raw_level = None
        
        self._block_signals = True
        self.slider.setValue(pct)
        self.spin_val.setValue(pct)
        self._update_display(pct, raw_level)
        self._block_signals = False
        
        if notify:
            self.laser_level_changed.emit(self.laser_index, raw_level)

    def set_level(self, raw_level: int, notify: bool = True):
        raw_level = max(0, min(4095, int(raw_level)))
        pct = laser_level_to_percent(raw_level)
        self.set_percent(pct, notify=notify)

    def update_from_telemetry(self, raw_level: int):
        """Update from background telemetry without overriding active user adjustments."""
        raw_level = max(0, min(4095, int(raw_level)))
        if self.slider.isSliderDown() or self.spin_val.hasFocus():
            return  # Do not override while user is actively dragging or typing
        if (time.time() - self._last_user_time) < 0.8:
            return  # Grace period prevents stale buffered telemetry from overriding recent changes
        if self.current_raw_level == raw_level:
            return  # Value unchanged

        pct = laser_level_to_percent(raw_level)
        self._block_signals = True
        self.slider.setValue(pct)
        self.spin_val.setValue(pct)
        self._update_display(pct, raw_level)
        self._block_signals = False
