"""
Reusable Telemetry Card Widget.
Displays a metric title, bold value, unit, and optional status badge / progress bar.
"""

from typing import Optional
from PyQt6.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar
from PyQt6.QtCore import Qt

class StatusCard(QFrame):
    def __init__(self, title: str, initial_value: str = "--", unit: str = "", parent=None):
        super().__init__(parent)
        self.setProperty("class", "metric-card")
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(2)
        
        # Header (Title + Badge)
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_title = QLabel(title)
        self.lbl_title.setProperty("class", "card-title")
        header_layout.addWidget(self.lbl_title)
        
        header_layout.addStretch()
        
        self.lbl_badge = QLabel("")
        self.lbl_badge.setProperty("class", "badge")
        self.lbl_badge.setVisible(False)
        header_layout.addWidget(self.lbl_badge)
        
        layout.addLayout(header_layout)
        
        # Value + Unit
        val_layout = QHBoxLayout()
        val_layout.setContentsMargins(0, 0, 0, 0)
        val_layout.setSpacing(5)
        
        self.lbl_value = QLabel(initial_value)
        self.lbl_value.setProperty("class", "metric-value")
        val_layout.addWidget(self.lbl_value)
        
        self.lbl_unit = QLabel(unit)
        self.lbl_unit.setProperty("class", "metric-unit")
        self.lbl_unit.setAlignment(Qt.AlignmentFlag.AlignBottom)
        val_layout.addWidget(self.lbl_unit)
        
        val_layout.addStretch()
        layout.addLayout(val_layout)
        
        # Optional progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setFixedHeight(5)
        self.progress_bar.setTextVisible(False)
        layout.addWidget(self.progress_bar)

    def set_value(self, val_text: str):
        self.lbl_value.setText(val_text)
        
    def set_unit(self, unit_text: str):
        self.lbl_unit.setText(unit_text)

    def set_badge(self, text: str, style: str = "green"):
        """style can be: green, red, yellow, blue, gray"""
        if not text:
            self.lbl_badge.setVisible(False)
            return
        self.lbl_badge.setText(text)
        self.lbl_badge.setProperty("class", f"badge badge-{style}")
        self.lbl_badge.style().unpolish(self.lbl_badge)
        self.lbl_badge.style().polish(self.lbl_badge)
        self.lbl_badge.setVisible(True)

    def set_progress(self, percent: float, visible: bool = True):
        self.progress_bar.setVisible(visible)
        self.progress_bar.setValue(int(max(0, min(100, percent))))
