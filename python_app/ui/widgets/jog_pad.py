"""
Jog Pad Widget for Interactive Motion Control.
Features directional cross-pad for Axis C (Base Rotativa) and Axis A (Pivot dos Lasers),
vertical up/down for Axis Z (Atuador Linear), customizable step sizes, and direction inversion.
"""

from PyQt6.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QCheckBox
)
from PyQt6.QtCore import pyqtSignal, Qt
from python_app.core.protocol_defs import GRAUS_POR_PASSO_CA

class JogPad(QFrame):
    # Emits (axis: 'C'|'A'|'Z', steps: int)
    jog_requested = pyqtSignal(str, int)
    home_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty("class", "card")
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 12, 14, 12)
        main_layout.setSpacing(10)
        
        # Header
        title_layout = QHBoxLayout()
        title = QLabel("Painel de Jog & Movimentação")
        title.setProperty("class", "section-title")
        title_layout.addWidget(title)
        title_layout.addStretch()
        main_layout.addLayout(title_layout)
        
        # Step Size Selector Row
        step_layout = QHBoxLayout()
        step_layout.setSpacing(10)
        
        step_layout.addWidget(QLabel("Passo C/A:"))
        self.combo_ca_step = QComboBox()
        self.combo_ca_step.addItems(["0.5°", "1.0°", "5.0°", "15.0°", "45.0°", "90.0°", "180.0°", "360.0°"])
        self.combo_ca_step.setCurrentText("15.0°")
        step_layout.addWidget(self.combo_ca_step)
        
        step_layout.addWidget(QLabel("Passo Z:"))
        self.combo_z_step = QComboBox()
        self.combo_z_step.addItems([
            "10 passos (0.1 mm)",
            "50 passos (0.5 mm)",
            "100 passos (1.0 mm)",
            "400 passos (4.0 mm)",
            "1000 passos (10.0 mm)",
            "2000 passos (20.0 mm)"
        ])
        self.combo_z_step.setCurrentIndex(3) # 400 steps default
        step_layout.addWidget(self.combo_z_step)
        
        step_layout.addStretch()
        main_layout.addLayout(step_layout)
        
        # Software Direction Inversion Row (Jog DIR)
        inv_row = QHBoxLayout()
        inv_row.setSpacing(14)
        inv_lbl = QLabel("Sentido no Jog:")
        inv_lbl.setStyleSheet("color: #64748b; font-size: 11px; font-weight: 600;")
        inv_row.addWidget(inv_lbl)
        
        self.chk_inv_c = QCheckBox("Inverter C")
        self.chk_inv_c.setStyleSheet("font-size: 11px;")
        inv_row.addWidget(self.chk_inv_c)
        
        self.chk_inv_a = QCheckBox("Inverter A")
        self.chk_inv_a.setStyleSheet("font-size: 11px;")
        inv_row.addWidget(self.chk_inv_a)
        
        self.chk_inv_z = QCheckBox("Inverter Z")
        self.chk_inv_z.setStyleSheet("font-size: 11px;")
        inv_row.addWidget(self.chk_inv_z)
        
        inv_row.addStretch()
        main_layout.addLayout(inv_row)
        
        # Grid layout for Pads
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(20)
        
        # --- C/A Cross Pad ---
        ca_group = QVBoxLayout()
        ca_title = QLabel("Eixo C (Base) / Eixo A (Pivot Lasers)")
        ca_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
        ca_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ca_group.addWidget(ca_title)
        
        ca_grid = QGridLayout()
        ca_grid.setSpacing(6)
        
        self.btn_a_pos = QPushButton("⟳ A+ (Pivot Horário)")
        self.btn_a_pos.setProperty("class", "jog-btn")
        self.btn_a_pos.setToolTip("Girar Pivot dos Lasers no sentido horário (+)")
        self.btn_a_pos.clicked.connect(lambda: self._on_jog_ca('A', 1))
        ca_grid.addWidget(self.btn_a_pos, 0, 1)
        
        self.btn_c_neg = QPushButton("◀ C- (Base Anti-horário)")
        self.btn_c_neg.setProperty("class", "jog-btn")
        self.btn_c_neg.setToolTip("Girar Base Rotativa no sentido anti-horário (-)")
        self.btn_c_neg.clicked.connect(lambda: self._on_jog_ca('C', -1))
        ca_grid.addWidget(self.btn_c_neg, 1, 0)
        
        self.btn_home_ca = QPushButton("🏠 Home C+A")
        self.btn_home_ca.setProperty("class", "btn-primary")
        self.btn_home_ca.setToolTip("Alinhar eixos C e A com as posições de Home")
        self.btn_home_ca.clicked.connect(lambda: self.home_requested.emit("CA"))
        ca_grid.addWidget(self.btn_home_ca, 1, 1)
        
        self.btn_c_pos = QPushButton("C+ (Base Horário) ▶")
        self.btn_c_pos.setProperty("class", "jog-btn")
        self.btn_c_pos.setToolTip("Girar Base Rotativa no sentido horário (+)")
        self.btn_c_pos.clicked.connect(lambda: self._on_jog_ca('C', 1))
        ca_grid.addWidget(self.btn_c_pos, 1, 2)
        
        self.btn_a_neg = QPushButton("⟲ A- (Pivot Anti-horário)")
        self.btn_a_neg.setProperty("class", "jog-btn")
        self.btn_a_neg.setToolTip("Girar Pivot dos Lasers no sentido anti-horário (-)")
        self.btn_a_neg.clicked.connect(lambda: self._on_jog_ca('A', -1))
        ca_grid.addWidget(self.btn_a_neg, 2, 1)
        
        ca_group.addLayout(ca_grid)
        controls_layout.addLayout(ca_group, 3)
        
        # --- Z Axis Vertical Pad ---
        z_group = QVBoxLayout()
        z_title = QLabel("Eixo Z (Atuador Linear)")
        z_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
        z_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        z_group.addWidget(z_title)
        
        z_vbox = QVBoxLayout()
        z_vbox.setSpacing(8)
        
        self.btn_z_up = QPushButton("⬆ Subir Z")
        self.btn_z_up.setProperty("class", "jog-btn")
        self.btn_z_up.setToolTip("Mover atuador vertical Z para cima (+)")
        self.btn_z_up.clicked.connect(lambda: self._on_jog_z(1))
        z_vbox.addWidget(self.btn_z_up)
        
        self.btn_home_z = QPushButton("🏠 Home Z")
        self.btn_home_z.setProperty("class", "btn-primary")
        self.btn_home_z.setToolTip("Retornar eixo Z ao sensor fim de curso")
        self.btn_home_z.clicked.connect(lambda: self.home_requested.emit("Z"))
        z_vbox.addWidget(self.btn_home_z)
        
        self.btn_z_down = QPushButton("⬇ Descer Z")
        self.btn_z_down.setProperty("class", "jog-btn")
        self.btn_z_down.setToolTip("Mover atuador vertical Z para baixo (-)")
        self.btn_z_down.clicked.connect(lambda: self._on_jog_z(-1))
        z_vbox.addWidget(self.btn_z_down)
        
        z_group.addLayout(z_vbox)
        controls_layout.addLayout(z_group, 2)
        
        main_layout.addLayout(controls_layout)

    def _get_ca_step_deg(self) -> float:
        text = self.combo_ca_step.currentText().split("°")[0].strip()
        try:
            return float(text)
        except ValueError:
            return 15.0

    def _get_z_step_steps(self) -> int:
        idx = self.combo_z_step.currentIndex()
        values = [10, 50, 100, 400, 1000, 2000]
        return values[idx] if idx < len(values) else 400

    def _on_jog_ca(self, axis: str, direction: int):
        if axis == 'C' and self.chk_inv_c.isChecked():
            direction *= -1
        elif axis == 'A' and self.chk_inv_a.isChecked():
            direction *= -1
            
        deg = self._get_ca_step_deg()
        steps = int(round((deg / GRAUS_POR_PASSO_CA) * direction))
        self.jog_requested.emit(axis, steps)

    def _on_jog_z(self, direction: int):
        if self.chk_inv_z.isChecked():
            direction *= -1
        steps = self._get_z_step_steps() * direction
        self.jog_requested.emit('Z', steps)
