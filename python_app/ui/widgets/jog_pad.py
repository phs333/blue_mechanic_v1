"""
Jog Pad Widget for Interactive Motion Control.
Features directional cross-pad for Axis C (Base Rotativa) and Axis A (Pivot dos Lasers),
vertical elevator for Axis Z (Atuador Linear), quick-select step pills, and force mode toggle.
"""

from PyQt6.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QCheckBox, QButtonGroup
)
from PyQt6.QtCore import pyqtSignal, Qt
import time
from python_app.core.protocol_defs import calc_ca_steps_for_degrees, calc_z_steps_for_mm
from python_app.core.state_model import DeviceState

class JogPad(QFrame):
    # Emits (axis: 'C'|'A'|'Z', steps: int, force_no_encoder: bool)
    jog_requested = pyqtSignal(str, int, bool)
    home_requested = pyqtSignal(str)

    def __init__(self, state: DeviceState, parent=None):
        super().__init__(parent)
        self.state = state
        self.setProperty("class", "card")
        
        self.current_step_deg = 15.0
        self.current_step_z_mm = 4.0
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 14, 16, 14)
        main_layout.setSpacing(12)
        
        # Header
        title_layout = QHBoxLayout()
        title = QLabel("Painel de Jog & Movimentação")
        title.setProperty("class", "section-title")
        title_layout.addWidget(title)
        title_layout.addStretch()
        
        self.chk_force = QCheckBox("🔓 Forçar sem correção (MOVE_F)")
        self.chk_force.setToolTip("Movimenta em malha aberta direta sem checagem de limites de encoder")
        self.chk_force.setStyleSheet("color: #f59e0b; font-size: 11px; font-weight: 600;")
        title_layout.addWidget(self.chk_force)
        
        main_layout.addLayout(title_layout)
        
        # Quick Step Pills for Axis C and A
        ca_pill_layout = QHBoxLayout()
        ca_pill_layout.setSpacing(6)
        lbl_ca = QLabel("Passo C/A:")
        lbl_ca.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px;")
        ca_pill_layout.addWidget(lbl_ca)
        
        self.ca_step_group = QButtonGroup(self)
        self.ca_pills = {}
        for deg in [0.5, 1.0, 5.0, 15.0, 45.0, 90.0]:
            btn = QPushButton(f"{deg:g}°")
            btn.setProperty("class", "step-pill-active" if deg == 15.0 else "step-pill")
            btn.clicked.connect(lambda checked, d=deg: self._set_ca_step(d))
            self.ca_step_group.addButton(btn)
            self.ca_pills[deg] = btn
            ca_pill_layout.addWidget(btn)
            
        ca_pill_layout.addStretch()
        main_layout.addLayout(ca_pill_layout)
        
        # Quick Step Pills for Axis Z
        z_pill_layout = QHBoxLayout()
        z_pill_layout.setSpacing(6)
        lbl_z = QLabel("Passo Z:")
        lbl_z.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px;")
        z_pill_layout.addWidget(lbl_z)
        
        self.z_step_group = QButtonGroup(self)
        self.z_pills = {}
        z_presets = [0.5, 1.0, 4.0, 10.0, 20.0]
        for mm in z_presets:
            lbl = f"{mm:g} mm"
            btn = QPushButton(lbl)
            btn.setProperty("class", "step-pill-active" if mm == 4.0 else "step-pill")
            btn.clicked.connect(lambda checked, value=mm: self._set_z_step(value))
            self.z_step_group.addButton(btn)
            self.z_pills[mm] = btn
            z_pill_layout.addWidget(btn)
            
        z_pill_layout.addStretch()
        main_layout.addLayout(z_pill_layout)
        
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
        
        # Controls Group (Cross Pad for C/A + Vertical Pad for Z)
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(24)
        
        # --- C/A Cross Pad ---
        ca_group = QVBoxLayout()
        ca_title = QLabel("Eixo C (Base) / Eixo A (Pivot Lasers)")
        ca_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
        ca_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ca_group.addWidget(ca_title)
        
        ca_grid = QGridLayout()
        ca_grid.setSpacing(8)
        
        self.btn_a_pos = QPushButton("▲ A+ (Pivot Horário)")
        self.btn_a_pos.setProperty("class", "jog-btn")
        self.btn_a_pos.setToolTip("Girar Pivot dos Lasers no sentido horário (+)")
        self.btn_a_pos.clicked.connect(lambda: self._on_jog_ca('A', 1))
        ca_grid.addWidget(self.btn_a_pos, 0, 1)
        
        self.btn_c_neg = QPushButton("◀ C- (Base Anti-horário)")
        self.btn_c_neg.setProperty("class", "jog-btn")
        self.btn_c_neg.setToolTip("Girar Base Rotativa no sentido anti-horário (-)")
        self.btn_c_neg.clicked.connect(lambda: self._on_jog_ca('C', -1))
        ca_grid.addWidget(self.btn_c_neg, 1, 0)
        
        self.btn_home_ca = QPushButton("🎯 Home C+A")
        self.btn_home_ca.setProperty("class", "btn-primary")
        self.btn_home_ca.setToolTip("Alinhar eixos C e A com as posições de Home")
        self.btn_home_ca.clicked.connect(lambda: self.home_requested.emit("CA"))
        ca_grid.addWidget(self.btn_home_ca, 1, 1)
        
        self.btn_c_pos = QPushButton("C+ (Base Horário) ▶")
        self.btn_c_pos.setProperty("class", "jog-btn")
        self.btn_c_pos.setToolTip("Girar Base Rotativa no sentido horário (+)")
        self.btn_c_pos.clicked.connect(lambda: self._on_jog_ca('C', 1))
        ca_grid.addWidget(self.btn_c_pos, 1, 2)
        
        self.btn_a_neg = QPushButton("▼ A- (Pivot Anti-horário)")
        self.btn_a_neg.setProperty("class", "jog-btn")
        self.btn_a_neg.setToolTip("Girar Pivot dos Lasers no sentido anti-horário (-)")
        self.btn_a_neg.clicked.connect(lambda: self._on_jog_ca('A', -1))
        ca_grid.addWidget(self.btn_a_neg, 2, 1)
        
        ca_group.addLayout(ca_grid)
        controls_layout.addLayout(ca_group, 3)
        
        # --- Z Vertical Column ---
        z_group = QVBoxLayout()
        z_title = QLabel("Eixo Z (Atuador Linear)")
        z_title.setStyleSheet("color: #10b981; font-weight: 700; font-size: 12px;")
        z_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        z_group.addWidget(z_title)
        
        z_grid = QVBoxLayout()
        z_grid.setSpacing(8)
        
        self.btn_z_up = QPushButton("⬆ Z+ (Subir)")
        self.btn_z_up.setProperty("class", "jog-btn")
        self.btn_z_up.setToolTip("Subir eixo Z")
        self.btn_z_up.clicked.connect(lambda: self._on_jog_z(1))
        z_grid.addWidget(self.btn_z_up)
        
        self.btn_home_z = QPushButton("🏁 Homing Z")
        self.btn_home_z.setProperty("class", "btn-success")
        self.btn_home_z.setToolTip("Executar Homing no switch fim de curso Z")
        self.btn_home_z.clicked.connect(lambda: self.home_requested.emit("Z"))
        z_grid.addWidget(self.btn_home_z)
        
        self.btn_z_down = QPushButton("⬇ Z- (Descer)")
        self.btn_z_down.setProperty("class", "jog-btn")
        self.btn_z_down.setToolTip("Descer eixo Z")
        self.btn_z_down.clicked.connect(lambda: self._on_jog_z(-1))
        z_grid.addWidget(self.btn_z_down)
        
        z_group.addLayout(z_grid)
        controls_layout.addLayout(z_group, 2)
        
        main_layout.addLayout(controls_layout)

    def _set_ca_step(self, deg: float):
        self.current_step_deg = deg
        for d, btn in self.ca_pills.items():
            btn.setProperty("class", "step-pill-active" if d == deg else "step-pill")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _set_z_step(self, mm: float):
        self.current_step_z_mm = mm
        for value, btn in self.z_pills.items():
            btn.setProperty("class", "step-pill-active" if value == mm else "step-pill")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _on_jog_ca(self, axis: str, direction_multiplier: int):
        now = time.time()
        if not hasattr(self, '_last_jog_time'):
            self._last_jog_time = 0.0
            self._last_jog_key = ""
        key = f"{axis}_{direction_multiplier}"
        # Throttle rapid duplicate clicks on the exact same button (120ms)
        if now - self._last_jog_time < 0.12 and key == self._last_jog_key:
            return
        self._last_jog_time = now
        self._last_jog_key = key

        if axis == 'C' and self.chk_inv_c.isChecked():
            direction_multiplier *= -1
        elif axis == 'A' and self.chk_inv_a.isChecked():
            direction_multiplier *= -1

        force = self.chk_force.isChecked()
        if not force:
            if axis == 'A':
                pos_a = self.state.telemetry.pos_a_deg
                min_a = self.state.parameters.limit_min_deg_a
                max_a = self.state.parameters.limit_max_deg_a
                # If already at max limit and trying to move positive, block command at UI level
                if direction_multiplier > 0 and pos_a >= (max_a - 0.2):
                    return
                # If already at min limit and trying to move negative, block command at UI level
                if direction_multiplier < 0 and pos_a <= (min_a + 0.2):
                    return
            elif axis == 'C':
                pos_c = self.state.telemetry.pos_c_deg
                min_c = self.state.parameters.limit_min_deg_c
                max_c = self.state.parameters.limit_max_deg_c
                if direction_multiplier > 0 and pos_c >= (max_c - 0.2):
                    return
                if direction_multiplier < 0 and pos_c <= (min_c + 0.2):
                    return

        axis_index = 0 if axis == 'C' else 1
        params = self.state.parameters
        steps = calc_ca_steps_for_degrees(
            self.current_step_deg,
            params.steps_per_rev[axis_index],
            params.tmc_microsteps[axis_index],
        )
        self.jog_requested.emit(axis, steps * direction_multiplier, force)

    def _on_jog_z(self, direction_multiplier: int):
        now = time.time()
        if not hasattr(self, '_last_jog_time'):
            self._last_jog_time = 0.0
            self._last_jog_key = ""
        key = f"Z_{direction_multiplier}"
        if now - self._last_jog_time < 0.12 and key == self._last_jog_key:
            return
        self._last_jog_time = now
        self._last_jog_key = key

        if self.chk_inv_z.isChecked():
            direction_multiplier *= -1

        force = self.chk_force.isChecked()
        if not force:
            pos_z = self.state.telemetry.pos_z_steps
            max_z = self.state.parameters.max_passos_z
            if direction_multiplier > 0 and pos_z >= max_z:
                return
            if direction_multiplier < 0 and pos_z <= 0:
                return

        params = self.state.parameters
        steps = calc_z_steps_for_mm(
            self.current_step_z_mm,
            params.z_pulley_teeth,
            params.steps_per_rev[2],
            params.tmc_microsteps[2],
        )
        self.jog_requested.emit('Z', steps * direction_multiplier, force)
