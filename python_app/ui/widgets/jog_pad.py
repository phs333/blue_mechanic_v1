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
    set_home_requested = pyqtSignal(str)

    def __init__(self, state: DeviceState, parent=None):
        super().__init__(parent)
        self.state = state
        self.setProperty("class", "card")
        
        self.current_step_deg = 15.0
        self.current_step_z_mm = 5.0
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 10, 14, 10)
        main_layout.setSpacing(8)
        
        # --- 1. Header ---
        title_layout = QHBoxLayout()
        title = QLabel("🕹️ Painel de Jog & Movimentação")
        title.setProperty("class", "section-title")
        title_layout.addWidget(title)
        title_layout.addStretch()
        
        self.chk_force = QCheckBox("🔓 Forçar sem correção (MOVE_F)")
        self.chk_force.setToolTip("Movimenta em malha aberta direta sem checagem de limites de encoder")
        self.chk_force.setStyleSheet("color: #f59e0b; font-size: 11px; font-weight: 600;")
        title_layout.addWidget(self.chk_force)
        
        main_layout.addLayout(title_layout)
        
        # --- 2. Quick Step Pills for Axis C and A ---
        ca_pill_layout = QHBoxLayout()
        ca_pill_layout.setSpacing(6)
        lbl_ca = QLabel("Passo C/A:")
        lbl_ca.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px; min-width: 65px;")
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
        
        # --- 3. Quick Step Pills for Axis Z ---
        z_pill_layout = QHBoxLayout()
        z_pill_layout.setSpacing(6)
        lbl_z = QLabel("Passo Z:")
        lbl_z.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px; min-width: 65px;")
        z_pill_layout.addWidget(lbl_z)
        
        self.z_step_group = QButtonGroup(self)
        self.z_pills = {}
        z_presets = [0.5, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0]
        for mm in z_presets:
            lbl = f"{mm:g} mm"
            btn = QPushButton(lbl)
            btn.setProperty("class", "step-pill-active-z" if mm == 5.0 else "step-pill")
            btn.clicked.connect(lambda checked, value=mm: self._set_z_step(value))
            self.z_step_group.addButton(btn)
            self.z_pills[mm] = btn
            z_pill_layout.addWidget(btn)
            
        z_pill_layout.addStretch()
        main_layout.addLayout(z_pill_layout)
        
        # --- 4. Software Direction Inversion Row ---
        inv_row = QHBoxLayout()
        inv_row.setSpacing(14)
        inv_lbl = QLabel("Sentido no Jog:")
        inv_lbl.setStyleSheet("color: #64748b; font-size: 11px; font-weight: 600; min-width: 65px;")
        inv_row.addWidget(inv_lbl)
        
        self.chk_inv_c = QCheckBox("Inverter C")
        self.chk_inv_c.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        inv_row.addWidget(self.chk_inv_c)
        
        self.chk_inv_a = QCheckBox("Inverter A")
        self.chk_inv_a.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        inv_row.addWidget(self.chk_inv_a)
        
        self.chk_inv_z = QCheckBox("Inverter Z")
        self.chk_inv_z.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        inv_row.addWidget(self.chk_inv_z)
        
        inv_row.addStretch()
        main_layout.addLayout(inv_row)
        
        main_layout.addSpacing(4)

        # --- 5. Controls Group (Enclosed Cards for C/A & Z) ---
        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(16)
        
        # =========================================================================
        # MODULE 1: AXIS C (Base) & AXIS A (Pivot) MODULE
        # =========================================================================
        frame_ca = QFrame()
        frame_ca.setProperty("class", "jog-module")
        ca_layout = QVBoxLayout(frame_ca)
        ca_layout.setContentsMargins(12, 10, 12, 10)
        ca_layout.setSpacing(8)
        ca_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        # C/A Module Header
        ca_hdr_box = QVBoxLayout()
        ca_hdr_box.setSpacing(1)
        ca_title = QLabel("🔄 Eixos C (Base) & A (Pivot)")
        ca_title.setProperty("class", "jog-module-title")
        ca_sub = QLabel("AS5600 12-bit Encoder | Rastreamento Multi-voltas")
        ca_sub.setProperty("class", "jog-module-sub")
        ca_hdr_box.addWidget(ca_title)
        ca_hdr_box.addWidget(ca_sub)
        ca_layout.addLayout(ca_hdr_box)
        
        # Directional Cross Pad
        ca_grid = QGridLayout()
        ca_grid.setSpacing(8)
        
        self.btn_a_pos = QPushButton("▲ A+ (Horário)")
        self.btn_a_pos.setProperty("class", "jog-btn")
        self.btn_a_pos.setToolTip("Girar Pivot dos Lasers no sentido horário (+)")
        self.btn_a_pos.clicked.connect(lambda checked=False: self._on_jog_ca('A', 1))
        ca_grid.addWidget(self.btn_a_pos, 0, 1)
        
        self.btn_c_neg = QPushButton("◀ C- (Anti-horário)")
        self.btn_c_neg.setProperty("class", "jog-btn")
        self.btn_c_neg.setToolTip("Girar Base Rotativa no sentido anti-horário (-)")
        self.btn_c_neg.clicked.connect(lambda checked=False: self._on_jog_ca('C', -1))
        ca_grid.addWidget(self.btn_c_neg, 1, 0)
        
        self.btn_home_ca = QPushButton("🎯 Home C+A")
        self.btn_home_ca.setProperty("class", "jog-btn-home")
        self.btn_home_ca.setToolTip("Alinhar eixos C e A com as posições de Home (0.00°)")
        self.btn_home_ca.clicked.connect(lambda checked=False: self.home_requested.emit("CA"))
        ca_grid.addWidget(self.btn_home_ca, 1, 1)
        
        self.btn_c_pos = QPushButton("C+ (Horário) ▶")
        self.btn_c_pos.setProperty("class", "jog-btn")
        self.btn_c_pos.setToolTip("Girar Base Rotativa no sentido horário (+)")
        self.btn_c_pos.clicked.connect(lambda checked=False: self._on_jog_ca('C', 1))
        ca_grid.addWidget(self.btn_c_pos, 1, 2)
        
        self.btn_a_neg = QPushButton("▼ A- (Anti-horário)")
        self.btn_a_neg.setProperty("class", "jog-btn")
        self.btn_a_neg.setToolTip("Girar Pivot dos Lasers no sentido anti-horário (-)")
        self.btn_a_neg.clicked.connect(lambda checked=False: self._on_jog_ca('A', -1))
        ca_grid.addWidget(self.btn_a_neg, 2, 1)
        
        ca_layout.addLayout(ca_grid)

        # Quick Calibration and Independent Home for C and A
        ca_quick_grid = QGridLayout()
        ca_quick_grid.setSpacing(6)
        
        self.btn_home_c = QPushButton("🏠 Home C")
        self.btn_home_c.setProperty("class", "jog-btn-action")
        self.btn_home_c.setToolTip("Alinhar eixo C com Home (0.00°)")
        self.btn_home_c.clicked.connect(lambda checked=False: self.home_requested.emit("C"))
        ca_quick_grid.addWidget(self.btn_home_c, 0, 0)
        
        self.btn_home_a = QPushButton("🏠 Home A")
        self.btn_home_a.setProperty("class", "jog-btn-action")
        self.btn_home_a.setToolTip("Alinhar eixo A com Home (0.00°)")
        self.btn_home_a.clicked.connect(lambda checked=False: self.home_requested.emit("A"))
        ca_quick_grid.addWidget(self.btn_home_a, 0, 1)

        self.btn_sethome_c = QPushButton("📍 Zero C (SETHOME)")
        self.btn_sethome_c.setProperty("class", "jog-btn-zero")
        self.btn_sethome_c.setToolTip("Gravar posição atual como Zero do Eixo C na NVS")
        self.btn_sethome_c.clicked.connect(lambda checked=False: self.set_home_requested.emit("C"))
        ca_quick_grid.addWidget(self.btn_sethome_c, 1, 0)
        
        self.btn_sethome_a = QPushButton("📍 Zero A (SETHOME)")
        self.btn_sethome_a.setProperty("class", "jog-btn-zero")
        self.btn_sethome_a.setToolTip("Gravar posição atual como Zero do Eixo A na NVS")
        self.btn_sethome_a.clicked.connect(lambda checked=False: self.set_home_requested.emit("A"))
        ca_quick_grid.addWidget(self.btn_sethome_a, 1, 1)

        self.btn_home_all = QPushButton("🌐 Home Geral (C, A, Z)")
        self.btn_home_all.setProperty("class", "btn-primary")
        self.btn_home_all.setToolTip("Executar Homing completo em todos os eixos da máquina")
        self.btn_home_all.clicked.connect(lambda checked=False: self.home_requested.emit("ALL"))
        ca_quick_grid.addWidget(self.btn_home_all, 2, 0, 1, 2)
        
        ca_layout.addLayout(ca_quick_grid)
        controls_layout.addWidget(frame_ca, 3)
        
        # =========================================================================
        # MODULE 2: AXIS Z (Linear) MODULE
        # =========================================================================
        frame_z = QFrame()
        frame_z.setProperty("class", "jog-module")
        z_layout = QVBoxLayout(frame_z)
        z_layout.setContentsMargins(12, 10, 12, 10)
        z_layout.setSpacing(8)
        z_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        # Z Module Header
        z_hdr_box = QVBoxLayout()
        z_hdr_box.setSpacing(1)
        z_title = QLabel("📏 Eixo Z (Atuador Linear)")
        z_title.setProperty("class", "jog-module-title-z")
        z_sub = QLabel("Fim de Curso Óptico/Mecânico")
        z_sub.setProperty("class", "jog-module-sub")
        z_hdr_box.addWidget(z_title)
        z_hdr_box.addWidget(z_sub)
        z_layout.addLayout(z_hdr_box)
        
        z_grid = QVBoxLayout()
        z_grid.setSpacing(8)
        
        self.btn_z_up = QPushButton("⬆ Subir Z+")
        self.btn_z_up.setProperty("class", "jog-btn-z")
        self.btn_z_up.setToolTip("Subir eixo Z")
        self.btn_z_up.clicked.connect(lambda checked=False: self._on_jog_z(1))
        z_grid.addWidget(self.btn_z_up)
        
        self.btn_home_z = QPushButton("🏁 Homing Z")
        self.btn_home_z.setProperty("class", "jog-btn-home-z")
        self.btn_home_z.setToolTip("Executar Homing no switch fim de curso Z")
        self.btn_home_z.clicked.connect(lambda checked=False: self.home_requested.emit("Z"))
        z_grid.addWidget(self.btn_home_z)
        
        self.btn_z_down = QPushButton("⬇ Descer Z-")
        self.btn_z_down.setProperty("class", "jog-btn-z")
        self.btn_z_down.setToolTip("Descer eixo Z")
        self.btn_z_down.clicked.connect(lambda checked=False: self._on_jog_z(-1))
        z_grid.addWidget(self.btn_z_down)
        
        z_layout.addLayout(z_grid)

        # Quick Z Helper
        self.btn_unlock_z = QPushButton("🔓 Reset Limite Z")
        self.btn_unlock_z.setProperty("class", "jog-btn-action")
        self.btn_unlock_z.setToolTip("Desbloquear eixo Z caso o alarme de fim de curso tenha atuado")
        self.btn_unlock_z.clicked.connect(lambda checked=False: self.state.telemetry_updated.emit(self.state.telemetry))
        z_layout.addWidget(self.btn_unlock_z)

        controls_layout.addWidget(frame_z, 2)
        
        main_layout.addLayout(controls_layout)
        main_layout.addStretch()

    def _set_ca_step(self, deg: float):
        self.current_step_deg = deg
        for d, btn in self.ca_pills.items():
            btn.setProperty("class", "step-pill-active" if d == deg else "step-pill")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _set_z_step(self, mm: float):
        self.current_step_z_mm = mm
        for value, btn in self.z_pills.items():
            btn.setProperty("class", "step-pill-active-z" if value == mm else "step-pill")
            btn.style().unpolish(btn)
            btn.style().polish(btn)

    def _on_jog_ca(self, axis: str, direction_multiplier: int):
        if axis == 'C' and self.chk_inv_c.isChecked():
            direction_multiplier *= -1
        elif axis == 'A' and self.chk_inv_a.isChecked():
            direction_multiplier *= -1

        now = time.time()
        dir_key = f"{axis}_{direction_multiplier}"
        # Instant reversal: if direction or axis changed, allow immediately!
        if getattr(self, '_last_jog_ca_key', None) == dir_key:
            if now - getattr(self, '_last_jog_ca_time', 0.0) < 0.08:
                return
        self._last_jog_ca_time = now
        self._last_jog_ca_key = dir_key

        force = self.chk_force.isChecked()
        step_deg = self.current_step_deg
        if not force:
            if axis == 'A':
                pos_a = self.state.telemetry.pos_a_deg
                min_a = self.state.parameters.limit_min_deg_a
                max_a = self.state.parameters.limit_max_deg_a
                if direction_multiplier > 0:
                    if pos_a >= (max_a - 0.05):
                        return
                    if pos_a + step_deg > max_a:
                        step_deg = max(0.0, max_a - pos_a)
                else:
                    if pos_a <= (min_a + 0.05):
                        return
                    if pos_a - step_deg < min_a:
                        step_deg = max(0.0, pos_a - min_a)
            elif axis == 'C':
                pos_c = self.state.telemetry.pos_c_deg
                min_c = self.state.parameters.limit_min_deg_c
                max_c = self.state.parameters.limit_max_deg_c
                if direction_multiplier > 0:
                    if pos_c >= (max_c - 0.05):
                        return
                    if pos_c + step_deg > max_c:
                        step_deg = max(0.0, max_c - pos_c)
                else:
                    if pos_c <= (min_c + 0.05):
                        return
                    if pos_c - step_deg < min_c:
                        step_deg = max(0.0, pos_c - min_c)

        if step_deg <= 0.001:
            return

        axis_index = 0 if axis == 'C' else 1
        params = self.state.parameters
        steps = calc_ca_steps_for_degrees(
            step_deg,
            params.steps_per_rev[axis_index],
            params.tmc_microsteps[axis_index],
        )
        if steps <= 0:
            return
        self.jog_requested.emit(axis, steps * direction_multiplier, force)

    def _on_jog_z(self, direction_multiplier: int):
        if self.chk_inv_z.isChecked():
            direction_multiplier *= -1

        now = time.time()
        dir_key = f"Z_{direction_multiplier}"
        # Instant reversal: if direction changed, allow immediately!
        if getattr(self, '_last_jog_z_key', None) == dir_key:
            if now - getattr(self, '_last_jog_z_time', 0.0) < 0.08:
                return
        self._last_jog_z_time = now
        self._last_jog_z_key = dir_key

        params = self.state.parameters
        steps = calc_z_steps_for_mm(
            self.current_step_z_mm,
            params.z_pulley_teeth,
            params.steps_per_rev[2],
            params.tmc_microsteps[2],
        )

        force = self.chk_force.isChecked()
        if not force:
            pos_z = self.state.telemetry.pos_z_steps
            max_z = self.state.parameters.max_passos_z
            if direction_multiplier > 0:
                if pos_z >= max_z:
                    return
                if pos_z + steps > max_z:
                    steps = max(0, max_z - pos_z)
            else:
                if pos_z <= 0:
                    return
                if pos_z - steps < 0:
                    steps = max(0, pos_z)

        if steps <= 0:
            return

        self.jog_requested.emit('Z', steps * direction_multiplier, force)
