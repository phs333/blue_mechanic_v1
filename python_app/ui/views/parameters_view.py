"""
Parameters and Hardware Configuration View (NVS, Kinematics, TMC2209 & CAN).

Unified, single-screen configuration management with clear visual hierarchy:
- Cinemática e Motores de Passo (Eixos C, A, Z)
- Limites Angulares dos Encoders Magnéticos (Eixos C, A)
- Drivers TMC2209 (Comunicação UART Digital)
- Barramento CAN (TWAI - Comunicação entre os 10 Nós)
- Ação Unificada de Gravação na NVS (Elimina botões intermediários dispersos)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QScrollArea,
    QLineEdit, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareParameters, DeviceState
from python_app.core.protocol_defs import (
    calc_z_mm_per_rev, calc_z_mm_per_step, calc_z_steps_per_mm,
    calc_z_steps_for_mm, calc_z_mm_for_steps, Z_BELT_PITCH_MM, tmc_quantize_ma
)
from python_app.ui.theme import add_class

class ParametersView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state

        # Main layout attached directly to self (no QScrollArea = no scrollbar)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 10, 14, 10)
        main_layout.setSpacing(8)

        # Center outer container to prevent cards from sprawling into empty gaps on wide displays
        center_outer = QHBoxLayout()
        center_outer.setContentsMargins(0, 0, 0, 0)
        center_outer.addStretch(1)

        center_box = QWidget()
        center_box.setMaximumWidth(1380)
        center_layout = QVBoxLayout(center_box)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(8)
        
        # ==============================================================
        # --- HEADER CARD (Compact) ---
        # ==============================================================
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(14, 8, 14, 8)
        header_layout.setSpacing(10)
        
        title_vbox = QVBoxLayout()
        title_vbox.setSpacing(2)
        title_lbl = QLabel("⚙️  Gerenciamento de Parâmetros & NVS")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 15px; font-weight: 700;")
        sub_lbl = QLabel("Cinemática, encoders magnéticos, drivers TMC2209 e barramento CAN dos nós")
        add_class(sub_lbl, "muted")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox, 1)
        
        self.btn_refresh = QPushButton("📥  Ler da Placa")
        self.btn_refresh.setStyleSheet("font-size: 12.5px; min-height: 28px; max-height: 32px; padding: 3px 12px;")
        self.btn_refresh.setToolTip("Solicita leitura completa das configurações gravadas no ESP32")
        self.btn_refresh.clicked.connect(self.comm.request_config_dump)
        header_layout.addWidget(self.btn_refresh)
        
        self.btn_save_nvs = QPushButton("💾  Gravar Todas na NVS")
        self.btn_save_nvs.setProperty("class", "btn-primary")
        self.btn_save_nvs.setStyleSheet("font-size: 12.5px; font-weight: 700; min-height: 28px; max-height: 32px; padding: 3px 14px;")
        self.btn_save_nvs.setToolTip("Transmite e grava todas as seções desta tela na memória flash não-volátil (NVS)")
        self.btn_save_nvs.clicked.connect(self._apply_all_parameters)
        header_layout.addWidget(self.btn_save_nvs)
        
        center_layout.addWidget(header_card)
        
        # ==============================================================
        # --- 2-COLUMN MAIN LAYOUT (Dividido ao meio sem rolagem) ---
        # ==============================================================
        cols_layout = QHBoxLayout()
        cols_layout.setContentsMargins(0, 0, 0, 0)
        cols_layout.setSpacing(10)
        
        # --------------------------------------------------------------
        # COLUNA ESQUERDA: Cinemática + Limites Angulares
        # --------------------------------------------------------------
        left_vbox = QVBoxLayout()
        left_vbox.setContentsMargins(0, 0, 0, 0)
        left_vbox.setSpacing(8)
        
        # 1. Cinemática e Movimento
        kin_card = QFrame()
        kin_card.setProperty("class", "card")
        kin_layout = QVBoxLayout(kin_card)
        kin_layout.setContentsMargins(12, 8, 12, 8)
        kin_layout.setSpacing(6)
        
        kin_title = QLabel("1. Cinemática e Movimento")
        kin_title.setProperty("class", "section-title")
        add_class(kin_title, "card-title")
        kin_layout.addWidget(kin_title)
        
        kin_grid = QGridLayout()
        kin_grid.setContentsMargins(0, 0, 0, 0)
        kin_grid.setSpacing(5)
        
        # Column Headers
        h_axis = QLabel("Eixo")
        add_class(h_axis, "table-header")
        h_axis.setFixedWidth(95)
        h_steps = QLabel("Passos")
        h_steps.setToolTip("Passos base do motor por volta")
        add_class(h_steps, "table-header")
        h_steps.setFixedWidth(80)
        h_speed = QLabel("Vel. Máx")
        h_speed.setToolTip("Velocidade Máxima (°/s ou mm/s)")
        add_class(h_speed, "table-header")
        h_speed.setFixedWidth(120)
        h_accel = QLabel("Acel. Máx")
        h_accel.setToolTip("Aceleração Máxima (°/s² ou mm/s²)")
        add_class(h_accel, "table-header")
        h_accel.setFixedWidth(125)
        h_ramp = QLabel("Rampa Inicial")
        h_ramp.setToolTip("Velocidade inicial de partida da rampa S-Curve (°/s ou mm/s)")
        add_class(h_ramp, "table-header")
        h_ramp.setFixedWidth(120)
        h_inv = QLabel("Inv.")
        h_inv.setToolTip("Inverter sentido no hardware")
        add_class(h_inv, "table-header")
        h_inv.setFixedWidth(48)
        
        kin_grid.addWidget(h_axis, 0, 0)
        kin_grid.addWidget(h_steps, 0, 1)
        kin_grid.addWidget(h_speed, 0, 2)
        kin_grid.addWidget(h_accel, 0, 3)
        kin_grid.addWidget(h_ramp, 0, 4)
        kin_grid.addWidget(h_inv, 0, 5)
        
        INPUT_STYLE = (
            "font-size: 13px; font-weight: 500; padding: 2px 6px; "
            "background-color: #0b1220; color: #f1f5f9; "
            "border: 1px solid #1f2e47; border-radius: 5px;"
        )
        COMBO_STYLE = (
            "font-size: 13px; font-weight: 500; padding: 2px 24px 2px 8px; "
            "background-color: #0b1220; color: #f1f5f9; "
            "border: 1px solid #1f2e47; border-radius: 5px;"
        )
        
        # Axis C
        lbl_c = QLabel("Eixo C (Base):")
        add_class(lbl_c, "row-label")
        lbl_c.setFixedWidth(95)
        kin_grid.addWidget(lbl_c, 1, 0)
        self.spin_steps_c = QSpinBox()
        self.spin_steps_c.setRange(20, 10000)
        self.spin_steps_c.setValue(200)
        self.spin_steps_c.setFixedWidth(80)
        self.spin_steps_c.setFixedHeight(28)
        add_class(self.spin_steps_c, "compact-input")
        self.spin_steps_c.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_c, 1, 1)
        self.spin_speed_c = QDoubleSpinBox()
        self.spin_speed_c.setRange(0.1, 10000.0)
        self.spin_speed_c.setDecimals(1)
        self.spin_speed_c.setValue(140.0)
        self.spin_speed_c.setSuffix(" °/s")
        self.spin_speed_c.setFixedWidth(120)
        self.spin_speed_c.setFixedHeight(28)
        add_class(self.spin_speed_c, "compact-input")
        kin_grid.addWidget(self.spin_speed_c, 1, 2)
        self.spin_accel_c = QDoubleSpinBox()
        self.spin_accel_c.setRange(1.0, 50000.0)
        self.spin_accel_c.setValue(1800.0)
        self.spin_accel_c.setSuffix(" °/s²")
        self.spin_accel_c.setFixedWidth(125)
        self.spin_accel_c.setFixedHeight(28)
        add_class(self.spin_accel_c, "compact-input")
        kin_grid.addWidget(self.spin_accel_c, 1, 3)
        self.spin_ramp_c = QDoubleSpinBox()
        self.spin_ramp_c.setRange(0.5, 100.0)
        self.spin_ramp_c.setDecimals(1)
        self.spin_ramp_c.setValue(10.0)
        self.spin_ramp_c.setSingleStep(0.5)
        self.spin_ramp_c.setSuffix(" °/s")
        self.spin_ramp_c.setFixedWidth(120)
        self.spin_ramp_c.setFixedHeight(28)
        add_class(self.spin_ramp_c, "compact-input")
        self.spin_ramp_c.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo C (salvo na NVS)")
        self.spin_ramp_c.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_c, 1, 4)
        self.chk_inv_hw_c = QCheckBox("Inv")
        self.chk_inv_hw_c.setChecked(True)
        self.chk_inv_hw_c.setFixedWidth(48)
        add_class(self.chk_inv_hw_c, "check-label")
        kin_grid.addWidget(self.chk_inv_hw_c, 1, 5)
        
        # Axis A
        lbl_a = QLabel("Eixo A (Pivot):")
        add_class(lbl_a, "row-label")
        lbl_a.setFixedWidth(95)
        kin_grid.addWidget(lbl_a, 2, 0)
        self.spin_steps_a = QSpinBox()
        self.spin_steps_a.setRange(20, 10000)
        self.spin_steps_a.setValue(200)
        self.spin_steps_a.setFixedWidth(80)
        self.spin_steps_a.setFixedHeight(28)
        add_class(self.spin_steps_a, "compact-input")
        self.spin_steps_a.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_a, 2, 1)
        self.spin_speed_a = QDoubleSpinBox()
        self.spin_speed_a.setRange(0.1, 10000.0)
        self.spin_speed_a.setDecimals(1)
        self.spin_speed_a.setValue(140.0)
        self.spin_speed_a.setSuffix(" °/s")
        self.spin_speed_a.setFixedWidth(120)
        self.spin_speed_a.setFixedHeight(28)
        add_class(self.spin_speed_a, "compact-input")
        kin_grid.addWidget(self.spin_speed_a, 2, 2)
        self.spin_accel_a = QDoubleSpinBox()
        self.spin_accel_a.setRange(1.0, 50000.0)
        self.spin_accel_a.setValue(1800.0)
        self.spin_accel_a.setSuffix(" °/s²")
        self.spin_accel_a.setFixedWidth(125)
        self.spin_accel_a.setFixedHeight(28)
        add_class(self.spin_accel_a, "compact-input")
        kin_grid.addWidget(self.spin_accel_a, 2, 3)
        self.spin_ramp_a = QDoubleSpinBox()
        self.spin_ramp_a.setRange(0.5, 100.0)
        self.spin_ramp_a.setDecimals(1)
        self.spin_ramp_a.setValue(10.0)
        self.spin_ramp_a.setSingleStep(0.5)
        self.spin_ramp_a.setSuffix(" °/s")
        self.spin_ramp_a.setFixedWidth(120)
        self.spin_ramp_a.setFixedHeight(28)
        add_class(self.spin_ramp_a, "compact-input")
        self.spin_ramp_a.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo A (salvo na NVS)")
        self.spin_ramp_a.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_a, 2, 4)
        self.chk_inv_hw_a = QCheckBox("Inv")
        self.chk_inv_hw_a.setChecked(True)
        self.chk_inv_hw_a.setFixedWidth(48)
        add_class(self.chk_inv_hw_a, "check-label")
        kin_grid.addWidget(self.chk_inv_hw_a, 2, 5)
        
        # Axis Z
        lbl_z = QLabel("Eixo Z (Linear):")
        add_class(lbl_z, "row-label")
        lbl_z.setFixedWidth(95)
        kin_grid.addWidget(lbl_z, 3, 0)
        self.spin_steps_z = QSpinBox()
        self.spin_steps_z.setRange(20, 10000)
        self.spin_steps_z.setValue(200)
        self.spin_steps_z.setFixedWidth(80)
        self.spin_steps_z.setFixedHeight(28)
        add_class(self.spin_steps_z, "compact-input")
        self.spin_steps_z.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_z, 3, 1)
        self.spin_speed_z = QDoubleSpinBox()
        self.spin_speed_z.setRange(0.01, 500.0)
        self.spin_speed_z.setDecimals(1)
        self.spin_speed_z.setValue(250.0)
        self.spin_speed_z.setSuffix(" mm/s")
        self.spin_speed_z.setFixedWidth(120)
        self.spin_speed_z.setFixedHeight(28)
        add_class(self.spin_speed_z, "compact-input")
        kin_grid.addWidget(self.spin_speed_z, 3, 2)
        self.spin_accel_z = QDoubleSpinBox()
        self.spin_accel_z.setRange(0.1, 5000.0)
        self.spin_accel_z.setDecimals(1)
        self.spin_accel_z.setValue(1000.0)
        self.spin_accel_z.setSuffix(" mm/s²")
        self.spin_accel_z.setFixedWidth(125)
        self.spin_accel_z.setFixedHeight(28)
        add_class(self.spin_accel_z, "compact-input")
        kin_grid.addWidget(self.spin_accel_z, 3, 3)
        self.spin_ramp_z = QDoubleSpinBox()
        self.spin_ramp_z.setRange(0.5, 150.0)
        self.spin_ramp_z.setDecimals(1)
        self.spin_ramp_z.setValue(15.0)
        self.spin_ramp_z.setSingleStep(1.0)
        self.spin_ramp_z.setSuffix(" mm/s")
        self.spin_ramp_z.setFixedWidth(120)
        self.spin_ramp_z.setFixedHeight(28)
        add_class(self.spin_ramp_z, "compact-input")
        self.spin_ramp_z.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo Z (salvo na NVS)")
        self.spin_ramp_z.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_z, 3, 4)
        self.chk_inv_hw_z = QCheckBox("Inv")
        self.chk_inv_hw_z.setChecked(False)
        self.chk_inv_hw_z.setFixedWidth(48)
        add_class(self.chk_inv_hw_z, "check-label")
        kin_grid.addWidget(self.chk_inv_hw_z, 3, 5)
        kin_grid.setColumnStretch(6, 1)
        
        kin_layout.addLayout(kin_grid)
        
        # Sub-Card: Parâmetros Físicos do Eixo Z (Polia GT2 e Limite em mm)
        z_extra_frame = QFrame()
        z_extra_frame.setProperty("class", "metric-card")
        z_extra_layout = QVBoxLayout(z_extra_frame)
        z_extra_layout.setContentsMargins(10, 6, 10, 6)
        z_extra_layout.setSpacing(4)
        
        z_config_row = QHBoxLayout()
        z_config_row.setContentsMargins(0, 0, 0, 0)
        z_config_row.setSpacing(12)
        
        lbl_pulley = QLabel("Polia Z:")
        add_class(lbl_pulley, "field-label")
        lbl_pulley.setFixedWidth(54)
        z_config_row.addWidget(lbl_pulley)
        self.spin_pulley_z = QSpinBox()
        self.spin_pulley_z.setRange(6, 200)
        self.spin_pulley_z.setValue(20)
        self.spin_pulley_z.setSuffix(" T")
        self.spin_pulley_z.setFixedWidth(80)
        self.spin_pulley_z.setFixedHeight(28)
        add_class(self.spin_pulley_z, "compact-input")
        self.spin_pulley_z.valueChanged.connect(self._update_kinematics_preview)
        z_config_row.addWidget(self.spin_pulley_z)
        
        lbl_max_z = QLabel("Curso Z:")
        add_class(lbl_max_z, "field-label")
        lbl_max_z.setFixedWidth(62)
        z_config_row.addWidget(lbl_max_z)
        self.spin_max_z_mm = QDoubleSpinBox()
        self.spin_max_z_mm.setRange(1.0, 500.0)
        self.spin_max_z_mm.setDecimals(1)
        self.spin_max_z_mm.setValue(480.0)
        self.spin_max_z_mm.setSingleStep(5.0)
        self.spin_max_z_mm.setSuffix(" mm")
        self.spin_max_z_mm.setFixedWidth(110)
        self.spin_max_z_mm.setFixedHeight(28)
        add_class(self.spin_max_z_mm, "compact-input")
        self.spin_max_z_mm.valueChanged.connect(self._update_kinematics_preview)
        self.spin_max_z = self.spin_max_z_mm
        z_config_row.addWidget(self.spin_max_z_mm)
        z_config_row.addStretch(1)
        z_extra_layout.addLayout(z_config_row)

        # Motor de movimento do firmware: S-curve no ISR + encadeamento (lookahead com jerk)
        motion_row = QHBoxLayout()
        motion_row.setContentsMargins(0, 0, 0, 0)
        motion_row.setSpacing(10)
        lbl_engine = QLabel("Motor:")
        lbl_engine.setProperty("class", "field-label")
        motion_row.addWidget(lbl_engine)
        self.combo_motion_engine = QComboBox()
        self.combo_motion_engine.addItems(["STREAM", "LEGACY"])
        self.combo_motion_engine.setToolTip(
            "STREAM: perfil S-curve no domínio do tempo gerado no ISR do RMT, eixos alinhados e "
            "encadeamento de movimentos.\nLEGACY: motor anterior (rampas por tabela, um movimento por vez)."
        )
        self.combo_motion_engine.setFixedWidth(100)
        self.combo_motion_engine.setFixedHeight(28)
        add_class(self.combo_motion_engine, "compact-combo")
        motion_row.addWidget(self.combo_motion_engine)
        self.chk_lookahead = QCheckBox("Encadear movimentos")
        self.chk_lookahead.setChecked(True)
        self.chk_lookahead.setToolTip(
            "Com comandos já na fila, passa de um movimento ao seguinte sem parar "
            "(salto de velocidade por eixo limitado pelo jerk)."
        )
        motion_row.addWidget(self.chk_lookahead)
        motion_row.addStretch(1)
        z_extra_layout.addLayout(motion_row)

        jerk_row = QHBoxLayout()
        jerk_row.setContentsMargins(0, 0, 0, 0)
        jerk_row.setSpacing(10)
        motion_row = jerk_row
        lbl_jerk = QLabel("Jerk C / A / Z:")
        lbl_jerk.setProperty("class", "field-label")
        lbl_jerk.setToolTip("Salto máximo de velocidade de cada eixo numa junção entre movimentos")
        motion_row.addWidget(lbl_jerk)
        self.spin_jerk = []
        for default, suffix in ((15.0, " °/s"), (15.0, " °/s"), (10.0, " mm/s")):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(1)
            spin.setValue(default)
            spin.setSuffix(suffix)
            spin.setFixedWidth(92)
            spin.setFixedHeight(28)
            add_class(spin, "compact-input")
            motion_row.addWidget(spin)
            self.spin_jerk.append(spin)
        motion_row.addStretch(1)
        z_extra_layout.addLayout(motion_row)
        
        # Dynamic preview calculation banner (with wordwrap enabled)
        self.lbl_kin_calc_info = QLabel("Cálculo cinemático: ...")
        self.lbl_kin_calc_info.setWordWrap(True)
        self.lbl_kin_calc_info.setStyleSheet("color: #38bdf8; font-weight: 600; font-size: 11.5px;")
        z_extra_layout.addWidget(self.lbl_kin_calc_info)
        
        kin_layout.addWidget(z_extra_frame)
        left_vbox.addWidget(kin_card)
        
        # 2. Limites Angulares dos Encoders
        limits_card = QFrame()
        limits_card.setProperty("class", "card")
        limits_layout = QVBoxLayout(limits_card)
        limits_layout.setContentsMargins(12, 8, 12, 8)
        limits_layout.setSpacing(6)
        
        limits_title = QLabel("2. Limites Angulares dos Encoders Magnéticos")
        limits_title.setProperty("class", "section-title")
        add_class(limits_title, "card-title")
        limits_layout.addWidget(limits_title)
        
        limits_grid = QGridLayout()
        limits_grid.setContentsMargins(0, 0, 0, 0)
        limits_grid.setSpacing(5)
        
        lh_axis = QLabel("Eixo")
        add_class(lh_axis, "table-header")
        lh_axis.setFixedWidth(95)
        lh_min = QLabel("Limite Mín")
        add_class(lh_min, "table-header")
        lh_min.setFixedWidth(120)
        lh_max = QLabel("Limite Máx")
        add_class(lh_max, "table-header")
        lh_max.setFixedWidth(120)
        lh_desc = QLabel("Operação")
        add_class(lh_desc, "table-header")
        lh_desc.setFixedWidth(135)
        
        limits_grid.addWidget(lh_axis, 0, 0)
        limits_grid.addWidget(lh_min, 0, 1)
        limits_grid.addWidget(lh_max, 0, 2)
        limits_grid.addWidget(lh_desc, 0, 3)
        
        # Eixo C
        lbl_lim_c = QLabel("Eixo C (Base):")
        add_class(lbl_lim_c, "row-label")
        lbl_lim_c.setFixedWidth(95)
        limits_grid.addWidget(lbl_lim_c, 1, 0)
        self.spin_limit_min_c = QDoubleSpinBox()
        self.spin_limit_min_c.setRange(-3600.0, 3600.0)
        self.spin_limit_min_c.setDecimals(1)
        self.spin_limit_min_c.setValue(-540.0)
        self.spin_limit_min_c.setSuffix(" °")
        self.spin_limit_min_c.setFixedWidth(120)
        self.spin_limit_min_c.setFixedHeight(28)
        add_class(self.spin_limit_min_c, "compact-input")
        limits_grid.addWidget(self.spin_limit_min_c, 1, 1)
        self.spin_limit_max_c = QDoubleSpinBox()
        self.spin_limit_max_c.setRange(-3600.0, 3600.0)
        self.spin_limit_max_c.setDecimals(1)
        self.spin_limit_max_c.setValue(540.0)
        self.spin_limit_max_c.setSuffix(" °")
        self.spin_limit_max_c.setFixedWidth(120)
        self.spin_limit_max_c.setFixedHeight(28)
        add_class(self.spin_limit_max_c, "compact-input")
        limits_grid.addWidget(self.spin_limit_max_c, 1, 2)
        lbl_desc_c = QLabel("Giro base contínuo")
        add_class(lbl_desc_c, "muted")
        lbl_desc_c.setFixedWidth(135)
        limits_grid.addWidget(lbl_desc_c, 1, 3)
        
        # Eixo A
        lbl_lim_a = QLabel("Eixo A (Pivot):")
        add_class(lbl_lim_a, "row-label")
        lbl_lim_a.setFixedWidth(95)
        limits_grid.addWidget(lbl_lim_a, 2, 0)
        self.spin_limit_min_a = QDoubleSpinBox()
        self.spin_limit_min_a.setRange(-3600.0, 3600.0)
        self.spin_limit_min_a.setDecimals(1)
        self.spin_limit_min_a.setValue(-540.0)
        self.spin_limit_min_a.setSuffix(" °")
        self.spin_limit_min_a.setFixedWidth(120)
        self.spin_limit_min_a.setFixedHeight(28)
        add_class(self.spin_limit_min_a, "compact-input")
        limits_grid.addWidget(self.spin_limit_min_a, 2, 1)
        self.spin_limit_max_a = QDoubleSpinBox()
        self.spin_limit_max_a.setRange(-3600.0, 3600.0)
        self.spin_limit_max_a.setDecimals(1)
        self.spin_limit_max_a.setValue(540.0)
        self.spin_limit_max_a.setSuffix(" °")
        self.spin_limit_max_a.setFixedWidth(120)
        self.spin_limit_max_a.setFixedHeight(28)
        add_class(self.spin_limit_max_a, "compact-input")
        limits_grid.addWidget(self.spin_limit_max_a, 2, 2)
        lbl_desc_a = QLabel("Pivot do cabeçote")
        add_class(lbl_desc_a, "muted")
        lbl_desc_a.setFixedWidth(135)
        limits_grid.addWidget(lbl_desc_a, 2, 3)
        limits_grid.setColumnStretch(4, 1)
        
        limits_layout.addLayout(limits_grid)
        left_vbox.addWidget(limits_card)
        left_vbox.addStretch()
        cols_layout.addLayout(left_vbox, 1)
        
        # --------------------------------------------------------------
        # COLUNA DIREITA: Drivers TMC2209 + Barramento CAN
        # --------------------------------------------------------------
        right_vbox = QVBoxLayout()
        right_vbox.setContentsMargins(0, 0, 0, 0)
        right_vbox.setSpacing(8)
        
        # 3. Drivers TMC2209
        tmc_card = QFrame()
        tmc_card.setProperty("class", "card")
        tmc_layout = QVBoxLayout(tmc_card)
        tmc_layout.setContentsMargins(12, 8, 12, 8)
        tmc_layout.setSpacing(6)
        
        # TMC Header Row (Title + Mode)
        tmc_header = QHBoxLayout()
        tmc_header.setContentsMargins(0, 0, 0, 0)
        tmc_header.setSpacing(8)
        tmc_title = QLabel("3. Drivers TMC2209 (UART)")
        tmc_title.setProperty("class", "section-title")
        add_class(tmc_title, "card-title")
        tmc_header.addWidget(tmc_title)
        tmc_header.addStretch()
        
        lbl_mode = QLabel("Modo:")
        add_class(lbl_mode, "field-label")
        tmc_header.addWidget(lbl_mode)
        self.combo_driver_mode = QComboBox()
        self.combo_driver_mode.addItems(["UART Digital", "STEP/DIR Legado"])
        self.combo_driver_mode.setCurrentIndex(0)
        self.combo_driver_mode.setFixedWidth(155)
        self.combo_driver_mode.setFixedHeight(28)
        add_class(self.combo_driver_mode, "compact-combo")
        tmc_header.addWidget(self.combo_driver_mode)
        tmc_layout.addLayout(tmc_header)
        
        tmc_grid = QGridLayout()
        tmc_grid.setContentsMargins(0, 0, 0, 0)
        tmc_grid.setSpacing(5)
        
        th_driver = QLabel("Driver")
        add_class(th_driver, "table-header")
        th_driver.setFixedWidth(95)
        th_addr = QLabel("Endr")
        th_addr.setToolTip("Endereço UART (0..3)")
        add_class(th_addr, "table-header")
        th_addr.setFixedWidth(55)
        th_irun = QLabel("iRun")
        th_irun.setToolTip("Corrente de trabalho em mA")
        add_class(th_irun, "table-header")
        th_irun.setFixedWidth(105)
        th_ihold = QLabel("iHold")
        th_ihold.setToolTip("Corrente de repouso em mA")
        add_class(th_ihold, "table-header")
        th_ihold.setFixedWidth(105)
        th_usteps = QLabel("Micropassos")
        add_class(th_usteps, "table-header")
        th_usteps.setFixedWidth(75)
        th_sc = QLabel("SC")
        th_sc.setToolTip("SpreadCycle ativado")
        add_class(th_sc, "table-header")
        th_sc.setFixedWidth(45)
        
        tmc_grid.addWidget(th_driver, 0, 0)
        tmc_grid.addWidget(th_addr, 0, 1)
        tmc_grid.addWidget(th_irun, 0, 2)
        tmc_grid.addWidget(th_ihold, 0, 3)
        tmc_grid.addWidget(th_usteps, 0, 4)
        tmc_grid.addWidget(th_sc, 0, 5)
        
        # Driver C
        lbl_drv_c = QLabel("Driver C (Base):")
        add_class(lbl_drv_c, "row-label")
        lbl_drv_c.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_c, 1, 0)
        self.spin_tmc_addr_c = QSpinBox()
        self.spin_tmc_addr_c.setRange(0, 3)
        self.spin_tmc_addr_c.setValue(0)
        self.spin_tmc_addr_c.setFixedWidth(55)
        self.spin_tmc_addr_c.setFixedHeight(28)
        add_class(self.spin_tmc_addr_c, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_addr_c, 1, 1)
        self.spin_tmc_irun_c = QSpinBox()
        self.spin_tmc_irun_c.setRange(50, 2000)
        self.spin_tmc_irun_c.setValue(897)
        self.spin_tmc_irun_c.setSuffix(" mA")
        self.spin_tmc_irun_c.setFixedWidth(105)
        self.spin_tmc_irun_c.setFixedHeight(28)
        add_class(self.spin_tmc_irun_c, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_irun_c, 1, 2)
        self.spin_tmc_ihold_c = QSpinBox()
        self.spin_tmc_ihold_c.setRange(50, 2000)
        self.spin_tmc_ihold_c.setValue(559)
        self.spin_tmc_ihold_c.setSuffix(" mA")
        self.spin_tmc_ihold_c.setFixedWidth(105)
        self.spin_tmc_ihold_c.setFixedHeight(28)
        add_class(self.spin_tmc_ihold_c, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_ihold_c, 1, 3)
        self.combo_tmc_usteps_c = QComboBox()
        self.combo_tmc_usteps_c.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_c.setCurrentText("16")
        self.combo_tmc_usteps_c.setFixedWidth(75)
        self.combo_tmc_usteps_c.setFixedHeight(28)
        add_class(self.combo_tmc_usteps_c, "compact-combo")
        self.combo_tmc_usteps_c.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_c, 1, 4)
        self.chk_tmc_sc_c = QCheckBox("SC")
        self.chk_tmc_sc_c.setFixedWidth(45)
        add_class(self.chk_tmc_sc_c, "check-label")
        tmc_grid.addWidget(self.chk_tmc_sc_c, 1, 5)
        
        # Driver A
        lbl_drv_a = QLabel("Driver A (Pivot):")
        add_class(lbl_drv_a, "row-label")
        lbl_drv_a.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_a, 2, 0)
        self.spin_tmc_addr_a = QSpinBox()
        self.spin_tmc_addr_a.setRange(0, 3)
        self.spin_tmc_addr_a.setValue(1)
        self.spin_tmc_addr_a.setFixedWidth(55)
        self.spin_tmc_addr_a.setFixedHeight(28)
        add_class(self.spin_tmc_addr_a, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_addr_a, 2, 1)
        self.spin_tmc_irun_a = QSpinBox()
        self.spin_tmc_irun_a.setRange(50, 2000)
        self.spin_tmc_irun_a.setValue(897)
        self.spin_tmc_irun_a.setSuffix(" mA")
        self.spin_tmc_irun_a.setFixedWidth(105)
        self.spin_tmc_irun_a.setFixedHeight(28)
        add_class(self.spin_tmc_irun_a, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_irun_a, 2, 2)
        self.spin_tmc_ihold_a = QSpinBox()
        self.spin_tmc_ihold_a.setRange(50, 2000)
        self.spin_tmc_ihold_a.setValue(559)
        self.spin_tmc_ihold_a.setSuffix(" mA")
        self.spin_tmc_ihold_a.setFixedWidth(105)
        self.spin_tmc_ihold_a.setFixedHeight(28)
        add_class(self.spin_tmc_ihold_a, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_ihold_a, 2, 3)
        self.combo_tmc_usteps_a = QComboBox()
        self.combo_tmc_usteps_a.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_a.setCurrentText("16")
        self.combo_tmc_usteps_a.setFixedWidth(75)
        self.combo_tmc_usteps_a.setFixedHeight(28)
        add_class(self.combo_tmc_usteps_a, "compact-combo")
        self.combo_tmc_usteps_a.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_a, 2, 4)
        self.chk_tmc_sc_a = QCheckBox("SC")
        self.chk_tmc_sc_a.setFixedWidth(45)
        add_class(self.chk_tmc_sc_a, "check-label")
        tmc_grid.addWidget(self.chk_tmc_sc_a, 2, 5)
        
        # Driver Z
        lbl_drv_z = QLabel("Driver Z (Linear):")
        add_class(lbl_drv_z, "row-label")
        lbl_drv_z.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_z, 3, 0)
        self.spin_tmc_addr_z = QSpinBox()
        self.spin_tmc_addr_z.setRange(0, 3)
        self.spin_tmc_addr_z.setValue(2)
        self.spin_tmc_addr_z.setFixedWidth(55)
        self.spin_tmc_addr_z.setFixedHeight(28)
        add_class(self.spin_tmc_addr_z, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_addr_z, 3, 1)
        self.spin_tmc_irun_z = QSpinBox()
        self.spin_tmc_irun_z.setRange(50, 2000)
        self.spin_tmc_irun_z.setValue(957)
        self.spin_tmc_irun_z.setSuffix(" mA")
        self.spin_tmc_irun_z.setFixedWidth(105)
        self.spin_tmc_irun_z.setFixedHeight(28)
        add_class(self.spin_tmc_irun_z, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_irun_z, 3, 2)
        self.spin_tmc_ihold_z = QSpinBox()
        self.spin_tmc_ihold_z.setRange(50, 2000)
        self.spin_tmc_ihold_z.setValue(418)
        self.spin_tmc_ihold_z.setSuffix(" mA")
        self.spin_tmc_ihold_z.setFixedWidth(105)
        self.spin_tmc_ihold_z.setFixedHeight(28)
        add_class(self.spin_tmc_ihold_z, "compact-input")
        tmc_grid.addWidget(self.spin_tmc_ihold_z, 3, 3)
        self.combo_tmc_usteps_z = QComboBox()
        self.combo_tmc_usteps_z.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_z.setCurrentText("16")
        self.combo_tmc_usteps_z.setFixedWidth(75)
        self.combo_tmc_usteps_z.setFixedHeight(28)
        add_class(self.combo_tmc_usteps_z, "compact-combo")
        self.combo_tmc_usteps_z.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_z, 3, 4)
        self.chk_tmc_sc_z = QCheckBox("SC")
        self.chk_tmc_sc_z.setFixedWidth(45)
        add_class(self.chk_tmc_sc_z, "check-label")
        tmc_grid.addWidget(self.chk_tmc_sc_z, 3, 5)

        # Limiar stealthChop -> spreadCycle (TPWMTHRS): silencioso devagar, torque em alta
        th_stealth = QLabel("Stealth até")
        th_stealth.setToolTip(
            "Velocidade até a qual o driver usa stealthChop (silencioso); acima dela troca para "
            "spreadCycle (mais torque em alta rotação). 0 = stealthChop sempre. Ignorado com SC marcado."
        )
        add_class(th_stealth, "table-header")
        tmc_grid.addWidget(th_stealth, 0, 6)
        self.spin_tmc_stealth = []
        for row, (default, suffix) in enumerate(((180.0, " °/s"), (180.0, " °/s"), (60.0, " mm/s")), start=1):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 5000.0)
            spin.setDecimals(0)
            spin.setValue(default)
            spin.setSuffix(suffix)
            spin.setFixedWidth(110)
            spin.setFixedHeight(28)
            add_class(spin, "compact-input")
            tmc_grid.addWidget(spin, row, 6)
            self.spin_tmc_stealth.append(spin)

        # A corrente do TMC2209 é quantizada em degraus de ~60 mA: ao terminar a edição o
        # campo mostra o valor que o driver realmente vai usar (antes o CONFIG DUMP voltava
        # "diferente" do digitado, ex.: 559 -> 539 mA).
        for spin in (self.spin_tmc_irun_c, self.spin_tmc_irun_a, self.spin_tmc_irun_z,
                     self.spin_tmc_ihold_c, self.spin_tmc_ihold_a, self.spin_tmc_ihold_z):
            spin.setToolTip("Corrente RMS; ajustada ao degrau real do TMC2209 (~60 mA)")
            spin.editingFinished.connect(lambda s=spin: s.setValue(tmc_quantize_ma(s.value())))
        tmc_grid.setColumnStretch(7, 1)
        
        tmc_layout.addLayout(tmc_grid)
        
        # Sub-Card: Diagnóstico e Acesso Direto a Registradores TMC2209
        reg_frame = QFrame()
        reg_frame.setProperty("class", "metric-card")
        reg_layout = QHBoxLayout(reg_frame)
        reg_layout.setContentsMargins(10, 6, 10, 6)
        reg_layout.setSpacing(6)
        
        lbl_diag = QLabel("Reg TMC:")
        add_class(lbl_diag, "field-label")
        lbl_diag.setFixedWidth(65)
        reg_layout.addWidget(lbl_diag)
        self.combo_reg_axis = QComboBox()
        self.combo_reg_axis.addItems(["C", "A", "Z"])
        self.combo_reg_axis.setFixedWidth(65)
        self.combo_reg_axis.setFixedHeight(28)
        add_class(self.combo_reg_axis, "compact-combo")
        reg_layout.addWidget(self.combo_reg_axis)
        self.txt_reg_addr = QLineEdit("0x06")
        self.txt_reg_addr.setFixedWidth(65)
        self.txt_reg_addr.setFixedHeight(28)
        self.txt_reg_addr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        add_class(self.txt_reg_addr, "compact-input")
        reg_layout.addWidget(self.txt_reg_addr)
        self.txt_reg_val = QLineEdit("0x00000000")
        self.txt_reg_val.setFixedWidth(105)
        self.txt_reg_val.setFixedHeight(28)
        self.txt_reg_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        add_class(self.txt_reg_val, "compact-input")
        reg_layout.addWidget(self.txt_reg_val)
        self.btn_reg_read = QPushButton("📖 Ler")
        self.btn_reg_read.setFixedWidth(65)
        self.btn_reg_read.setFixedHeight(28)
        self.btn_reg_read.setStyleSheet("font-size: 12px; padding: 2px 8px;")
        self.btn_reg_read.clicked.connect(self._read_tmc_reg)
        reg_layout.addWidget(self.btn_reg_read)
        self.btn_reg_write = QPushButton("✏️ Gravar")
        self.btn_reg_write.setFixedWidth(75)
        self.btn_reg_write.setFixedHeight(28)
        self.btn_reg_write.setStyleSheet("font-size: 12px; padding: 2px 8px;")
        self.btn_reg_write.clicked.connect(self._write_tmc_reg)
        reg_layout.addWidget(self.btn_reg_write)
        reg_layout.addStretch(1)
        tmc_layout.addWidget(reg_frame)
        right_vbox.addWidget(tmc_card)
        
        # 4. Barramento CAN / TWAI
        can_card = QFrame()
        can_card.setProperty("class", "card")
        can_layout = QVBoxLayout(can_card)
        can_layout.setContentsMargins(12, 8, 12, 8)
        can_layout.setSpacing(6)
        
        can_header = QHBoxLayout()
        can_header.setContentsMargins(0, 0, 0, 0)
        can_header.setSpacing(8)
        can_title = QLabel("4. Barramento CAN / TWAI (10 Nós)")
        can_title.setProperty("class", "section-title")
        add_class(can_title, "card-title")
        can_header.addWidget(can_title)
        can_header.addStretch()
        
        self.chk_can_enabled = QCheckBox("Habilitar CAN")
        add_class(self.chk_can_enabled, "accent")
        self.chk_can_enabled.toggled.connect(self._on_can_enabled_toggled)
        can_header.addWidget(self.chk_can_enabled)
        
        self.lbl_can_status_badge = QLabel("DESATIVADO")
        self.lbl_can_status_badge.setProperty("class", "badge badge-gray")
        self.lbl_can_status_badge.setStyleSheet("font-size: 11px; padding: 2px 8px; font-weight: 700;")
        can_header.addWidget(self.lbl_can_status_badge)
        can_layout.addLayout(can_header)
        
        can_grid = QGridLayout()
        can_grid.setContentsMargins(0, 0, 0, 0)
        can_grid.setSpacing(6)
        
        lbl_node = QLabel("Node ID (1..10):")
        add_class(lbl_node, "field-label")
        lbl_node.setFixedWidth(105)
        can_grid.addWidget(lbl_node, 0, 0)
        self.spin_node_id = QSpinBox()
        self.spin_node_id.setRange(1, 10)
        self.spin_node_id.setValue(1)
        self.spin_node_id.setFixedWidth(75)
        self.spin_node_id.setFixedHeight(28)
        add_class(self.spin_node_id, "compact-input")
        can_grid.addWidget(self.spin_node_id, 0, 1)
        
        lbl_bitrate = QLabel("Bitrate:")
        add_class(lbl_bitrate, "field-label")
        lbl_bitrate.setFixedWidth(65)
        can_grid.addWidget(lbl_bitrate, 0, 2)
        self.combo_can_bitrate = QComboBox()
        self.combo_can_bitrate.addItems(["125000", "250000", "500000", "1000000"])
        self.combo_can_bitrate.setCurrentText("500000")
        self.combo_can_bitrate.setFixedWidth(105)
        self.combo_can_bitrate.setFixedHeight(28)
        add_class(self.combo_can_bitrate, "compact-combo")
        can_grid.addWidget(self.combo_can_bitrate, 0, 3)
        
        lbl_base_cmd = QLabel("Base Cmd:")
        add_class(lbl_base_cmd, "field-label")
        lbl_base_cmd.setFixedWidth(105)
        can_grid.addWidget(lbl_base_cmd, 1, 0)
        self.txt_base_cmd = QLineEdit("0x200")
        self.txt_base_cmd.setFixedWidth(75)
        self.txt_base_cmd.setFixedHeight(28)
        self.txt_base_cmd.setAlignment(Qt.AlignmentFlag.AlignCenter)
        add_class(self.txt_base_cmd, "compact-input")
        can_grid.addWidget(self.txt_base_cmd, 1, 1)
        
        lbl_base_st = QLabel("Base Status:")
        add_class(lbl_base_st, "field-label")
        lbl_base_st.setFixedWidth(85)
        can_grid.addWidget(lbl_base_st, 1, 2)
        self.txt_base_status = QLineEdit("0x280")
        self.txt_base_status.setFixedWidth(105)
        self.txt_base_status.setFixedHeight(28)
        self.txt_base_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        add_class(self.txt_base_status, "compact-input")
        can_grid.addWidget(self.txt_base_status, 1, 3)
        
        lbl_base_ev = QLabel("Base Eventos:")
        add_class(lbl_base_ev, "field-label")
        lbl_base_ev.setFixedWidth(105)
        can_grid.addWidget(lbl_base_ev, 2, 0)
        self.txt_base_event = QLineEdit("0x300")
        self.txt_base_event.setFixedWidth(75)
        self.txt_base_event.setFixedHeight(28)
        self.txt_base_event.setAlignment(Qt.AlignmentFlag.AlignCenter)
        add_class(self.txt_base_event, "compact-input")
        can_grid.addWidget(self.txt_base_event, 2, 1)
        can_grid.setColumnStretch(4, 1)
        
        can_layout.addLayout(can_grid)
        right_vbox.addWidget(can_card)
        right_vbox.addStretch()
        cols_layout.addLayout(right_vbox, 1)
        
        center_layout.addLayout(cols_layout)
        
        # ==============================================================
        # --- PAINEL DE AÇÃO UNIFICADO (RODAPÉ COMPACTO) ---
        # ==============================================================
        action_card = QFrame()
        action_card.setProperty("class", "card")
        action_card.setStyleSheet("border: 1px solid #1e3a8a; background-color: #0f172a;")
        action_layout = QHBoxLayout(action_card)
        action_layout.setContentsMargins(14, 8, 14, 8)
        action_layout.setSpacing(12)
        
        lbl_act_desc = QLabel("💡 Ao gravar, todos os parâmetros das 4 seções acima são persistidos em lote na memória NVS do ESP32.")
        lbl_act_desc.setWordWrap(True)
        lbl_act_desc.setStyleSheet("color: #94a3b8; font-size: 12px;")
        action_layout.addWidget(lbl_act_desc, 1)
        
        self.btn_refresh_bottom = QPushButton("📥  Ler da Placa")
        self.btn_refresh_bottom.setStyleSheet("font-size: 12.5px; min-height: 28px; max-height: 32px; padding: 4px 14px;")
        self.btn_refresh_bottom.clicked.connect(self.comm.request_config_dump)
        action_layout.addWidget(self.btn_refresh_bottom)
        
        self.btn_save_all_bottom = QPushButton("💾  Gravar Todas na NVS")
        self.btn_save_all_bottom.setProperty("class", "btn-primary")
        self.btn_save_all_bottom.setStyleSheet("font-weight: 700; font-size: 12.5px; min-height: 28px; max-height: 32px; padding: 4px 16px;")
        self.btn_save_all_bottom.clicked.connect(self._apply_all_parameters)
        action_layout.addWidget(self.btn_save_all_bottom)
        
        center_layout.addWidget(action_card)
        
        # Adiciona a caixa centralizada (com largura contida) ao layout principal
        center_outer.addWidget(center_box)
        center_outer.addStretch(1)
        main_layout.addLayout(center_outer)
        main_layout.addStretch()
        
        self.state.parameters_updated.connect(self._on_parameters_updated)
        self._update_kinematics_preview()

    def _on_can_enabled_toggled(self, checked: bool):
        if checked:
            self.lbl_can_status_badge.setText("ATIVO")
            self.lbl_can_status_badge.setProperty("class", "badge badge-success")
            self.lbl_can_status_badge.setStyleSheet("color: #4ade80; font-weight: 700; font-size: 11px; padding: 2px 8px;")
        else:
            self.lbl_can_status_badge.setText("DESATIVADO")
            self.lbl_can_status_badge.setProperty("class", "badge badge-gray")
            self.lbl_can_status_badge.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 11px; padding: 2px 8px;")

    def _update_kinematics_preview(self):
        teeth = self.spin_pulley_z.value()
        spr_z = self.spin_steps_z.value()
        spr_c = self.spin_steps_c.value()
        try:
            usteps_z = int(self.combo_tmc_usteps_z.currentText())
            usteps_c = int(self.combo_tmc_usteps_c.currentText())
        except ValueError:
            usteps_z = 16
            usteps_c = 16
            
        mm_rev = calc_z_mm_per_rev(teeth)
        mm_step = calc_z_mm_per_step(teeth, spr_z, usteps_z)
        steps_mm = calc_z_steps_per_mm(teeth, spr_z, usteps_z)
        res_um = mm_step * 1000.0
        max_mm = self.spin_max_z_mm.value()
        calc_steps_z = calc_z_steps_for_mm(max_mm, teeth, spr_z, usteps_z)
        ramp_c = self.spin_ramp_c.value()
        ramp_a = self.spin_ramp_a.value()
        ramp_z = self.spin_ramp_z.value()
        
        deg_step_c = 360.0 / (spr_c * usteps_c) if (spr_c * usteps_c) > 0 else 0.1125
        
        self.lbl_kin_calc_info.setText(
            f"Eixo C/A: {spr_c * usteps_c} micropassos/volta ({deg_step_c:.4f}°/passo @ {usteps_c}x) | Rampas: C={ramp_c:.1f}°/s, A={ramp_a:.1f}°/s | "
            f"Eixo Z: {teeth}T GT2 → {mm_rev:.2f} mm/volta ({steps_mm:.1f} passos/mm @ {usteps_z}x) | "
            f"Limite Z: {max_mm:.1f} mm ({calc_steps_z} passos) | Rampa Z: {ramp_z:.1f} mm/s"
        )

    def _read_tmc_reg(self):
        try:
            axis = self.combo_reg_axis.currentText()
            reg = int(self.txt_reg_addr.text(), 0)
            self.comm.read_tmc_reg(axis, reg)
        except ValueError:
            QMessageBox.warning(self, "Valor Inválido", "Endereço do registrador em formato inválido.")

    def _write_tmc_reg(self):
        try:
            axis = self.combo_reg_axis.currentText()
            reg = int(self.txt_reg_addr.text(), 0)
            val = int(self.txt_reg_val.text(), 0)
            self.comm.write_tmc_reg(axis, reg, val)
        except ValueError:
            QMessageBox.warning(self, "Valor Inválido", "Endereço ou valor do registrador em formato inválido.")

    def _on_parameters_updated(self, p: HardwareParameters):
        widgets = [
            self.spin_steps_c, self.spin_steps_a, self.spin_steps_z,
            self.spin_speed_c, self.spin_speed_a, self.spin_speed_z,
            self.spin_accel_c, self.spin_accel_a, self.spin_accel_z,
            self.chk_inv_hw_c, self.chk_inv_hw_a, self.chk_inv_hw_z,
            self.spin_pulley_z, self.spin_max_z_mm,
            self.spin_ramp_c, self.spin_ramp_a, self.spin_ramp_z,
            self.combo_driver_mode,
            self.spin_tmc_addr_c, self.spin_tmc_addr_a, self.spin_tmc_addr_z,
            self.spin_tmc_irun_c, self.spin_tmc_irun_a, self.spin_tmc_irun_z,
            self.spin_tmc_ihold_c, self.spin_tmc_ihold_a, self.spin_tmc_ihold_z,
            self.combo_tmc_usteps_c, self.combo_tmc_usteps_a, self.combo_tmc_usteps_z,
            self.chk_tmc_sc_c, self.chk_tmc_sc_a, self.chk_tmc_sc_z,
            self.chk_can_enabled,
            self.spin_node_id, self.combo_can_bitrate, self.txt_base_cmd, self.txt_base_status, self.txt_base_event,
            self.spin_limit_min_c, self.spin_limit_max_c, self.spin_limit_min_a, self.spin_limit_max_a,
            self.combo_motion_engine, self.chk_lookahead,
        ]
        for w in widgets:
            w.blockSignals(True)

        def set_spin(w, val):
            if not w.hasFocus() and w.value() != val:
                w.setValue(val)

        def set_combo(w, text):
            if not w.hasFocus() and w.currentText() != text:
                w.setCurrentText(text)

        def set_chk(w, checked):
            if not w.hasFocus() and w.isChecked() != checked:
                w.setChecked(checked)

        def set_txt(w, text):
            if not w.hasFocus() and w.text() != text:
                w.setText(text)

        if len(p.steps_per_rev) >= 3:
            set_spin(self.spin_steps_c, p.steps_per_rev[0])
            set_spin(self.spin_steps_a, p.steps_per_rev[1])
            set_spin(self.spin_steps_z, p.steps_per_rev[2])

        if len(p.speed) >= 3:
            set_spin(self.spin_speed_c, p.speed[0])
            set_spin(self.spin_speed_a, p.speed[1])
            set_spin(self.spin_speed_z, p.speed[2])

        if len(p.accel) >= 3:
            set_spin(self.spin_accel_c, p.accel[0])
            set_spin(self.spin_accel_a, p.accel[1])
            set_spin(self.spin_accel_z, p.accel[2])

        if len(p.inverter) >= 3:
            set_chk(self.chk_inv_hw_c, p.inverter[0])
            set_chk(self.chk_inv_hw_a, p.inverter[1])
            set_chk(self.chk_inv_hw_z, p.inverter[2])

        set_spin(self.spin_pulley_z, p.z_pulley_teeth if p.z_pulley_teeth > 0 else DEFAULT_Z_PULLEY_TEETH)
        set_spin(self.spin_ramp_c, getattr(p, 'c_start_speed_deg', 10.0))
        set_spin(self.spin_ramp_a, getattr(p, 'a_start_speed_deg', 10.0))
        set_spin(self.spin_ramp_z, getattr(p, 'z_start_speed_mm', 15.0))
        teeth_z = p.z_pulley_teeth if p.z_pulley_teeth > 0 else DEFAULT_Z_PULLEY_TEETH
        spr_z = p.steps_per_rev[2] if len(p.steps_per_rev) > 2 and p.steps_per_rev[2] > 0 else 200
        usteps_z = p.tmc_microsteps[2] if len(p.tmc_microsteps) > 2 and p.tmc_microsteps[2] > 0 else 16
        max_mm = calc_z_mm_for_steps(p.max_passos_z, teeth_z, spr_z, usteps_z)
        set_spin(self.spin_max_z_mm, min(500.0, max(1.0, round(max_mm, 1))))
        
        if not self.combo_driver_mode.hasFocus():
            self.combo_driver_mode.setCurrentIndex(0 if p.driver_bus_mode == 1 else 1)

        set_spin(self.spin_limit_min_c, p.limit_min_deg_c)
        set_spin(self.spin_limit_max_c, p.limit_max_deg_c)
        set_spin(self.spin_limit_min_a, p.limit_min_deg_a)
        set_spin(self.spin_limit_max_a, p.limit_max_deg_a)

        if len(p.tmc_slave_addr) >= 3:
            set_spin(self.spin_tmc_addr_c, p.tmc_slave_addr[0])
            set_spin(self.spin_tmc_addr_a, p.tmc_slave_addr[1])
            set_spin(self.spin_tmc_addr_z, p.tmc_slave_addr[2])

        if len(p.tmc_irun_ma) >= 3:
            set_spin(self.spin_tmc_irun_c, p.tmc_irun_ma[0])
            set_spin(self.spin_tmc_irun_a, p.tmc_irun_ma[1])
            set_spin(self.spin_tmc_irun_z, p.tmc_irun_ma[2])

        if len(p.tmc_ihold_ma) >= 3:
            set_spin(self.spin_tmc_ihold_c, p.tmc_ihold_ma[0])
            set_spin(self.spin_tmc_ihold_a, p.tmc_ihold_ma[1])
            set_spin(self.spin_tmc_ihold_z, p.tmc_ihold_ma[2])

        if len(p.tmc_microsteps) >= 3:
            set_combo(self.combo_tmc_usteps_c, str(p.tmc_microsteps[0]))
            set_combo(self.combo_tmc_usteps_a, str(p.tmc_microsteps[1]))
            set_combo(self.combo_tmc_usteps_z, str(p.tmc_microsteps[2]))

        if len(p.tmc_spreadcycle) >= 3:
            set_chk(self.chk_tmc_sc_c, p.tmc_spreadcycle[0])
            set_chk(self.chk_tmc_sc_a, p.tmc_spreadcycle[1])
            set_chk(self.chk_tmc_sc_z, p.tmc_spreadcycle[2])

        if not self.combo_motion_engine.hasFocus():
            self.combo_motion_engine.setCurrentText(p.motion_engine)
        set_chk(self.chk_lookahead, p.lookahead)
        for spin, value in zip(self.spin_jerk, p.jerk):
            spin.blockSignals(True)
            set_spin(spin, value)
            spin.blockSignals(False)

        for spin, value in zip(self.spin_tmc_stealth, p.tmc_stealth_max):
            spin.blockSignals(True)
            set_spin(spin, value)
            spin.blockSignals(False)

        # CAN status sync
        set_chk(self.chk_can_enabled, p.can_enabled)
        self._on_can_enabled_toggled(p.can_enabled)

        set_spin(self.spin_node_id, p.node_id)
        set_combo(self.combo_can_bitrate, str(p.can_bitrate))
        set_txt(self.txt_base_cmd, f"0x{p.can_command_base_id:03X}")
        set_txt(self.txt_base_status, f"0x{p.can_status_base_id:03X}")
        set_txt(self.txt_base_event, f"0x{p.can_event_base_id:03X}")

        for w in widgets:
            w.blockSignals(False)

        self._update_kinematics_preview()

    AXES = ("C", "A", "Z")
    CONFIRM_TIMEOUT_MS = 4000

    def _collect_desired(self) -> dict:
        """Captura TODOS os valores da tela no momento do clique.

        Antes cada comando lia o widget só na hora de ser enviado; respostas do nó no meio
        da sequência (ex.: polia/rampa) disparavam parameters_updated e recarregavam os
        campos com os valores antigos — e as correntes antigas eram reenviadas.
        """
        teeth = self.spin_pulley_z.value()
        spr = [self.spin_steps_c.value(), self.spin_steps_a.value(), self.spin_steps_z.value()]
        usteps = []
        for combo in (self.combo_tmc_usteps_c, self.combo_tmc_usteps_a, self.combo_tmc_usteps_z):
            try:
                usteps.append(int(combo.currentText()))
            except ValueError:
                usteps.append(16)
        return {
            "steps_per_rev": spr,
            "speed": [self.spin_speed_c.value(), self.spin_speed_a.value(), self.spin_speed_z.value()],
            "accel": [self.spin_accel_c.value(), self.spin_accel_a.value(), self.spin_accel_z.value()],
            "inverter": [self.chk_inv_hw_c.isChecked(), self.chk_inv_hw_a.isChecked(), self.chk_inv_hw_z.isChecked()],
            "z_pulley_teeth": teeth,
            "max_passos_z": calc_z_steps_for_mm(self.spin_max_z_mm.value(), teeth, spr[2], usteps[2]),
            "ramp": [self.spin_ramp_c.value(), self.spin_ramp_a.value(), self.spin_ramp_z.value()],
            "limits_c": (self.spin_limit_min_c.value(), self.spin_limit_max_c.value()),
            "limits_a": (self.spin_limit_min_a.value(), self.spin_limit_max_a.value()),
            "tmc_addr": [self.spin_tmc_addr_c.value(), self.spin_tmc_addr_a.value(), self.spin_tmc_addr_z.value()],
            "tmc_irun": [tmc_quantize_ma(w.value()) for w in (self.spin_tmc_irun_c, self.spin_tmc_irun_a, self.spin_tmc_irun_z)],
            "tmc_ihold": [tmc_quantize_ma(w.value()) for w in (self.spin_tmc_ihold_c, self.spin_tmc_ihold_a, self.spin_tmc_ihold_z)],
            "tmc_usteps": usteps,
            "tmc_sc": [self.chk_tmc_sc_c.isChecked(), self.chk_tmc_sc_a.isChecked(), self.chk_tmc_sc_z.isChecked()],
            "tmc_stealth": [w.value() for w in self.spin_tmc_stealth],
            "driver_bus_mode": 1 if "UART" in self.combo_driver_mode.currentText() else 0,
            "motion_engine": self.combo_motion_engine.currentText(),
            "lookahead": self.chk_lookahead.isChecked(),
            "jerk": [w.value() for w in self.spin_jerk],
            "can_enabled": self.chk_can_enabled.isChecked(),
            "node_id": self.spin_node_id.value(),
            "can_bitrate": int(self.combo_can_bitrate.currentText()),
            "can_bases": (int(self.txt_base_cmd.text(), 16), int(self.txt_base_status.text(), 16),
                          int(self.txt_base_event.text(), 16)),
        }

    @staticmethod
    def _differs(a, b, tol: float = 0.01) -> bool:
        return abs(float(a) - float(b)) > tol

    def _diff_against_device(self, d: dict, p: HardwareParameters) -> list:
        """Lista (descrição, verificador) do que difere entre a tela e o nó."""
        diffs = []
        for i, ax in enumerate(self.AXES):
            if d["steps_per_rev"][i] != p.steps_per_rev[i]:
                diffs.append((f"Passos/volta {ax}", "steps_per_rev", i))
            if self._differs(d["speed"][i], p.speed[i], 0.05):
                diffs.append((f"Velocidade {ax}", "speed", i))
            if self._differs(d["accel"][i], p.accel[i], 0.05):
                diffs.append((f"Aceleração {ax}", "accel", i))
            if d["inverter"][i] != p.inverter[i]:
                diffs.append((f"Inversão {ax}", "inverter", i))
            if self._differs(d["ramp"][i], (p.c_start_speed_deg, p.a_start_speed_deg, p.z_start_speed_mm)[i], 0.05):
                diffs.append((f"Rampa inicial {ax}", "ramp", i))
            if d["tmc_addr"][i] != p.tmc_slave_addr[i]:
                diffs.append((f"Endereço TMC {ax}", "tmc_addr", i))
            if d["tmc_irun"][i] != tmc_quantize_ma(p.tmc_irun_ma[i]) or \
                    d["tmc_ihold"][i] != tmc_quantize_ma(p.tmc_ihold_ma[i]):
                diffs.append((f"Corrente TMC {ax}", "tmc_current", i))
            if d["tmc_usteps"][i] != p.tmc_microsteps[i]:
                diffs.append((f"Micropassos {ax}", "tmc_usteps", i))
            if d["tmc_sc"][i] != p.tmc_spreadcycle[i]:
                diffs.append((f"SpreadCycle {ax}", "tmc_sc", i))
            if self._differs(d["tmc_stealth"][i], p.tmc_stealth_max[i], 0.5):
                diffs.append((f"Stealth até {ax}", "tmc_stealth", i))
            if self._differs(d["jerk"][i], p.jerk[i], 0.05):
                diffs.append((f"Jerk {ax}", "jerk", i))
        if d["motion_engine"] != p.motion_engine:
            diffs.append(("Motor de movimento", "motion_engine", None))
        if d["lookahead"] != p.lookahead:
            diffs.append(("Encadear movimentos", "lookahead", None))
        if d["z_pulley_teeth"] != p.z_pulley_teeth:
            diffs.append(("Polia Z", "z_pulley_teeth", None))
        if d["max_passos_z"] != p.max_passos_z:
            diffs.append(("Curso máximo Z", "max_passos_z", None))
        if self._differs(d["limits_c"][0], p.limit_min_deg_c) or self._differs(d["limits_c"][1], p.limit_max_deg_c):
            diffs.append(("Limites C", "limits_c", None))
        if self._differs(d["limits_a"][0], p.limit_min_deg_a) or self._differs(d["limits_a"][1], p.limit_max_deg_a):
            diffs.append(("Limites A", "limits_a", None))
        if d["driver_bus_mode"] != p.driver_bus_mode:
            diffs.append(("Modo do driver", "driver_bus_mode", None))
        if (d["can_enabled"] != p.can_enabled or d["node_id"] != p.node_id or d["can_bitrate"] != p.can_bitrate or
                d["can_bases"] != (p.can_command_base_id, p.can_status_base_id, p.can_event_base_id)):
            diffs.append(("Barramento CAN", "can", None))
        return diffs

    def _commands_for(self, client, d: dict, diffs: list) -> list:
        """Comandos só para o que mudou. DRIVER APPLY roda uma vez, e só se precisar."""
        cmds = []
        needs_driver_apply = False
        keys = {key for _, key, _ in diffs}
        for label, key, i in diffs:
            ax = self.AXES[i] if i is not None else None
            if key == "steps_per_rev":
                cmds.append((label, lambda ax=ax, v=d["steps_per_rev"][i]: client.set_steps_per_rev(ax, v)))
            elif key == "speed":
                cmds.append((label, lambda ax=ax, v=d["speed"][i]: client.set_axis_speed(ax, v)))
            elif key == "accel":
                cmds.append((label, lambda ax=ax, v=d["accel"][i]: client.set_axis_accel(ax, v)))
            elif key == "inverter":
                cmds.append((label, lambda ax=ax, v=d["inverter"][i]: client.set_driver_invert(ax, v)))
            elif key == "ramp":
                cmds.append((label, lambda ax=ax, v=d["ramp"][i]: client.set_axis_ramp_speed(ax, v)))
            elif key == "tmc_addr":
                cmds.append((label, lambda ax=ax, v=d["tmc_addr"][i]: client.set_tmc_address(ax, v)))
                needs_driver_apply = True
            elif key == "tmc_current":
                cmds.append((label, lambda ax=ax, h=d["tmc_ihold"][i], r=d["tmc_irun"][i]:
                             client.set_tmc_uart_current(ax, h, r, 6)))
            elif key == "tmc_usteps":
                cmds.append((label, lambda ax=ax, v=d["tmc_usteps"][i]: client.set_tmc_microsteps(ax, v)))
            elif key == "tmc_sc":
                cmds.append((label, lambda ax=ax, v=d["tmc_sc"][i]: client.set_tmc_spreadcycle(ax, v)))
            elif key == "tmc_stealth":
                cmds.append((label, lambda ax=ax, v=d["tmc_stealth"][i]: client.set_tmc_stealth_max(ax, v)))
            elif key == "jerk":
                cmds.append((label, lambda ax=ax, v=d["jerk"][i]: client.set_jerk(ax, v)))
            elif key == "motion_engine":
                cmds.append((label, lambda v=d["motion_engine"]: client.set_motion_engine(v)))
            elif key == "lookahead":
                cmds.append((label, lambda v=d["lookahead"]: client.set_lookahead(v)))
            elif key == "z_pulley_teeth":
                cmds.append((label, lambda v=d["z_pulley_teeth"]: client.set_z_pulley_teeth(v)))
            elif key == "max_passos_z":
                cmds.append((label, lambda v=d["max_passos_z"]: client.set_length_z(v)))
            elif key == "limits_c":
                cmds.append((label, lambda v=d["limits_c"]: client.set_axis_limits("C", v[0], v[1])))
            elif key == "limits_a":
                cmds.append((label, lambda v=d["limits_a"]: client.set_axis_limits("A", v[0], v[1])))
        if "driver_bus_mode" in keys:
            # set_driver_mode já envia DRIVER APPLY
            mode = "UART" if d["driver_bus_mode"] == 1 else "STEPDIR"
            cmds.append(("Modo do driver", lambda m=mode: client.set_driver_mode(m)))
        elif needs_driver_apply:
            cmds.append(("Reaplicar drivers", client.apply_driver_settings))
        if "can" in keys:
            b = d["can_bases"]
            cmds.append(("Barramento CAN", lambda: client.configure_can(
                d["node_id"], d["can_bitrate"], b[0], b[1], b[2], enabled=d["can_enabled"])))
        return cmds

    def _apply_all_parameters(self):
        client = self.comm.active_client
        if not client or not self.comm.is_connected:
            QMessageBox.warning(self, "Sem conexão", "Conecte ao hardware antes de aplicar parâmetros.")
            return

        # Desfoca campos para confirmar valores digitados
        self.setFocus()

        # Validações prévias
        if self.spin_limit_min_c.value() >= self.spin_limit_max_c.value():
            QMessageBox.warning(self, "Limites Inválidos", "O limite mínimo do Eixo C deve ser estritamente menor que o máximo.")
            return

        if self.spin_limit_min_a.value() >= self.spin_limit_max_a.value():
            QMessageBox.warning(self, "Limites Inválidos", "O limite mínimo do Eixo A deve ser estritamente menor que o máximo.")
            return

        try:
            desired = self._collect_desired()
        except ValueError as e:
            QMessageBox.warning(self, "Valor Inválido", f"Formato hexadecimal incorreto nas bases CAN: {e}")
            return

        diffs = self._diff_against_device(desired, self.state.parameters)
        if not diffs:
            QMessageBox.information(self, "Nada a Gravar",
                                    "Os valores da tela já são os do nó. Nenhum comando foi enviado.")
            return

        cmds = self._commands_for(client, desired, diffs)
        # Grava de uma vez na NVS (o firmware agrupa as alterações) e confirma a escrita
        cmds.append(("Gravar NVS", client.save_nvs))
        labels = [label for label, _ in cmds]
        self._run_command_sequence([func for _, func in cmds],
                                   lambda results: self._on_apply_all_finished(results, labels, desired, diffs))

    def _run_command_sequence(self, cmds, on_finished, interval_ms: int = 40):
        """Envia os comandos espaçados por QTimer (40 ms evita overrun da RX FIFO da UART do
        ESP32) sem bloquear a thread da interface — antes eram ~30 time.sleep() seguidos."""
        results = []
        self.btn_save_nvs.setEnabled(False)
        self.btn_save_all_bottom.setEnabled(False)

        def step(index: int = 0):
            if index >= len(cmds) or not self.comm.is_connected:
                self.btn_save_nvs.setEnabled(True)
                self.btn_save_all_bottom.setEnabled(True)
                on_finished(results + [False] * (len(cmds) - len(results)))
                return
            try:
                results.append(bool(cmds[index]()))
            except Exception as exc:
                self.state.error_occurred.emit(f"Falha ao aplicar parâmetro {index + 1}: {exc}")
                results.append(False)
            QTimer.singleShot(interval_ms, lambda: step(index + 1))

        step()

    def _on_apply_all_finished(self, results, labels, desired: dict, diffs: list):
        failed = [label for label, ok in zip(labels, results) if not ok]
        if failed:
            QMessageBox.warning(
                self,
                "Aviso de Gravação",
                "Não foi possível transmitir: " + ", ".join(failed) + ".\nVerifique o terminal para detalhes.",
            )
            return

        # Confirmação real: relê a configuração do nó e compara com o que foi pedido
        self._finish_confirmation()  # descarta conferência anterior ainda pendente
        self._confirm_token = getattr(self, "_confirm_token", 0) + 1
        token = self._confirm_token
        self._pending_confirmation = (desired, diffs)
        self.state.config_dump_completed.connect(self._on_confirmation_dump)
        QTimer.singleShot(self.CONFIRM_TIMEOUT_MS, lambda: self._on_confirmation_timeout(token))
        QTimer.singleShot(400, self.comm.request_config_dump)

    def _finish_confirmation(self):
        try:
            self.state.config_dump_completed.disconnect(self._on_confirmation_dump)
        except (TypeError, RuntimeError):
            pass
        pending = getattr(self, "_pending_confirmation", None)
        self._pending_confirmation = None
        return pending

    def _on_confirmation_dump(self):
        pending = self._finish_confirmation()
        if pending is None:
            return
        desired, diffs = pending
        still_diff = {label for label, _, _ in self._diff_against_device(desired, self.state.parameters)}
        requested = [label for label, _, _ in diffs]
        mismatched = [label for label in requested if label in still_diff]
        if mismatched:
            QMessageBox.warning(
                self,
                "Gravação Incompleta",
                "O nó não confirmou: " + ", ".join(mismatched) +
                ".\n\nOs demais parâmetros foram gravados e conferidos. Tente gravar novamente.",
            )
        else:
            QMessageBox.information(
                self,
                "Configurações Gravadas",
                f"{len(requested)} parâmetro(s) alterado(s) gravado(s) na NVS e conferido(s) no nó:\n\n• " +
                "\n• ".join(requested),
            )

    def _on_confirmation_timeout(self, token: int):
        if token != getattr(self, "_confirm_token", 0):
            return  # timeout de uma gravação anterior
        pending = self._finish_confirmation()
        if pending is None:
            return  # já confirmado pelo dump
        _, diffs = pending
        QMessageBox.information(
            self,
            "Configurações Enviadas",
            f"{len(diffs)} parâmetro(s) enviado(s), mas o nó não respondeu ao CONFIG DUMP de conferência "
            f"(backend sem leitura de configuração ou nó ocupado). Confira os valores no terminal.",
        )
