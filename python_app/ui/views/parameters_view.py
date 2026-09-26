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
import time
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareParameters, DeviceState
from python_app.core.protocol_defs import (
    calc_z_mm_per_rev, calc_z_mm_per_step, calc_z_steps_per_mm,
    calc_z_steps_for_mm, calc_z_mm_for_steps, Z_BELT_PITCH_MM
)

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
        sub_lbl.setStyleSheet("color: #64748b; font-size: 12px;")
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
        kin_title.setStyleSheet("font-size: 13.5px; font-weight: 700; color: #38bdf8;")
        kin_layout.addWidget(kin_title)
        
        kin_grid = QGridLayout()
        kin_grid.setContentsMargins(0, 0, 0, 0)
        kin_grid.setSpacing(5)
        
        # Column Headers
        h_axis = QLabel("Eixo")
        h_axis.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_axis.setFixedWidth(95)
        h_steps = QLabel("Passos")
        h_steps.setToolTip("Passos base do motor por volta")
        h_steps.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_steps.setFixedWidth(80)
        h_speed = QLabel("Vel. Máx")
        h_speed.setToolTip("Velocidade Máxima (°/s ou mm/s)")
        h_speed.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_speed.setFixedWidth(120)
        h_accel = QLabel("Acel. Máx")
        h_accel.setToolTip("Aceleração Máxima (°/s² ou mm/s²)")
        h_accel.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_accel.setFixedWidth(125)
        h_ramp = QLabel("Rampa Inicial")
        h_ramp.setToolTip("Velocidade inicial de partida da rampa S-Curve (°/s ou mm/s)")
        h_ramp.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_ramp.setFixedWidth(120)
        h_inv = QLabel("Inv.")
        h_inv.setToolTip("Inverter sentido no hardware")
        h_inv.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
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
        lbl_c.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_c.setFixedWidth(95)
        kin_grid.addWidget(lbl_c, 1, 0)
        self.spin_steps_c = QSpinBox()
        self.spin_steps_c.setRange(20, 10000)
        self.spin_steps_c.setValue(200)
        self.spin_steps_c.setFixedWidth(80)
        self.spin_steps_c.setFixedHeight(28)
        self.spin_steps_c.setStyleSheet(INPUT_STYLE)
        self.spin_steps_c.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_c, 1, 1)
        self.spin_speed_c = QDoubleSpinBox()
        self.spin_speed_c.setRange(0.1, 10000.0)
        self.spin_speed_c.setDecimals(1)
        self.spin_speed_c.setValue(140.0)
        self.spin_speed_c.setSuffix(" °/s")
        self.spin_speed_c.setFixedWidth(120)
        self.spin_speed_c.setFixedHeight(28)
        self.spin_speed_c.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_speed_c, 1, 2)
        self.spin_accel_c = QDoubleSpinBox()
        self.spin_accel_c.setRange(1.0, 50000.0)
        self.spin_accel_c.setValue(1800.0)
        self.spin_accel_c.setSuffix(" °/s²")
        self.spin_accel_c.setFixedWidth(125)
        self.spin_accel_c.setFixedHeight(28)
        self.spin_accel_c.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_accel_c, 1, 3)
        self.spin_ramp_c = QDoubleSpinBox()
        self.spin_ramp_c.setRange(0.5, 100.0)
        self.spin_ramp_c.setDecimals(1)
        self.spin_ramp_c.setValue(10.0)
        self.spin_ramp_c.setSingleStep(0.5)
        self.spin_ramp_c.setSuffix(" °/s")
        self.spin_ramp_c.setFixedWidth(120)
        self.spin_ramp_c.setFixedHeight(28)
        self.spin_ramp_c.setStyleSheet(INPUT_STYLE)
        self.spin_ramp_c.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo C (salvo na NVS)")
        self.spin_ramp_c.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_c, 1, 4)
        self.chk_inv_hw_c = QCheckBox("Inv")
        self.chk_inv_hw_c.setChecked(True)
        self.chk_inv_hw_c.setFixedWidth(48)
        self.chk_inv_hw_c.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        kin_grid.addWidget(self.chk_inv_hw_c, 1, 5)
        
        # Axis A
        lbl_a = QLabel("Eixo A (Pivot):")
        lbl_a.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_a.setFixedWidth(95)
        kin_grid.addWidget(lbl_a, 2, 0)
        self.spin_steps_a = QSpinBox()
        self.spin_steps_a.setRange(20, 10000)
        self.spin_steps_a.setValue(200)
        self.spin_steps_a.setFixedWidth(80)
        self.spin_steps_a.setFixedHeight(28)
        self.spin_steps_a.setStyleSheet(INPUT_STYLE)
        self.spin_steps_a.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_a, 2, 1)
        self.spin_speed_a = QDoubleSpinBox()
        self.spin_speed_a.setRange(0.1, 10000.0)
        self.spin_speed_a.setDecimals(1)
        self.spin_speed_a.setValue(140.0)
        self.spin_speed_a.setSuffix(" °/s")
        self.spin_speed_a.setFixedWidth(120)
        self.spin_speed_a.setFixedHeight(28)
        self.spin_speed_a.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_speed_a, 2, 2)
        self.spin_accel_a = QDoubleSpinBox()
        self.spin_accel_a.setRange(1.0, 50000.0)
        self.spin_accel_a.setValue(1800.0)
        self.spin_accel_a.setSuffix(" °/s²")
        self.spin_accel_a.setFixedWidth(125)
        self.spin_accel_a.setFixedHeight(28)
        self.spin_accel_a.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_accel_a, 2, 3)
        self.spin_ramp_a = QDoubleSpinBox()
        self.spin_ramp_a.setRange(0.5, 100.0)
        self.spin_ramp_a.setDecimals(1)
        self.spin_ramp_a.setValue(10.0)
        self.spin_ramp_a.setSingleStep(0.5)
        self.spin_ramp_a.setSuffix(" °/s")
        self.spin_ramp_a.setFixedWidth(120)
        self.spin_ramp_a.setFixedHeight(28)
        self.spin_ramp_a.setStyleSheet(INPUT_STYLE)
        self.spin_ramp_a.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo A (salvo na NVS)")
        self.spin_ramp_a.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_a, 2, 4)
        self.chk_inv_hw_a = QCheckBox("Inv")
        self.chk_inv_hw_a.setChecked(True)
        self.chk_inv_hw_a.setFixedWidth(48)
        self.chk_inv_hw_a.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        kin_grid.addWidget(self.chk_inv_hw_a, 2, 5)
        
        # Axis Z
        lbl_z = QLabel("Eixo Z (Linear):")
        lbl_z.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_z.setFixedWidth(95)
        kin_grid.addWidget(lbl_z, 3, 0)
        self.spin_steps_z = QSpinBox()
        self.spin_steps_z.setRange(20, 10000)
        self.spin_steps_z.setValue(200)
        self.spin_steps_z.setFixedWidth(80)
        self.spin_steps_z.setFixedHeight(28)
        self.spin_steps_z.setStyleSheet(INPUT_STYLE)
        self.spin_steps_z.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_z, 3, 1)
        self.spin_speed_z = QDoubleSpinBox()
        self.spin_speed_z.setRange(0.01, 500.0)
        self.spin_speed_z.setDecimals(1)
        self.spin_speed_z.setValue(250.0)
        self.spin_speed_z.setSuffix(" mm/s")
        self.spin_speed_z.setFixedWidth(120)
        self.spin_speed_z.setFixedHeight(28)
        self.spin_speed_z.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_speed_z, 3, 2)
        self.spin_accel_z = QDoubleSpinBox()
        self.spin_accel_z.setRange(0.1, 5000.0)
        self.spin_accel_z.setDecimals(1)
        self.spin_accel_z.setValue(1000.0)
        self.spin_accel_z.setSuffix(" mm/s²")
        self.spin_accel_z.setFixedWidth(125)
        self.spin_accel_z.setFixedHeight(28)
        self.spin_accel_z.setStyleSheet(INPUT_STYLE)
        kin_grid.addWidget(self.spin_accel_z, 3, 3)
        self.spin_ramp_z = QDoubleSpinBox()
        self.spin_ramp_z.setRange(0.5, 150.0)
        self.spin_ramp_z.setDecimals(1)
        self.spin_ramp_z.setValue(15.0)
        self.spin_ramp_z.setSingleStep(1.0)
        self.spin_ramp_z.setSuffix(" mm/s")
        self.spin_ramp_z.setFixedWidth(120)
        self.spin_ramp_z.setFixedHeight(28)
        self.spin_ramp_z.setStyleSheet(INPUT_STYLE)
        self.spin_ramp_z.setToolTip("Velocidade inicial de partida da rampa S-Curve no eixo Z (salvo na NVS)")
        self.spin_ramp_z.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_ramp_z, 3, 4)
        self.chk_inv_hw_z = QCheckBox("Inv")
        self.chk_inv_hw_z.setChecked(False)
        self.chk_inv_hw_z.setFixedWidth(48)
        self.chk_inv_hw_z.setStyleSheet("font-size: 12px; color: #cbd5e1;")
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
        lbl_pulley.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_pulley.setFixedWidth(54)
        z_config_row.addWidget(lbl_pulley)
        self.spin_pulley_z = QSpinBox()
        self.spin_pulley_z.setRange(6, 200)
        self.spin_pulley_z.setValue(20)
        self.spin_pulley_z.setSuffix(" T")
        self.spin_pulley_z.setFixedWidth(80)
        self.spin_pulley_z.setFixedHeight(28)
        self.spin_pulley_z.setStyleSheet(INPUT_STYLE)
        self.spin_pulley_z.valueChanged.connect(self._update_kinematics_preview)
        z_config_row.addWidget(self.spin_pulley_z)
        
        lbl_max_z = QLabel("Curso Z:")
        lbl_max_z.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
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
        self.spin_max_z_mm.setStyleSheet(INPUT_STYLE)
        self.spin_max_z_mm.valueChanged.connect(self._update_kinematics_preview)
        self.spin_max_z = self.spin_max_z_mm
        z_config_row.addWidget(self.spin_max_z_mm)
        z_config_row.addStretch(1)
        z_extra_layout.addLayout(z_config_row)
        
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
        limits_title.setStyleSheet("font-size: 13.5px; font-weight: 700; color: #38bdf8;")
        limits_layout.addWidget(limits_title)
        
        limits_grid = QGridLayout()
        limits_grid.setContentsMargins(0, 0, 0, 0)
        limits_grid.setSpacing(5)
        
        lh_axis = QLabel("Eixo")
        lh_axis.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_axis.setFixedWidth(95)
        lh_min = QLabel("Limite Mín")
        lh_min.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_min.setFixedWidth(120)
        lh_max = QLabel("Limite Máx")
        lh_max.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_max.setFixedWidth(120)
        lh_desc = QLabel("Operação")
        lh_desc.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_desc.setFixedWidth(135)
        
        limits_grid.addWidget(lh_axis, 0, 0)
        limits_grid.addWidget(lh_min, 0, 1)
        limits_grid.addWidget(lh_max, 0, 2)
        limits_grid.addWidget(lh_desc, 0, 3)
        
        # Eixo C
        lbl_lim_c = QLabel("Eixo C (Base):")
        lbl_lim_c.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_lim_c.setFixedWidth(95)
        limits_grid.addWidget(lbl_lim_c, 1, 0)
        self.spin_limit_min_c = QDoubleSpinBox()
        self.spin_limit_min_c.setRange(-3600.0, 3600.0)
        self.spin_limit_min_c.setDecimals(1)
        self.spin_limit_min_c.setValue(-540.0)
        self.spin_limit_min_c.setSuffix(" °")
        self.spin_limit_min_c.setFixedWidth(120)
        self.spin_limit_min_c.setFixedHeight(28)
        self.spin_limit_min_c.setStyleSheet(INPUT_STYLE)
        limits_grid.addWidget(self.spin_limit_min_c, 1, 1)
        self.spin_limit_max_c = QDoubleSpinBox()
        self.spin_limit_max_c.setRange(-3600.0, 3600.0)
        self.spin_limit_max_c.setDecimals(1)
        self.spin_limit_max_c.setValue(540.0)
        self.spin_limit_max_c.setSuffix(" °")
        self.spin_limit_max_c.setFixedWidth(120)
        self.spin_limit_max_c.setFixedHeight(28)
        self.spin_limit_max_c.setStyleSheet(INPUT_STYLE)
        limits_grid.addWidget(self.spin_limit_max_c, 1, 2)
        lbl_desc_c = QLabel("Giro base contínuo")
        lbl_desc_c.setStyleSheet("color: #64748b; font-size: 12px;")
        lbl_desc_c.setFixedWidth(135)
        limits_grid.addWidget(lbl_desc_c, 1, 3)
        
        # Eixo A
        lbl_lim_a = QLabel("Eixo A (Pivot):")
        lbl_lim_a.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_lim_a.setFixedWidth(95)
        limits_grid.addWidget(lbl_lim_a, 2, 0)
        self.spin_limit_min_a = QDoubleSpinBox()
        self.spin_limit_min_a.setRange(-3600.0, 3600.0)
        self.spin_limit_min_a.setDecimals(1)
        self.spin_limit_min_a.setValue(-540.0)
        self.spin_limit_min_a.setSuffix(" °")
        self.spin_limit_min_a.setFixedWidth(120)
        self.spin_limit_min_a.setFixedHeight(28)
        self.spin_limit_min_a.setStyleSheet(INPUT_STYLE)
        limits_grid.addWidget(self.spin_limit_min_a, 2, 1)
        self.spin_limit_max_a = QDoubleSpinBox()
        self.spin_limit_max_a.setRange(-3600.0, 3600.0)
        self.spin_limit_max_a.setDecimals(1)
        self.spin_limit_max_a.setValue(540.0)
        self.spin_limit_max_a.setSuffix(" °")
        self.spin_limit_max_a.setFixedWidth(120)
        self.spin_limit_max_a.setFixedHeight(28)
        self.spin_limit_max_a.setStyleSheet(INPUT_STYLE)
        limits_grid.addWidget(self.spin_limit_max_a, 2, 2)
        lbl_desc_a = QLabel("Pivot do cabeçote")
        lbl_desc_a.setStyleSheet("color: #64748b; font-size: 12px;")
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
        tmc_title.setStyleSheet("font-size: 13.5px; font-weight: 700; color: #38bdf8;")
        tmc_header.addWidget(tmc_title)
        tmc_header.addStretch()
        
        lbl_mode = QLabel("Modo:")
        lbl_mode.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        tmc_header.addWidget(lbl_mode)
        self.combo_driver_mode = QComboBox()
        self.combo_driver_mode.addItems(["UART Digital", "STEP/DIR Legado"])
        self.combo_driver_mode.setCurrentIndex(0)
        self.combo_driver_mode.setFixedWidth(155)
        self.combo_driver_mode.setFixedHeight(28)
        self.combo_driver_mode.setStyleSheet(COMBO_STYLE)
        tmc_header.addWidget(self.combo_driver_mode)
        tmc_layout.addLayout(tmc_header)
        
        tmc_grid = QGridLayout()
        tmc_grid.setContentsMargins(0, 0, 0, 0)
        tmc_grid.setSpacing(5)
        
        th_driver = QLabel("Driver")
        th_driver.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_driver.setFixedWidth(95)
        th_addr = QLabel("Endr")
        th_addr.setToolTip("Endereço UART (0..3)")
        th_addr.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_addr.setFixedWidth(55)
        th_irun = QLabel("iRun")
        th_irun.setToolTip("Corrente de trabalho em mA")
        th_irun.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_irun.setFixedWidth(105)
        th_ihold = QLabel("iHold")
        th_ihold.setToolTip("Corrente de repouso em mA")
        th_ihold.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_ihold.setFixedWidth(105)
        th_usteps = QLabel("Micropassos")
        th_usteps.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_usteps.setFixedWidth(75)
        th_sc = QLabel("SC")
        th_sc.setToolTip("SpreadCycle ativado")
        th_sc.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_sc.setFixedWidth(45)
        
        tmc_grid.addWidget(th_driver, 0, 0)
        tmc_grid.addWidget(th_addr, 0, 1)
        tmc_grid.addWidget(th_irun, 0, 2)
        tmc_grid.addWidget(th_ihold, 0, 3)
        tmc_grid.addWidget(th_usteps, 0, 4)
        tmc_grid.addWidget(th_sc, 0, 5)
        
        # Driver C
        lbl_drv_c = QLabel("Driver C (Base):")
        lbl_drv_c.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_drv_c.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_c, 1, 0)
        self.spin_tmc_addr_c = QSpinBox()
        self.spin_tmc_addr_c.setRange(0, 3)
        self.spin_tmc_addr_c.setValue(0)
        self.spin_tmc_addr_c.setFixedWidth(55)
        self.spin_tmc_addr_c.setFixedHeight(28)
        self.spin_tmc_addr_c.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_addr_c, 1, 1)
        self.spin_tmc_irun_c = QSpinBox()
        self.spin_tmc_irun_c.setRange(50, 2000)
        self.spin_tmc_irun_c.setValue(897)
        self.spin_tmc_irun_c.setSuffix(" mA")
        self.spin_tmc_irun_c.setFixedWidth(105)
        self.spin_tmc_irun_c.setFixedHeight(28)
        self.spin_tmc_irun_c.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_irun_c, 1, 2)
        self.spin_tmc_ihold_c = QSpinBox()
        self.spin_tmc_ihold_c.setRange(50, 2000)
        self.spin_tmc_ihold_c.setValue(559)
        self.spin_tmc_ihold_c.setSuffix(" mA")
        self.spin_tmc_ihold_c.setFixedWidth(105)
        self.spin_tmc_ihold_c.setFixedHeight(28)
        self.spin_tmc_ihold_c.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_ihold_c, 1, 3)
        self.combo_tmc_usteps_c = QComboBox()
        self.combo_tmc_usteps_c.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_c.setCurrentText("16")
        self.combo_tmc_usteps_c.setFixedWidth(75)
        self.combo_tmc_usteps_c.setFixedHeight(28)
        self.combo_tmc_usteps_c.setStyleSheet(COMBO_STYLE)
        self.combo_tmc_usteps_c.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_c, 1, 4)
        self.chk_tmc_sc_c = QCheckBox("SC")
        self.chk_tmc_sc_c.setFixedWidth(45)
        self.chk_tmc_sc_c.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        tmc_grid.addWidget(self.chk_tmc_sc_c, 1, 5)
        
        # Driver A
        lbl_drv_a = QLabel("Driver A (Pivot):")
        lbl_drv_a.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_drv_a.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_a, 2, 0)
        self.spin_tmc_addr_a = QSpinBox()
        self.spin_tmc_addr_a.setRange(0, 3)
        self.spin_tmc_addr_a.setValue(1)
        self.spin_tmc_addr_a.setFixedWidth(55)
        self.spin_tmc_addr_a.setFixedHeight(28)
        self.spin_tmc_addr_a.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_addr_a, 2, 1)
        self.spin_tmc_irun_a = QSpinBox()
        self.spin_tmc_irun_a.setRange(50, 2000)
        self.spin_tmc_irun_a.setValue(897)
        self.spin_tmc_irun_a.setSuffix(" mA")
        self.spin_tmc_irun_a.setFixedWidth(105)
        self.spin_tmc_irun_a.setFixedHeight(28)
        self.spin_tmc_irun_a.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_irun_a, 2, 2)
        self.spin_tmc_ihold_a = QSpinBox()
        self.spin_tmc_ihold_a.setRange(50, 2000)
        self.spin_tmc_ihold_a.setValue(559)
        self.spin_tmc_ihold_a.setSuffix(" mA")
        self.spin_tmc_ihold_a.setFixedWidth(105)
        self.spin_tmc_ihold_a.setFixedHeight(28)
        self.spin_tmc_ihold_a.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_ihold_a, 2, 3)
        self.combo_tmc_usteps_a = QComboBox()
        self.combo_tmc_usteps_a.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_a.setCurrentText("16")
        self.combo_tmc_usteps_a.setFixedWidth(75)
        self.combo_tmc_usteps_a.setFixedHeight(28)
        self.combo_tmc_usteps_a.setStyleSheet(COMBO_STYLE)
        self.combo_tmc_usteps_a.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_a, 2, 4)
        self.chk_tmc_sc_a = QCheckBox("SC")
        self.chk_tmc_sc_a.setFixedWidth(45)
        self.chk_tmc_sc_a.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        tmc_grid.addWidget(self.chk_tmc_sc_a, 2, 5)
        
        # Driver Z
        lbl_drv_z = QLabel("Driver Z (Linear):")
        lbl_drv_z.setStyleSheet("color: #e2e8f0; font-size: 12px; font-weight: 600;")
        lbl_drv_z.setFixedWidth(95)
        tmc_grid.addWidget(lbl_drv_z, 3, 0)
        self.spin_tmc_addr_z = QSpinBox()
        self.spin_tmc_addr_z.setRange(0, 3)
        self.spin_tmc_addr_z.setValue(2)
        self.spin_tmc_addr_z.setFixedWidth(55)
        self.spin_tmc_addr_z.setFixedHeight(28)
        self.spin_tmc_addr_z.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_addr_z, 3, 1)
        self.spin_tmc_irun_z = QSpinBox()
        self.spin_tmc_irun_z.setRange(50, 2000)
        self.spin_tmc_irun_z.setValue(957)
        self.spin_tmc_irun_z.setSuffix(" mA")
        self.spin_tmc_irun_z.setFixedWidth(105)
        self.spin_tmc_irun_z.setFixedHeight(28)
        self.spin_tmc_irun_z.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_irun_z, 3, 2)
        self.spin_tmc_ihold_z = QSpinBox()
        self.spin_tmc_ihold_z.setRange(50, 2000)
        self.spin_tmc_ihold_z.setValue(418)
        self.spin_tmc_ihold_z.setSuffix(" mA")
        self.spin_tmc_ihold_z.setFixedWidth(105)
        self.spin_tmc_ihold_z.setFixedHeight(28)
        self.spin_tmc_ihold_z.setStyleSheet(INPUT_STYLE)
        tmc_grid.addWidget(self.spin_tmc_ihold_z, 3, 3)
        self.combo_tmc_usteps_z = QComboBox()
        self.combo_tmc_usteps_z.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_z.setCurrentText("16")
        self.combo_tmc_usteps_z.setFixedWidth(75)
        self.combo_tmc_usteps_z.setFixedHeight(28)
        self.combo_tmc_usteps_z.setStyleSheet(COMBO_STYLE)
        self.combo_tmc_usteps_z.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_z, 3, 4)
        self.chk_tmc_sc_z = QCheckBox("SC")
        self.chk_tmc_sc_z.setFixedWidth(45)
        self.chk_tmc_sc_z.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        tmc_grid.addWidget(self.chk_tmc_sc_z, 3, 5)
        tmc_grid.setColumnStretch(6, 1)
        
        tmc_layout.addLayout(tmc_grid)
        
        # Sub-Card: Diagnóstico e Acesso Direto a Registradores TMC2209
        reg_frame = QFrame()
        reg_frame.setProperty("class", "metric-card")
        reg_layout = QHBoxLayout(reg_frame)
        reg_layout.setContentsMargins(10, 6, 10, 6)
        reg_layout.setSpacing(6)
        
        lbl_diag = QLabel("Reg TMC:")
        lbl_diag.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_diag.setFixedWidth(65)
        reg_layout.addWidget(lbl_diag)
        self.combo_reg_axis = QComboBox()
        self.combo_reg_axis.addItems(["C", "A", "Z"])
        self.combo_reg_axis.setFixedWidth(65)
        self.combo_reg_axis.setFixedHeight(28)
        self.combo_reg_axis.setStyleSheet(COMBO_STYLE)
        reg_layout.addWidget(self.combo_reg_axis)
        self.txt_reg_addr = QLineEdit("0x06")
        self.txt_reg_addr.setFixedWidth(65)
        self.txt_reg_addr.setFixedHeight(28)
        self.txt_reg_addr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_reg_addr.setStyleSheet(INPUT_STYLE)
        reg_layout.addWidget(self.txt_reg_addr)
        self.txt_reg_val = QLineEdit("0x00000000")
        self.txt_reg_val.setFixedWidth(105)
        self.txt_reg_val.setFixedHeight(28)
        self.txt_reg_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_reg_val.setStyleSheet(INPUT_STYLE)
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
        can_title.setStyleSheet("font-size: 13.5px; font-weight: 700; color: #38bdf8;")
        can_header.addWidget(can_title)
        can_header.addStretch()
        
        self.chk_can_enabled = QCheckBox("Habilitar CAN")
        self.chk_can_enabled.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 12px;")
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
        lbl_node.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_node.setFixedWidth(105)
        can_grid.addWidget(lbl_node, 0, 0)
        self.spin_node_id = QSpinBox()
        self.spin_node_id.setRange(1, 10)
        self.spin_node_id.setValue(1)
        self.spin_node_id.setFixedWidth(75)
        self.spin_node_id.setFixedHeight(28)
        self.spin_node_id.setStyleSheet(INPUT_STYLE)
        can_grid.addWidget(self.spin_node_id, 0, 1)
        
        lbl_bitrate = QLabel("Bitrate:")
        lbl_bitrate.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_bitrate.setFixedWidth(65)
        can_grid.addWidget(lbl_bitrate, 0, 2)
        self.combo_can_bitrate = QComboBox()
        self.combo_can_bitrate.addItems(["125000", "250000", "500000", "1000000"])
        self.combo_can_bitrate.setCurrentText("500000")
        self.combo_can_bitrate.setFixedWidth(105)
        self.combo_can_bitrate.setFixedHeight(28)
        self.combo_can_bitrate.setStyleSheet(COMBO_STYLE)
        can_grid.addWidget(self.combo_can_bitrate, 0, 3)
        
        lbl_base_cmd = QLabel("Base Cmd:")
        lbl_base_cmd.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_base_cmd.setFixedWidth(105)
        can_grid.addWidget(lbl_base_cmd, 1, 0)
        self.txt_base_cmd = QLineEdit("0x200")
        self.txt_base_cmd.setFixedWidth(75)
        self.txt_base_cmd.setFixedHeight(28)
        self.txt_base_cmd.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_base_cmd.setStyleSheet(INPUT_STYLE)
        can_grid.addWidget(self.txt_base_cmd, 1, 1)
        
        lbl_base_st = QLabel("Base Status:")
        lbl_base_st.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_base_st.setFixedWidth(85)
        can_grid.addWidget(lbl_base_st, 1, 2)
        self.txt_base_status = QLineEdit("0x280")
        self.txt_base_status.setFixedWidth(105)
        self.txt_base_status.setFixedHeight(28)
        self.txt_base_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_base_status.setStyleSheet(INPUT_STYLE)
        can_grid.addWidget(self.txt_base_status, 1, 3)
        
        lbl_base_ev = QLabel("Base Eventos:")
        lbl_base_ev.setStyleSheet("color: #94a3b8; font-size: 12px; font-weight: 600;")
        lbl_base_ev.setFixedWidth(105)
        can_grid.addWidget(lbl_base_ev, 2, 0)
        self.txt_base_event = QLineEdit("0x300")
        self.txt_base_event.setFixedWidth(75)
        self.txt_base_event.setFixedHeight(28)
        self.txt_base_event.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_base_event.setStyleSheet(INPUT_STYLE)
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
            self.spin_limit_min_c, self.spin_limit_max_c, self.spin_limit_min_a, self.spin_limit_max_a
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
            cmd_base = int(self.txt_base_cmd.text(), 16)
            status_base = int(self.txt_base_status.text(), 16)
            event_base = int(self.txt_base_event.text(), 16)
        except ValueError as e:
            QMessageBox.warning(self, "Valor Inválido", f"Formato hexadecimal incorreto nas bases CAN: {e}")
            return

        teeth = self.spin_pulley_z.value()
        spr_z = self.spin_steps_z.value()
        try:
            usteps_z = int(self.combo_tmc_usteps_z.currentText())
        except ValueError:
            usteps_z = 16

        steps_z = calc_z_steps_for_mm(self.spin_max_z_mm.value(), teeth, spr_z, usteps_z)
        target_mode = "UART" if "UART" in self.combo_driver_mode.currentText() else "STEPDIR"
        can_enabled = self.chk_can_enabled.isChecked()

        cmds = [
            # 1. Driver Bus Mode
            lambda: client.set_driver_mode("UART"),
            # 2. Steps per revolution
            lambda: client.set_steps_per_rev('C', self.spin_steps_c.value()),
            lambda: client.set_steps_per_rev('A', self.spin_steps_a.value()),
            lambda: client.set_steps_per_rev('Z', self.spin_steps_z.value()),
            # 3. Speeds & Accel
            lambda: client.set_axis_speed('C', self.spin_speed_c.value()),
            lambda: client.set_axis_speed('A', self.spin_speed_a.value()),
            lambda: client.set_axis_speed('Z', self.spin_speed_z.value()),
            lambda: client.set_axis_accel('C', self.spin_accel_c.value()),
            lambda: client.set_axis_accel('A', self.spin_accel_a.value()),
            lambda: client.set_axis_accel('Z', self.spin_accel_z.value()),
            # 4. Invert DIR
            lambda: client.set_driver_invert('C', self.chk_inv_hw_c.isChecked()),
            lambda: client.set_driver_invert('A', self.chk_inv_hw_a.isChecked()),
            lambda: client.set_driver_invert('Z', self.chk_inv_hw_z.isChecked()),
            # 5. Pulley Z, Max Z & Rampas (C, A, Z)
            lambda: client.set_z_pulley_teeth(teeth),
            lambda: client.set_length_z(steps_z),
            lambda: client.set_axis_ramp_speed('C', self.spin_ramp_c.value()),
            lambda: client.set_axis_ramp_speed('A', self.spin_ramp_a.value()),
            lambda: client.set_axis_ramp_speed('Z', self.spin_ramp_z.value()),
            # 6. Angular Limits C & A
            lambda: client.set_axis_limits('C', self.spin_limit_min_c.value(), self.spin_limit_max_c.value()),
            lambda: client.set_axis_limits('A', self.spin_limit_min_a.value(), self.spin_limit_max_a.value()),
            # 7. TMC2209 Parameters
            lambda: client.set_tmc_uart_current('C', self.spin_tmc_ihold_c.value(), self.spin_tmc_irun_c.value(), 6),
            lambda: client.set_tmc_uart_current('A', self.spin_tmc_ihold_a.value(), self.spin_tmc_irun_a.value(), 6),
            lambda: client.set_tmc_uart_current('Z', self.spin_tmc_ihold_z.value(), self.spin_tmc_irun_z.value(), 6),
            lambda: client.set_tmc_spreadcycle('C', self.chk_tmc_sc_c.isChecked()),
            lambda: client.set_tmc_spreadcycle('A', self.chk_tmc_sc_a.isChecked()),
            lambda: client.set_tmc_spreadcycle('Z', self.chk_tmc_sc_z.isChecked()),
            lambda: client.set_tmc_microsteps('C', int(self.combo_tmc_usteps_c.currentText())),
            lambda: client.set_tmc_microsteps('A', int(self.combo_tmc_usteps_a.currentText())),
            lambda: client.set_tmc_microsteps('Z', int(self.combo_tmc_usteps_z.currentText())),
            lambda: client.set_driver_mode(target_mode),
            lambda: client.apply_driver_settings(),
            # 8. CAN Settings
            lambda: client.configure_can(
                self.spin_node_id.value(),
                int(self.combo_can_bitrate.currentText()),
                cmd_base,
                status_base,
                event_base,
                enabled=can_enabled
            )
        ]

        results = []
        for cmd_func in cmds:
            results.append(cmd_func())
            time.sleep(0.04)  # 40ms inter-command delay prevents ESP32 UART RX FIFO overrun

        # Solicita leitura de confirmação da placa
        QTimer.singleShot(600, self.comm.request_config_dump)

        if all(results):
            can_str = "Habilitado" if can_enabled else "Desativado"
            QMessageBox.information(
                self,
                "Configurações Gravadas",
                f"Todas as configurações foram transmitidas e gravadas na NVS com sucesso!\n\n"
                f"• Cinemática: Passos C={self.spin_steps_c.value()}, A={self.spin_steps_a.value()}, Z={self.spin_steps_z.value()}\n"
                f"• Rampas S-Curve: C={self.spin_ramp_c.value():.1f}°/s | A={self.spin_ramp_a.value():.1f}°/s | Z={self.spin_ramp_z.value():.1f} mm/s\n"
                f"• Eixo Z: Polia={teeth}T | Curso Máx={self.spin_max_z_mm.value():.1f} mm ({steps_z} passos)\n"
                f"• Limites C: [{self.spin_limit_min_c.value():.1f}°, {self.spin_limit_max_c.value():.1f}°]\n"
                f"• Limites A: [{self.spin_limit_min_a.value():.1f}°, {self.spin_limit_max_a.value():.1f}°]\n"
                f"• Drivers TMC: Modo {target_mode} | Correntes Run/Hold atualizadas\n"
                f"• Barramento CAN: {can_str} | Node ID={self.spin_node_id.value()} | Bitrate={self.combo_can_bitrate.currentText()} bps\n\n"
                f"Os parâmetros foram confirmados pelo hardware e salvos na memória não-volátil."
            )
        else:
            QMessageBox.warning(
                self,
                "Aviso de Gravação",
                "Alguns parâmetros podem não ter sido confirmados pelo hardware. Verifique o terminal para detalhes.",
            )
