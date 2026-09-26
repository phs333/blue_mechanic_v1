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

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setContentsMargins(20, 16, 20, 20)
        main_layout.setSpacing(16)
        
        # ==============================================================
        # --- HEADER CARD ---
        # ==============================================================
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(18, 14, 18, 14)
        
        title_vbox = QVBoxLayout()
        title_lbl = QLabel("⚙️  Gerenciamento de Parâmetros & NVS")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 17px; font-weight: 700;")
        sub_lbl = QLabel("Configuração unificada e persistente de cinemática, drivers TMC2209, limites angulares e barramento CAN")
        sub_lbl.setStyleSheet("color: #64748b; font-size: 12px;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        self.btn_refresh = QPushButton("📥  Ler da Placa (CONFIG DUMP)")
        self.btn_refresh.setToolTip("Solicita leitura completa das configurações gravadas no ESP32")
        self.btn_refresh.clicked.connect(self.comm.request_config_dump)
        header_layout.addWidget(self.btn_refresh)
        
        self.btn_save_nvs = QPushButton("💾  Gravar Todas as Configurações na NVS")
        self.btn_save_nvs.setProperty("class", "btn-primary")
        self.btn_save_nvs.setStyleSheet("font-weight: 700; padding: 8px 18px;")
        self.btn_save_nvs.setToolTip("Transmite e grava todas as seções desta tela na memória flash não-volátil (NVS)")
        self.btn_save_nvs.clicked.connect(self._apply_all_parameters)
        header_layout.addWidget(self.btn_save_nvs)
        
        main_layout.addWidget(header_card)
        
        # ==============================================================
        # --- 1. CINEMÁTICA, PASSOS DO MOTOR E VELOCIDADES ---
        # ==============================================================
        kin_card = QFrame()
        kin_card.setProperty("class", "card")
        kin_layout = QVBoxLayout(kin_card)
        kin_layout.setContentsMargins(18, 16, 18, 16)
        kin_layout.setSpacing(14)
        
        kin_title = QLabel("1. Cinemática, Passos do Motor e Dinâmica de Movimento")
        kin_title.setProperty("class", "section-title")
        kin_layout.addWidget(kin_title)
        
        kin_grid = QGridLayout()
        kin_grid.setSpacing(10)
        
        # Column Headers
        h_axis = QLabel("Eixo")
        h_axis.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_steps = QLabel("Passos Base Motor (ex: 200/400)")
        h_steps.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_speed = QLabel("Velocidade Máx (°/s ou mm/s)")
        h_speed.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_accel = QLabel("Aceleração Máx (°/s² ou mm/s²)")
        h_accel.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        h_inv = QLabel("Inverter Sentido (DIR)")
        h_inv.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        
        kin_grid.addWidget(h_axis, 0, 0)
        kin_grid.addWidget(h_steps, 0, 1)
        kin_grid.addWidget(h_speed, 0, 2)
        kin_grid.addWidget(h_accel, 0, 3)
        kin_grid.addWidget(h_inv, 0, 4)
        
        # Axis C (Base Rotativa)
        lbl_c = QLabel("Eixo C (Base Rotativa):")
        lbl_c.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        kin_grid.addWidget(lbl_c, 1, 0)
        
        self.spin_steps_c = QSpinBox()
        self.spin_steps_c.setRange(20, 10000)
        self.spin_steps_c.setValue(200)
        self.spin_steps_c.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_c, 1, 1)
        
        self.spin_speed_c = QDoubleSpinBox()
        self.spin_speed_c.setRange(0.1, 10000.0)
        self.spin_speed_c.setDecimals(2)
        self.spin_speed_c.setValue(140.625)
        self.spin_speed_c.setSuffix(" °/s")
        kin_grid.addWidget(self.spin_speed_c, 1, 2)
        
        self.spin_accel_c = QDoubleSpinBox()
        self.spin_accel_c.setRange(1.0, 50000.0)
        self.spin_accel_c.setValue(1800.0)
        self.spin_accel_c.setSuffix(" °/s²")
        kin_grid.addWidget(self.spin_accel_c, 1, 3)
        
        self.chk_inv_hw_c = QCheckBox("Inverter C")
        kin_grid.addWidget(self.chk_inv_hw_c, 1, 4)
        
        # Axis A (Pivot dos Lasers)
        lbl_a = QLabel("Eixo A (Pivot Lasers):")
        lbl_a.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        kin_grid.addWidget(lbl_a, 2, 0)
        
        self.spin_steps_a = QSpinBox()
        self.spin_steps_a.setRange(20, 10000)
        self.spin_steps_a.setValue(200)
        self.spin_steps_a.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_a, 2, 1)
        
        self.spin_speed_a = QDoubleSpinBox()
        self.spin_speed_a.setRange(0.1, 10000.0)
        self.spin_speed_a.setDecimals(2)
        self.spin_speed_a.setValue(140.625)
        self.spin_speed_a.setSuffix(" °/s")
        kin_grid.addWidget(self.spin_speed_a, 2, 2)
        
        self.spin_accel_a = QDoubleSpinBox()
        self.spin_accel_a.setRange(1.0, 50000.0)
        self.spin_accel_a.setValue(1800.0)
        self.spin_accel_a.setSuffix(" °/s²")
        kin_grid.addWidget(self.spin_accel_a, 2, 3)
        
        self.chk_inv_hw_a = QCheckBox("Inverter A")
        kin_grid.addWidget(self.chk_inv_hw_a, 2, 4)
        
        # Axis Z (Atuador Linear)
        lbl_z = QLabel("Eixo Z (Atuador Linear):")
        lbl_z.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        kin_grid.addWidget(lbl_z, 3, 0)
        
        self.spin_steps_z = QSpinBox()
        self.spin_steps_z.setRange(20, 10000)
        self.spin_steps_z.setValue(200)
        self.spin_steps_z.valueChanged.connect(self._update_kinematics_preview)
        kin_grid.addWidget(self.spin_steps_z, 3, 1)
        
        self.spin_speed_z = QDoubleSpinBox()
        self.spin_speed_z.setRange(0.01, 500.0)
        self.spin_speed_z.setDecimals(2)
        self.spin_speed_z.setValue(12.5)
        self.spin_speed_z.setSuffix(" mm/s")
        kin_grid.addWidget(self.spin_speed_z, 3, 2)
        
        self.spin_accel_z = QDoubleSpinBox()
        self.spin_accel_z.setRange(0.1, 5000.0)
        self.spin_accel_z.setDecimals(2)
        self.spin_accel_z.setValue(300.0)
        self.spin_accel_z.setSuffix(" mm/s²")
        kin_grid.addWidget(self.spin_accel_z, 3, 3)
        
        self.chk_inv_hw_z = QCheckBox("Inverter Z")
        kin_grid.addWidget(self.chk_inv_hw_z, 3, 4)
        
        kin_layout.addLayout(kin_grid)
        
        # Sub-Card: Parâmetros Físicos do Eixo Z (Polia GT2 e Limite em mm)
        z_extra_frame = QFrame()
        z_extra_frame.setProperty("class", "metric-card")
        z_extra_layout = QVBoxLayout(z_extra_frame)
        z_extra_layout.setContentsMargins(14, 12, 14, 12)
        z_extra_layout.setSpacing(10)
        
        z_config_grid = QGridLayout()
        z_config_grid.setSpacing(12)
        
        z_config_grid.addWidget(QLabel("⚙️  Dentes da Polia Motor Z (GT2 Passo 2.0mm):"), 0, 0)
        self.spin_pulley_z = QSpinBox()
        self.spin_pulley_z.setRange(6, 200)
        self.spin_pulley_z.setValue(16)
        self.spin_pulley_z.setToolTip("Número de dentes da polia dentada GT2 montada no motor Z (ex: 16T, 20T, 36T)")
        self.spin_pulley_z.valueChanged.connect(self._update_kinematics_preview)
        z_config_grid.addWidget(self.spin_pulley_z, 0, 1)
        
        z_config_grid.addWidget(QLabel("📏  Curso Linear Máximo do Eixo Z:"), 0, 2)
        self.spin_max_z_mm = QDoubleSpinBox()
        self.spin_max_z_mm.setRange(1.0, 500.0)
        self.spin_max_z_mm.setDecimals(1)
        self.spin_max_z_mm.setValue(500.0)
        self.spin_max_z_mm.setSingleStep(5.0)
        self.spin_max_z_mm.setSuffix(" mm")
        self.spin_max_z_mm.setToolTip("Limite máximo de curso linear em milímetros (até 500.0 mm). O app converte para passos ao gravar na NVS.")
        self.spin_max_z_mm.valueChanged.connect(self._update_kinematics_preview)
        self.spin_max_z = self.spin_max_z_mm  # alias para compatibilidade
        z_config_grid.addWidget(self.spin_max_z_mm, 0, 3)
        
        z_extra_layout.addLayout(z_config_grid)
        
        # Dynamic preview calculation banner
        self.lbl_kin_calc_info = QLabel("Cálculo cinemático: ...")
        self.lbl_kin_calc_info.setStyleSheet("color: #38bdf8; font-weight: 600; font-size: 11px;")
        z_extra_layout.addWidget(self.lbl_kin_calc_info)
        
        kin_layout.addWidget(z_extra_frame)
        main_layout.addWidget(kin_card)
        
        # ==============================================================
        # --- 2. LIMITES ANGULARES DOS ENCODERS (C E A) ---
        # ==============================================================
        limits_card = QFrame()
        limits_card.setProperty("class", "card")
        limits_layout = QVBoxLayout(limits_card)
        limits_layout.setContentsMargins(18, 16, 18, 16)
        limits_layout.setSpacing(12)
        
        limits_title = QLabel("2. Limites Angulares dos Encoders Magnéticos (NVS)")
        limits_title.setProperty("class", "section-title")
        limits_layout.addWidget(limits_title)
        
        limits_grid = QGridLayout()
        limits_grid.setSpacing(10)
        
        # Column Headers
        lh_axis = QLabel("Eixo")
        lh_axis.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_min = QLabel("Limite Mínimo (Graus)")
        lh_min.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_max = QLabel("Limite Máximo (Graus)")
        lh_max.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        lh_desc = QLabel("Operação & Curso")
        lh_desc.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        
        limits_grid.addWidget(lh_axis, 0, 0)
        limits_grid.addWidget(lh_min, 0, 1)
        limits_grid.addWidget(lh_max, 0, 2)
        limits_grid.addWidget(lh_desc, 0, 3)
        
        # Eixo C
        lbl_lim_c = QLabel("Eixo C (Base Rotativa):")
        lbl_lim_c.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        limits_grid.addWidget(lbl_lim_c, 1, 0)
        
        self.spin_limit_min_c = QDoubleSpinBox()
        self.spin_limit_min_c.setRange(-3600.0, 3600.0)
        self.spin_limit_min_c.setDecimals(2)
        self.spin_limit_min_c.setValue(-540.0)
        self.spin_limit_min_c.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_min_c, 1, 1)
        
        self.spin_limit_max_c = QDoubleSpinBox()
        self.spin_limit_max_c.setRange(-3600.0, 3600.0)
        self.spin_limit_max_c.setDecimals(2)
        self.spin_limit_max_c.setValue(540.0)
        self.spin_limit_max_c.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_max_c, 1, 2)
        
        lbl_desc_c = QLabel("Rotação contínua da base de 0° a 360° (Curso angular NVS)")
        lbl_desc_c.setStyleSheet("color: #64748b; font-size: 11px;")
        limits_grid.addWidget(lbl_desc_c, 1, 3)
        
        # Eixo A
        lbl_lim_a = QLabel("Eixo A (Pivot dos Lasers):")
        lbl_lim_a.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        limits_grid.addWidget(lbl_lim_a, 2, 0)
        
        self.spin_limit_min_a = QDoubleSpinBox()
        self.spin_limit_min_a.setRange(-3600.0, 3600.0)
        self.spin_limit_min_a.setDecimals(2)
        self.spin_limit_min_a.setValue(-540.0)
        self.spin_limit_min_a.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_min_a, 2, 1)
        
        self.spin_limit_max_a = QDoubleSpinBox()
        self.spin_limit_max_a.setRange(-3600.0, 3600.0)
        self.spin_limit_max_a.setDecimals(2)
        self.spin_limit_max_a.setValue(540.0)
        self.spin_limit_max_a.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_max_a, 2, 2)
        
        lbl_desc_a = QLabel("Inclinação do cabeçote colinear dos lasers (Curso angular NVS)")
        lbl_desc_a.setStyleSheet("color: #64748b; font-size: 11px;")
        limits_grid.addWidget(lbl_desc_a, 2, 3)
        
        limits_layout.addLayout(limits_grid)
        main_layout.addWidget(limits_card)
        
        # ==============================================================
        # --- 3. DRIVERS TMC2209 (COMUNICAÇÃO UART DIGITAL) ---
        # ==============================================================
        tmc_card = QFrame()
        tmc_card.setProperty("class", "card")
        tmc_layout = QVBoxLayout(tmc_card)
        tmc_layout.setContentsMargins(18, 16, 18, 16)
        tmc_layout.setSpacing(14)
        
        tmc_title = QLabel("3. Drivers de Passo TMC2209 (Comunicação UART)")
        tmc_title.setProperty("class", "section-title")
        tmc_layout.addWidget(tmc_title)
        
        # Bus Mode Selection Row
        mode_row = QHBoxLayout()
        mode_row.setSpacing(12)
        lbl_mode = QLabel("Modo do Barramento dos Drivers:")
        lbl_mode.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        mode_row.addWidget(lbl_mode)
        
        self.combo_driver_mode = QComboBox()
        self.combo_driver_mode.addItems(["UART TMC2209 Digital (UART)", "STEP/DIR Puro Legado (STEPDIR)"])
        self.combo_driver_mode.setCurrentIndex(0)
        mode_row.addWidget(self.combo_driver_mode)
        
        lbl_mode_hint = QLabel("💡 Em modo UART, correntes e micropassos são ajustados digitalmente na placa via GPIO8.")
        lbl_mode_hint.setStyleSheet("color: #64748b; font-size: 11px;")
        mode_row.addWidget(lbl_mode_hint)
        mode_row.addStretch()
        tmc_layout.addLayout(mode_row)
        
        tmc_grid = QGridLayout()
        tmc_grid.setSpacing(10)
        
        # Column Headers
        th_driver = QLabel("Driver")
        th_driver.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_addr = QLabel("Endereço UART (0..3)")
        th_addr.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_irun = QLabel("Corrente Trabalho (irun mA)")
        th_irun.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_ihold = QLabel("Corrente Parada (ihold mA)")
        th_ihold.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_usteps = QLabel("Micropassos")
        th_usteps.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        th_sc = QLabel("Modo SpreadCycle")
        th_sc.setStyleSheet("color: #94a3b8; font-weight: 700; font-size: 12px;")
        
        tmc_grid.addWidget(th_driver, 0, 0)
        tmc_grid.addWidget(th_addr, 0, 1)
        tmc_grid.addWidget(th_irun, 0, 2)
        tmc_grid.addWidget(th_ihold, 0, 3)
        tmc_grid.addWidget(th_usteps, 0, 4)
        tmc_grid.addWidget(th_sc, 0, 5)
        
        # Driver C (Base)
        lbl_drv_c = QLabel("Driver C (Base Rotativa):")
        lbl_drv_c.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        tmc_grid.addWidget(lbl_drv_c, 1, 0)
        
        self.spin_tmc_addr_c = QSpinBox()
        self.spin_tmc_addr_c.setRange(0, 3)
        self.spin_tmc_addr_c.setValue(0)
        tmc_grid.addWidget(self.spin_tmc_addr_c, 1, 1)
        
        self.spin_tmc_irun_c = QSpinBox()
        self.spin_tmc_irun_c.setRange(50, 2000)
        self.spin_tmc_irun_c.setValue(900)
        self.spin_tmc_irun_c.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_irun_c, 1, 2)
        
        self.spin_tmc_ihold_c = QSpinBox()
        self.spin_tmc_ihold_c.setRange(50, 2000)
        self.spin_tmc_ihold_c.setValue(350)
        self.spin_tmc_ihold_c.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_ihold_c, 1, 3)
        
        self.combo_tmc_usteps_c = QComboBox()
        self.combo_tmc_usteps_c.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_c.setCurrentText("16")
        self.combo_tmc_usteps_c.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_c, 1, 4)
        
        self.chk_tmc_sc_c = QCheckBox("SpreadCycle C")
        self.chk_tmc_sc_c.setToolTip("Marque para SpreadCycle (maior torque/alta rotação) ou desmarque para StealthChop (ultra-silencioso)")
        tmc_grid.addWidget(self.chk_tmc_sc_c, 1, 5)
        
        # Driver A (Pivot)
        lbl_drv_a = QLabel("Driver A (Pivot Lasers):")
        lbl_drv_a.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        tmc_grid.addWidget(lbl_drv_a, 2, 0)
        
        self.spin_tmc_addr_a = QSpinBox()
        self.spin_tmc_addr_a.setRange(0, 3)
        self.spin_tmc_addr_a.setValue(1)
        tmc_grid.addWidget(self.spin_tmc_addr_a, 2, 1)
        
        self.spin_tmc_irun_a = QSpinBox()
        self.spin_tmc_irun_a.setRange(50, 2000)
        self.spin_tmc_irun_a.setValue(900)
        self.spin_tmc_irun_a.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_irun_a, 2, 2)
        
        self.spin_tmc_ihold_a = QSpinBox()
        self.spin_tmc_ihold_a.setRange(50, 2000)
        self.spin_tmc_ihold_a.setValue(350)
        self.spin_tmc_ihold_a.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_ihold_a, 2, 3)
        
        self.combo_tmc_usteps_a = QComboBox()
        self.combo_tmc_usteps_a.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_a.setCurrentText("16")
        self.combo_tmc_usteps_a.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_a, 2, 4)
        
        self.chk_tmc_sc_a = QCheckBox("SpreadCycle A")
        self.chk_tmc_sc_a.setToolTip("Marque para SpreadCycle ou desmarque para StealthChop")
        tmc_grid.addWidget(self.chk_tmc_sc_a, 2, 5)
        
        # Driver Z (Linear)
        lbl_drv_z = QLabel("Driver Z (Atuador Linear):")
        lbl_drv_z.setStyleSheet("color: #e2e8f0; font-weight: 600;")
        tmc_grid.addWidget(lbl_drv_z, 3, 0)
        
        self.spin_tmc_addr_z = QSpinBox()
        self.spin_tmc_addr_z.setRange(0, 3)
        self.spin_tmc_addr_z.setValue(2)
        tmc_grid.addWidget(self.spin_tmc_addr_z, 3, 1)
        
        self.spin_tmc_irun_z = QSpinBox()
        self.spin_tmc_irun_z.setRange(50, 2000)
        self.spin_tmc_irun_z.setValue(950)
        self.spin_tmc_irun_z.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_irun_z, 3, 2)
        
        self.spin_tmc_ihold_z = QSpinBox()
        self.spin_tmc_ihold_z.setRange(50, 2000)
        self.spin_tmc_ihold_z.setValue(400)
        self.spin_tmc_ihold_z.setSuffix(" mA")
        tmc_grid.addWidget(self.spin_tmc_ihold_z, 3, 3)
        
        self.combo_tmc_usteps_z = QComboBox()
        self.combo_tmc_usteps_z.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_z.setCurrentText("16")
        self.combo_tmc_usteps_z.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_z, 3, 4)
        
        self.chk_tmc_sc_z = QCheckBox("SpreadCycle Z")
        self.chk_tmc_sc_z.setToolTip("Marque para SpreadCycle ou desmarque para StealthChop")
        tmc_grid.addWidget(self.chk_tmc_sc_z, 3, 5)
        
        tmc_layout.addLayout(tmc_grid)
        
        # Sub-Card: Diagnóstico e Acesso Direto a Registradores TMC2209
        reg_frame = QFrame()
        reg_frame.setProperty("class", "metric-card")
        reg_layout = QHBoxLayout(reg_frame)
        reg_layout.setContentsMargins(14, 10, 14, 10)
        reg_layout.setSpacing(12)
        
        lbl_diag = QLabel("Diagnóstico TMC (Acesso Direto):")
        lbl_diag.setStyleSheet("color: #94a3b8; font-weight: 600;")
        reg_layout.addWidget(lbl_diag)
        
        self.combo_reg_axis = QComboBox()
        self.combo_reg_axis.addItems(["C", "A", "Z"])
        self.combo_reg_axis.setToolTip("Eixo alvo para leitura ou gravação de registrador TMC")
        reg_layout.addWidget(self.combo_reg_axis)
        
        self.txt_reg_addr = QLineEdit("0x06")
        self.txt_reg_addr.setPlaceholderText("Reg Hex (ex: 0x06)")
        self.txt_reg_addr.setMaximumWidth(90)
        self.txt_reg_addr.setToolTip("Endereço hexadecimal do registrador (ex: 0x00 GCONF, 0x06 FACT_RESET, 0x6F SGTHRS)")
        reg_layout.addWidget(self.txt_reg_addr)
        
        self.txt_reg_val = QLineEdit("0x00000000")
        self.txt_reg_val.setPlaceholderText("Val Hex (32 bits)")
        self.txt_reg_val.setMaximumWidth(120)
        self.txt_reg_val.setToolTip("Valor hexadecimal de 32 bits a gravar")
        reg_layout.addWidget(self.txt_reg_val)
        
        self.btn_reg_read = QPushButton("📖 Ler Reg")
        self.btn_reg_read.clicked.connect(self._read_tmc_reg)
        reg_layout.addWidget(self.btn_reg_read)
        
        self.btn_reg_write = QPushButton("✏️ Gravar Reg")
        self.btn_reg_write.clicked.connect(self._write_tmc_reg)
        reg_layout.addWidget(self.btn_reg_write)
        
        reg_layout.addStretch()
        tmc_layout.addWidget(reg_frame)
        main_layout.addWidget(tmc_card)
        
        # ==============================================================
        # --- 4. BARRAMENTO CAN (TWAI - COMUNICAÇÃO ENTRE NÓS) ---
        # ==============================================================
        can_card = QFrame()
        can_card.setProperty("class", "card")
        can_layout = QVBoxLayout(can_card)
        can_layout.setContentsMargins(18, 16, 18, 16)
        can_layout.setSpacing(14)
        
        can_title = QLabel("4. Barramento CAN / TWAI (Comunicação dos 10 Nós)")
        can_title.setProperty("class", "section-title")
        can_layout.addWidget(can_title)
        
        # Enable CAN Toggle Row
        can_toggle_row = QHBoxLayout()
        can_toggle_row.setSpacing(14)
        
        self.chk_can_enabled = QCheckBox("Habilitar Barramento CAN (TWAI)")
        self.chk_can_enabled.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 13px;")
        self.chk_can_enabled.setToolTip("Ativa ou desativa a controladora de barramento CAN (TWAI) do ESP32-S3")
        self.chk_can_enabled.toggled.connect(self._on_can_enabled_toggled)
        can_toggle_row.addWidget(self.chk_can_enabled)
        
        self.lbl_can_status_badge = QLabel("DESATIVADO")
        self.lbl_can_status_badge.setProperty("class", "badge badge-gray")
        can_toggle_row.addWidget(self.lbl_can_status_badge)
        
        lbl_can_note = QLabel("Quando desativado, o ESP32 não transmite nem consome mensagens CAN, operando em modo Serial/USB autônomo.")
        lbl_can_note.setStyleSheet("color: #64748b; font-size: 11px;")
        can_toggle_row.addWidget(lbl_can_note)
        can_toggle_row.addStretch()
        
        can_layout.addLayout(can_toggle_row)
        
        can_grid = QGridLayout()
        can_grid.setSpacing(12)
        
        # CAN Settings Inputs
        can_grid.addWidget(QLabel("Node ID do ESP32 (1..10):"), 0, 0)
        self.spin_node_id = QSpinBox()
        self.spin_node_id.setRange(1, 10)
        self.spin_node_id.setValue(1)
        self.spin_node_id.setToolTip("Endereço único deste nó no barramento CAN (1..10)")
        can_grid.addWidget(self.spin_node_id, 0, 1)
        
        can_grid.addWidget(QLabel("Bitrate da Rede CAN:"), 0, 2)
        self.combo_can_bitrate = QComboBox()
        self.combo_can_bitrate.addItems(["125000", "250000", "500000", "1000000"])
        self.combo_can_bitrate.setCurrentText("500000")
        self.combo_can_bitrate.setToolTip("Taxa de transmissão em bps (Padrão: 500000 bps = 500 kbps)")
        can_grid.addWidget(self.combo_can_bitrate, 0, 3)
        
        can_grid.addWidget(QLabel("Base ID Comandos (Hex):"), 1, 0)
        self.txt_base_cmd = QLineEdit("0x200")
        self.txt_base_cmd.setToolTip("Base hexadecimal para IDs de comando CAN recebidos")
        can_grid.addWidget(self.txt_base_cmd, 1, 1)
        
        can_grid.addWidget(QLabel("Base ID Status (Hex):"), 1, 2)
        self.txt_base_status = QLineEdit("0x280")
        self.txt_base_status.setToolTip("Base hexadecimal para IDs de status periódico enviados")
        can_grid.addWidget(self.txt_base_status, 1, 3)
        
        can_grid.addWidget(QLabel("Base ID Eventos (Hex):"), 2, 0)
        self.txt_base_event = QLineEdit("0x300")
        self.txt_base_event.setToolTip("Base hexadecimal para IDs de telemetria/eventos enviados")
        can_grid.addWidget(self.txt_base_event, 2, 1)
        
        can_layout.addLayout(can_grid)
        main_layout.addWidget(can_card)
        
        # ==============================================================
        # --- PAINEL DE AÇÃO UNIFICADO (RODAPÉ) ---
        # ==============================================================
        action_card = QFrame()
        action_card.setProperty("class", "card")
        action_card.setStyleSheet("border: 1px solid #1e3a8a; background-color: #0f172a;")
        action_layout = QHBoxLayout(action_card)
        action_layout.setContentsMargins(18, 14, 18, 14)
        action_layout.setSpacing(16)
        
        info_vbox = QVBoxLayout()
        info_vbox.setSpacing(4)
        lbl_act_title = QLabel("💾  Gravação Unificada na Memória NVS Permanente")
        lbl_act_title.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 13px;")
        lbl_act_desc = QLabel("Ao gravar, todos os parâmetros acima (Cinemática, Limites, Drivers TMC2209 e CAN) são transmitidos em lote e persistidos no ESP32.")
        lbl_act_desc.setStyleSheet("color: #94a3b8; font-size: 11px;")
        info_vbox.addWidget(lbl_act_title)
        info_vbox.addWidget(lbl_act_desc)
        action_layout.addLayout(info_vbox, 1)
        
        self.btn_refresh_bottom = QPushButton("📥  Ler da Placa")
        self.btn_refresh_bottom.clicked.connect(self.comm.request_config_dump)
        action_layout.addWidget(self.btn_refresh_bottom)
        
        self.btn_save_all_bottom = QPushButton("💾  Gravar Todas as Configurações na NVS")
        self.btn_save_all_bottom.setProperty("class", "btn-primary")
        self.btn_save_all_bottom.setStyleSheet("font-weight: 700; font-size: 13px; padding: 10px 22px;")
        self.btn_save_all_bottom.clicked.connect(self._apply_all_parameters)
        action_layout.addWidget(self.btn_save_all_bottom)
        
        main_layout.addWidget(action_card)
        main_layout.addStretch()
        
        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)
        
        self.state.parameters_updated.connect(self._on_parameters_updated)
        self._update_kinematics_preview()

    def _on_can_enabled_toggled(self, checked: bool):
        if checked:
            self.lbl_can_status_badge.setText("ATIVO")
            self.lbl_can_status_badge.setProperty("class", "badge badge-success")
            self.lbl_can_status_badge.setStyleSheet("color: #4ade80; font-weight: 700;")
        else:
            self.lbl_can_status_badge.setText("DESATIVADO")
            self.lbl_can_status_badge.setProperty("class", "badge badge-gray")
            self.lbl_can_status_badge.setStyleSheet("color: #94a3b8; font-weight: 700;")

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
        
        deg_step_c = 360.0 / (spr_c * usteps_c) if (spr_c * usteps_c) > 0 else 0.1125
        
        self.lbl_kin_calc_info.setText(
            f"Eixo C/A: {spr_c * usteps_c} micropassos/volta ({deg_step_c:.4f}°/passo @ {usteps_c}x) | "
            f"Eixo Z: {teeth}T GT2 → {mm_rev:.2f} mm/volta | Resolução: {res_um:.2f} µm/passo ({steps_mm:.1f} passos/mm @ {usteps_z}x) | Limite Z: {max_mm:.1f} mm ({calc_steps_z} passos)"
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

        set_spin(self.spin_pulley_z, p.z_pulley_teeth if p.z_pulley_teeth > 0 else 16)
        teeth_z = p.z_pulley_teeth if p.z_pulley_teeth > 0 else 16
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
            # 5. Pulley Z & Max Z em passos
            lambda: client.set_z_pulley_teeth(teeth),
            lambda: client.set_length_z(steps_z),
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
