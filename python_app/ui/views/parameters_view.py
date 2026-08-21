"""
Parameters and Hardware Configuration View (NVS, Kinematics, TMC2209 & CAN).

Kinematics:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (2 Lasers Colineares Opostos)
- Eixo Z: Atuador Linear (Correia e Polia GT2 com dentes configuráveis)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QScrollArea,
    QLineEdit, QMessageBox
)
from PyQt6.QtCore import Qt
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import HardwareParameters, DeviceState
from python_app.core.protocol_defs import calc_z_mm_per_rev, calc_z_mm_per_step, calc_z_steps_per_mm, Z_BELT_PITCH_MM

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
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(16)
        
        # --- Header ---
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)
        
        title_vbox = QVBoxLayout()
        title_lbl = QLabel("Gerenciamento de Parâmetros & NVS")
        title_lbl.setStyleSheet("color: #38bdf8; font-size: 16px; font-weight: 700;")
        sub_lbl = QLabel("Ajuste completo da cinemática C/A/Z, Driver Mode, Inversão de DIR, TMC2209 e CAN")
        sub_lbl.setStyleSheet("color: #64748b; font-size: 12px;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        self.btn_refresh = QPushButton("📥 Ler do Hardware (STATUS)")
        self.btn_refresh.clicked.connect(self.comm.request_status)
        header_layout.addWidget(self.btn_refresh)
        
        self.btn_save_nvs = QPushButton("💾 Gravar Toda Configuração na NVS")
        self.btn_save_nvs.setProperty("class", "btn-primary")
        self.btn_save_nvs.clicked.connect(self._apply_all_parameters)
        header_layout.addWidget(self.btn_save_nvs)
        
        main_layout.addWidget(header_card)
        
        # --- 0. Driver Mode & Hardware DIR Inversion Card ---
        driver_mode_card = QFrame()
        driver_mode_card.setProperty("class", "card")
        driver_mode_layout = QVBoxLayout(driver_mode_card)
        driver_mode_layout.setContentsMargins(16, 14, 16, 14)
        driver_mode_layout.setSpacing(12)
        
        dm_title = QLabel("1. Modo do Barramento dos Drivers & Inversão de Sentido (DIR)")
        dm_title.setProperty("class", "section-title")
        driver_mode_layout.addWidget(dm_title)
        
        dm_grid = QGridLayout()
        dm_grid.setSpacing(12)
        
        dm_grid.addWidget(QLabel("Modo do Barramento (DRIVER MODE):"), 0, 0)
        self.combo_driver_mode = QComboBox()
        self.combo_driver_mode.addItems(["STEP/DIR Puro (STEPDIR)", "UART TMC2209 Digital (UART)"])
        self.combo_driver_mode.setCurrentIndex(1)
        dm_grid.addWidget(self.combo_driver_mode, 0, 1)
        
        self.btn_apply_mode = QPushButton("⚡ Aplicar Modo (DRIVER APPLY)")
        self.btn_apply_mode.clicked.connect(self._apply_driver_mode)
        dm_grid.addWidget(self.btn_apply_mode, 0, 2)
        
        # Inversion checkboxes
        inv_box = QHBoxLayout()
        inv_box.setSpacing(16)
        inv_lbl = QLabel("Inversão de Hardware (DRIVER INVERT):")
        inv_lbl.setStyleSheet("color: #94a3b8; font-weight: 600;")
        inv_box.addWidget(inv_lbl)
        
        self.chk_inv_hw_c = QCheckBox("Inverter DIR C (Base)")
        self.chk_inv_hw_c.toggled.connect(lambda chk: self.comm.set_driver_invert('C', chk))
        inv_box.addWidget(self.chk_inv_hw_c)
        
        self.chk_inv_hw_a = QCheckBox("Inverter DIR A (Pivot)")
        self.chk_inv_hw_a.toggled.connect(lambda chk: self.comm.set_driver_invert('A', chk))
        inv_box.addWidget(self.chk_inv_hw_a)
        
        self.chk_inv_hw_z = QCheckBox("Inverter DIR Z (Linear)")
        self.chk_inv_hw_z.toggled.connect(lambda chk: self.comm.set_driver_invert('Z', chk))
        inv_box.addWidget(self.chk_inv_hw_z)
        
        inv_box.addStretch()
        driver_mode_layout.addLayout(dm_grid)
        driver_mode_layout.addLayout(inv_box)
        main_layout.addWidget(driver_mode_card)
        
        # --- 1. Kinematics & Limits Card ---
        kin_card = QFrame()
        kin_card.setProperty("class", "card")
        kin_layout = QVBoxLayout(kin_card)
        kin_layout.setContentsMargins(16, 14, 16, 14)
        kin_layout.setSpacing(12)
        
        kin_title = QLabel("2. Cinemática, Passos do Motor, Velocidade e Aceleração")
        kin_title.setProperty("class", "section-title")
        kin_layout.addWidget(kin_title)
        
        kin_grid = QGridLayout()
        kin_grid.setSpacing(10)
        
        # Headers
        kin_grid.addWidget(QLabel("Eixo"), 0, 0)
        kin_grid.addWidget(QLabel("Passos Base Motor (ex: 200/400)"), 0, 1)
        kin_grid.addWidget(QLabel("Velocidade (deg/s ou mm/s)"), 0, 2)
        kin_grid.addWidget(QLabel("Aceleração (deg/s² ou mm/s²)"), 0, 3)
        
        # Axis C (Base Rotativa)
        kin_grid.addWidget(QLabel("Eixo C (Base):"), 1, 0)
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
        
        # Axis A (Pivot dos Lasers)
        kin_grid.addWidget(QLabel("Eixo A (Pivot):"), 2, 0)
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
        
        # Axis Z (Linear)
        kin_grid.addWidget(QLabel("Eixo Z (Linear):"), 3, 0)
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
        
        kin_layout.addLayout(kin_grid)
        
        # Extra Z configuration rows (Pulley teeth & max steps)
        z_extra_frame = QFrame()
        z_extra_frame.setProperty("class", "metric-card")
        z_extra_layout = QVBoxLayout(z_extra_frame)
        z_extra_layout.setContentsMargins(12, 10, 12, 10)
        z_extra_layout.setSpacing(8)
        
        z_config_grid = QGridLayout()
        z_config_grid.setSpacing(10)
        
        z_config_grid.addWidget(QLabel("⚙️ Dentes da Polia Motor Z (GT2 2.0mm):"), 0, 0)
        self.spin_pulley_z = QSpinBox()
        self.spin_pulley_z.setRange(6, 200)
        self.spin_pulley_z.setValue(16)
        self.spin_pulley_z.setToolTip("Número de dentes da polia dentada GT2 montada no motor Z (ex: 16T, 20T, 36T)")
        self.spin_pulley_z.valueChanged.connect(self._update_kinematics_preview)
        z_config_grid.addWidget(self.spin_pulley_z, 0, 1)
        
        z_config_grid.addWidget(QLabel("📏 Limite Máximo Z (max_passos_z):"), 0, 2)
        self.spin_max_z = QSpinBox()
        self.spin_max_z.setRange(1000, 100000)
        self.spin_max_z.setValue(20000)
        self.spin_max_z.setSingleStep(500)
        self.spin_max_z.valueChanged.connect(self._update_kinematics_preview)
        z_config_grid.addWidget(self.spin_max_z, 0, 3)
        
        z_extra_layout.addLayout(z_config_grid)
        
        # Dynamic preview banner
        self.lbl_kin_calc_info = QLabel("Cálculo: ...")
        self.lbl_kin_calc_info.setStyleSheet("color: #38bdf8; font-weight: 600; font-size: 11px;")
        z_extra_layout.addWidget(self.lbl_kin_calc_info)
        
        kin_layout.addWidget(z_extra_frame)
        main_layout.addWidget(kin_card)
        
        # --- Limites Angulares dos Eixos C e A Card ---
        limits_card = QFrame()
        limits_card.setProperty("class", "card")
        limits_layout = QVBoxLayout(limits_card)
        limits_layout.setContentsMargins(16, 14, 16, 14)
        limits_layout.setSpacing(12)
        
        limits_title = QLabel("3. Limites Angulares de Curso dos Encoders (NVS)")
        limits_title.setProperty("class", "section-title")
        limits_layout.addWidget(limits_title)
        
        limits_grid = QGridLayout()
        limits_grid.setSpacing(10)
        
        limits_grid.addWidget(QLabel("Eixo"), 0, 0)
        limits_grid.addWidget(QLabel("Limite Mínimo (°)"), 0, 1)
        limits_grid.addWidget(QLabel("Limite Máximo (°)"), 0, 2)
        limits_grid.addWidget(QLabel("Ação"), 0, 3)
        
        # Eixo C
        limits_grid.addWidget(QLabel("Eixo C (Base Rotativa):"), 1, 0)
        self.spin_limit_min_c = QDoubleSpinBox()
        self.spin_limit_min_c.setRange(0.0, 360.0)
        self.spin_limit_min_c.setDecimals(2)
        self.spin_limit_min_c.setValue(10.0)
        self.spin_limit_min_c.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_min_c, 1, 1)
        
        self.spin_limit_max_c = QDoubleSpinBox()
        self.spin_limit_max_c.setRange(0.0, 360.0)
        self.spin_limit_max_c.setDecimals(2)
        self.spin_limit_max_c.setValue(190.0)
        self.spin_limit_max_c.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_max_c, 1, 2)
        
        self.btn_save_limit_c = QPushButton("💾 Salvar Limites C")
        self.btn_save_limit_c.clicked.connect(self._save_limit_c)
        limits_grid.addWidget(self.btn_save_limit_c, 1, 3)
        
        # Eixo A
        limits_grid.addWidget(QLabel("Eixo A (Pivot Lasers):"), 2, 0)
        self.spin_limit_min_a = QDoubleSpinBox()
        self.spin_limit_min_a.setRange(0.0, 360.0)
        self.spin_limit_min_a.setDecimals(2)
        self.spin_limit_min_a.setValue(10.0)
        self.spin_limit_min_a.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_min_a, 2, 1)
        
        self.spin_limit_max_a = QDoubleSpinBox()
        self.spin_limit_max_a.setRange(0.0, 360.0)
        self.spin_limit_max_a.setDecimals(2)
        self.spin_limit_max_a.setValue(190.0)
        self.spin_limit_max_a.setSuffix(" °")
        limits_grid.addWidget(self.spin_limit_max_a, 2, 2)
        
        self.btn_save_limit_a = QPushButton("💾 Salvar Limites A")
        self.btn_save_limit_a.clicked.connect(self._save_limit_a)
        limits_grid.addWidget(self.btn_save_limit_a, 2, 3)
        
        limits_layout.addLayout(limits_grid)
        main_layout.addWidget(limits_card)
        
        # --- 2. TMC2209 Driver UART Config Card ---
        tmc_card = QFrame()
        tmc_card.setProperty("class", "card")
        tmc_layout = QVBoxLayout(tmc_card)
        tmc_layout.setContentsMargins(16, 14, 16, 14)
        tmc_layout.setSpacing(12)
        
        tmc_title = QLabel("4. Configuração Avançada dos Drivers TMC2209")
        tmc_title.setProperty("class", "section-title")
        tmc_layout.addWidget(tmc_title)
        
        tmc_grid = QGridLayout()
        tmc_grid.setSpacing(10)
        
        tmc_grid.addWidget(QLabel("Eixo"), 0, 0)
        tmc_grid.addWidget(QLabel("Endereço UART (0..3)"), 0, 1)
        tmc_grid.addWidget(QLabel("Corrente Trabalho (irun mA)"), 0, 2)
        tmc_grid.addWidget(QLabel("Corrente Parada (ihold mA)"), 0, 3)
        tmc_grid.addWidget(QLabel("Microsteps"), 0, 4)
        tmc_grid.addWidget(QLabel("Modo SpreadCycle"), 0, 5)
        
        # C (Base)
        tmc_grid.addWidget(QLabel("Driver C (Base):"), 1, 0)
        self.spin_tmc_addr_c = QSpinBox()
        self.spin_tmc_addr_c.setRange(0, 3)
        self.spin_tmc_addr_c.setValue(0)
        tmc_grid.addWidget(self.spin_tmc_addr_c, 1, 1)
        
        self.spin_tmc_irun_c = QSpinBox()
        self.spin_tmc_irun_c.setRange(50, 2000)
        self.spin_tmc_irun_c.setValue(900)
        tmc_grid.addWidget(self.spin_tmc_irun_c, 1, 2)
        
        self.spin_tmc_ihold_c = QSpinBox()
        self.spin_tmc_ihold_c.setRange(50, 2000)
        self.spin_tmc_ihold_c.setValue(350)
        tmc_grid.addWidget(self.spin_tmc_ihold_c, 1, 3)
        
        self.combo_tmc_usteps_c = QComboBox()
        self.combo_tmc_usteps_c.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_c.setCurrentText("16")
        self.combo_tmc_usteps_c.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_c, 1, 4)
        
        self.chk_tmc_sc_c = QCheckBox("SpreadCycle")
        tmc_grid.addWidget(self.chk_tmc_sc_c, 1, 5)
        
        # A (Pivot)
        tmc_grid.addWidget(QLabel("Driver A (Pivot):"), 2, 0)
        self.spin_tmc_addr_a = QSpinBox()
        self.spin_tmc_addr_a.setRange(0, 3)
        self.spin_tmc_addr_a.setValue(1)
        tmc_grid.addWidget(self.spin_tmc_addr_a, 2, 1)
        
        self.spin_tmc_irun_a = QSpinBox()
        self.spin_tmc_irun_a.setRange(50, 2000)
        self.spin_tmc_irun_a.setValue(900)
        tmc_grid.addWidget(self.spin_tmc_irun_a, 2, 2)
        
        self.spin_tmc_ihold_a = QSpinBox()
        self.spin_tmc_ihold_a.setRange(50, 2000)
        self.spin_tmc_ihold_a.setValue(350)
        tmc_grid.addWidget(self.spin_tmc_ihold_a, 2, 3)
        
        self.combo_tmc_usteps_a = QComboBox()
        self.combo_tmc_usteps_a.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_a.setCurrentText("16")
        self.combo_tmc_usteps_a.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_a, 2, 4)
        
        self.chk_tmc_sc_a = QCheckBox("SpreadCycle")
        tmc_grid.addWidget(self.chk_tmc_sc_a, 2, 5)
        
        # Z (Linear)
        tmc_grid.addWidget(QLabel("Driver Z (Linear):"), 3, 0)
        self.spin_tmc_addr_z = QSpinBox()
        self.spin_tmc_addr_z.setRange(0, 3)
        self.spin_tmc_addr_z.setValue(2)
        tmc_grid.addWidget(self.spin_tmc_addr_z, 3, 1)
        
        self.spin_tmc_irun_z = QSpinBox()
        self.spin_tmc_irun_z.setRange(50, 2000)
        self.spin_tmc_irun_z.setValue(950)
        tmc_grid.addWidget(self.spin_tmc_irun_z, 3, 2)
        
        self.spin_tmc_ihold_z = QSpinBox()
        self.spin_tmc_ihold_z.setRange(50, 2000)
        self.spin_tmc_ihold_z.setValue(400)
        tmc_grid.addWidget(self.spin_tmc_ihold_z, 3, 3)
        
        self.combo_tmc_usteps_z = QComboBox()
        self.combo_tmc_usteps_z.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_z.setCurrentText("16")
        self.combo_tmc_usteps_z.currentTextChanged.connect(self._update_kinematics_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_z, 3, 4)
        
        self.chk_tmc_sc_z = QCheckBox("SpreadCycle")
        tmc_grid.addWidget(self.chk_tmc_sc_z, 3, 5)
        
        tmc_layout.addLayout(tmc_grid)
        
        # TMC Apply Button & Direct Register Access Tool
        tmc_btn_row = QHBoxLayout()
        self.btn_apply_tmc = QPushButton("⚡ Aplicar Configuração UART TMC2209")
        self.btn_apply_tmc.clicked.connect(self._apply_tmc_settings)
        tmc_btn_row.addWidget(self.btn_apply_tmc)
        tmc_btn_row.addStretch()
        tmc_layout.addLayout(tmc_btn_row)
        
        # Direct Register Tool
        reg_frame = QFrame()
        reg_frame.setProperty("class", "metric-card")
        reg_layout = QHBoxLayout(reg_frame)
        reg_layout.setContentsMargins(10, 8, 10, 8)
        reg_layout.setSpacing(10)
        
        reg_layout.addWidget(QLabel("Diagnóstico TMC:"))
        self.combo_reg_axis = QComboBox()
        self.combo_reg_axis.addItems(["C", "A", "Z"])
        reg_layout.addWidget(self.combo_reg_axis)
        
        self.txt_reg_addr = QLineEdit("0x06")
        self.txt_reg_addr.setPlaceholderText("Reg Hex (ex: 0x06)")
        self.txt_reg_addr.setMaximumWidth(80)
        reg_layout.addWidget(self.txt_reg_addr)
        
        self.txt_reg_val = QLineEdit("0x00000000")
        self.txt_reg_val.setPlaceholderText("Val Hex")
        self.txt_reg_val.setMaximumWidth(110)
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
        
        # --- 3. CAN Network Settings Card ---
        can_card = QFrame()
        can_card.setProperty("class", "card")
        can_layout = QVBoxLayout(can_card)
        can_layout.setContentsMargins(16, 14, 16, 14)
        can_layout.setSpacing(12)
        
        can_title = QLabel("4. Configurações da Rede CAN (TWAI ESP32-S3)")
        can_title.setProperty("class", "section-title")
        can_layout.addWidget(can_title)
        
        can_grid = QGridLayout()
        can_grid.setSpacing(10)
        
        can_grid.addWidget(QLabel("Node ID do Slave (1..127):"), 0, 0)
        self.spin_node_id = QSpinBox()
        self.spin_node_id.setRange(1, 127)
        self.spin_node_id.setValue(1)
        can_grid.addWidget(self.spin_node_id, 0, 1)
        
        can_grid.addWidget(QLabel("Bitrate CAN:"), 0, 2)
        self.combo_can_bitrate = QComboBox()
        self.combo_can_bitrate.addItems(["125000", "250000", "500000", "1000000"])
        self.combo_can_bitrate.setCurrentText("500000")
        can_grid.addWidget(self.combo_can_bitrate, 0, 3)
        
        can_grid.addWidget(QLabel("Base ID Comandos (Hex):"), 1, 0)
        self.txt_base_cmd = QLineEdit("0x200")
        can_grid.addWidget(self.txt_base_cmd, 1, 1)
        
        can_grid.addWidget(QLabel("Base ID Status (Hex):"), 1, 2)
        self.txt_base_status = QLineEdit("0x280")
        can_grid.addWidget(self.txt_base_status, 1, 3)
        
        can_grid.addWidget(QLabel("Base ID Eventos (Hex):"), 2, 0)
        self.txt_base_event = QLineEdit("0x300")
        can_grid.addWidget(self.txt_base_event, 2, 1)
        
        can_layout.addLayout(can_grid)
        
        can_btn_row = QHBoxLayout()
        self.btn_apply_can = QPushButton("📡 Aplicar Configurações CAN (CAN APPLY)")
        self.btn_apply_can.clicked.connect(self._apply_can_settings)
        can_btn_row.addWidget(self.btn_apply_can)
        can_btn_row.addStretch()
        can_layout.addLayout(can_btn_row)
        
        main_layout.addWidget(can_card)
        main_layout.addStretch()
        
        scroll.setWidget(container)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(scroll)
        
        self.state.parameters_updated.connect(self._on_parameters_updated)
        self._update_kinematics_preview()

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
        max_mm = self.spin_max_z.value() * mm_step
        
        deg_step_c = 360.0 / (spr_c * usteps_c) if (spr_c * usteps_c) > 0 else 0.1125
        
        self.lbl_kin_calc_info.setText(
            f"Eixo C/A: {spr_c * usteps_c} micropassos/volta ({deg_step_c:.4f}°/passo @ {usteps_c}x) | "
            f"Eixo Z: {teeth}T GT2 → {mm_rev:.2f} mm/volta | Resolução: {res_um:.2f} µm/passo ({steps_mm:.1f} passos/mm @ {usteps_z}x) | Curso: {max_mm:.1f} mm"
        )

    def _apply_driver_mode(self):
        mode = "UART" if "UART" in self.combo_driver_mode.currentText() else "STEPDIR"
        self.comm.set_driver_mode(mode)

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
            self.spin_pulley_z, self.spin_max_z,
            self.combo_driver_mode,
            self.spin_tmc_addr_c, self.spin_tmc_addr_a, self.spin_tmc_addr_z,
            self.spin_tmc_irun_c, self.spin_tmc_irun_a, self.spin_tmc_irun_z,
            self.spin_tmc_ihold_c, self.spin_tmc_ihold_a, self.spin_tmc_ihold_z,
            self.combo_tmc_usteps_c, self.combo_tmc_usteps_a, self.combo_tmc_usteps_z,
            self.chk_tmc_sc_c, self.chk_tmc_sc_a, self.chk_tmc_sc_z,
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
        set_spin(self.spin_max_z, p.max_passos_z)
        if not self.combo_driver_mode.hasFocus():
            self.combo_driver_mode.setCurrentIndex(1 if p.driver_bus_mode == 1 else 0)

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

        set_spin(self.spin_node_id, p.node_id)
        set_combo(self.combo_can_bitrate, str(p.can_bitrate))
        set_txt(self.txt_base_cmd, f"0x{p.can_command_base_id:03X}")
        set_txt(self.txt_base_status, f"0x{p.can_status_base_id:03X}")
        set_txt(self.txt_base_event, f"0x{p.can_event_base_id:03X}")

        for w in widgets:
            w.blockSignals(False)

        self._update_kinematics_preview()

    def _save_limit_c(self):
        min_v = self.spin_limit_min_c.value()
        max_v = self.spin_limit_max_c.value()
        if min_v >= max_v:
            QMessageBox.warning(self, "Limites Inválidos", "O limite mínimo de C deve ser menor que o máximo.")
            return
        if self.comm.set_axis_limits('C', min_v, max_v):
            QMessageBox.information(self, "Sucesso", f"Limites Eixo C gravados na NVS: [{min_v:.2f}, {max_v:.2f}]°")
        else:
            QMessageBox.warning(self, "Erro", "Não foi possível enviar comando LIMIT C.")

    def _save_limit_a(self):
        min_v = self.spin_limit_min_a.value()
        max_v = self.spin_limit_max_a.value()
        if min_v >= max_v:
            QMessageBox.warning(self, "Limites Inválidos", "O limite mínimo de A deve ser menor que o máximo.")
            return
        if self.comm.set_axis_limits('A', min_v, max_v):
            QMessageBox.information(self, "Sucesso", f"Limites Eixo A gravados na NVS: [{min_v:.2f}, {max_v:.2f}]°")
        else:
            QMessageBox.warning(self, "Erro", "Não foi possível enviar comando LIMIT A.")

    def _apply_tmc_settings(self):
        client = self.comm.active_client
        if client:
            target_mode = "UART" if "UART" in self.combo_driver_mode.currentText() else "STEPDIR"
            # Always ensure UART is active to write TMC registers
            results = [
                client.set_driver_mode("UART"),
                client.apply_driver_settings(),
                client.set_tmc_uart_current('C', self.spin_tmc_ihold_c.value(), self.spin_tmc_irun_c.value(), 6),
                client.set_tmc_uart_current('A', self.spin_tmc_ihold_a.value(), self.spin_tmc_irun_a.value(), 6),
                client.set_tmc_uart_current('Z', self.spin_tmc_ihold_z.value(), self.spin_tmc_irun_z.value(), 6),
                client.set_tmc_spreadcycle('C', self.chk_tmc_sc_c.isChecked()),
                client.set_tmc_spreadcycle('A', self.chk_tmc_sc_a.isChecked()),
                client.set_tmc_spreadcycle('Z', self.chk_tmc_sc_z.isChecked()),
                client.set_tmc_microsteps('C', int(self.combo_tmc_usteps_c.currentText())),
                client.set_tmc_microsteps('A', int(self.combo_tmc_usteps_a.currentText())),
                client.set_tmc_microsteps('Z', int(self.combo_tmc_usteps_z.currentText())),
            ]

            # Restore and apply configured driver mode
            results.extend([
                client.set_driver_mode(target_mode),
                client.apply_driver_settings(),
            ])
            return all(results)
        return False

    def _apply_can_settings(self):
        try:
            node_id = self.spin_node_id.value()
            bitrate = int(self.combo_can_bitrate.currentText())
            cmd_base = int(self.txt_base_cmd.text(), 16)
            status_base = int(self.txt_base_status.text(), 16)
            event_base = int(self.txt_base_event.text(), 16)
            if hasattr(self.comm.active_client, 'configure_can'):
                return self.comm.active_client.configure_can(node_id, bitrate, cmd_base, status_base, event_base)
        except ValueError as e:
            QMessageBox.warning(self, "Valor Inválido", f"Formato hexadecimal incorreto nas bases CAN: {e}")
        return False

    def _apply_all_parameters(self):
        client = self.comm.active_client
        if client:
            # 1. Steps
            results = [
                client.set_steps_per_rev('C', self.spin_steps_c.value()),
                client.set_steps_per_rev('A', self.spin_steps_a.value()),
                client.set_steps_per_rev('Z', self.spin_steps_z.value()),
            ]
            
            # 2. Speeds & Accel
            results.extend([
                client.set_axis_speed('C', self.spin_speed_c.value()),
                client.set_axis_speed('A', self.spin_speed_a.value()),
                client.set_axis_speed('Z', self.spin_speed_z.value()),
                client.set_axis_accel('C', self.spin_accel_c.value()),
                client.set_axis_accel('A', self.spin_accel_a.value()),
                client.set_axis_accel('Z', self.spin_accel_z.value()),
            ])
            
            # 3. Inverts
            results.extend([
                client.set_driver_invert('C', self.chk_inv_hw_c.isChecked()),
                client.set_driver_invert('A', self.chk_inv_hw_a.isChecked()),
                client.set_driver_invert('Z', self.chk_inv_hw_z.isChecked()),
            ])
            
            # 4. Pulley Z & Max Z
            results.extend([
                client.set_z_pulley_teeth(self.spin_pulley_z.value()),
                client.set_length_z(self.spin_max_z.value()),
            ])

            # 5. Angular Limits C & A
            results.extend([
                client.set_axis_limits('C', self.spin_limit_min_c.value(), self.spin_limit_max_c.value()),
                client.set_axis_limits('A', self.spin_limit_min_a.value(), self.spin_limit_max_a.value()),
            ])
            
            # 6. TMC settings (handles temporary UART mode and restore)
            results.append(self._apply_tmc_settings())
            
            # 7. CAN settings
            results.append(self._apply_can_settings())
            
            # 8. Refresh and read back
            results.append(self.comm.request_status())

            if all(results):
                QMessageBox.information(self, "Sucesso", "Todas as configurações foram enviadas ao hardware.")
            else:
                QMessageBox.warning(
                    self,
                    "Aplicação Parcial",
                    "Algumas configurações não são suportadas pela interface ativa ou não puderam ser enviadas. "
                    "Consulte a barra de status/terminal.",
                )
        else:
            QMessageBox.warning(self, "Sem conexão", "Conecte ao hardware antes de aplicar parâmetros.")
