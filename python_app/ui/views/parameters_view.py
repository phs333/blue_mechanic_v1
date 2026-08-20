"""
Parameters and Hardware Configuration View (NVS, TMC2209 & CAN).

Kinematics:
- Eixo C: Base Rotativa
- Eixo A: Pivot dos Lasers (2 Lasers Colineares Opostos)
- Eixo Z: Atuador Linear (Correia e Polia GT2 com número de dentes configurável)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QGroupBox, QScrollArea,
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
        sub_lbl = QLabel("Ajuste cinemática dos Eixos C (Base), A (Pivot), Z (Linear e Polia GT2), TMC2209 e CAN")
        sub_lbl.setStyleSheet("color: #64748b; font-size: 12px;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(sub_lbl)
        header_layout.addLayout(title_vbox)
        
        header_layout.addStretch()
        
        self.btn_refresh = QPushButton("📥 Ler do Hardware")
        self.btn_refresh.clicked.connect(self.comm.request_status)
        header_layout.addWidget(self.btn_refresh)
        
        self.btn_save_nvs = QPushButton("💾 Gravar na NVS")
        self.btn_save_nvs.setProperty("class", "btn-primary")
        self.btn_save_nvs.clicked.connect(self._apply_all_parameters)
        header_layout.addWidget(self.btn_save_nvs)
        
        main_layout.addWidget(header_card)
        
        # --- 1. Kinematics & Limits Card ---
        kin_card = QFrame()
        kin_card.setProperty("class", "card")
        kin_layout = QVBoxLayout(kin_card)
        kin_layout.setContentsMargins(16, 14, 16, 14)
        kin_layout.setSpacing(12)
        
        kin_title = QLabel("1. Cinemática e Limites de Velocidade/Aceleração")
        kin_title.setProperty("class", "section-title")
        kin_layout.addWidget(kin_title)
        
        kin_grid = QGridLayout()
        kin_grid.setSpacing(10)
        
        # Headers
        kin_grid.addWidget(QLabel("Eixo"), 0, 0)
        kin_grid.addWidget(QLabel("Passos/Volta"), 0, 1)
        kin_grid.addWidget(QLabel("Velocidade (deg/s ou mm/s)"), 0, 2)
        kin_grid.addWidget(QLabel("Aceleração (deg/s² ou mm/s²)"), 0, 3)
        kin_grid.addWidget(QLabel("Inverter Rotação"), 0, 4)
        
        # Axis C (Base Rotativa)
        kin_grid.addWidget(QLabel("Eixo C (Base):"), 1, 0)
        self.spin_steps_c = QSpinBox()
        self.spin_steps_c.setRange(200, 10000)
        self.spin_steps_c.setValue(200)
        kin_grid.addWidget(self.spin_steps_c, 1, 1)
        
        self.spin_speed_c = QDoubleSpinBox()
        self.spin_speed_c.setRange(1.0, 10000.0)
        self.spin_speed_c.setValue(1500.0)
        kin_grid.addWidget(self.spin_speed_c, 1, 2)
        
        self.spin_accel_c = QDoubleSpinBox()
        self.spin_accel_c.setRange(10.0, 50000.0)
        self.spin_accel_c.setValue(5000.0)
        kin_grid.addWidget(self.spin_accel_c, 1, 3)
        
        self.chk_invert_c = QCheckBox("Inverter C")
        kin_grid.addWidget(self.chk_invert_c, 1, 4)
        
        # Axis A (Pivot dos Lasers)
        kin_grid.addWidget(QLabel("Eixo A (Pivot):"), 2, 0)
        self.spin_steps_a = QSpinBox()
        self.spin_steps_a.setRange(200, 10000)
        self.spin_steps_a.setValue(200)
        kin_grid.addWidget(self.spin_steps_a, 2, 1)
        
        self.spin_speed_a = QDoubleSpinBox()
        self.spin_speed_a.setRange(1.0, 10000.0)
        self.spin_speed_a.setValue(1500.0)
        kin_grid.addWidget(self.spin_speed_a, 2, 2)
        
        self.spin_accel_a = QDoubleSpinBox()
        self.spin_accel_a.setRange(10.0, 50000.0)
        self.spin_accel_a.setValue(5000.0)
        kin_grid.addWidget(self.spin_accel_a, 2, 3)
        
        self.chk_invert_a = QCheckBox("Inverter A")
        kin_grid.addWidget(self.chk_invert_a, 2, 4)
        
        # Axis Z (Linear)
        kin_grid.addWidget(QLabel("Eixo Z (Linear):"), 3, 0)
        self.spin_steps_z = QSpinBox()
        self.spin_steps_z.setRange(200, 10000)
        self.spin_steps_z.setValue(200)
        self.spin_steps_z.valueChanged.connect(self._update_z_calc_preview)
        kin_grid.addWidget(self.spin_steps_z, 3, 1)
        
        self.spin_speed_z = QDoubleSpinBox()
        self.spin_speed_z.setRange(0.1, 500.0)
        self.spin_speed_z.setValue(80.0)
        kin_grid.addWidget(self.spin_speed_z, 3, 2)
        
        self.spin_accel_z = QDoubleSpinBox()
        self.spin_accel_z.setRange(1.0, 5000.0)
        self.spin_accel_z.setValue(1000.0)
        kin_grid.addWidget(self.spin_accel_z, 3, 3)
        
        self.chk_invert_z = QCheckBox("Inverter Z")
        kin_grid.addWidget(self.chk_invert_z, 3, 4)
        
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
        self.spin_pulley_z.setToolTip("Número de dentes da polia dentada GT2 montada no eixo do motor Z (ex: 16T, 20T, 36T)")
        self.spin_pulley_z.valueChanged.connect(self._update_z_calc_preview)
        z_config_grid.addWidget(self.spin_pulley_z, 0, 1)
        
        z_config_grid.addWidget(QLabel("📏 Limite Máximo Z (max_passos_z):"), 0, 2)
        self.spin_max_z = QSpinBox()
        self.spin_max_z.setRange(1000, 100000)
        self.spin_max_z.setValue(20000)
        self.spin_max_z.setSingleStep(500)
        self.spin_max_z.valueChanged.connect(self._update_z_calc_preview)
        z_config_grid.addWidget(self.spin_max_z, 0, 3)
        
        z_extra_layout.addLayout(z_config_grid)
        
        # Dynamic preview banner
        self.lbl_z_calc_info = QLabel("Cálculo: 16 dentes GT2 (passo 2.0mm) → 32.00 mm/volta | Resolução: 10.00 µm/passo (100.0 passos/mm)")
        self.lbl_z_calc_info.setStyleSheet("color: #38bdf8; font-weight: 600; font-size: 11px;")
        z_extra_layout.addWidget(self.lbl_z_calc_info)
        
        kin_layout.addWidget(z_extra_frame)
        main_layout.addWidget(kin_card)
        
        # --- 2. TMC2209 Driver UART Config Card ---
        tmc_card = QFrame()
        tmc_card.setProperty("class", "card")
        tmc_layout = QVBoxLayout(tmc_card)
        tmc_layout.setContentsMargins(16, 14, 16, 14)
        tmc_layout.setSpacing(12)
        
        tmc_title = QLabel("2. Configuração Avançada dos Drivers TMC2209")
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
        self.spin_tmc_irun_c.setValue(800)
        tmc_grid.addWidget(self.spin_tmc_irun_c, 1, 2)
        
        self.spin_tmc_ihold_c = QSpinBox()
        self.spin_tmc_ihold_c.setRange(50, 2000)
        self.spin_tmc_ihold_c.setValue(300)
        tmc_grid.addWidget(self.spin_tmc_ihold_c, 1, 3)
        
        self.combo_tmc_usteps_c = QComboBox()
        self.combo_tmc_usteps_c.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_c.setCurrentText("16")
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
        self.spin_tmc_irun_a.setValue(800)
        tmc_grid.addWidget(self.spin_tmc_irun_a, 2, 2)
        
        self.spin_tmc_ihold_a = QSpinBox()
        self.spin_tmc_ihold_a.setRange(50, 2000)
        self.spin_tmc_ihold_a.setValue(300)
        tmc_grid.addWidget(self.spin_tmc_ihold_a, 2, 3)
        
        self.combo_tmc_usteps_a = QComboBox()
        self.combo_tmc_usteps_a.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_a.setCurrentText("16")
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
        self.spin_tmc_irun_z.setValue(800)
        tmc_grid.addWidget(self.spin_tmc_irun_z, 3, 2)
        
        self.spin_tmc_ihold_z = QSpinBox()
        self.spin_tmc_ihold_z.setRange(50, 2000)
        self.spin_tmc_ihold_z.setValue(300)
        tmc_grid.addWidget(self.spin_tmc_ihold_z, 3, 3)
        
        self.combo_tmc_usteps_z = QComboBox()
        self.combo_tmc_usteps_z.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.combo_tmc_usteps_z.setCurrentText("16")
        self.combo_tmc_usteps_z.currentTextChanged.connect(self._update_z_calc_preview)
        tmc_grid.addWidget(self.combo_tmc_usteps_z, 3, 4)
        
        self.chk_tmc_sc_z = QCheckBox("SpreadCycle")
        tmc_grid.addWidget(self.chk_tmc_sc_z, 3, 5)
        
        tmc_layout.addLayout(tmc_grid)
        
        # TMC Apply Button
        tmc_btn_row = QHBoxLayout()
        self.btn_apply_tmc = QPushButton("⚡ Aplicar Configuração UART TMC2209 (DRIVER APPLY)")
        self.btn_apply_tmc.clicked.connect(self._apply_tmc_settings)
        tmc_btn_row.addWidget(self.btn_apply_tmc)
        tmc_btn_row.addStretch()
        tmc_layout.addLayout(tmc_btn_row)
        
        main_layout.addWidget(tmc_card)
        
        # --- 3. CAN Network Settings Card ---
        can_card = QFrame()
        can_card.setProperty("class", "card")
        can_layout = QVBoxLayout(can_card)
        can_layout.setContentsMargins(16, 14, 16, 14)
        can_layout.setSpacing(12)
        
        can_title = QLabel("3. Configurações da Rede CAN (TWAI ESP32-S3)")
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
        self._update_z_calc_preview()

    def _update_z_calc_preview(self):
        teeth = self.spin_pulley_z.value()
        spr = self.spin_steps_z.value()
        try:
            usteps = int(self.combo_tmc_usteps_z.currentText())
        except ValueError:
            usteps = 16
        mm_rev = calc_z_mm_per_rev(teeth)
        mm_step = calc_z_mm_per_step(teeth, spr, usteps)
        steps_mm = calc_z_steps_per_mm(teeth, spr, usteps)
        res_um = mm_step * 1000.0
        max_mm = self.spin_max_z.value() * mm_step
        
        self.lbl_z_calc_info.setText(
            f"Cálculo Z: {teeth} dentes GT2 ({Z_BELT_PITCH_MM:.1f}mm) → {mm_rev:.2f} mm/volta | "
            f"Resolução: {res_um:.2f} µm/passo ({steps_mm:.1f} passos/mm @ {usteps}x) | Curso Total: {max_mm:.1f} mm"
        )

    def _on_parameters_updated(self, p: HardwareParameters):
        self.spin_pulley_z.blockSignals(True)
        self.spin_pulley_z.setValue(p.z_pulley_teeth if p.z_pulley_teeth > 0 else 16)
        self.spin_pulley_z.blockSignals(False)
        self.spin_max_z.blockSignals(True)
        self.spin_max_z.setValue(p.max_passos_z)
        self.spin_max_z.blockSignals(False)
        self._update_z_calc_preview()

    def _apply_tmc_settings(self):
        if hasattr(self.comm.active_client, 'set_tmc_uart_current'):
            client = self.comm.active_client
            client.set_tmc_uart_current('C', self.spin_tmc_ihold_c.value(), self.spin_tmc_irun_c.value(), 6)
            client.set_tmc_uart_current('A', self.spin_tmc_ihold_a.value(), self.spin_tmc_irun_a.value(), 6)
            client.set_tmc_uart_current('Z', self.spin_tmc_ihold_z.value(), self.spin_tmc_irun_z.value(), 6)
            client.set_tmc_spreadcycle('C', self.chk_tmc_sc_c.isChecked())
            client.set_tmc_spreadcycle('A', self.chk_tmc_sc_a.isChecked())
            client.set_tmc_spreadcycle('Z', self.chk_tmc_sc_z.isChecked())
            client.set_tmc_microsteps('C', int(self.combo_tmc_usteps_c.currentText()))
            client.set_tmc_microsteps('A', int(self.combo_tmc_usteps_a.currentText()))
            client.set_tmc_microsteps('Z', int(self.combo_tmc_usteps_z.currentText()))
            client.apply_driver_settings()

    def _apply_can_settings(self):
        try:
            node_id = self.spin_node_id.value()
            bitrate = int(self.combo_can_bitrate.currentText())
            cmd_base = int(self.txt_base_cmd.text(), 16)
            status_base = int(self.txt_base_status.text(), 16)
            event_base = int(self.txt_base_event.text(), 16)
            if hasattr(self.comm.active_client, 'configure_can'):
                self.comm.active_client.configure_can(node_id, bitrate, cmd_base, status_base, event_base)
        except ValueError as e:
            QMessageBox.warning(self, "Valor Inválido", f"Formato hexadecimal incorreto nas bases CAN: {e}")

    def _apply_all_parameters(self):
        if hasattr(self.comm.active_client, 'set_steps_per_rev'):
            client = self.comm.active_client
            # Steps
            client.set_steps_per_rev('C', self.spin_steps_c.value())
            client.set_steps_per_rev('A', self.spin_steps_a.value())
            client.set_steps_per_rev('Z', self.spin_steps_z.value())
            # Speeds
            client.set_axis_speed('C', self.spin_speed_c.value())
            client.set_axis_speed('A', self.spin_speed_a.value())
            client.set_axis_speed('Z', self.spin_speed_z.value())
            # Inverts
            client.set_driver_invert('C', self.chk_invert_c.isChecked())
            client.set_driver_invert('A', self.chk_invert_a.isChecked())
            client.set_driver_invert('Z', self.chk_invert_z.isChecked())
            # Pulley Z & Max Z
            client.set_z_pulley_teeth(self.spin_pulley_z.value())
            client.set_length_z(self.spin_max_z.value())
            
        self._apply_tmc_settings()
        self._apply_can_settings()
        QMessageBox.information(self, "Sucesso", "Parâmetros e número de dentes da polia Z enviados ao hardware!")
