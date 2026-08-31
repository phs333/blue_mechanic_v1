"""
Terminal and Diagnostics View.
Features raw Serial Monitor with command history and live CAN Frame Sniffer table.
"""

import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame,
    QLineEdit, QPlainTextEdit, QTableWidget, QTableWidgetItem, QHeaderView,
    QTabWidget, QCheckBox
)
from PyQt6.QtGui import QTextCursor, QColor
from PyQt6.QtCore import Qt
from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState

class TerminalView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self.history = []
        self.history_idx = -1
        self._terminal_mode = "ESP32"
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(12)
        
        # Tabs for Serial Console and CAN Sniffer
        self.tab_widget = QTabWidget()
        
        # --- TAB 1: Serial Monitor Console ---
        serial_tab = QWidget()
        serial_layout = QVBoxLayout(serial_tab)
        serial_layout.setContentsMargins(12, 12, 12, 12)
        serial_layout.setSpacing(10)
        
        # Console Toolbar
        tool_layout = QHBoxLayout()
        tool_layout.addWidget(QLabel("Monitor de Comunicação Serial / ASCII:"))
        tool_layout.addStretch()
        
        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        tool_layout.addWidget(self.chk_autoscroll)
        
        self.btn_clear_console = QPushButton("🗑️ Limpar")
        self.btn_clear_console.clicked.connect(self._clear_console)
        tool_layout.addWidget(self.btn_clear_console)
        
        serial_layout.addLayout(tool_layout)
        
        # Text Console
        self.console = QPlainTextEdit()
        self.console.setProperty("class", "console")
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        serial_layout.addWidget(self.console, 1)
        
        # Quick Presets Buttons
        preset_row = QHBoxLayout()
        preset_row.setSpacing(6)
        self.preset_buttons = []
        for index, cmd in enumerate(["STATUS", "HELP", "TEMP", "DRIVER STATUS", "CAN STATUS"]):
            btn = QPushButton(cmd)
            btn.clicked.connect(lambda checked, i=index: self._send_preset(i))
            preset_row.addWidget(btn)
            self.preset_buttons.append(btn)
        preset_row.addStretch()
        serial_layout.addLayout(preset_row)
        
        # Command Input Line
        input_layout = QHBoxLayout()
        self.txt_cmd = QLineEdit()
        self.txt_cmd.setPlaceholderText("Digite um comando (ex: STATUS, MOVE C 400, MOVE A -200, DRIVER ENABLED ON)...")
        self.txt_cmd.returnPressed.connect(self._send_command)
        input_layout.addWidget(self.txt_cmd, 1)
        
        self.btn_send = QPushButton("Enviar")
        self.btn_send.setProperty("class", "btn-primary")
        self.btn_send.clicked.connect(self._send_command)
        input_layout.addWidget(self.btn_send)
        
        serial_layout.addLayout(input_layout)
        self.tab_widget.addTab(serial_tab, "📟 Monitor ASCII / Serial")
        
        # --- TAB 2: CAN Frame Sniffer ---
        can_tab = QWidget()
        can_layout = QVBoxLayout(can_tab)
        can_layout.setContentsMargins(12, 12, 12, 12)
        can_layout.setSpacing(10)
        
        can_top = QHBoxLayout()
        can_top.addWidget(QLabel("Sniffer de Frames CAN (PeakCAN / PCAN):"))
        can_top.addStretch()
        
        self.btn_clear_can = QPushButton("🗑️ Limpar Sniffer")
        self.btn_clear_can.clicked.connect(self._clear_can_table)
        can_top.addWidget(self.btn_clear_can)
        can_layout.addLayout(can_top)
        
        # Sniffer Table
        self.can_table = QTableWidget()
        self.can_table.setColumnCount(6)
        self.can_table.setHorizontalHeaderLabels(["Hora", "Dir", "ID CAN", "DLC", "Dados Hex", "Interpretação / Evento"])
        self.can_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.can_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.can_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.can_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        self.can_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        self.can_table.horizontalHeader().setSectionResizeMode(5, QHeaderView.ResizeMode.Stretch)
        can_layout.addWidget(self.can_table)
        
        self.tab_widget.addTab(can_tab, "📡 CAN Bus Sniffer")
        
        main_layout.addWidget(self.tab_widget)
        
        # Signals
        self.state.raw_message_received.connect(self._append_console_msg)
        self.state.can_frame_received.connect(self._append_can_frame)
        self.state.connection_changed.connect(self._on_connection_changed)

    def _send_preset(self, index: int):
        if self._terminal_mode == "TEENSY":
            node = self.comm.teensy_serial_client.node_id
            commands = [
                f"R {node}",
                f"P {node} 10 20",
                f"E {node} 1",
                f"S {node} 3",
                f"F {node} 2",
            ]
            self.comm.send_raw(commands[index])
            return

        if index == 0:
            self.comm.request_status()
            return

        commands = ["STATUS", "HELP", "TEMP", "DRIVER STATUS", "CAN STATUS"]
        self.comm.send_raw(commands[index])

    def _on_connection_changed(self, connected: bool, backend: str):
        if connected and backend.startswith("Teensy USB/CAN"):
            self._terminal_mode = "TEENSY"
            node = self.comm.teensy_serial_client.node_id
            labels = [f"R {node}", f"P {node} 10 20", f"E {node} 1", f"S {node} 3", f"F {node} 2"]
            self.txt_cmd.setPlaceholderText(
                f"Teensy Node {node}: M {node} C 800, MF {node} A -400, H {node} Z, R {node}..."
            )
        else:
            self._terminal_mode = "ESP32"
            labels = ["STATUS", "HELP", "TEMP", "DRIVER STATUS", "CAN STATUS"]
            self.txt_cmd.setPlaceholderText(
                "Digite um comando (ex: STATUS, MOVE C 400, MOVE A -200, DRIVER ENABLED ON)..."
            )

        for button, label in zip(self.preset_buttons, labels):
            button.setText(label)

    def _clear_console(self):
        self.console.clear()

    def _clear_can_table(self):
        self.can_table.setRowCount(0)

    def _send_command(self):
        cmd = self.txt_cmd.text().strip()
        if not cmd:
            return
        self.history.append(cmd)
        self.history_idx = len(self.history)
        self.comm.send_raw(cmd)
        self.txt_cmd.clear()

    def _append_console_msg(self, direction: str, text: str):
        timestamp = time.strftime("%H:%M:%S")
        if direction == "TX":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <span style="color:#38bdf8; font-weight:bold;">&gt;&gt; TX:</span> <span style="color:#f8fafc;">{text}</span>'
        else:
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <span style="color:#34d399; font-weight:bold;">&lt;&lt; RX:</span> <span style="color:#e2e8f0;">{text}</span>'
        
        self.console.appendHtml(line)
        if self.chk_autoscroll.isChecked():
            self.console.moveCursor(QTextCursor.MoveOperation.End)

    def _append_can_frame(self, frame_info: dict):
        row = self.can_table.rowCount()
        self.can_table.insertRow(row)
        
        # Limit rows to 500
        if row > 500:
            self.can_table.removeRow(0)
            row = self.can_table.rowCount() - 1
            
        time_str = time.strftime("%H:%M:%S", time.localtime(frame_info.get("timestamp", time.time())))
        direction = frame_info.get("dir", "RX")
        can_id = frame_info.get("id", "0x000")
        dlc = str(frame_info.get("dlc", 0))
        data_hex = frame_info.get("data", "")
        desc = frame_info.get("desc", "")
        
        item_time = QTableWidgetItem(time_str)
        item_dir = QTableWidgetItem(direction)
        item_id = QTableWidgetItem(can_id)
        item_dlc = QTableWidgetItem(dlc)
        item_data = QTableWidgetItem(data_hex)
        item_desc = QTableWidgetItem(desc)
        
        if direction == "TX":
            item_dir.setForeground(QColor("#38bdf8"))
        else:
            item_dir.setForeground(QColor("#34d399"))
            
        self.can_table.setItem(row, 0, item_time)
        self.can_table.setItem(row, 1, item_dir)
        self.can_table.setItem(row, 2, item_id)
        self.can_table.setItem(row, 3, item_dlc)
        self.can_table.setItem(row, 4, item_data)
        self.can_table.setItem(row, 5, item_desc)
        
        self.can_table.scrollToBottom()
