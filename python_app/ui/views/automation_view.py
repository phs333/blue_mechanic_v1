"""
Test Automation and Looping View for Blue Mechanic V1.
Provides a script editor, preset loader, execution metrics, live logs,
and continuous looping execution via Teensy 4.1.
"""

import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QComboBox, QSpinBox, QCheckBox, QPlainTextEdit, QProgressBar,
    QFileDialog, QMessageBox, QSplitter
)
from PyQt6.QtGui import QTextCursor, QFont
from PyQt6.QtCore import Qt, QTimer

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.test_automation import (
    AUTOMATION_PRESETS,
    PRESET_SWEEP_CAZ,
    AutomationWorker,
    ScriptExpansionError,
    expand_automation_script,
    parse_script,
)


class AutomationView(QWidget):
    def __init__(self, comm: CommManager, state: DeviceState, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state

        self.worker: AutomationWorker = AutomationWorker(self.comm, self)
        self._setup_worker_signals()

        self.start_time = 0.0
        self.elapsed_timer = QTimer(self)
        self.elapsed_timer.setInterval(1000)
        self.elapsed_timer.timeout.connect(self._update_elapsed_time)

        self._init_ui()

    def _setup_worker_signals(self):
        self.worker.state_changed.connect(self._on_state_changed)
        self.worker.step_started.connect(self._on_step_started)
        self.worker.step_completed.connect(self._on_step_completed)
        self.worker.loop_completed.connect(self._on_loop_completed)
        self.worker.finished.connect(self._on_finished)
        self.worker.log_message.connect(self._append_log)
        self.worker.progress_updated.connect(self._on_progress_updated)

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(14)

        # ==========================================
        # 1. TOP HEADER & ACTION CONTROLS
        # ==========================================
        header_card = QFrame()
        header_card.setProperty("class", "card")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)
        header_layout.setSpacing(12)

        title_box = QVBoxLayout()
        lbl_title = QLabel("🔁 Automação de Testes & Sequenciador em Loop")
        lbl_title.setStyleSheet("color: #38bdf8; font-size: 16px; font-weight: 800;")
        lbl_desc = QLabel("Execução contínua de comandos ASCII/CAN via Teensy 4.1 com delays e suporte a broadcast.")
        lbl_desc.setStyleSheet("color: #64748b; font-size: 11px;")
        title_box.addWidget(lbl_title)
        title_box.addWidget(lbl_desc)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        # Elapsed time display
        self.lbl_elapsed = QLabel("⏱️ 00:00:00")
        self.lbl_elapsed.setStyleSheet("color: #94a3b8; font-family: 'Consolas', monospace; font-size: 13px; font-weight: 700; margin-right: 8px;")
        header_layout.addWidget(self.lbl_elapsed)

        # Status badge
        self.lbl_status_badge = QLabel("PARADO")
        self.lbl_status_badge.setProperty("class", "badge badge-gray")
        header_layout.addWidget(self.lbl_status_badge)

        # Action Buttons
        self.btn_start = QPushButton("▶️ Iniciar Automação")
        self.btn_start.setProperty("class", "btn-success")
        self.btn_start.clicked.connect(self._start_automation)
        header_layout.addWidget(self.btn_start)

        self.btn_pause = QPushButton("⏸️ Pausar")
        self.btn_pause.setProperty("class", "btn-warning")
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self._toggle_pause)
        header_layout.addWidget(self.btn_pause)

        self.btn_stop = QPushButton("⏹️ Parar")
        self.btn_stop.setProperty("class", "btn-danger")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._stop_automation)
        header_layout.addWidget(self.btn_stop)

        main_layout.addWidget(header_card)

        # ==========================================
        # 2. CONFIGURATION BAR
        # ==========================================
        config_card = QFrame()
        config_card.setProperty("class", "metric-card")
        config_layout = QHBoxLayout(config_card)
        config_layout.setContentsMargins(14, 10, 14, 10)
        config_layout.setSpacing(16)

        config_layout.addWidget(QLabel("Modo de Execução:"))
        self.combo_mode = QComboBox()
        self.combo_mode.addItems([
            "Loop Contínuo (Infinito)",
            "Repetir N Vezes",
            "Executar Uma Vez",
        ])
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        config_layout.addWidget(self.combo_mode)

        self.lbl_target_loops = QLabel("Repetições:")
        self.spin_target_loops = QSpinBox()
        self.spin_target_loops.setRange(1, 100000)
        self.spin_target_loops.setValue(10)
        self.lbl_target_loops.setVisible(False)
        self.spin_target_loops.setVisible(False)
        config_layout.addWidget(self.lbl_target_loops)
        config_layout.addWidget(self.spin_target_loops)

        config_layout.addWidget(QLabel("Delay Padrão (ms):"))
        self.spin_default_delay = QSpinBox()
        self.spin_default_delay.setRange(20, 10000)
        self.spin_default_delay.setValue(500)
        self.spin_default_delay.setSingleStep(50)
        self.spin_default_delay.setSuffix(" ms")
        config_layout.addWidget(self.spin_default_delay)

        self.chk_force_broadcast = QCheckBox("📢 Forçar Broadcast (Substituir nós por 0)")
        self.chk_force_broadcast.setChecked(False)
        self.chk_force_broadcast.setToolTip(
            "Desativado por padrão: executa os comandos exatamente com os nós informados no script (ex: L 1, L 2, L 0).\n"
            "Quando ativado: força todos os comandos de atuação para o Node 0 (Broadcast)."
        )
        config_layout.addWidget(self.chk_force_broadcast)

        self.chk_stop_on_error = QCheckBox("⛔ Parar ao Detectar Erro")
        self.chk_stop_on_error.setChecked(False)
        config_layout.addWidget(self.chk_stop_on_error)

        config_layout.addStretch()
        main_layout.addWidget(config_card)

        # ==========================================
        # 3. SPLIT WORKSPACE: EDITOR & LIVE MONITOR
        # ==========================================
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(8)

        # --- LEFT: SCRIPT EDITOR ---
        editor_widget = QWidget()
        editor_layout = QVBoxLayout(editor_widget)
        editor_layout.setContentsMargins(0, 0, 0, 0)
        editor_layout.setSpacing(8)

        editor_top = QHBoxLayout()
        editor_top.addWidget(QLabel("📝 Sequência de Comandos:"))
        editor_top.addStretch()

        self.combo_presets = QComboBox()
        self.combo_presets.addItem("⚡ Carregar Preset Pronto...")
        for name in AUTOMATION_PRESETS.keys():
            self.combo_presets.addItem(name)
        self.combo_presets.currentIndexChanged.connect(self._load_preset)
        editor_top.addWidget(self.combo_presets)

        btn_preview = QPushButton("👁️ Prévia")
        btn_preview.setToolTip("Compila e exibe uma prévia dos comandos gerados por macros e loops")
        btn_preview.clicked.connect(self._preview_expansion)
        editor_top.addWidget(btn_preview)

        btn_open = QPushButton("📂 Abrir")
        btn_open.clicked.connect(self._open_script_file)
        editor_top.addWidget(btn_open)

        btn_save = QPushButton("💾 Salvar")
        btn_save.clicked.connect(self._save_script_file)
        editor_top.addWidget(btn_save)

        btn_clear = QPushButton("🗑️ Limpar")
        btn_clear.clicked.connect(self._clear_editor)
        editor_top.addWidget(btn_clear)

        editor_layout.addLayout(editor_top)

        # PlainTextEdit script editor
        self.txt_script = QPlainTextEdit()
        font = QFont("Consolas", 11)
        font.setStyleHint(QFont.StyleHint.Monospace)
        self.txt_script.setFont(font)
        self.txt_script.setPlaceholderText(
            "# Escreva comandos ASCII para o Teensy (ex: M 0 C 800, MS 0 45 -45 5, L 0 1 4095)\n"
            "# Utilize WAIT <ms> para pausas temporizadas.\n"
            "# Exemplo:\n"
            "E 0 1\n"
            "WAIT 500\n"
            "M 0 C 400\n"
            "WAIT 1000\n"
            "M 0 C -400\n"
            "WAIT 1000\n"
        )
        self.txt_script.setPlainText(PRESET_SWEEP_CAZ)
        editor_layout.addWidget(self.txt_script, 1)

        # Quick Insert Tool Buttons (Hardware & Movement)
        insert_row_hw = QHBoxLayout()
        insert_row_hw.setSpacing(6)
        lbl_hw = QLabel("Hardware:")
        lbl_hw.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700;")
        insert_row_hw.addWidget(lbl_hw)

        quick_hw_buttons = [
            ("+ Move C", "M 0 C 400\nWAIT 1000\n"),
            ("+ Move A", "M 0 A 400\nWAIT 1000\n"),
            ("+ Move Z", "M 0 Z 500\nWAIT 1000\n"),
            ("+ Sync C+A+Z", "MS 0 45.0 -30.0 10.00\nWAIT 1500\n"),
            ("+ Homing", "H 0 C\nWAIT 1500\n"),
            ("+ Delay", "WAIT 500\n"),
            ("+ Blackout", "BLACKOUT 0\nWAIT 100\n"),
        ]
        for label, snippet in quick_hw_buttons:
            btn = QPushButton(label)
            btn.setStyleSheet("font-size: 11px; padding: 3px 6px;")
            btn.clicked.connect(lambda checked, s=snippet: self._insert_snippet(s))
            insert_row_hw.addWidget(btn)
        insert_row_hw.addStretch()
        editor_layout.addLayout(insert_row_hw)

        # Quick Insert Tool Buttons (Macros, Loops & Lasers)
        insert_row_macros = QHBoxLayout()
        insert_row_macros.setSpacing(6)
        lbl_macro = QLabel("Animação:")
        lbl_macro.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: 700;")
        insert_row_macros.addWidget(lbl_macro)

        quick_macro_buttons = [
            ("+ Loop FOR", "FOR P = 0 TO 300 STEP 15\n  L 1 2 {P}\n  L 4 2 {300 - P}\n  WAIT 15\nEND\n"),
            ("+ REPEAT", "REPEAT 10\n  L 1 2 300\n  WAIT 50\n  L 1 2 0\n  WAIT 50\nEND\n"),
            ("+ FADE", "FADE 1 2 0 300 10 5\n"),
            ("+ SYNC_FADE", "SYNC_FADE 1 2 4 2 0 300 10 3\n"),
            ("+ STROBE", "STROBE 1 2 300 20 20 50\n"),
            ("+ STROBE_CROSS", "STROBE_CROSS 1 2 4 2 300 20 100\n"),
        ]
        for label, snippet in quick_macro_buttons:
            btn = QPushButton(label)
            btn.setStyleSheet("font-size: 11px; padding: 3px 6px; color: #38bdf8;")
            btn.clicked.connect(lambda checked, s=snippet: self._insert_snippet(s))
            insert_row_macros.addWidget(btn)
        insert_row_macros.addStretch()
        editor_layout.addLayout(insert_row_macros)

        splitter.addWidget(editor_widget)

        # --- RIGHT: METRICS & LIVE MONITOR ---
        monitor_widget = QWidget()
        monitor_layout = QVBoxLayout(monitor_widget)
        monitor_layout.setContentsMargins(0, 0, 0, 0)
        monitor_layout.setSpacing(10)

        # Metrics Cards Grid
        metrics_grid = QGridLayout()
        metrics_grid.setSpacing(8)

        # Card: Current Loop
        card_loop = QFrame()
        card_loop.setProperty("class", "metric-card")
        cl_box = QVBoxLayout(card_loop)
        cl_box.setContentsMargins(10, 8, 10, 8)
        lbl_cl_title = QLabel("LOOP ATUAL")
        lbl_cl_title.setProperty("class", "card-title")
        self.lbl_metric_loop = QLabel("0 / ∞")
        self.lbl_metric_loop.setProperty("class", "metric-value")
        self.lbl_metric_loop.setStyleSheet("color: #38bdf8; font-size: 20px;")
        cl_box.addWidget(lbl_cl_title)
        cl_box.addWidget(self.lbl_metric_loop)
        metrics_grid.addWidget(card_loop, 0, 0)

        # Card: Current Step
        card_step = QFrame()
        card_step.setProperty("class", "metric-card")
        cs_box = QVBoxLayout(card_step)
        cs_box.setContentsMargins(10, 8, 10, 8)
        lbl_cs_title = QLabel("PASSO ATUAL")
        lbl_cs_title.setProperty("class", "card-title")
        self.lbl_metric_step = QLabel("0 / 0")
        self.lbl_metric_step.setProperty("class", "metric-value")
        self.lbl_metric_step.setStyleSheet("color: #34d399; font-size: 20px;")
        cs_box.addWidget(lbl_cs_title)
        cs_box.addWidget(self.lbl_metric_step)
        metrics_grid.addWidget(card_step, 0, 1)

        # Card: Commands Sent
        card_cmds = QFrame()
        card_cmds.setProperty("class", "metric-card")
        cc_box = QVBoxLayout(card_cmds)
        cc_box.setContentsMargins(10, 8, 10, 8)
        lbl_cc_title = QLabel("COMANDOS TX")
        lbl_cc_title.setProperty("class", "card-title")
        self.lbl_metric_cmds = QLabel("0")
        self.lbl_metric_cmds.setProperty("class", "metric-value")
        self.lbl_metric_cmds.setStyleSheet("color: #f8fafc; font-size: 20px;")
        cc_box.addWidget(lbl_cc_title)
        cc_box.addWidget(self.lbl_metric_cmds)
        metrics_grid.addWidget(card_cmds, 1, 0)

        # Card: Errors Detected
        card_errs = QFrame()
        card_errs.setProperty("class", "metric-card")
        ce_box = QVBoxLayout(card_errs)
        ce_box.setContentsMargins(10, 8, 10, 8)
        lbl_ce_title = QLabel("FALHAS / ERROS")
        lbl_ce_title.setProperty("class", "card-title")
        self.lbl_metric_errs = QLabel("0")
        self.lbl_metric_errs.setProperty("class", "metric-value")
        self.lbl_metric_errs.setStyleSheet("color: #f87171; font-size: 20px;")
        ce_box.addWidget(lbl_ce_title)
        ce_box.addWidget(self.lbl_metric_errs)
        metrics_grid.addWidget(card_errs, 1, 1)

        monitor_layout.addLayout(metrics_grid)

        # Current Command banner & Progress Bar
        active_card = QFrame()
        active_card.setProperty("class", "card")
        active_layout = QVBoxLayout(active_card)
        active_layout.setContentsMargins(12, 10, 12, 10)
        active_layout.setSpacing(6)

        lbl_active_hdr = QLabel("Comando em Execução:")
        lbl_active_hdr.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700;")
        self.lbl_active_command = QLabel("Aguardando início...")
        self.lbl_active_command.setStyleSheet("color: #38bdf8; font-family: 'Consolas', monospace; font-size: 13px; font-weight: 800;")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        active_layout.addWidget(lbl_active_hdr)
        active_layout.addWidget(self.lbl_active_command)
        active_layout.addWidget(self.progress_bar)

        monitor_layout.addWidget(active_card)

        # Log Output Header
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("📡 Log de Execução da Automação:"))
        log_header.addStretch()

        self.chk_autoscroll = QCheckBox("Auto-scroll")
        self.chk_autoscroll.setChecked(True)
        log_header.addWidget(self.chk_autoscroll)

        btn_clear_log = QPushButton("🗑️ Limpar Log")
        btn_clear_log.clicked.connect(self._clear_log)
        log_header.addWidget(btn_clear_log)

        monitor_layout.addLayout(log_header)

        # Execution Log Text Console
        self.console_log = QPlainTextEdit()
        self.console_log.setProperty("class", "console")
        self.console_log.setReadOnly(True)
        self.console_log.setMaximumBlockCount(3000)
        monitor_layout.addWidget(self.console_log, 1)

        splitter.addWidget(monitor_widget)
        splitter.setSizes([540, 480])

        main_layout.addWidget(splitter, 1)

    def _insert_snippet(self, text: str):
        self.txt_script.insertPlainText(text)
        self.txt_script.ensureCursorVisible()

    def _clear_editor(self):
        self.txt_script.clear()

    def _clear_log(self):
        self.console_log.clear()

    def _on_mode_changed(self, index: int):
        # 0 = Infinite, 1 = Count, 2 = Once
        is_count = index == 1
        self.lbl_target_loops.setVisible(is_count)
        self.spin_target_loops.setVisible(is_count)

    def _load_preset(self, index: int):
        if index <= 0:
            return
        preset_name = self.combo_presets.currentText()
        if preset_name in AUTOMATION_PRESETS:
            self.txt_script.setPlainText(AUTOMATION_PRESETS[preset_name])
        self.combo_presets.setCurrentIndex(0)

    def _open_script_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Abrir Script de Automação", "", "Arquivos de Script (*.txt *.seq);;Todos os Arquivos (*)"
        )
        if filename:
            try:
                with open(filename, "r", encoding="utf-8") as f:
                    self.txt_script.setPlainText(f.read())
            except Exception as e:
                QMessageBox.critical(self, "Erro", f"Não foi possível carregar o arquivo: {e}")

    def _save_script_file(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, "Salvar Script de Automação", "automacao_teste.txt", "Arquivos de Script (*.txt *.seq);;Todos os Arquivos (*)"
        )
        if filename:
            try:
                with open(filename, "w", encoding="utf-8") as f:
                    f.write(self.txt_script.toPlainText())
                QMessageBox.information(self, "Sucesso", "Script salvo com sucesso!")
            except Exception as e:
                QMessageBox.critical(self, "Erro", f"Não foi possível salvar o arquivo: {e}")

    def _preview_expansion(self):
        script_text = self.txt_script.toPlainText()
        if not script_text.strip():
            QMessageBox.information(self, "Prévia da Automação", "O editor de comandos está vazio.")
            return

        try:
            expanded = expand_automation_script(script_text)
            lines = [l for l in expanded.splitlines() if l.strip() and not l.strip().startswith(("#", "//"))]
            count = len(lines)
            preview_sample = "\n".join(lines[:12])
            if count > 12:
                preview_sample += f"\n... (+{count - 12} passos adicionais)"

            QMessageBox.information(
                self,
                "Prévia da Expansão Procedural",
                f"✅ Script válido e compilado com sucesso!\n\n"
                f"Total de passos executáveis gerados: {count}\n\n"
                f"Amostra dos passos compilados:\n"
                f"----------------------------------------\n"
                f"{preview_sample}",
            )
        except ScriptExpansionError as e:
            QMessageBox.critical(
                self,
                "Erro de Sintaxe no Script",
                f"Erro na interpretação de macros ou loops:\n\n{e}",
            )
        except Exception as e:
            QMessageBox.critical(
                self,
                "Erro",
                f"Falha inesperada ao processar o script:\n\n{e}",
            )


    def _start_automation(self):
        if not self.comm.is_connected:
            QMessageBox.warning(self, "Aviso", "Conecte-se ao hardware (Teensy ou Simulador) antes de iniciar a automação.")
            return

        script_text = self.txt_script.toPlainText()
        mode_idx = self.combo_mode.currentIndex()
        mode_str = "INFINITE" if mode_idx == 0 else ("COUNT" if mode_idx == 1 else "ONCE")
        target_loops = self.spin_target_loops.value()
        default_delay = self.spin_default_delay.value()
        force_bcast = self.chk_force_broadcast.isChecked()
        stop_on_err = self.chk_stop_on_error.isChecked()

        # Get default node from comm
        default_node = 1
        if hasattr(self.comm.active_client, "node_id"):
            default_node = getattr(self.comm.active_client, "node_id", 1)

        try:
            steps = parse_script(
                script_text,
                default_delay_ms=default_delay,
                force_broadcast=force_bcast,
                default_node=default_node,
            )
        except ScriptExpansionError as err:
            QMessageBox.critical(
                self,
                "Erro de Sintaxe no Script",
                f"Erro na interpretação de macros ou loops:\n\n{err}",
            )
            return
        except Exception as ex:
            QMessageBox.critical(
                self,
                "Erro no Script",
                f"Não foi possível interpretar o script:\n\n{ex}",
            )
            return

        exec_count = sum(1 for s in steps if s.step_type in ("COMMAND", "DELAY"))
        if exec_count == 0:
            QMessageBox.warning(self, "Aviso", "O script não contém nenhum comando executável.")
            return

        # Configure worker and start
        self.worker.configure(
            steps=steps,
            loop_mode=mode_str,
            target_loops=target_loops,
            default_delay_ms=default_delay,
            stop_on_error=stop_on_err,
        )

        self.start_time = time.time()
        self.elapsed_timer.start()

        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_pause.setText("⏸️ Pausar")
        self.btn_stop.setEnabled(True)

        self.worker.start()

    def _toggle_pause(self):
        if self.worker.isRunning():
            if self.worker._pause_requested:
                self.worker.request_resume()
                self.btn_pause.setText("⏸️ Pausar")
            else:
                self.worker.request_pause()
                self.btn_pause.setText("▶️ Retomar")

    def _stop_automation(self):
        self.worker.request_stop()
        self.btn_stop.setEnabled(False)

    def _on_state_changed(self, state: str):
        if state == "RUNNING":
            loop_txt = f"Loop #{self.worker.current_loop}"
            self.lbl_status_badge.setText(f"EXECUTANDO ({loop_txt})")
            self.lbl_status_badge.setProperty("class", "badge badge-green")
        elif state == "PAUSED":
            self.lbl_status_badge.setText("PAUSADO")
            self.lbl_status_badge.setProperty("class", "badge badge-yellow")
        elif state == "ERROR":
            self.lbl_status_badge.setText("ERRO DETECTADO")
            self.lbl_status_badge.setProperty("class", "badge badge-red")
        else:
            self.lbl_status_badge.setText("PARADO")
            self.lbl_status_badge.setProperty("class", "badge badge-gray")

        self.lbl_status_badge.style().unpolish(self.lbl_status_badge)
        self.lbl_status_badge.style().polish(self.lbl_status_badge)

    def _on_step_started(self, step_idx: int, total_steps: int, command: str, loop_num: int):
        mode_idx = self.combo_mode.currentIndex()
        if mode_idx == 0:
            self.lbl_metric_loop.setText(f"{loop_num} / ∞")
        elif mode_idx == 1:
            self.lbl_metric_loop.setText(f"{loop_num} / {self.spin_target_loops.value()}")
        else:
            self.lbl_metric_loop.setText(f"{loop_num} / 1")

        self.lbl_metric_step.setText(f"{step_idx} / {total_steps}")
        self.lbl_metric_cmds.setText(str(self.worker.total_commands_sent))
        self.lbl_metric_errs.setText(str(self.worker.error_count))

        self.lbl_active_command.setText(f"[{step_idx}/{total_steps}] {command}")
        pct = int((step_idx / max(1, total_steps)) * 100)
        self.progress_bar.setValue(pct)

    def _on_step_completed(self, step_idx: int, success: bool, message: str):
        self.lbl_metric_cmds.setText(str(self.worker.total_commands_sent))
        self.lbl_metric_errs.setText(str(self.worker.error_count))

    def _on_loop_completed(self, loop_num: int):
        pass

    def _on_finished(self, total_loops: int, total_cmds: int, errors: int):
        self.elapsed_timer.stop()
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_pause.setText("⏸️ Pausar")
        self.btn_stop.setEnabled(False)
        self.lbl_active_command.setText("Automação finalizada.")
        self.progress_bar.setValue(100)

    def _on_progress_updated(self, current: int, total: int):
        pct = int((current / max(1, total)) * 100)
        self.progress_bar.setValue(pct)

    def _update_elapsed_time(self):
        if self.start_time > 0:
            sec = int(time.time() - self.start_time)
            m, s = divmod(sec, 60)
            h, m = divmod(m, 60)
            self.lbl_elapsed.setText(f"⏱️ {h:02d}:{m:02d}:{s:02d}")

    def _append_log(self, timestamp: str, direction: str, message: str, tag: str):
        if tag == "highlight":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <b style="color:#c084fc;">{message}</b>'
        elif tag == "tx":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <b style="color:#38bdf8;">&gt;&gt; TX:</b> <span style="color:#f8fafc;">{message}</span>'
        elif tag == "wait":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <b style="color:#fbbf24;">⏳ WAIT:</b> <span style="color:#fde68a;">{message}</span>'
        elif tag == "error":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <b style="color:#f87171;">⛔ ERRO:</b> <span style="color:#fca5a5;">{message}</span>'
        elif tag == "success":
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <b style="color:#34d399;">✓ OK:</b> <span style="color:#a7f3d0;">{message}</span>'
        else:
            line = f'<span style="color:#64748b;">[{timestamp}]</span> <span style="color:#94a3b8;">[SYS] {message}</span>'

        self.console_log.appendHtml(line)
        if self.chk_autoscroll.isChecked():
            self.console_log.moveCursor(QTextCursor.MoveOperation.End)
