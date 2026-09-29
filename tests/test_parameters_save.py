import unittest

from PyQt6.QtWidgets import QApplication

from python_app.core.comm_manager import CommManager
from python_app.core.protocol_defs import tmc_quantize_ma, tmc_ma_to_cs, tmc_cs_to_ma
from python_app.core.serial_client import SerialClient
from python_app.core.state_model import DeviceState
from python_app.ui.views.parameters_view import ParametersView


class RecordingSerialClient(SerialClient):
    """Registra os comandos; ecoa atualizações de parâmetros como o firmware faria."""

    def __init__(self, state):
        super().__init__(state)
        self.commands = []

    def send_raw(self, cmd):
        self.commands.append(cmd)
        return True


class CurrentQuantizationTests(unittest.TestCase):
    def test_matches_firmware_steps(self):
        self.assertEqual(tmc_ma_to_cs(559), 8)
        self.assertEqual(tmc_quantize_ma(559), 539)   # antes o dump "mudava" 559 -> 538
        self.assertEqual(tmc_quantize_ma(897), 898)
        self.assertEqual(tmc_quantize_ma(tmc_quantize_ma(700)), tmc_quantize_ma(700))
        self.assertEqual(tmc_cs_to_ma(31), 1915)


class ParametersSaveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)
        self.client = RecordingSerialClient(self.state)
        self.client.is_connected = True
        self.comm.active_client = self.client
        self.view = ParametersView(self.comm, self.state)
        # Tela sincronizada com o nó (como após o CONFIG DUMP da conexão)
        self.state.parameters_updated.emit(self.state.parameters)

    def tearDown(self):
        # Destrói o widget enquanto a QApplication existe (senão o GC do Python o destrói
        # depois dela no fim do processo -> access violation no teardown do Qt)
        self.view.deleteLater()
        QApplication.processEvents()

    def _run_commands(self):
        desired = self.view._collect_desired()
        diffs = self.view._diff_against_device(desired, self.state.parameters)
        cmds = self.view._commands_for(self.client, desired, diffs)
        for _, func in cmds:
            func()
        return diffs

    def test_nothing_changed_sends_nothing(self):
        diffs = self._run_commands()
        self.assertEqual(diffs, [])
        self.assertEqual(self.client.commands, [])

    def test_only_changed_current_is_sent_and_no_driver_apply(self):
        self.view.spin_tmc_irun_a.setValue(1200)
        self._run_commands()
        self.assertEqual(len(self.client.commands), 1)
        self.assertTrue(self.client.commands[0].startswith("DRIVER UART CURRENT A "))
        self.assertIn(f" {tmc_quantize_ma(1200)} ", self.client.commands[0] + " ")
        self.assertNotIn("DRIVER APPLY", self.client.commands)

    def test_values_are_captured_before_device_echo_reloads_the_form(self):
        # Cenário do bug: polia + corrente alteradas; a resposta da polia recarrega a
        # tela com os valores antigos antes de a corrente ser enviada.
        self.view.spin_pulley_z.setValue(16)
        self.view.spin_tmc_irun_c.setValue(1200)
        desired = self.view._collect_desired()
        diffs = self.view._diff_against_device(desired, self.state.parameters)
        cmds = self.view._commands_for(self.client, desired, diffs)
        for _, func in cmds:
            func()
            self.state.parameters_updated.emit(self.state.parameters)  # eco do nó
        current_cmds = [c for c in self.client.commands if c.startswith("DRIVER UART CURRENT C")]
        self.assertEqual(len(current_cmds), 1)
        self.assertIn(str(tmc_quantize_ma(1200)), current_cmds[0])

    def test_address_change_triggers_single_driver_apply(self):
        self.view.spin_tmc_addr_z.setValue(3)
        self._run_commands()
        self.assertEqual(self.client.commands, ["DRIVER UART ADDR Z 3", "DRIVER APPLY"])

    def test_confirmation_detects_value_not_accepted(self):
        self.view.spin_tmc_irun_c.setValue(1200)
        desired = self.view._collect_desired()
        diffs = self.view._diff_against_device(desired, self.state.parameters)
        # Após o CONFIG DUMP o nó ainda reporta a corrente antiga -> continua na lista
        still = [label for label, _, _ in self.view._diff_against_device(desired, self.state.parameters)]
        self.assertIn("Corrente TMC C", still)
        # Nó confirmou a nova corrente
        self.state.parameters.tmc_irun_ma[0] = tmc_quantize_ma(1200)
        still = [label for label, _, _ in self.view._diff_against_device(desired, self.state.parameters)]
        self.assertNotIn("Corrente TMC C", still)
        self.assertEqual(len(diffs), 1)

    def test_config_dump_parses_stealth_and_signals_completion(self):
        done = []
        self.state.config_dump_completed.connect(lambda: done.append(True))
        for line in ("=== CONFIG DUMP ===", "CONFIG STEALTH_MAX C=200.00 A=150.00 Z=40.00", "==================="):
            self.client._parse_response_line(line)
        self.assertEqual(self.state.parameters.tmc_stealth_max, [200.0, 150.0, 40.0])
        self.assertEqual(done, [True])

    def test_motion_engine_change_is_sent(self):
        self.view.combo_motion_engine.setCurrentText("LEGACY")
        self.view.chk_lookahead.setChecked(False)
        self.view.spin_jerk[2].setValue(25.0)
        self._run_commands()
        self.assertIn("MOTION ENGINE LEGACY", self.client.commands)
        self.assertIn("MOTION LOOKAHEAD OFF", self.client.commands)
        self.assertIn("MOTION JERK Z 25.00", self.client.commands)

    def test_structured_pos_line(self):
        self.client._parse_response_line("@POS C=-45.25 A=nan Z=1200 ZMAX=38400 HOMED=101 MOVING=1")
        t = self.state.telemetry
        self.assertAlmostEqual(t.pos_c_deg, -45.25)
        self.assertTrue(t.pos_c_valid)
        self.assertFalse(t.pos_a_valid)
        self.assertEqual((t.pos_z_steps, t.max_z_steps), (1200, 38400))
        self.assertEqual(t.homed, [True, False, True])
        self.assertTrue(t.in_motion)

    def test_pos_line_carries_z_lock_and_alarm(self):
        self.state.update_telemetry(z_bloqueado=True, alarme_z_ativo=False)
        self.client._parse_response_line("@POS C=0.00 A=0.00 Z=0 ZMAX=38400 HOMED=001 MOVING=0 ZLOCK=0 ALARM=1")
        t = self.state.telemetry
        self.assertFalse(t.z_bloqueado)
        self.assertTrue(t.alarme_z_ativo)

    def test_home_z_messages_update_lock_state(self):
        # Firmware antigo: sem ZLOCK no @POS, o estado vem das mensagens do HOME/alarme
        self.client._parse_response_line("Home Z falhou: ESP_ERR_TIMEOUT")
        self.assertTrue(self.state.telemetry.z_bloqueado)
        self.client._parse_response_line("Home Z finalizado.")
        self.assertFalse(self.state.telemetry.z_bloqueado)
        self.client._parse_response_line(
            "E (1234) blue_mechanic: ALARME: fim de curso Z acionado inesperadamente. Movimentos interrompidos")
        self.assertTrue(self.state.telemetry.z_bloqueado)
        self.client._parse_response_line("ALARME Z DESATIVADO. Fim de curso ignorado.")
        self.assertEqual((self.state.telemetry.z_bloqueado, self.state.telemetry.alarme_z_ativo), (False, False))
        self.client._parse_response_line("ALARME Z ATIVADO. Fim de curso em vigia.")
        self.assertTrue(self.state.telemetry.alarme_z_ativo)

    def test_config_motion_line(self):
        self.client._parse_response_line("CONFIG MOTION ENGINE=LEGACY LOOKAHEAD=0 JERK C=20.00 A=12.50 Z=8.00")
        p = self.state.parameters
        self.assertEqual((p.motion_engine, p.lookahead, p.jerk), ("LEGACY", False, [20.0, 12.5, 8.0]))


if __name__ == "__main__":
    unittest.main()
