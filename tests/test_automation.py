import time
import unittest
from PyQt6.QtWidgets import QApplication

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.test_automation import AutomationWorker, parse_script, AUTOMATION_PRESETS


class AutomationWorkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)
        # Use simulator backend for clean test execution
        self.comm.connect_simulator()
        self.worker = AutomationWorker(self.comm)

    def tearDown(self):
        if self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(1000)
        self.comm.disconnect_all()

    def test_all_presets_parse_valid_executable_steps(self):
        for preset_name, script_text in AUTOMATION_PRESETS.items():
            steps = parse_script(script_text, default_delay_ms=200, force_broadcast=True)
            exec_steps = [s for s in steps if s.step_type in ("COMMAND", "DELAY")]
            self.assertGreater(
                len(exec_steps),
                0,
                f"Preset '{preset_name}' deve ter passos executáveis",
            )
            # Verify all command steps have node 0 when force_broadcast is true
            for step in exec_steps:
                if step.step_type == "COMMAND":
                    parts = step.command.split()
                    if parts[0] in ("M", "MF", "MS", "MSF", "H", "E", "S", "L", "F"):
                        self.assertEqual(
                            parts[1],
                            "0",
                            f"Comando {step.command} deveria ter node 0 em broadcast",
                        )

    def test_worker_runs_count_loops_and_finishes(self):
        script = """
        E 0 1
        WAIT 30
        M 0 C 100
        WAIT 30
        """
        steps = parse_script(script, default_delay_ms=30)
        self.worker.configure(
            steps=steps,
            loop_mode="COUNT",
            target_loops=2,
            default_delay_ms=30,
        )

        completed_loops = []
        self.worker.loop_completed.connect(completed_loops.append)

        self.worker.start()
        self.worker.wait(3000)
        self.app.processEvents()

        self.assertEqual(self.worker.current_loop, 2)
        self.assertEqual(len(completed_loops), 2)
        self.assertGreaterEqual(self.worker.total_commands_sent, 4)

    def test_worker_can_be_stopped_early_in_infinite_mode(self):
        script = """
        M 0 C 100
        WAIT 50
        """
        steps = parse_script(script, default_delay_ms=50)
        self.worker.configure(
            steps=steps,
            loop_mode="INFINITE",
            default_delay_ms=50,
        )

        self.worker.start()
        # Let it run briefly
        time.sleep(0.15)
        self.worker.request_stop()
        self.worker.wait(1000)

        self.assertFalse(self.worker.isRunning())
        self.assertGreater(self.worker.current_loop, 0)

    def test_for_loop_expansion_and_variable_math(self):
        script = """
        FOR P = 0 TO 100 STEP 50
          L 1 2 {P}
          L 4 2 {100 - P}
          WAIT 10
        END
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertEqual(
            cmds,
            [
                "L 1 2 0", "L 4 2 100",
                "L 1 2 50", "L 4 2 50",
                "L 1 2 100", "L 4 2 0",
            ],
        )

    def test_repeat_and_nested_loops(self):
        script = """
        REPEAT 2
          FOR I = 1 TO 2
            L 1 2 {I * 100}
            WAIT 5
          END
        END
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertEqual(cmds, ["L 1 2 100", "L 1 2 200", "L 1 2 100", "L 1 2 200"])

    def test_if_else_branching(self):
        script = """
        FOR I = 1 TO 4
          IF {I} % 2 == 0
            L 1 2 300
          ELSE
            L 4 2 300
          ENDIF
        END
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertEqual(cmds, ["L 4 2 300", "L 1 2 300", "L 4 2 300", "L 1 2 300"])

    def test_user_defined_function_def_and_call(self):
        script = """
        DEF FLASH_PAIR pwr
          L 1 2 {pwr}
          L 4 2 {pwr}
          WAIT 20
          L 1 2 0
          L 4 2 0
          WAIT 20
        ENDDEF

        FLASH_PAIR 250
        CALL FLASH_PAIR 300
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertEqual(
            cmds,
            [
                "L 1 2 250", "L 4 2 250", "L 1 2 0", "L 4 2 0",
                "L 1 2 300", "L 4 2 300", "L 1 2 0", "L 4 2 0",
            ],
        )

    def test_fade_and_sync_fade_macros(self):
        script = """
        FADE 1 2 0 40 10 20
        SYNC_FADE 1 2 4 2 100 60 15 20
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        # FADE: 0, 20, 40
        self.assertIn("L 1 2 0", cmds)
        self.assertIn("L 1 2 20", cmds)
        self.assertIn("L 1 2 40", cmds)
        # SYNC_FADE: 100, 80, 60 on both nodes 1 and 4
        self.assertIn("L 4 2 100", cmds)
        self.assertIn("L 4 2 80", cmds)
        self.assertIn("L 4 2 60", cmds)

    def test_strobe_and_strobe_cross_macros(self):
        script = """
        STROBE 1 2 300 20 20 2
        STROBE_CROSS 1 2 4 2 300 25 1
        BLACKOUT 1
        """
        steps = parse_script(script)
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertEqual(
            cmds,
            [
                "L 1 2 300", "L 1 2 0",
                "L 1 2 300", "L 1 2 0",
                "L 1 2 300", "L 4 2 0",
                "L 1 2 0", "L 4 2 300",
                "L 1 1 0", "L 1 2 0",
            ],
        )

    def test_unclosed_block_raises_expansion_error(self):
        from python_app.core.test_automation import ScriptExpansionError
        script = """
        FOR P = 1 TO 10
          L 1 2 100
        """
        with self.assertRaises(ScriptExpansionError):
            parse_script(script)


if __name__ == "__main__":
    unittest.main()

