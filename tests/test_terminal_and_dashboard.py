import unittest
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QEvent
from PyQt6.QtGui import QKeyEvent

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState, HardwareTelemetry
from python_app.core.simulator import SimulatorClient
from python_app.ui.views.terminal_view import TerminalView
from python_app.ui.views.dashboard_view import DashboardView
from python_app.ui.widgets.jog_pad import JogPad

class TestTerminalAndDashboard(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)
        self.sim = SimulatorClient(self.state)
        self.comm.active_client = self.sim

    def test_terminal_command_history_up_down(self):
        term = TerminalView(self.comm, self.state)
        
        # Send 3 commands
        cmds = ["STATUS", "MOVE C 400", "MOVE A -200"]
        for c in cmds:
            term.txt_cmd.setText(c)
            term._send_command()
            
        self.assertEqual(term.history, cmds)
        self.assertEqual(term.txt_cmd.text(), "")
        
        # Press UP arrow: should recall last command "MOVE A -200"
        up_event = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Up, Qt.KeyboardModifier.NoModifier)
        term.eventFilter(term.txt_cmd, up_event)
        self.assertEqual(term.txt_cmd.text(), "MOVE A -200")
        
        # Press UP arrow again: should recall "MOVE C 400"
        term.eventFilter(term.txt_cmd, up_event)
        self.assertEqual(term.txt_cmd.text(), "MOVE C 400")
        
        # Press UP arrow again: should recall "STATUS"
        term.eventFilter(term.txt_cmd, up_event)
        self.assertEqual(term.txt_cmd.text(), "STATUS")
        
        # Press DOWN arrow: should recall "MOVE C 400"
        down_event = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Down, Qt.KeyboardModifier.NoModifier)
        term.eventFilter(term.txt_cmd, down_event)
        self.assertEqual(term.txt_cmd.text(), "MOVE C 400")
        
        # Press DOWN arrow: should recall "MOVE A -200"
        term.eventFilter(term.txt_cmd, down_event)
        self.assertEqual(term.txt_cmd.text(), "MOVE A -200")
        
        # Press DOWN arrow again: past newest, line should be cleared
        term.eventFilter(term.txt_cmd, down_event)
        self.assertEqual(term.txt_cmd.text(), "")

    def test_dashboard_angular_progress_bars(self):
        dash = DashboardView(self.comm, self.state)
        self.state.parameters.limit_min_deg_c = 0.0
        self.state.parameters.limit_max_deg_c = 200.0
        self.state.parameters.limit_min_deg_a = 0.0
        self.state.parameters.limit_max_deg_a = 100.0
        self.state.parameters.max_passos_z = 160000
        
        t = HardwareTelemetry()
        t.pos_c_valid = True
        t.pos_c_deg = 100.0 # 50%
        t.pos_a_valid = True
        t.pos_a_deg = 25.0 # 25%
        t.max_z_steps = 160000
        t.pos_z_steps = 80000 # 50%
        
        dash.show()
        dash.update_telemetry(t)
        self.app.processEvents()
        
        self.assertFalse(dash.card_c.progress_bar.isHidden())
        self.assertEqual(dash.card_c.progress_bar.value(), 50)
        self.assertFalse(dash.card_a.progress_bar.isHidden())
        self.assertEqual(dash.card_a.progress_bar.value(), 25)
        self.assertFalse(dash.card_z.progress_bar.isHidden())
        self.assertEqual(dash.card_z.progress_bar.value(), 50)
        dash.close()

    def test_jog_pad_immediate_direction_reversal(self):
        self.state.telemetry.pos_c_deg = 50.0 # Middle of range, not hitting limits
        jog = JogPad(self.state)
        emitted_moves = []
        jog.jog_requested.connect(lambda axis, steps, force: emitted_moves.append((axis, steps)))
        
        # Jog + then immediately jog -
        jog._on_jog_ca('C', 1)
        self.assertEqual(len(emitted_moves), 1)
        self.assertGreater(emitted_moves[0][1], 0)
        
        # Reversal must NOT be throttled even if immediately called
        jog._on_jog_ca('C', -1)
        self.assertEqual(len(emitted_moves), 2)
        self.assertLess(emitted_moves[1][1], 0)

    def test_jog_z_step_pills_include_50_100_200(self):
        jog = JogPad(self.state)
        self.assertIn(50.0, jog.z_pills)
        self.assertIn(100.0, jog.z_pills)
        self.assertIn(200.0, jog.z_pills)

    def test_jog_z_clamps_steps_to_travel_limits(self):
        self.state.parameters.max_passos_z = 10000
        self.state.telemetry.pos_z_steps = 9500
        jog = JogPad(self.state)
        jog.current_step_z_mm = 50.0  # 50mm = 5000 steps with 16T / 16 usteps
        emitted_moves = []
        jog.jog_requested.connect(lambda axis, steps, force: emitted_moves.append((axis, steps)))
        
        # Jog +: should be clamped to 500 steps instead of requesting 5000
        jog._on_jog_z(1)
        self.assertEqual(len(emitted_moves), 1)
        self.assertEqual(emitted_moves[0], ('Z', 500))

        # Position is 300 steps (only 300 steps until 0 bottom limit)
        self.state.telemetry.pos_z_steps = 300
        jog._last_jog_z_key = None
        jog._on_jog_z(-1)
        self.assertEqual(len(emitted_moves), 2)
        self.assertEqual(emitted_moves[1], ('Z', -300))

if __name__ == '__main__':
    unittest.main()
