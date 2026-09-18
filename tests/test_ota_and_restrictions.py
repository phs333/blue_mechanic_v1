import os
import tempfile
import time
import unittest
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QMouseEvent

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.protocol_defs import CanOpcode, CanEvent
from python_app.core.can_client import CanClient
from python_app.core.simulator import SimulatorClient
from python_app.core.ota_manager import OtaWorker
from python_app.ui.views.teensy_dashboard_view import TeensyDashboardView, NodeCardWidget
from python_app.ui.views.ota_view import OtaView
from python_app.ui.main_window import MainWindow


class OtaAndRestrictionsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)

    def test_ota_opcodes_and_events_definitions(self):
        self.assertEqual(CanOpcode.OTA_START, 0x40)
        self.assertEqual(CanOpcode.OTA_DATA, 0x41)
        self.assertEqual(CanOpcode.OTA_END, 0x42)
        self.assertEqual(CanOpcode.OTA_ABORT, 0x43)

        self.assertEqual(CanEvent.OTA_READY, 0x90)
        self.assertEqual(CanEvent.OTA_PROGRESS, 0x91)
        self.assertEqual(CanEvent.OTA_DONE, 0x92)
        self.assertEqual(CanEvent.OTA_ERROR, 0x93)

    def test_state_model_ota_signals(self):
        ready_events = []
        progress_events = []
        done_events = []
        error_events = []

        self.state.ota_ready.connect(lambda nid, err: ready_events.append((nid, err)))
        self.state.ota_progress.connect(lambda nid, pct: progress_events.append((nid, pct)))
        self.state.ota_done.connect(lambda nid: done_events.append(nid))
        self.state.ota_error.connect(lambda nid, err: error_events.append((nid, err)))

        self.state.ota_ready.emit(1, 0)
        self.state.ota_progress.emit(1, 45)
        self.state.ota_done.emit(1)
        self.state.ota_error.emit(2, 5)

        self.assertEqual(ready_events, [(1, 0)])
        self.assertEqual(progress_events, [(1, 45)])
        self.assertEqual(done_events, [1])
        self.assertEqual(error_events, [(2, 5)])

    def test_node_card_widget_blocks_offline_click(self):
        selected_nodes = []
        card = NodeCardWidget(node_id=3, on_select_callback=selected_nodes.append)

        # Initially offline
        self.assertFalse(card.is_online)
        self.assertEqual(card.cursor().shape(), Qt.CursorShape.ForbiddenCursor)
        self.assertIn("OFFLINE", card.toolTip().upper())

        # Click while offline
        ev = QMouseEvent(
            QMouseEvent.Type.MouseButtonPress,
            QPointF(10, 10),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier
        )
        card.mousePressEvent(ev)
        self.assertEqual(len(selected_nodes), 0) # Focus was blocked!

        # Now simulate node coming online
        t = self.state.get_node_telemetry(3)
        card.update_data(t, is_online=True)
        self.assertTrue(card.is_online)
        self.assertEqual(card.cursor().shape(), Qt.CursorShape.PointingHandCursor)

        # Click while online
        card.mousePressEvent(ev)
        self.assertEqual(selected_nodes, [3]) # Focus was allowed!

    def test_teensy_dashboard_blocks_offline_node_selection(self):
        dash = TeensyDashboardView(self.comm, self.state)

        # Put Node 1 and Node 2 online; Node 5 offline
        self.state.update_node_telemetry(1, drivers_enabled=True)
        self.state.update_node_telemetry(2, drivers_enabled=True)

        self.assertTrue(self.state.is_node_online(1))
        self.assertTrue(self.state.is_node_online(2))
        self.assertFalse(self.state.is_node_online(5))
        self.assertGreater(self.state.get_online_nodes_count(), 0)

        # Set focus to Node 1
        dash.select_node(1)
        self.assertEqual(dash.selected_node, 1)

        # Attempt to focus Node 5 (offline)
        dash.select_node(5)
        self.assertEqual(dash.selected_node, 1) # Must remain 1, 5 is blocked!

        # Focus Node 2 (online)
        dash.select_node(2)
        self.assertEqual(dash.selected_node, 2) # Allowed!

    def test_main_window_blocks_unsupported_screens_in_teensy_mode(self):
        win = MainWindow(self.comm, self.state)

        # Backend 1 = Teensy USB/CAN
        win.combo_backend.setCurrentIndex(1)
        self.app.processEvents()

        # Check unsupported buttons in Teensy mode (Motion, Periph, Params)
        self.assertFalse(win.btn_nav_motion.isEnabled())
        self.assertFalse(win.btn_nav_periph.isEnabled())
        self.assertFalse(win.btn_nav_params.isEnabled())

        # Supported buttons in Teensy mode (Dash, Term, Auto, OTA)
        self.assertTrue(win.btn_nav_dash.isEnabled())
        self.assertTrue(win.btn_nav_term.isEnabled())
        self.assertTrue(win.btn_nav_auto.isEnabled())
        self.assertTrue(win.btn_nav_ota.isEnabled())

        # Attempt navigation to blocked page (e.g. page 1 - Motion)
        win._set_page(1)
        self.assertEqual(win.stack.currentIndex(), 0) # Kept at 0 (Dashboard)!

        # Navigation to OTA (page 6) is allowed in Teensy mode!
        win._set_page(6)
        self.assertEqual(win.stack.currentIndex(), 6)

        # Switch to PeakCAN (Index 2)
        win.combo_backend.setCurrentIndex(2)
        self.app.processEvents()

        # Screens should be re-enabled
        self.assertTrue(win.btn_nav_motion.isEnabled())
        self.assertTrue(win.btn_nav_periph.isEnabled())
        self.assertTrue(win.btn_nav_params.isEnabled())
        self.assertTrue(win.btn_nav_ota.isEnabled())

        win.close()

    def test_ota_worker_simulation_run(self):
        sim = SimulatorClient(self.state)
        sim.connect()
        self.comm.active_client = sim

        # Create a dummy firmware binary
        dummy_data = b"\xAA\xBB\xCC\xDD\xEE\xFF" * 16 # 96 bytes
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(dummy_data)
            temp_path = f.name

        try:
            progress_updates = []
            steps = []
            results = []

            worker = OtaWorker(
                comm=self.comm,
                state=self.state,
                binary_path=temp_path,
                target_node=1,
                chunk_delay_s=0.0001
            )
            worker.progress_changed.connect(lambda sent, total, spd, eta: progress_updates.append((sent, total)))
            worker.step_changed.connect(steps.append)
            worker.finished.connect(lambda ok, msg: results.append((ok, msg)))

            worker.run() # Synchronous run in test

            self.assertTrue(len(results) > 0)
            ok, msg = results[-1]
            self.assertTrue(ok)
            self.assertTrue(len(progress_updates) > 0)
            last_sent, total_bytes = progress_updates[-1]
            self.assertEqual(last_sent, len(dummy_data))
            self.assertEqual(total_bytes, len(dummy_data))
        finally:
            sim.disconnect()
            if os.path.exists(temp_path):
                os.remove(temp_path)

    def test_teensy_serial_client_ota_methods_and_parsing(self):
        from python_app.core.teensy_serial_client import TeensySerialClient
        teensy = TeensySerialClient(self.state)

        # Test command generation
        sent_commands = []
        teensy.send_raw = lambda cmd: (sent_commands.append(cmd), True)[1]

        self.assertFalse(teensy._ota_active)
        teensy.ota_start(target_node=0, image_size=1048576)
        self.assertEqual(sent_commands[-1], "OTA_START 0 1048576")
        self.assertTrue(teensy._ota_active)

        teensy.ota_end(target_node=0, checksum=12345)
        self.assertEqual(sent_commands[-1], "OTA_END 0 12345")
        self.assertFalse(teensy._ota_active)

        teensy.ota_start(target_node=1, image_size=50000)
        self.assertTrue(teensy._ota_active)
        teensy.ota_abort(target_node=1)
        self.assertEqual(sent_commands[-1], "OTA_ABORT 1")
        self.assertFalse(teensy._ota_active)

        # Test response parsing
        ready_received = []
        prog_received = []
        done_received = []
        err_received = []

        self.state.ota_ready.connect(lambda n, s: ready_received.append((n, s)))
        self.state.ota_progress.connect(lambda n, p: prog_received.append((n, p)))
        self.state.ota_done.connect(done_received.append)
        self.state.ota_error.connect(lambda n, e: err_received.append((n, e)))

        teensy._parse_response_line("OTA_READY 2 0")
        teensy._parse_response_line("OTA_PROGRESS 2 50")
        teensy._parse_response_line("OTA_DONE 2")
        teensy._parse_response_line("OTA_ERROR 3 105")

        self.assertEqual(ready_received, [(2, 0)])
        self.assertEqual(prog_received, [(2, 50)])
        self.assertEqual(done_received, [2])
        self.assertEqual(err_received, [(3, 105)])


if __name__ == "__main__":
    unittest.main()
