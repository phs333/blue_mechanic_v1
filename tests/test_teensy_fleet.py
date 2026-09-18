import time
import unittest
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QCoreApplication

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.teensy_serial_client import TeensySerialClient
from python_app.ui.views.teensy_dashboard_view import TeensyDashboardView


class TeensyFleetTelemetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)
        self.client = TeensySerialClient(self.state)

    def test_state_model_initializes_ten_nodes(self):
        self.assertEqual(len(self.state.nodes_telemetry), 10)
        for node_id in range(1, 11):
            self.assertIn(node_id, self.state.nodes_telemetry)
            telem = self.state.get_node_telemetry(node_id)
            self.assertIsNotNone(telem)
            self.assertFalse(self.state.is_node_online(node_id))

    def test_state_model_updates_individual_node_telemetry(self):
        signals_received = []
        summary_received = []

        self.state.node_telemetry_updated.connect(lambda nid, t: signals_received.append((nid, t)))
        self.state.nodes_summary_updated.connect(lambda count: summary_received.append(count))

        self.state.update_node_telemetry(
            1,
            pos_c_deg=12.5,
            pos_a_deg=-4.2,
            pos_z_steps=1000,
            temperature_c=32.0,
            drivers_enabled=True,
        )
        self.state.update_node_telemetry(
            2,
            pos_c_deg=90.0,
            pos_a_deg=0.0,
            pos_z_steps=0,
            temperature_c=29.5,
            drivers_enabled=False,
        )

        t1 = self.state.get_node_telemetry(1)
        t2 = self.state.get_node_telemetry(2)

        self.assertAlmostEqual(t1.pos_c_deg, 12.5)
        self.assertAlmostEqual(t1.pos_a_deg, -4.2)
        self.assertEqual(t1.pos_z_steps, 1000)
        self.assertAlmostEqual(t1.temperature_c, 32.0)
        self.assertTrue(t1.drivers_enabled)
        self.assertTrue(self.state.is_node_online(1))

        self.assertAlmostEqual(t2.pos_c_deg, 90.0)
        self.assertAlmostEqual(t2.temperature_c, 29.5)
        self.assertFalse(t2.drivers_enabled)
        self.assertTrue(self.state.is_node_online(2))

        # Node 3 was never updated, must be offline
        self.assertFalse(self.state.is_node_online(3))

        self.assertEqual(self.state.get_online_nodes_count(), 2)
        self.assertGreaterEqual(len(signals_received), 2)
        self.assertGreaterEqual(len(summary_received), 2)
        self.assertEqual(summary_received[-1], 2)

    def test_teensy_serial_client_parses_status_for_multiple_nodes(self):
        # Format: STATUS <node> <flags> <laser1> <laser2> <fan_on> <fan_mode> <speed_level>
        # Flags: 61 has DRIVERS_ENABLED (0x01) and CAN_ONLINE (0x10) and TEMP_VALID (0x04) and ALARME_Z (0x08) and TMC_READY (0x20)
        self.client._parse_response_line("STATUS 1 61 4095 2048 1 2 4")
        self.client._parse_response_line("STATUS 3 0 0 0 0 0 1")

        t1 = self.state.get_node_telemetry(1)
        t3 = self.state.get_node_telemetry(3)

        self.assertTrue(t1.drivers_enabled)
        self.assertEqual(t1.laser1_level, 4095)
        self.assertEqual(t1.laser2_level, 2048)
        self.assertTrue(t1.fan_output_on)

        self.assertFalse(t3.drivers_enabled)
        self.assertEqual(t3.laser1_level, 0)
        self.assertEqual(t3.laser2_level, 0)
        self.assertFalse(t3.fan_output_on)

        self.assertTrue(self.state.is_node_online(1))
        self.assertTrue(self.state.is_node_online(3))
        self.assertFalse(self.state.is_node_online(2))

    def test_teensy_serial_client_parses_pos_for_multiple_nodes(self):
        # Format: POS <node> <pos_c> <pos_a> <pos_z> <temp>
        self.client._parse_response_line("POS 1 25.50 10.00 1520 33.4")
        self.client._parse_response_line("POS 7 180.00 45.00 500 27.1")

        t1 = self.state.get_node_telemetry(1)
        t7 = self.state.get_node_telemetry(7)

        self.assertAlmostEqual(t1.pos_c_deg, 25.50)
        self.assertAlmostEqual(t1.pos_a_deg, 10.00)
        self.assertEqual(t1.pos_z_steps, 1520)
        self.assertAlmostEqual(t1.temperature_c, 33.4)

        self.assertAlmostEqual(t7.pos_c_deg, 180.00)
        self.assertAlmostEqual(t7.pos_a_deg, 45.00)
        self.assertEqual(t7.pos_z_steps, 500)
        self.assertAlmostEqual(t7.temperature_c, 27.1)

    def test_teensy_serial_client_parses_heartbeat_events(self):
        self.client._parse_response_line("HEARTBEAT 5")
        t5 = self.state.get_node_telemetry(5)
        self.assertGreater(t5.last_heartbeat_timestamp, 0)
        self.assertTrue(self.state.is_node_online(5))

    def test_teensy_dashboard_view_and_node_selection(self):
        dash = TeensyDashboardView(self.comm, self.state)
        self.assertEqual(len(dash.node_cards), 10)

        # Verify initial card states
        for nid, card in dash.node_cards.items():
            self.assertEqual(card.node_id, nid)
            self.assertFalse(card.is_online)

        # Select node 4
        selected_nodes = []
        dash.selected_node_changed.connect(selected_nodes.append)
        dash.select_node(4)

        self.assertEqual(dash.selected_node, 4)
        self.assertIn("NODE 4", dash.lbl_focus_title.text().upper())
        self.assertIn(4, selected_nodes)

        # Push telemetry to Node 4 and ensure card updates
        self.state.update_node_telemetry(
            4,
            pos_c_deg=18.3,
            pos_a_deg=-5.0,
            pos_c_valid=True,
            pos_a_valid=True,
            pos_z_steps=250,
            temperature_c=34.0,
            temp_valid=True,
            drivers_enabled=True,
        )
        self.app.processEvents()

        card4 = dash.node_cards[4]
        self.assertTrue(card4.is_online)
        self.assertIn("18.30", card4.lbl_pos_c.text())
        self.assertIn("-5.00", card4.lbl_pos_a.text())
        self.assertIn("250p", card4.lbl_pos_z.text())
        self.assertIn("34.0", card4.lbl_temp.text())

    def test_teensy_serial_client_handles_can_tx_failed_with_backoff(self):
        errors_reported = []
        self.state.error_occurred.connect(errors_reported.append)

        # Node 6 fails transmission on CAN
        self.client._parse_response_line("TEENSY_ERROR CAN_TX_FAILED 6 -3")

        self.assertIn(6, self.client.node_offline_until)
        self.assertGreater(self.client.node_offline_until[6], time.time())
        self.assertFalse(self.state.is_node_online(6))
        # Routine offline polling miss does not spam error dialogs
        self.assertEqual(len(errors_reported), 0)

        # Later, node 6 boots and sends a heartbeat
        self.client._parse_response_line("HEARTBEAT 6")
        self.assertNotIn(6, self.client.node_offline_until)
        self.assertTrue(self.state.is_node_online(6))

    def test_can_tx_failed_does_not_flicker_recently_active_node(self):
        # Node 1 is online from recent heartbeat
        self.client._parse_response_line("HEARTBEAT 1")
        self.assertTrue(self.state.is_node_online(1))

        # A transient CAN TX failure should NOT immediately wipe node 1 to offline
        self.client._parse_response_line("TEENSY_ERROR CAN_TX_FAILED 1 -3")
        self.assertTrue(self.state.is_node_online(1))


if __name__ == "__main__":
    unittest.main()
