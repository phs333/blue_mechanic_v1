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

    def test_poll_all_nodes_queries_all_ten_nodes(self):
        commands_sent = []
        self.comm.send_raw = lambda cmd: commands_sent.append(cmd) or True
        dash = TeensyDashboardView(self.comm, self.state)

        # Even when all nodes are offline, _poll_all_nodes MUST query all 10 nodes
        dash._poll_all_nodes()
        expected = [f"R {i}" for i in range(1, 11)]
        self.assertEqual(commands_sent, expected)

    def test_request_status_all_sends_commands_to_all_ten_nodes(self):
        raw_commands = []
        self.client.send_raw = lambda cmd: raw_commands.append(cmd) or True

        result = self.client.request_status_all()
        self.assertTrue(result)
        expected = [f"R {i}" for i in range(1, 11)]
        self.assertEqual(raw_commands, expected)

    def test_pos_parsing_handles_nan_and_valid_values(self):
        # Node 2 sends POS with nan angles (uncalibrated / read error) and -99.9 temp
        self.client._parse_response_line("POS 2 nan nan 450 -99.9")
        t2 = self.state.get_node_telemetry(2)
        self.assertFalse(t2.pos_c_valid)
        self.assertFalse(t2.pos_a_valid)
        self.assertEqual(t2.pos_z_steps, 450)
        self.assertFalse(t2.temp_valid)
        self.assertTrue(self.state.is_node_online(2))

        # Node 2 sends valid POS with negative angle and 31.5 C
        self.client._parse_response_line("POS 2 -15.5 42.0 1200 31.5")
        self.assertTrue(t2.pos_c_valid)
        self.assertAlmostEqual(t2.pos_c_deg, -15.5)
        self.assertTrue(t2.pos_a_valid)
        self.assertAlmostEqual(t2.pos_a_deg, 42.0)
        self.assertEqual(t2.pos_z_steps, 1200)
        self.assertTrue(t2.temp_valid)
        self.assertAlmostEqual(t2.temperature_c, 31.5)

    def test_device_state_update_telemetry_syncs_with_nodes_telemetry(self):
        self.state.parameters.node_id = 3
        self.state.update_telemetry(
            pos_c_deg=35.0,
            pos_c_valid=True,
            temperature_c=28.0,
            temp_valid=True,
        )
        t3 = self.state.get_node_telemetry(3)
        self.assertAlmostEqual(t3.pos_c_deg, 35.0)
        self.assertTrue(t3.pos_c_valid)
        self.assertAlmostEqual(t3.temperature_c, 28.0)
        self.assertTrue(t3.temp_valid)
    def test_send_unified_formats_command_correctly(self):
        sent = []
        self.client.send_raw = lambda cmd: sent.append(cmd) or True
        self.client.target_node = 2

        # Envia coordenadas absolutas C=45.2, A=-12.5, Z=150.00, Laser1=2048, Laser2=1024
        result = self.client.send_unified(45.2, -12.5, 150.0, laser1=2048, laser2=1024)
        self.assertTrue(result)
        self.assertEqual(sent[-1], "U 2 45.20 -12.50 150.00 2048 1024")

    def test_send_unified_broadcast_and_force(self):
        sent = []
        self.client.send_raw = lambda cmd: sent.append(cmd) or True

        # Broadcast para todos os nós (node=0) com force_no_encoder=True (UF)
        result = self.client.send_unified(0.0, 0.0, 0.0, laser1=4095, laser2=4095, force_no_encoder=True, node_id=0)
        self.assertTrue(result)
        self.assertEqual(sent[-1], "UF 0 0.00 0.00 0.00 4095 4095")

    def test_send_unified_clamps_laser_values(self):
        sent = []
        self.client.send_raw = lambda cmd: sent.append(cmd) or True

        # Lasers acima de 4095 ou negativos devem ser clampados
        self.client.send_unified(10.0, 20.0, 30.0, laser1=5000, laser2=-50, node_id=4)
        self.assertEqual(sent[-1], "U 4 10.00 20.00 30.00 4095 0")

    def test_move_sync_deg_formats_absolute_command(self):
        sent = []
        self.client.send_raw = lambda cmd: sent.append(cmd) or True
        self.client.target_node = 5

        result = self.client.move_sync_deg(30.5, -5.0, 80.25)
        self.assertTrue(result)
        self.assertEqual(sent[-1], "MS 5 30.5 -5.0 80.25")

        # Com force_no_encoder
        self.client.move_sync_deg(15.0, 10.0, 50.0, force_no_encoder=True)
        self.assertEqual(sent[-1], "MSF 5 15.0 10.0 50.00")

    def test_comm_manager_forwards_unified_and_move_sync_deg(self):
        sent = []
        self.client.send_raw = lambda cmd: sent.append(cmd) or True
        self.comm.active_client = self.client

        self.comm.send_unified(25.0, -10.0, 100.0, laser1=1000, laser2=2000, node_id=3)
        self.assertEqual(sent[-1], "U 3 25.00 -10.00 100.00 1000 2000")

        self.comm.move_sync_deg(10.0, 20.0, 30.0, node_id=3)
        self.assertEqual(sent[-1], "MS 3 10.0 20.0 30.00")


if __name__ == "__main__":
    unittest.main()
