import os
import struct
import tempfile
import threading
import unittest

import can
from PyQt6.QtWidgets import QApplication

from python_app.core.comm_manager import CommManager
from python_app.core.ota_manager import OtaWorker
from python_app.core.protocol_defs import CanOpcode, ESP_ERRORS, STOP_FLAG_LASERS_OFF
from python_app.core.serial_client import SerialClient
from python_app.core.simulator import SimulatorClient
from python_app.core.state_model import DeviceState

from test_protocol import RecordingCanClient, RecordingTeensyClient


class RecordingSerialClient(SerialClient):
    def __init__(self, state):
        super().__init__(state)
        self.commands = []

    def send_raw(self, cmd):
        self.commands.append(cmd)
        return True


class EstopTests(unittest.TestCase):
    def setUp(self):
        self.state = DeviceState()

    def test_serial_estop_and_stop_commands(self):
        client = RecordingSerialClient(self.state)
        self.assertTrue(client.stop_all(lasers_off=True))
        self.assertTrue(client.stop_all(lasers_off=False))
        self.assertEqual(client.commands, ["ESTOP", "STOP"])

    def test_can_estop_is_broadcast_with_laser_flag(self):
        client = RecordingCanClient(self.state)
        client.node_id = 7
        self.assertTrue(client.stop_all(lasers_off=True))
        frame_id, data, _ = client.frames[-1]
        self.assertEqual(frame_id, client.cmd_base_id)  # broadcast, não só o nó selecionado
        self.assertEqual(data, bytes([CanOpcode.STOP, STOP_FLAG_LASERS_OFF]))

    def test_teensy_estop_is_broadcast_stop_opcode(self):
        client = RecordingTeensyClient(self.state, node_id=3)
        self.assertTrue(client.stop_all(lasers_off=True))
        self.assertTrue(client.stop_all(lasers_off=False))
        self.assertEqual(client.commands, ["ESTOP 0", "STOP 0"])

    def test_teensy_pos_accepts_signed_angles_and_nan(self):
        client = RecordingTeensyClient(self.state, node_id=2)
        client._parse_response_line("POS 2 -45.3 540.0 1200 31.5")
        t = self.state.get_node_telemetry(2)
        self.assertTrue(t.pos_c_valid and t.pos_a_valid)
        self.assertAlmostEqual(t.pos_c_deg, -45.3)
        self.assertAlmostEqual(t.pos_a_deg, 540.0)
        client._parse_response_line("POS 2 nan 10.0 1200 31.5")
        self.assertFalse(self.state.get_node_telemetry(2).pos_c_valid)

    def test_can_pos_v2_decoded_after_status_flag(self):
        client = RecordingCanClient(self.state)
        client.node_id = 1
        status = can.Message(arbitration_id=0x281, data=bytes([0x82, 1, 0x40 | 0x01, 0, 0, 0, 0, 0]))
        pos = can.Message(arbitration_id=0x291,
                          data=struct.pack('<hhHh', -1234, -32768, 500, 250))
        client._process_rx_frame(status)
        client._process_rx_frame(pos)
        self.assertTrue(self.state.telemetry.pos_c_valid)
        self.assertAlmostEqual(self.state.telemetry.pos_c_deg, -123.4)
        self.assertFalse(self.state.telemetry.pos_a_valid)

    def test_can_pos_v1_still_decoded_for_old_nodes(self):
        client = RecordingCanClient(self.state)
        client.node_id = 1
        status = can.Message(arbitration_id=0x281, data=bytes([0x82, 1, 0x01, 0, 0, 0, 0, 0]))
        pos = can.Message(arbitration_id=0x291, data=struct.pack('<HHHh', 18050, 0xFFFF, 0, 250))
        client._process_rx_frame(status)
        client._process_rx_frame(pos)
        self.assertAlmostEqual(self.state.telemetry.pos_c_deg, 180.5)
        self.assertFalse(self.state.telemetry.pos_a_valid)

    def test_simulator_estop_freezes_targets(self):
        sim = SimulatorClient(self.state)
        sim.drivers_en = True
        sim.sim_c_deg = 12.0
        sim.target_c_deg = 80.0
        sim.laser1 = 2000
        self.assertTrue(sim.stop_all())
        self.assertEqual(sim.target_c_deg, 12.0)
        self.assertEqual(sim.laser1, 0)

    def test_new_error_codes_are_named(self):
        self.assertEqual(ESP_ERRORS[0x0C], "ESP_ERR_NOT_FINISHED")
        self.assertEqual(ESP_ERRORS[0x09], "ESP_ERR_INVALID_CRC")


class ScriptedOtaClient(SimulatorClient):
    """Simulador com respostas configuráveis para o handshake OTA."""

    def __init__(self, state, ready=True, end_error=None):
        super().__init__(state)
        self.ready = ready
        self.end_error = end_error
        self.chunks = 0
        self.aborts = 0

    def ota_start(self, target_node, image_size):
        if self.ready:
            threading.Timer(0.02, lambda: self.state.ota_ready.emit(target_node, 0)).start()
        return True

    def ota_send_chunk(self, seq_num, chunk, target_node=0):
        self.chunks += 1
        return True

    def ota_end(self, target_node, checksum=0):
        if self.end_error is None:
            threading.Timer(0.02, lambda: self.state.ota_done.emit(target_node)).start()
        else:
            threading.Timer(0.02, lambda: self.state.ota_error.emit(target_node, self.end_error)).start()
        return True

    def ota_abort(self, target_node=0):
        self.aborts += 1
        return True


class OtaHandshakeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = CommManager(self.state)
        fd, self.path = tempfile.mkstemp(suffix=".bin")
        with os.fdopen(fd, "wb") as f:
            f.write(bytes(range(60)))

    def tearDown(self):
        os.remove(self.path)

    def _run(self, client, **worker_kwargs):
        self.comm.active_client = client
        worker = OtaWorker(self.comm, self.state, self.path, target_node=2, chunk_delay_s=0)
        for key, value in worker_kwargs.items():
            setattr(worker, key, value)
        results = []
        worker.finished.connect(lambda ok, msg: results.append((ok, msg)))
        worker.run()
        self.addCleanup(worker.deleteLater)
        return results[-1], worker

    def test_success_requires_ota_done(self):
        client = ScriptedOtaClient(self.state)
        (ok, _), _ = self._run(client)
        self.assertTrue(ok)
        self.assertEqual(client.chunks, 10)

    def test_node_rejecting_image_is_reported_as_failure(self):
        client = ScriptedOtaClient(self.state, end_error=0x09)
        (ok, msg), _ = self._run(client)
        self.assertFalse(ok)
        self.assertIn("Falharam: [2]", msg)

    def test_no_ready_means_no_data_is_streamed(self):
        client = ScriptedOtaClient(self.state, ready=False)
        (ok, _), _ = self._run(client, READY_TIMEOUT_S=0.2)
        self.assertFalse(ok)
        self.assertEqual(client.chunks, 0)
        self.assertEqual(client.aborts, 1)

    def test_worker_disconnects_state_signals_after_run(self):
        client = ScriptedOtaClient(self.state)
        _, worker = self._run(client)
        self.state.ota_ready.emit(5, 0)  # não deve mais alimentar o worker encerrado
        self.assertNotIn(5, worker._ready)


if __name__ == "__main__":
    unittest.main()
