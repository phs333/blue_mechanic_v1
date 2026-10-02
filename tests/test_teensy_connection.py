"""Conexão do app com o bridge Teensy (COM) e estado online dos nodes."""

import threading
import time
import unittest
from unittest import mock

from PyQt6.QtWidgets import QApplication

from python_app.core.comm_manager import CommManager
from python_app.core.state_model import DeviceState
from python_app.core.teensy_serial_client import TeensySerialClient


class FakeTeensyPort:
    """COM do Teensy: aberta na construção (como serial.Serial(port=...)), sem nodes no CAN."""

    instances = []

    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.dtr = False
        self.rts = False
        self.written = []
        self.pending = bytearray()
        self.reset_calls = 0
        self.lock = threading.Lock()
        FakeTeensyPort.instances.append(self)

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.pending)

    def read(self, n=1):
        with self.lock:
            if not self.pending:
                data = b""
            else:
                data = bytes(self.pending[:n])
                del self.pending[:n]
        if not data:
            time.sleep(0.01)
        return data

    def feed(self, text):
        with self.lock:
            self.pending.extend(text.encode("ascii"))

    def reset_input_buffer(self):
        self.reset_calls += 1
        with self.lock:
            self.pending.clear()

    def write(self, data):
        self.written.append(data.decode("ascii").strip())

    def flush(self):
        pass

    def close(self):
        self.is_open = False


class TeensyConnectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        FakeTeensyPort.instances.clear()
        patcher = mock.patch("python_app.core.teensy_serial_client.serial.Serial", FakeTeensyPort)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.state = DeviceState()
        self.client = TeensySerialClient(self.state)
        self.addCleanup(self.client.disconnect)

    def _pump(self, seconds=0.05):
        end = time.time() + seconds
        while time.time() < end:
            self.app.processEvents()
            time.sleep(0.005)

    def test_connect_without_nodes_reports_no_node_online(self):
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        self.client._parse_response_line("HEARTBEAT 1")
        self.client.disconnect()
        # Reconexao: o reset de can_online do node focado nao pode contar como resposta
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        self._pump()
        # Abrir a COM não prova que algum node existe: nada respondeu no CAN
        self.assertEqual(self.state.get_online_nodes_count(), 0)
        self.assertFalse(self.state.is_node_online(1))

    def test_disconnect_marks_every_node_offline(self):
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        self.client._parse_response_line("HEARTBEAT 1")  # node focado: can_online passa a True
        self.client._parse_response_line("HEARTBEAT 2")
        self.client._parse_response_line("HEARTBEAT 3")
        self.assertEqual(self.state.get_online_nodes_count(), 3)

        self.client.disconnect()
        self._pump()
        self.assertEqual(self.state.get_online_nodes_count(), 0)

    def test_stale_lines_buffered_before_connect_are_discarded(self):
        # Linhas que o Teensy acumulou com a COM fechada (heartbeats antigos) não podem
        # marcar nodes como online na conexão nova
        original_init = FakeTeensyPort.__init__

        def init_with_backlog(port_self, *args, **kwargs):
            original_init(port_self, *args, **kwargs)
            port_self.feed("HEARTBEAT 7\r\nHEARTBEAT 8\r\n")

        with mock.patch.object(FakeTeensyPort, "__init__", init_with_backlog):
            self.assertTrue(self.client.connect(port="COM9", node_id=1))
        self._pump(0.1)
        self.assertFalse(self.state.is_node_online(7))
        self.assertFalse(self.state.is_node_online(8))

    def test_live_heartbeat_after_connect_marks_node_online(self):
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        FakeTeensyPort.instances[-1].feed("HEARTBEAT 4\r\n")
        self._pump(0.15)
        self.assertTrue(self.state.is_node_online(4))

    def test_lost_port_reports_disconnect_once(self):
        changes = []
        self.state.connection_changed.connect(lambda ok, name: changes.append(ok))
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        changes.clear()

        port = FakeTeensyPort.instances[-1]

        def unplugged(*args, **kwargs):
            import serial
            raise serial.SerialException("dispositivo removido")

        port.read = unplugged
        self._pump(0.2)
        self.assertFalse(self.client.is_connected)
        self.assertEqual(changes, [False])
        self.assertEqual(self.state.get_online_nodes_count(), 0)

    def test_old_reader_thread_does_not_drop_new_session(self):
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        old_session = self.client._session
        self.assertTrue(self.client.connect(port="COM9", node_id=1))
        # Um fim tardio da thread da sessão anterior não pode derrubar a conexão atual
        self.client._on_link_lost(old_session, "teste")
        self._pump()
        self.assertTrue(self.client.is_connected)
        self.assertTrue(self.state.is_connected)


class NodeResponsesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.client = TeensySerialClient(self.state)

    def test_pong_marks_node_online(self):
        self.client._parse_response_line("PONG 5 12 34")
        self.assertTrue(self.state.is_node_online(5))

    def test_node_error_keeps_node_online(self):
        self.client._parse_response_line("ERROR 6 25 08")
        self.assertTrue(self.state.is_node_online(6))

    def test_teensy_ok_lines_are_ignored_without_errors(self):
        errors = []
        self.state.error_occurred.connect(errors.append)
        for line in ("TEENSY_OK USYNC 1", "TEENSY_OK CANBR 1000000", "TEENSY_OK STOP 0 1", "TEENSY_READY 1"):
            self.client._parse_response_line(line)
        self.assertEqual(errors, [])
        self.assertEqual(self.state.get_online_nodes_count(), 0)


class OnlineSummaryRefreshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def test_status_bar_count_drops_when_nodes_time_out(self):
        from python_app.ui.main_window import MainWindow

        state = DeviceState()
        comm = CommManager(state)
        win = MainWindow(comm, state)
        self.addCleanup(win.deleteLater)
        win.combo_backend.setCurrentIndex(1)  # Teensy

        state.update_node_telemetry(3, last_heartbeat_timestamp=time.time())
        self.assertIn("1/10", win.lbl_status_nodes.text())

        # Node some (sem heartbeat): nenhum sinal novo chega, mas o tick precisa atualizar
        node = state.get_node_telemetry(3)
        node.last_heartbeat_timestamp = node.last_seen_timestamp = time.time() - 10.0
        win._on_tick()
        self.assertIn("0/10", win.lbl_status_nodes.text())


if __name__ == "__main__":
    unittest.main()
