import unittest
from unittest import mock

from python_app.core.serial_client import SerialClient
from python_app.core.state_model import DeviceState


class FakeSerial:
    """Porta serial falsa: registra a ordem das mudanças de DTR/RTS e o que é escrito."""

    instances = []

    def __init__(self, *args, **kwargs):
        self.events = []
        self.written = []
        self.is_open = False
        self._dtr = True   # padrão do pyserial
        self._rts = True
        self.port = self.baudrate = self.timeout = self.write_timeout = None
        FakeSerial.instances.append(self)

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        self.events.append(("dtr", value, self.is_open))

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value
        self.events.append(("rts", value, self.is_open))

    def open(self):
        self.is_open = True
        self.events.append(("open", self._dtr, self._rts))

    def close(self):
        self.is_open = False

    in_waiting = 0

    def read(self, n=1):
        return b""

    def write(self, data):
        self.written.append(data.decode().strip())

    def flush(self):
        pass


class SerialResetTests(unittest.TestCase):
    def setUp(self):
        FakeSerial.instances.clear()
        patcher = mock.patch("python_app.core.serial_client.serial.Serial", FakeSerial)
        patcher.start()
        self.addCleanup(patcher.stop)
        original_pulse = SerialClient.RESET_PULSE_S
        SerialClient.RESET_PULSE_S = 0.0
        self.addCleanup(setattr, SerialClient, "RESET_PULSE_S", original_pulse)
        self.state = DeviceState()
        self.client = SerialClient(self.state)
        self.addCleanup(self.client.disconnect)

    def test_opens_with_lines_inactive_and_pulses_en_with_io0_high(self):
        self.assertTrue(self.client.connect(port="COM9", reset_on_connect=True))
        port = FakeSerial.instances[-1]
        # Abre com DTR e RTS já inativos (sem pulso espúrio que leve ao bootloader)
        self.assertEqual(port.events[:3], [("dtr", False, False), ("rts", False, False), ("open", False, False)])
        # Reset: IO0 alto (DTR inativo), EN baixo (RTS ativo) e solta
        self.assertEqual(port.events[3:], [("dtr", False, True), ("rts", True, True), ("rts", False, True)])

    def test_initial_read_waits_for_boot_marker(self):
        self.client.connect(port="COM9", reset_on_connect=True)
        port = FakeSerial.instances[-1]
        self.assertEqual(port.written, [])  # nada durante o boot
        self.client._parse_response_line("=== SISTEMA PRONTO PARA COMANDOS ===")
        self.assertEqual(port.written, ["CONFIG DUMP", "STATUS"])
        # Um reinício posterior (crash, reset manual) relê a configuração de novo
        self.client._parse_response_line("=== SISTEMA PRONTO PARA COMANDOS ===")
        self.assertEqual(port.written[-2:], ["CONFIG DUMP", "STATUS"])

    def test_boot_timeout_falls_back_to_initial_read(self):
        SerialClient.BOOT_TIMEOUT_S = 0.05
        try:
            self.client.connect(port="COM9", reset_on_connect=True)
            port = FakeSerial.instances[-1]
            self.client._boot_timer.join(1.0)
            self.assertEqual(port.written, ["CONFIG DUMP", "STATUS"])
        finally:
            SerialClient.BOOT_TIMEOUT_S = 6.0

    def test_without_reset_reads_immediately(self):
        self.client.connect(port="COM9", reset_on_connect=False)
        port = FakeSerial.instances[-1]
        self.assertNotIn(("rts", True, True), port.events)
        self.assertEqual(port.written, ["CONFIG DUMP", "STATUS"])


if __name__ == "__main__":
    unittest.main()
