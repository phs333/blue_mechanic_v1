import math
import unittest

from PyQt6.QtWidgets import QApplication

from python_app.core.protocol_defs import percent_to_laser_level
from python_app.core.state_model import DeviceState
from python_app.ui.widgets.mouse_control_pad import MouseControlPad, MouseJogController


class RecordingComm:
    def __init__(self):
        self.is_connected = True
        self.moves = []
        self.speeds = []
        self.lasers = []

    def move_sync(self, steps_c=0, steps_a=0, steps_z=0, **kwargs):
        self.moves.append((steps_c, steps_a, steps_z))
        self.speeds.append((kwargs.get("speed_c"), kwargs.get("speed_a"), kwargs.get("speed_z")))
        return True

    def set_laser(self, laser_index, level):
        self.lasers.append((laser_index, level))
        return True


class MouseJogControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = RecordingComm()
        self.ctl = MouseJogController(self.comm, self.state)
        self.ctl.send_interval_ms = 40  # caminho de fallback (backend sem JOG): segmentos de 40 ms
        self.ctl.activate()

    def tearDown(self):
        self.ctl.deactivate()
        self.ctl.deleteLater()
        QApplication.processEvents()

    def _run(self, seconds, start=0.0, per_tick=None):
        """Avança o controlador com relógio simulado; per_tick(i) injeta movimento do mouse."""
        interval = self.ctl.send_interval_ms / 1000.0
        n = int(round(seconds / interval))
        for i in range(n):
            if per_tick:
                per_tick(i)
            self.ctl.tick(now=start + i * interval)
        return start + n * interval

    def _totals(self):
        return tuple(sum(m[i] for m in self.comm.moves) for i in range(3))

    def test_mouse_axes_mapping(self):
        # 10 px para a direita = 2 graus em C; 10 px para cima = 2 graus em A; 1 clique da roda = 1 mm em Z
        self.ctl.on_move(10, -10)
        self.ctl.on_wheel(1)
        self._run(2.0)
        # Percorre exatamente o alvo (em passos inteiros), sem ultrapassar
        self.assertEqual(self._totals(), (18, 18, 80))  # 2° @ 3200 passos/volta; 1 mm @ 80 passos/mm

    def test_no_move_when_nothing_pending(self):
        self.assertFalse(self.ctl.tick(now=0.0))
        self.assertEqual(self.comm.moves, [])

    def test_first_segments_go_together_then_one_per_tick(self):
        interval = self.ctl.send_interval_ms / 1000.0
        sent_per_tick = []

        def mouse(i):
            self.ctl.on_move(4, 0)  # arrasto contínuo

        for i in range(20):
            mouse(i)
            before = len(self.comm.moves)
            self.ctl.tick(now=i * interval)
            sent_per_tick.append(len(self.comm.moves) - before)
        first = next(i for i, n in enumerate(sent_per_tick) if n)
        # A folga inicial sai de uma vez; depois, um segmento por intervalo (fila no mesmo nível)
        self.assertGreaterEqual(sent_per_tick[first], 2)
        self.assertTrue(all(n == 1 for n in sent_per_tick[first + 1:]))

    def test_segment_speeds_make_each_segment_last_the_interval(self):
        interval = self.ctl.send_interval_ms / 1000.0
        self._run(1.5, per_tick=lambda i: self.ctl.on_move(6, -3) if i < 20 else None)
        self.assertTrue(self.comm.moves)
        for (c, a, _), (sc, sa, _) in zip(self.comm.moves, self.comm.speeds):
            durations = set()
            for steps, speed in ((c, sc), (a, sa)):
                if steps:
                    durations.add(round(abs(steps) * 0.1125 / speed, 9))
                else:
                    self.assertIsNone(speed)
            # Todos os eixos do segmento terminam juntos, num múltiplo do intervalo
            # (em baixa velocidade um passo pode levar mais de um intervalo)
            self.assertEqual(len(durations), 1)
            ticks = durations.pop() / interval
            self.assertAlmostEqual(ticks, round(ticks), delta=1e-6)
            self.assertGreaterEqual(round(ticks), 1)

    def test_velocity_changes_between_segments_stay_within_jerk(self):
        jerk = self.state.parameters.jerk[0]
        interval = self.ctl.send_interval_ms / 1000.0
        # Arranque brusco, parada brusca e inversão de sentido
        # (5 px por tick = 25°/s, abaixo do limite: nada é descartado)
        self._run(2.5, per_tick=lambda i: self.ctl.on_move(5 if i < 10 else (-5 if 20 <= i < 30 else 0), 0))
        v = [math.copysign(s[0] or 0.0, m[0]) for m, s in zip(self.comm.moves, self.comm.speeds)]
        quantum = 0.1125 / interval  # 1 passo a mais ou a menos no segmento
        for prev, cur in zip(v, v[1:]):
            self.assertLessEqual(abs(cur - prev), jerk + quantum + 1e-6)
        self.assertEqual(self._totals()[0], 0)  # ida e volta iguais: termina onde começou

    def test_fast_mouse_is_clamped_to_machine_speed(self):
        vmax, _ = self.ctl._limits("C")
        self.ctl.on_move(5000, 0)
        self.assertLessEqual(self.ctl.pending["C"], vmax * self.ctl.MAX_LAG_S + 1e-6)
        self._run(1.0)
        interval = self.ctl.send_interval_ms / 1000.0
        for c, _, _ in self.comm.moves:
            self.assertLessEqual(abs(c) * 0.1125 / interval, vmax + 0.1125 / interval + 1e-6)

    def test_fraction_of_step_stays_pending(self):
        self.ctl.on_move(0.25, 0)  # 0,05 grau < meio passo
        self._run(0.5)
        self.assertEqual(self.comm.moves, [])
        self.assertAlmostEqual(self.ctl.pending["C"], 0.05)

    def test_double_click_toggles_laser(self):
        self.ctl.on_double_click(1)
        self.assertEqual(self.comm.lasers[-1], (1, percent_to_laser_level(100)))
        self.ctl.on_double_click(1)
        self.assertEqual(self.comm.lasers[-1], (1, 0))
        self.ctl.on_double_click(2)
        self.assertEqual(self.comm.lasers[-1], (2, percent_to_laser_level(100)))

    def test_hold_fades_up_and_release_keeps_level(self):
        self.ctl.fade_ms = 400
        self.ctl.on_press(1)
        self.ctl._start_fade(1)            # como se o tempo de "segurar" tivesse passado
        for _ in range(4):
            self.ctl.fade_tick()           # 4 x 40 ms de 400 ms = 40%
        self.ctl.on_release(1)
        level = self.ctl.laser_pct[1]
        self.assertAlmostEqual(level, 40.0, delta=0.5)
        self.ctl.fade_tick()               # soltou: não sobe mais
        self.assertAlmostEqual(self.ctl.laser_pct[1], level)
        self.assertEqual(self.comm.lasers[-1], (1, percent_to_laser_level(int(level))))

    def test_fade_stops_at_100(self):
        self.ctl.fade_ms = 100
        self.ctl._start_fade(2)
        for _ in range(10):
            self.ctl.fade_tick()
        self.assertEqual(self.ctl.laser_pct[2], 100.0)
        self.assertFalse(self.ctl.fading[2])

    def test_deactivate_discards_pending_motion(self):
        self.ctl.on_move(50, 50)
        self.ctl.deactivate()
        self.assertFalse(self.ctl.tick(now=0.0))
        self.assertEqual(self.comm.moves, [])


class JogComm(RecordingComm):
    supports_jog = True

    def __init__(self):
        super().__init__()
        self.jogs = []

    def jog(self, d_c=0.0, d_a=0.0, d_z=0.0):
        self.jogs.append((d_c, d_a, d_z))
        return True


class NodeJogTests(unittest.TestCase):
    """Serial direta: o deslocamento do mouse vai direto ao nó (JOG), sem suavizar nem descartar."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.comm = JogComm()
        self.ctl = MouseJogController(self.comm, DeviceState())
        self.ctl.activate()

    def tearDown(self):
        self.ctl.deactivate()
        self.ctl.deleteLater()
        QApplication.processEvents()

    def test_mouse_delta_is_forwarded_exactly(self):
        self.ctl.on_move(10, -5)
        self.ctl.on_wheel(2)
        self.assertTrue(self.ctl.tick(now=0.0))
        c, a, z = self.comm.jogs[-1]
        self.assertAlmostEqual(c, 2.0)
        self.assertAlmostEqual(a, 1.0)
        self.assertAlmostEqual(z, 2.0)
        self.assertEqual(self.comm.moves, [])  # nada de MOVE_SYNC
        self.assertFalse(self.ctl.tick(now=0.02))  # nada novo: não envia

    def test_fast_flick_is_not_discarded_on_the_app(self):
        self.ctl.on_move(5000, 0)
        self.ctl.tick(now=0.0)
        self.assertAlmostEqual(self.comm.jogs[-1][0], 1000.0)  # o nó limita (curso/atraso)

    def test_serial_jog_line_format(self):
        from unittest.mock import MagicMock
        from python_app.core.serial_client import SerialClient
        state = DeviceState()
        client = SerialClient(state)
        client.is_connected = True
        client.serial_port = MagicMock()
        terminal = []
        state.raw_message_received.connect(lambda d, t: terminal.append(t))
        self.assertTrue(client.jog(1.25, 0.0, -0.5))
        client.serial_port.write.assert_called_with(b"JOG C 1.25000 Z -0.50000\r\n")
        self.assertEqual(terminal, [])  # não inunda o terminal

    def test_old_firmware_without_jog_falls_back(self):
        from unittest.mock import MagicMock
        from python_app.core.comm_manager import CommManager
        from python_app.core.serial_client import SerialClient
        state = DeviceState()
        client = SerialClient(state)
        client.is_connected = True
        client.serial_port = MagicMock()
        client.jog(1.0)
        client._parse_response_line("Comando desconhecido. Use HELP.")
        self.assertFalse(client.supports_jog)
        comm = CommManager.__new__(CommManager)
        comm.active_client = client
        self.assertFalse(comm.supports_jog)  # o controle por mouse volta ao MOVE_SYNC


class MouseControlPadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def test_requires_connection_and_releases(self):
        comm = RecordingComm()
        ctl = MouseJogController(comm, DeviceState())
        pad = MouseControlPad(ctl)
        comm.is_connected = False
        pad.activate()
        self.assertFalse(pad.is_active())
        comm.is_connected = True
        pad.activate()
        self.assertTrue(pad.is_active() and ctl.active)
        pad.deactivate()
        self.assertFalse(pad.is_active() or ctl.active)
        pad.deleteLater()
        ctl.deleteLater()
        QApplication.processEvents()


if __name__ == "__main__":
    unittest.main()
