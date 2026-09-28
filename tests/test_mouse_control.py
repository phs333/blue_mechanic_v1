import unittest

from PyQt6.QtWidgets import QApplication

from python_app.core.protocol_defs import percent_to_laser_level
from python_app.core.state_model import DeviceState
from python_app.ui.widgets.mouse_control_pad import MouseControlPad, MouseJogController


class RecordingComm:
    def __init__(self):
        self.is_connected = True
        self.moves = []
        self.lasers = []

    def move_sync(self, steps_c=0, steps_a=0, steps_z=0, **kwargs):
        self.moves.append((steps_c, steps_a, steps_z))
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
        self.ctl.activate()

    def tearDown(self):
        self.ctl.deactivate()
        self.ctl.deleteLater()
        QApplication.processEvents()

    def test_mouse_axes_mapping(self):
        # 10 px para a direita = 2 graus em C; 10 px para cima = 2 graus em A; 1 clique da roda = 1 mm em Z
        self.ctl.on_move(10, -10)
        self.ctl.on_wheel(1)
        self.assertTrue(self.ctl.tick())
        c, a, z = self.comm.moves[-1]
        self.assertEqual(c, 18)   # 2 graus @ 3200 passos/volta
        self.assertEqual(a, 18)   # 2 graus @ 3200 passos/volta
        self.assertEqual(z, 80)   # 1 mm @ 80 passos/mm (polia 20T)

    def test_no_move_when_nothing_pending(self):
        self.assertFalse(self.ctl.tick())
        self.assertEqual(self.comm.moves, [])

    def test_fast_mouse_is_clamped_to_machine_speed(self):
        # 140 graus/s a cada 80 ms -> no máximo 11,2 graus por lote e 22,4 de pendência
        self.ctl.on_move(5000, 0)
        self.assertLessEqual(self.ctl.pending["C"], 22.4 + 1e-6)
        self.ctl.tick()
        c, _, _ = self.comm.moves[-1]
        self.assertLessEqual(abs(c), round(11.2 / 0.1125) + 1)

    def test_fraction_of_step_stays_pending(self):
        self.ctl.on_move(0.25, 0)  # 0,05 grau < 1 passo
        self.assertFalse(self.ctl.tick())
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
        self.assertFalse(self.ctl.tick())
        self.assertEqual(self.comm.moves, [])


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
