import unittest
from unittest import mock

from PyQt6.QtWidgets import QApplication

from python_app.core.protocol_defs import calc_ca_steps_for_degrees, calc_z_steps_for_mm
from python_app.core.state_model import DeviceState
from python_app.ui.views.dashboard_view import DashboardView


class DashboardDirectMoveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["--platform", "offscreen"])

    def setUp(self):
        self.state = DeviceState()
        self.comm = mock.MagicMock()
        self.view = DashboardView(self.comm, self.state)

    def tearDown(self):
        self.view.deleteLater()
        QApplication.processEvents()

    def _select_axis(self, idx):
        self.view.combo_axis.setCurrentIndex(idx)

    def test_degrees_for_c_with_invert_and_custom_accel(self):
        p = self.state.parameters
        self._select_axis(0)
        self.view.spin_distance.setValue(45.0)
        self.view.chk_invert_direct.setChecked(True)
        self.view.spin_speed.setValue(90.0)
        self.view.spin_accel.setValue(1200.0)
        self.view._execute_direct_move()
        expected = calc_ca_steps_for_degrees(-45.0, p.steps_per_rev[0], p.tmc_microsteps[0])
        self.comm.move_axis.assert_called_once_with("C", expected, speed=90.0, accel=1200.0,
                                                    force_no_encoder=False)
        self.assertLess(expected, 0)

    def test_mm_for_z_and_nvs_defaults(self):
        p = self.state.parameters
        self._select_axis(2)
        self.assertTrue(self.view.spin_distance.suffix().strip() == "mm")
        self.view.spin_distance.setValue(12.5)
        self.view.chk_force_direct.setChecked(True)  # ignorado no Z
        self.view._execute_direct_move()
        expected = calc_z_steps_for_mm(12.5, p.z_pulley_teeth, p.steps_per_rev[2], p.tmc_microsteps[2])
        self.comm.move_axis.assert_called_once_with("Z", expected, speed=None, accel=None,
                                                    force_no_encoder=False)

    def test_distance_is_remembered_per_axis(self):
        self._select_axis(0)
        self.view.spin_distance.setValue(30.0)
        self._select_axis(2)
        self.view.spin_distance.setValue(5.0)
        self._select_axis(0)
        self.assertAlmostEqual(self.view.spin_distance.value(), 30.0)
        self.assertEqual(self.view.spin_distance.suffix().strip(), "°")

    def test_progress_follows_new_limits_without_new_telemetry(self):
        self.state.update_telemetry(pos_c_deg=0.0, pos_c_valid=True)
        self.assertEqual(self.view.card_c.progress_bar.value(), 50)  # [-540, 540]
        # Limites novos chegam (ex.: LIMIT C gravado) sem nova leitura de posição
        self.state.update_parameters(limit_min_deg_c=0.0, limit_max_deg_c=100.0)
        self.assertEqual(self.view.card_c.progress_bar.value(), 0)
        self.state.update_parameters(limit_min_deg_c=-100.0, limit_max_deg_c=100.0)
        self.assertEqual(self.view.card_c.progress_bar.value(), 50)

    def test_z_progress_follows_new_travel(self):
        self.state.update_telemetry(pos_z_steps=1000, max_z_steps=4000)
        self.state.update_parameters(max_passos_z=2000)
        self.assertEqual(self.view.card_z.progress_bar.value(), 50)


if __name__ == "__main__":
    unittest.main()
