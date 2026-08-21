import struct
import unittest

import can

from python_app.core.can_client import CanClient
from python_app.core.protocol_defs import (
    CanOpcode,
    calc_ca_degrees_per_step,
    calc_ca_steps_for_degrees,
    calc_z_steps_for_mm,
)
from python_app.core.serial_client import SerialClient
from python_app.core.state_model import DeviceState


class RecordingCanClient(CanClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.frames = []

    def send_frame(self, arbitration_id, data, desc=""):
        self.frames.append((arbitration_id, bytes(data), desc))
        return True


class KinematicsTests(unittest.TestCase):
    def test_ca_steps_follow_configured_microsteps(self):
        self.assertEqual(calc_ca_steps_for_degrees(90.0, 200, 16), 800)
        self.assertEqual(calc_ca_steps_for_degrees(90.0, 400, 32), 3200)
        self.assertAlmostEqual(calc_ca_degrees_per_step(200, 16), 0.1125)

    def test_z_steps_follow_pulley_and_microsteps(self):
        self.assertEqual(calc_z_steps_for_mm(10.0, 16, 200, 16), 1000)
        self.assertEqual(calc_z_steps_for_mm(10.0, 20, 200, 16), 800)


class CanProtocolTests(unittest.TestCase):
    def setUp(self):
        self.state = DeviceState()
        self.client = RecordingCanClient(self.state)

    def test_move_force_sends_one_shot_profile_before_move(self):
        self.assertTrue(self.client.move_axis("A", 800, speed=123.5, accel=1800.0,
                                              force_no_encoder=True))
        self.assertEqual(len(self.client.frames), 2)

        _, profile, _ = self.client.frames[0]
        self.assertEqual(len(profile), 8)
        self.assertEqual(profile[0], CanOpcode.MOVE_PROFILE)
        self.assertEqual(profile[1], ord("A"))
        speed, accel = struct.unpack("<fH", profile[2:8])
        self.assertAlmostEqual(speed, 123.5, places=3)
        self.assertEqual(accel, 1800)

        _, move, _ = self.client.frames[1]
        self.assertEqual(move[0], CanOpcode.MOVE_FORCE)
        self.assertEqual(move[1], ord("A"))
        self.assertEqual(struct.unpack("<i", move[2:6])[0], 800)

    def test_position_and_temperature_frame_accepts_zero_celsius(self):
        payload = struct.pack("<HHHh", 18000, 35999, 1234, 0)
        message = can.Message(arbitration_id=0x291, data=payload, is_extended_id=False)
        self.client._process_rx_frame(message)

        telemetry = self.state.telemetry
        self.assertTrue(telemetry.pos_c_valid)
        self.assertTrue(telemetry.pos_a_valid)
        self.assertAlmostEqual(telemetry.pos_c_deg, 180.0)
        self.assertAlmostEqual(telemetry.pos_a_deg, 359.99)
        self.assertEqual(telemetry.pos_z_steps, 1234)
        self.assertTrue(telemetry.temp_valid)
        self.assertEqual(telemetry.temperature_c, 0.0)

    def test_invalid_telemetry_sentinels_clear_validity(self):
        self.state.update_telemetry(pos_c_valid=True, pos_a_valid=True, temp_valid=True,
                                    temperature_c=25.0)
        payload = struct.pack("<HHHh", 0xFFFF, 0xFFFF, 0, -32768)
        message = can.Message(arbitration_id=0x291, data=payload, is_extended_id=False)
        self.client._process_rx_frame(message)

        telemetry = self.state.telemetry
        self.assertFalse(telemetry.pos_c_valid)
        self.assertFalse(telemetry.pos_a_valid)
        self.assertFalse(telemetry.temp_valid)
        self.assertEqual(telemetry.temperature_c, 25.0)

    def test_axis_speed_uses_float32_can_payload(self):
        self.assertTrue(self.client.set_axis_speed("C", 456.25))
        _, payload, _ = self.client.frames[-1]
        self.assertEqual(payload[0], CanOpcode.AXIS_SPEED)
        self.assertEqual(payload[1], ord("C"))
        self.assertAlmostEqual(struct.unpack("<f", payload[2:6])[0], 456.25, places=3)


class SerialConfigParsingTests(unittest.TestCase):
    def test_current_and_max_profiles_are_kept_separate(self):
        state = DeviceState()
        client = SerialClient(state)
        client._parse_response_line("CONFIG SPEED C=100.00 A=200.00 Z=25.00")
        client._parse_response_line("CONFIG SPEED_MAX C=700.00 A=800.00 Z=60.00")
        client._parse_response_line("CONFIG ACCEL C=1000.00 A=1100.00 Z=300.00")
        client._parse_response_line("CONFIG ACCEL_MAX C=3000.00 A=3200.00 Z=800.00")

        self.assertEqual(state.parameters.speed, [100.0, 200.0, 25.0])
        self.assertEqual(state.parameters.speed_max, [700.0, 800.0, 60.0])
        self.assertEqual(state.parameters.accel, [1000.0, 1100.0, 300.0])
        self.assertEqual(state.parameters.accel_max, [3000.0, 3200.0, 800.0])


if __name__ == "__main__":
    unittest.main()
