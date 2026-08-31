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
from python_app.core.teensy_serial_client import TeensySerialClient


class RecordingCanClient(CanClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.frames = []

    def send_frame(self, arbitration_id, data, desc=""):
        self.frames.append((arbitration_id, bytes(data), desc))
        return True


class RecordingTeensyClient(TeensySerialClient):
    def __init__(self, state: DeviceState, node_id: int = 1):
        super().__init__(state)
        self.node_id = node_id
        self.commands = []

    def send_raw(self, data: str):
        self.commands.append(data)
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
        self.assertEqual(len(payload), 6)
        self.assertEqual(payload[0], CanOpcode.AXIS_SPEED)
        self.assertEqual(payload[1], ord("C"))
        self.assertAlmostEqual(struct.unpack("<f", payload[2:6])[0], 456.25, places=3)

    def test_commands_use_exact_dlc_without_sequence_byte(self):
        cases = [
            (lambda: self.client.request_status(), 1),
            (lambda: self.client.ping(10, 20), 3),
            (lambda: self.client.set_driver_enabled(True), 2),
            (lambda: self.client.home_axis("A"), 2),
            (lambda: self.client.move_axis("C", 100), 6),
            (lambda: self.client.set_laser(1, 4095), 4),
            (lambda: self.client.set_fan(2), 2),
            (lambda: self.client.set_speed_level(3), 2),
            (lambda: self.client.set_axis_accel("Z", 300.0), 6),
            (lambda: self.client.move_sync(800, -400, 1000), 8),
        ]

        for send, expected_dlc in cases:
            with self.subTest(expected_dlc=expected_dlc):
                self.assertTrue(send())
                self.assertEqual(len(self.client.frames[-1][1]), expected_dlc)

    def test_move_sync_encodes_physical_units_in_one_trigger_frame(self):
        self.assertTrue(self.client.move_sync(800, -400, 1000, force_no_encoder=True))
        self.assertEqual(len(self.client.frames), 1)

        _, payload, _ = self.client.frames[0]
        opcode, angle_c_deci, angle_a_deci, distance_z_centi, flags = struct.unpack(
            "<BhhhB", payload
        )
        self.assertEqual(opcode, CanOpcode.MOVE_SYNC)
        self.assertEqual(angle_c_deci, 900)
        self.assertEqual(angle_a_deci, -450)
        self.assertEqual(distance_z_centi, 1000)
        self.assertEqual(flags, 0x01)

    def test_move_sync_sends_axis_profiles_before_trigger_when_requested(self):
        self.assertTrue(
            self.client.move_sync(
                800,
                -400,
                1000,
                speed_c=140.0,
                speed_a=150.0,
                speed_z=12.5,
                accel=300.0,
            )
        )
        self.assertEqual(len(self.client.frames), 4)
        self.assertEqual([frame[1][0] for frame in self.client.frames[:3]], [
            CanOpcode.MOVE_PROFILE,
            CanOpcode.MOVE_PROFILE,
            CanOpcode.MOVE_PROFILE,
        ])
        self.assertEqual(self.client.frames[-1][1][0], CanOpcode.MOVE_SYNC)
        self.assertEqual(len(self.client.frames[-1][1]), 8)


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


class TeensySerialProtocolTests(unittest.TestCase):
    def setUp(self):
        self.state = DeviceState()
        self.client = RecordingTeensyClient(self.state, node_id=3)

    def test_high_level_commands_use_teensy_compact_protocol(self):
        self.assertTrue(self.client.request_status())
        self.assertTrue(self.client.ping(10, 20))
        self.assertTrue(self.client.set_driver_enabled(True))
        self.assertTrue(self.client.home_axis("X"))
        self.assertTrue(self.client.move_axis("Y", -400))
        self.assertTrue(self.client.move_axis("Z", 500, force_no_encoder=True))
        self.assertTrue(self.client.set_speed_level(4))
        self.assertTrue(self.client.set_laser(2, 4095))
        self.assertTrue(self.client.set_fan(2))

        self.assertEqual(
            self.client.commands,
            [
                "R 3",
                "P 3 10 20",
                "E 3 1",
                "H 3 C",
                "M 3 A -400",
                "MF 3 Z 500",
                "S 3 4",
                "L 3 2 4095",
                "F 3 2",
            ],
        )

    def test_teensy_does_not_send_esp32_only_profile_commands(self):
        self.assertFalse(self.client.move_axis("C", 100, speed=200.0))
        self.assertFalse(self.client.set_axis_accel("A", 500.0))
        self.assertEqual(self.client.commands, [])

    def test_move_sync_uses_teensy_physical_command(self):
        self.assertTrue(
            self.client.move_sync(
                800,
                -400,
                1000,
                speed_c=140.0,
                speed_a=140.0,
                speed_z=12.5,
                force_no_encoder=True,
            )
        )
        self.assertEqual(self.client.commands, ["MSF 3 90.0 -45.0 10.00"])

    def test_status_and_position_lines_update_selected_node(self):
        self.client._parse_response_line("STATUS 3 61 4095 2048 1 2 4")
        self.client._parse_response_line("POS 3 120.45 89.90 1500 28.5")

        telemetry = self.state.telemetry
        self.assertTrue(telemetry.drivers_enabled)
        self.assertFalse(telemetry.z_bloqueado)
        self.assertTrue(telemetry.alarme_z_ativo)
        self.assertTrue(telemetry.tmc_uart_ready)
        self.assertTrue(telemetry.can_online)
        self.assertEqual(telemetry.laser1_level, 4095)
        self.assertEqual(telemetry.laser2_level, 2048)
        self.assertTrue(telemetry.fan_output_on)
        self.assertEqual(telemetry.fan_mode.value, 2)
        self.assertEqual(telemetry.speed_level, 4)
        self.assertTrue(telemetry.pos_c_valid)
        self.assertTrue(telemetry.pos_a_valid)
        self.assertAlmostEqual(telemetry.pos_c_deg, 120.45)
        self.assertAlmostEqual(telemetry.pos_a_deg, 89.90)
        self.assertEqual(telemetry.pos_z_steps, 1500)
        self.assertTrue(telemetry.temp_valid)
        self.assertAlmostEqual(telemetry.temperature_c, 28.5)

        self.client._parse_response_line("POS 3 -1.00 -1.00 0 -99.9")
        self.assertFalse(telemetry.pos_c_valid)
        self.assertFalse(telemetry.pos_a_valid)
        self.assertFalse(telemetry.temp_valid)

    def test_telemetry_from_other_node_does_not_replace_selected_node(self):
        self.client._parse_response_line("STATUS 4 63 100 200 1 1 5")
        self.client._parse_response_line("POS 4 10.00 20.00 300 25.0")

        telemetry = self.state.telemetry
        self.assertEqual(telemetry.laser1_level, 0)
        self.assertFalse(telemetry.pos_c_valid)
        self.assertEqual(telemetry.pos_z_steps, 0)

    def test_can_error_line_is_decoded(self):
        errors = []
        self.state.error_occurred.connect(errors.append)

        self.client._parse_response_line("ERROR 3 20 03")

        self.assertEqual(self.state.telemetry.error_count, 1)
        self.assertEqual(len(errors), 1)
        self.assertIn("MOVE", errors[0])
        self.assertIn("ESP_ERR_INVALID_STATE", errors[0])


if __name__ == "__main__":
    unittest.main()
