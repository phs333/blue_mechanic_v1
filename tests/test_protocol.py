import struct
import unittest

import can

from python_app.core.can_client import CanClient
from python_app.core.protocol_defs import (
    CanOpcode,
    calc_ca_degrees_per_step,
    calc_ca_steps_for_degrees,
    calc_z_steps_for_mm,
    calc_z_mm_for_steps,
)
from python_app.core.serial_client import SerialClient
from python_app.core.state_model import DeviceState
from python_app.core.teensy_serial_client import TeensySerialClient
from python_app.core.test_automation import parse_script


class RecordingCanClient(CanClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.frames = []

    def send_frame(self, arbitration_id, data, desc="", log=True):
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
        self.assertAlmostEqual(calc_z_mm_for_steps(1000, 16, 200, 16), 10.0)
        self.assertAlmostEqual(calc_z_mm_for_steps(50000, 16, 200, 16), 500.0)


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
        opcode, angle_c_deci, angle_a_deci, distance_z_units, flags = struct.unpack(
            "<BhhhB", payload
        )
        self.assertEqual(opcode, CanOpcode.MOVE_SYNC)
        self.assertEqual(angle_c_deci, 900)
        self.assertEqual(angle_a_deci, -450)
        self.assertEqual(distance_z_units, 625)  # 1000 passos = 12,5 mm, em unidades de 0,02 mm
        self.assertEqual(flags, 0x01)

    def test_move_sync_reaches_full_z_travel(self):
        # 38400 passos = 480 mm: nao cabia no int16 de 0,01 mm (max 327,67 mm)
        self.assertTrue(self.client.move_sync(0, 0, 38400))
        _, payload, _ = self.client.frames[-1]
        _, _, _, distance_z_units, _ = struct.unpack("<BhhhB", payload)
        self.assertEqual(distance_z_units, 24000)

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
        self.assertEqual(self.client.commands, ["MSF 3 90.0 -45.0 12.50"])

    def test_move_sync_full_z_travel_via_teensy(self):
        self.assertTrue(self.client.move_sync(0, 0, 38400))
        self.assertEqual(self.client.commands[-1], "MS 3 0.0 0.0 480.00")

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

    def test_teensy_broadcast_mode_targets_node_zero_for_actuation_only(self):
        self.client.set_broadcast_mode(True)
        self.assertTrue(self.client.broadcast_mode)
        self.assertEqual(self.client.target_node, 0)
        self.assertEqual(self.client.node_id, 3)

        self.assertTrue(self.client.set_driver_enabled(True))
        self.assertTrue(self.client.home_axis("C"))
        self.assertTrue(self.client.move_axis("A", 400))
        self.assertTrue(self.client.move_axis("Z", -500, force_no_encoder=True))
        self.assertTrue(self.client.move_sync(800, -400, 1000))
        self.assertTrue(self.client.set_laser(1, 4095))
        self.assertTrue(self.client.set_fan(2))
        self.assertTrue(self.client.set_speed_level(5))

        # Status and ping MUST retain specific node 3 to prevent CAN flood / TEENSY_ERROR
        self.assertTrue(self.client.request_status())
        self.assertTrue(self.client.ping(10, 20))

        self.assertEqual(
            self.client.commands,
            [
                "E 0 1",
                "H 0 C",
                "M 0 A 400",
                "MF 0 Z -500",
                "MS 0 90.0 -45.0 12.50",
                "L 0 1 4095",
                "F 0 2",
                "S 0 5",
                "R 3",
                "P 3 10 20",
            ],
        )

    def test_automation_script_parser_and_broadcast_override(self):
        script = """
        # Test script
        E {node} 1
        WAIT 500
        M 2 C 800
        DELAY 1.5s
        MS 1 45.0 -30.0 10.0
        """
        # Parsing with force_broadcast=True
        steps = parse_script(script, default_delay_ms=400, force_broadcast=True, default_node=1)

        # Comments are retained as COMMENT steps
        comments = [s for s in steps if s.step_type == "COMMENT"]
        self.assertEqual(len(comments), 1)

        exec_steps = [s for s in steps if s.step_type in ("COMMAND", "DELAY")]
        self.assertEqual(len(exec_steps), 5)

        # Check commands have node replaced with 0
        self.assertEqual(exec_steps[0].command, "E 0 1")
        self.assertEqual(exec_steps[1].step_type, "DELAY")
        self.assertEqual(exec_steps[1].delay_ms, 500)
        self.assertEqual(exec_steps[2].command, "M 0 C 800")
        self.assertEqual(exec_steps[3].step_type, "DELAY")
        self.assertEqual(exec_steps[3].delay_ms, 1500)
        self.assertEqual(exec_steps[4].command, "MS 0 45.0 -30.0 10.0")

    def test_automation_script_preserves_explicit_nodes_by_default(self):
        # Exactly the user's script
        script = """
        L 1 2 300
        WAIT 500
        L 2 2 300
        WAIT 500
        L 3 2 300
        WAIT 500
        L 4 2 300
        WAIT 500
        L 0 2 0
        WAIT 500
        """
        # force_broadcast=False is the default
        steps = parse_script(script, default_delay_ms=500, force_broadcast=False, default_node=1)
        commands = [s.command for s in steps if s.step_type == "COMMAND"]

        self.assertEqual(
            commands,
            [
                "L 1 2 300",
                "L 2 2 300",
                "L 3 2 300",
                "L 4 2 300",
                "L 0 2 0",
            ],
        )


class RecordingSerialClient(SerialClient):
    def __init__(self, state: DeviceState):
        super().__init__(state)
        self.commands = []
        self.is_connected = True

    def send_raw(self, cmd: str) -> bool:
        self.commands.append(cmd)
        return True


class SerialProtocolTests(unittest.TestCase):
    def setUp(self):
        self.state = DeviceState()
        self.client = RecordingSerialClient(self.state)

    def test_serial_home_routing_ca_and_all(self):
        self.client.home_axis("CA")
        self.assertEqual(self.client.commands, ["HOME C", "HOME A"])

        self.client.commands.clear()
        self.client.home_axis("ALL")
        self.assertEqual(self.client.commands, ["HOME C", "HOME A", "HOME Z"])

        self.client.commands.clear()
        self.client.home_axis("Z")
        self.assertEqual(self.client.commands, ["HOME Z"])

    def test_serial_set_home_command(self):
        self.state.telemetry.pos_c_deg = 123.45
        self.state.telemetry.pos_a_deg = 67.89
        self.client.set_home("C")
        self.assertEqual(self.state.telemetry.pos_c_deg, 0.0)
        self.client.set_home("A")
        self.assertEqual(self.state.telemetry.pos_a_deg, 0.0)
        self.assertEqual(self.client.commands, ["SETHOME C", "SETHOME A"])

        # Also test response line parsing
        self.state.telemetry.pos_c_deg = 99.99
        self.client._parse_response_line("Home C gravado em 0.00 deg (posicao atual zerada, voltas resetadas na NVS).")
        self.assertEqual(self.state.telemetry.pos_c_deg, 0.0)

    def test_serial_config_dump_with_negative_limits_and_large_z(self):
        lines = [
            "=== CONFIG DUMP ===",
            "CONFIG STEPS C=200 A=200 Z=200",
            "CONFIG SPEED C=140.62 A=140.62 Z=12.50",
            "CONFIG SPEED_MAX C=720.00 A=720.00 Z=60.00",
            "CONFIG ACCEL_MAX C=3600.00 A=3600.00 Z=800.00",
            "CONFIG ACCEL C=1800.00 A=1800.00 Z=300.00",
            "CONFIG INVERT C=0 A=0 Z=1",
            "CONFIG PULLEY_Z=16 MAX_Z_MM=1600.00 MAX_PASSOS_Z=160000",
            "CONFIG LIMITS C=-540.00..540.00 A=-540.00..540.00",
            "CONFIG DRIVER_BUS_MODE=1",
            "CONFIG TMC C addr=0 ihold=359 irun=897 delay=6 usteps=16 spread=0",
            "CONFIG TMC A addr=1 ihold=359 irun=897 delay=6 usteps=16 spread=0",
            "CONFIG TMC Z addr=2 ihold=418 irun=957 delay=6 usteps=16 spread=0",
            "CONFIG CAN node=1 bitrate=500000 cmd=0x200 status=0x280 event=0x300",
            "===================",
        ]
        for l in lines:
            self.client._parse_response_line(l)

        p = self.state.parameters
        self.assertEqual(p.z_pulley_teeth, 16)
        self.assertEqual(p.max_passos_z, 160000)
        self.assertEqual(self.state.telemetry.max_z_steps, 160000)
        self.assertAlmostEqual(p.limit_min_deg_c, -540.00)
        self.assertAlmostEqual(p.limit_max_deg_c, 540.00)
        self.assertAlmostEqual(p.limit_min_deg_a, -540.00)
        self.assertAlmostEqual(p.limit_max_deg_a, 540.00)
        self.assertAlmostEqual(p.speed[0], 140.62)
        self.assertAlmostEqual(p.speed[1], 140.62)
        self.assertAlmostEqual(p.speed[2], 12.50)


    def test_serial_can_enabled_parsing_and_commands(self):
        self.client.set_can_enabled(True)
        self.assertEqual(self.client.commands[-1], "CAN ON")
        self.assertTrue(self.state.parameters.can_enabled)

        self.client.set_can_enabled(False)
        self.assertEqual(self.client.commands[-1], "CAN OFF")
        self.assertFalse(self.state.parameters.can_enabled)

        # Parse CONFIG CAN with enabled=1
        self.client._parse_response_line("CONFIG CAN enabled=1 node=3 bitrate=250000 cmd=0x210 status=0x290 event=0x310")
        self.assertTrue(self.state.parameters.can_enabled)
        self.assertEqual(self.state.parameters.node_id, 3)
        self.assertEqual(self.state.parameters.can_bitrate, 250000)

        # Parse CONFIG CAN with enabled=0
        self.client._parse_response_line("CONFIG CAN enabled=0 node=5 bitrate=500000 cmd=0x200 status=0x280 event=0x300")
        self.assertFalse(self.state.parameters.can_enabled)
        self.assertEqual(self.state.parameters.node_id, 5)

        # Parse backward compatible without enabled
        self.client._parse_response_line("CONFIG CAN node=7 bitrate=1000000 cmd=0x200 status=0x280 event=0x300")
        self.assertEqual(self.state.parameters.node_id, 7)
        self.assertEqual(self.state.parameters.can_bitrate, 1000000)

    def test_wheel_focus_filter_behavior(self):
        from PyQt6.QtWidgets import QApplication, QSpinBox, QComboBox, QScrollArea, QWidget, QVBoxLayout
        from PyQt6.QtCore import QPoint, QPointF, Qt
        from PyQt6.QtGui import QWheelEvent
        from python_app.ui.wheel_filter import WheelFocusFilter

        app = QApplication.instance() or QApplication([])
        scroll = QScrollArea()
        container = QWidget()
        vbox = QVBoxLayout(container)
        sp = QSpinBox()
        sp.setValue(20)
        vbox.addWidget(sp)
        scroll.setWidget(container)
        scroll.show()

        filt = WheelFocusFilter(app)
        app.installEventFilter(filt)

        wheel_up = QWheelEvent(
            QPointF(5, 5), QPointF(5, 5),
            QPoint(0, 0), QPoint(0, 120),
            Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
            Qt.ScrollPhase.NoScrollPhase, False
        )

        # Unfocused: value must NOT change!
        app.sendEvent(sp, wheel_up)
        self.assertEqual(sp.value(), 20)

        # Focused: value changes
        app.setActiveWindow(sp)
        sp.setFocus()
        app.sendEvent(sp, wheel_up)
        self.assertEqual(sp.value(), 21)

        app.removeEventFilter(filt)
        scroll.close()
        scroll.deleteLater()
        QApplication.processEvents()

    def test_laser_slider_throttling_and_telemetry_grace(self):
        import time
        from PyQt6.QtWidgets import QApplication
        from python_app.ui.widgets.laser_slider import LaserSlider

        app = QApplication.instance() or QApplication(["--platform", "offscreen"])
        slider = LaserSlider(1, "Test Laser")
        emitted = []
        slider.laser_level_changed.connect(lambda idx, val: emitted.append((idx, val)))

        # Direct preset
        slider.set_percent(50)
        self.assertEqual(slider.current_percent, 50)
        self.assertTrue(len(emitted) > 0)
        emitted.clear()

        # Telemetry arriving immediately within grace period (< 0.8s) is ignored
        slider.update_from_telemetry(0)
        self.assertEqual(slider.current_percent, 50) # Not overridden!

        # Manually backdate _last_user_time past grace period
        slider._last_user_time = time.time() - 2.0
        slider.update_from_telemetry(0)
        self.assertEqual(slider.current_percent, 0) # Now accepted!

        slider.deleteLater()
        QApplication.processEvents()

    def test_translate_teensy_to_serial_move_variants(self):
        from python_app.core.protocol_defs import translate_teensy_to_serial

        # Format MSM with target node
        self.assertEqual(translate_teensy_to_serial("MSM 0 Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("MSM 1 C -200"), ["MOVE C -200"])
        self.assertEqual(translate_teensy_to_serial("MSMF 0 Z 400"), ["MOVE_F Z 400"])

        # Format MSM without target node
        self.assertEqual(translate_teensy_to_serial("MSM Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("MSM A 150"), ["MOVE A 150"])
        self.assertEqual(translate_teensy_to_serial("MSMF Z -300"), ["MOVE_F Z -300"])

        # Standard M / MF with or without node
        self.assertEqual(translate_teensy_to_serial("M 0 Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("M Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("MF 0 Z 400"), ["MOVE_F Z 400"])
        self.assertEqual(translate_teensy_to_serial("MF Z 400"), ["MOVE_F Z 400"])

        # Native MOVE / MOVE_F with or without node
        self.assertEqual(translate_teensy_to_serial("MOVE 0 Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("MOVE Z 400"), ["MOVE Z 400"])
        self.assertEqual(translate_teensy_to_serial("MOVE_F 0 Z 400"), ["MOVE_F Z 400"])

        # With speed and accel suffixes
        self.assertEqual(translate_teensy_to_serial("MSM 0 C 400 S=100 F=500"), ["MOVE C 400 S=100 F=500"])

    def test_multi_axis_ramp_speed_serial_and_parser(self):
        state = DeviceState()
        client = SerialClient(state)
        sent = []
        client.send_raw = lambda cmd: sent.append(cmd) or True

        # Test setting ramp speeds
        client.set_axis_ramp_speed("C", 12.5)
        self.assertEqual(sent[-1], "RAMP C 12.50")
        self.assertEqual(state.parameters.c_start_speed_deg, 12.5)

        client.set_axis_ramp_speed("A", 25.0)
        self.assertEqual(sent[-1], "RAMP A 25.00")
        self.assertEqual(state.parameters.a_start_speed_deg, 25.0)

        client.set_axis_ramp_speed("Z", 18.0)
        self.assertEqual(sent[-1], "RAMP Z 18.00")
        self.assertEqual(state.parameters.z_start_speed_mm, 18.0)

        # Test parsing CONFIG RAMP with multi-axis values
        client._parse_response_line("CONFIG RAMP_C=14.50 RAMP_A=22.30 RAMP_Z=35.00")
        self.assertEqual(state.parameters.c_start_speed_deg, 14.50)
        self.assertEqual(state.parameters.a_start_speed_deg, 22.30)
        self.assertEqual(state.parameters.z_start_speed_mm, 35.00)

    def test_simulator_multi_axis_ramp_speed(self):
        from python_app.core.simulator import SimulatorClient
        state = DeviceState()
        sim = SimulatorClient(state)

        sim.set_axis_ramp_speed("C", 8.0)
        sim.set_axis_ramp_speed("A", 12.0)
        sim.set_axis_ramp_speed("Z", 20.0)

        self.assertEqual(state.parameters.c_start_speed_deg, 8.0)
        self.assertEqual(state.parameters.a_start_speed_deg, 12.0)
        self.assertEqual(state.parameters.z_start_speed_mm, 20.0)

        # Test sending command through send_raw
        sim.send_raw("RAMP C 16.5")
        self.assertEqual(state.parameters.c_start_speed_deg, 16.5)


if __name__ == "__main__":
    unittest.main()

