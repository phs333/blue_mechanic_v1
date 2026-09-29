import math
import re
import time
import unittest

from python_app.core.protocol_defs import translate_teensy_to_serial
from python_app.core.state_model import DeviceState
from python_app.core.test_automation import (
    AUTOMATION_PRESETS,
    AutomationWorker,
    ScriptExpansionError,
    ScriptKinematics,
    _turret_aim,
    expand_automation_script,
    parse_script,
)

KIN = ScriptKinematics()  # 200 passos x 16 micropassos = 0,1125 °/passo


def _moves(expanded: str):
    out = []
    for line in expanded.splitlines():
        m = re.match(r"^MS \S+ (\S+) (\S+) (\S+)(.*)$", line)
        if m:
            opts = dict(kv.split("=") for kv in m.group(4).split())
            out.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), opts))
    return out


def _waits(expanded: str):
    return [float(m.group(1)) for m in re.finditer(r"^WAIT (\S+)$", expanded, re.MULTILINE)]


class TurretGeometryTests(unittest.TestCase):
    def test_beam_projects_circle_of_requested_diameter(self):
        dist, diam = 3.0, 1.0
        theta = math.atan(diam / 2 / dist)
        for elev in (0.0, 20.0):
            for k in range(36):
                t = 2 * math.pi * k / 36
                az, el = _turret_aim(theta, t, elev)
                az_r, el_r = math.radians(az), math.radians(el + elev)
                beam = (math.cos(el_r) * math.cos(az_r), math.cos(el_r) * math.sin(az_r), math.sin(el_r))
                e0 = math.radians(elev)
                axis = (math.cos(e0), 0.0, math.sin(e0))
                # Interseção com o plano perpendicular ao eixo central a 3 m
                along = sum(b * a for b, a in zip(beam, axis))
                hit = [b * dist / along for b in beam]
                center = [a * dist for a in axis]
                radius = math.dist(hit, center)
                self.assertAlmostEqual(radius, diam / 2, places=9)


class TurretMacroTests(unittest.TestCase):
    def test_lap_closes_exactly_in_steps(self):
        text = expand_automation_script("TURRET_CIRCLE 0 3.0 1.0 4 72 3", kinematics=KIN)
        moves = _moves(text)
        self.assertEqual(len(moves), 72 * 3)
        for lap in range(3):
            lap_moves = moves[lap * 72:(lap + 1) * 72]
            sc = sum(round(m[0] / KIN.deg_per_step_c) for m in lap_moves)
            sa = sum(round(m[1] / KIN.deg_per_step_a) for m in lap_moves)
            self.assertEqual((sc, sa), (0, 0))
        # Todo deslocamento é um número inteiro de passos (sem arredondamento na tradução)
        for dc, da, dz, _ in moves:
            self.assertAlmostEqual(dc / KIN.deg_per_step_c, round(dc / KIN.deg_per_step_c), places=6)
            self.assertAlmostEqual(da / KIN.deg_per_step_a, round(da / KIN.deg_per_step_a), places=6)
            self.assertEqual(dz, 0.0)

    def test_segment_speeds_give_constant_duration(self):
        moves = _moves(expand_automation_script("TURRET_CIRCLE 0 3 1 4 72 1", kinematics=KIN))
        seg_s = 4 / 72
        for dc, da, _, opts in moves:
            # O planejador usa a menor taxa entre os eixos: cada eixo leva exatamente seg_s
            for delta, key in ((dc, "SC"), (da, "SA")):
                if delta:
                    self.assertAlmostEqual(abs(delta) / float(opts[key]), seg_s, delta=2e-4)
                else:
                    self.assertNotIn(key, opts)

    def test_feed_timeline_matches_motion_time(self):
        text = expand_automation_script("TURRET_CIRCLE 0 3 1 4 72 2", kinematics=KIN)
        waits = _waits(text)
        seg_ms = 4000 / 72
        total = 72 * 2
        # (total - prefill) esperas de um segmento + a espera final da fila
        self.assertEqual(len(waits), total - 3 + 1)
        self.assertAlmostEqual(sum(waits[:-1]), (total - 3) * seg_ms, delta=0.01 * total)
        self.assertGreaterEqual(waits[-1], 3 * seg_ms)
        # Os 3 primeiros segmentos saem juntos (fila à frente para o lookahead)
        lines = [l for l in text.splitlines() if not l.startswith("#")]
        self.assertTrue(all(l.startswith("MS ") for l in lines[:3]))

    def test_start_and_end_moves_mirror_first_point(self):
        start = _moves(expand_automation_script("TURRET_CIRCLE_START 0 3 1", kinematics=KIN))
        end = _moves(expand_automation_script("TURRET_CIRCLE_END 0 3 1", kinematics=KIN))
        theta = math.degrees(math.atan(0.5 / 3))
        self.assertAlmostEqual(start[0][0], theta, delta=KIN.deg_per_step_c)
        self.assertEqual(start[0][1], 0.0)
        self.assertEqual((end[0][0], end[0][1]), (-start[0][0], -start[0][1]))

    def test_translation_to_esp_serial_keeps_exact_steps_and_speeds(self):
        state = DeviceState()
        steps = parse_script("TURRET_CIRCLE 0 3 1 4 72 1", default_node=1,
                             kinematics=ScriptKinematics.from_parameters(state.parameters))
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        native = [translate_teensy_to_serial(c, state)[0] for c in cmds]
        self.assertTrue(all(n.startswith("MOVE_SYNC C ") for n in native))
        total_c = sum(int(re.search(r"MOVE_SYNC C (-?\d+)", n).group(1)) for n in native)
        total_a = sum(int(re.search(r" A (-?\d+)", n).group(1)) for n in native)
        self.assertEqual((total_c, total_a), (0, 0))
        self.assertIn("SC=", native[5])

    def test_invalid_arguments(self):
        for bad in ("TURRET_CIRCLE 0 3 1", "TURRET_CIRCLE 0 -3 1 4", "TURRET_CIRCLE 0 3 1 4 4",
                    "TURRET_CIRCLE_START 0 3"):
            with self.assertRaises(ScriptExpansionError):
                expand_automation_script(bad, kinematics=KIN)

    def test_preset_parses(self):
        steps = parse_script(AUTOMATION_PRESETS["9. Círculo de Laser 1 m a 3 m (Torre C + A)"])
        cmds = [s.command for s in steps if s.step_type == "COMMAND"]
        self.assertIn("MS 0 0 0 300", cmds)
        self.assertIn("L 0 1 300", cmds)
        self.assertIn("L 0 2 300", cmds)
        self.assertGreater(sum(c.startswith("MS 0 ") for c in cmds), 200)


class TimelineSleepTests(unittest.TestCase):
    def test_waits_do_not_accumulate_sleep_overshoot(self):
        worker = AutomationWorker(comm=None)
        worker._stop_requested = False
        t0 = time.monotonic()
        for _ in range(40):
            self.assertTrue(worker._sleep_cancellable(5.5))
        elapsed = time.monotonic() - t0
        # 40 x 5,5 ms = 220 ms; sem linha do tempo o Windows arredonda cada sleep para ~15 ms (600 ms)
        self.assertLess(elapsed, 0.30)
        self.assertGreaterEqual(elapsed, 0.219)


if __name__ == "__main__":
    unittest.main()
