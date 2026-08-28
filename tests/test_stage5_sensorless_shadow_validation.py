import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage5_sensorless_shadow_validation as sensorless


def frame(esmo_angle=0.0, flux_angle=0.0, abz_angle=0.0,
          esmo_speed=0.0, flux_speed=0.0, abz_speed=0.0):
    values = [0.0]*36
    values[10] = esmo_angle
    values[11] = flux_angle
    values[12] = abz_angle
    values[16] = esmo_speed
    values[17] = flux_speed
    values[18] = abz_speed
    values[25] = abz_speed
    values[26] = 1.0
    values[27] = sensorless.MODE_SPEED_TORQUE_RUN
    values[29] = 24.0
    return tuple(values)


class SensorlessShadowTests(unittest.TestCase):
    def test_plan_keeps_abz_in_control(self):
        plan = sensorless.experiment_plan()
        self.assertEqual(plan["control_angle"], "ABZ")
        self.assertFalse(plan["observers_have_control_authority"])
        self.assertEqual(max(abs(value) for value in plan["speed_sequence_rpm"]),
                         6500.0)

    def test_circular_error_wraps(self):
        error = sensorless.circular_error(math.radians(1.0),
                                          math.radians(359.0))
        self.assertAlmostEqual(math.degrees(error), 2.0, places=6)

    def test_summary_uses_absolute_angle_and_speed_error(self):
        electrical_speed = 1500.0*2.0*math.pi/60.0*10.0
        samples = [frame(
            esmo_angle=math.radians(5.0),
            flux_angle=math.radians(2.0),
            abz_angle=0.0,
            esmo_speed=electrical_speed,
            flux_speed=electrical_speed,
            abz_speed=electrical_speed,
        ) for _ in range(20)]
        summary = sensorless.summarize_hold(1500.0, samples)
        self.assertAlmostEqual(
            summary["observers"]["esmo"]["angle_error_p95_abs_deg"],
            5.0,
        )
        self.assertTrue(summary["observers"]["active_flux"]
                        ["passed_initial_gate"])

    def test_minimum_reliable_requires_all_higher_points(self):
        holds = []
        for rpm, passed in ((500.0, False), (1000.0, True), (1500.0, True)):
            holds.append({
                "target_rpm": rpm,
                "observers": {
                    "esmo": {"passed_initial_gate": passed},
                    "active_flux": {"passed_initial_gate": True},
                },
            })
        for rpm, passed in ((-500.0, False), (-1000.0, True), (-1500.0, True)):
            holds.append({
                "target_rpm": rpm,
                "observers": {
                    "esmo": {"passed_initial_gate": passed},
                    "active_flux": {"passed_initial_gate": True},
                },
            })
        minimum = sensorless.minimum_reliable_speeds(holds)
        self.assertEqual(minimum["esmo"]["positive"], 1000.0)
        self.assertEqual(minimum["active_flux"]["negative"], 500.0)


if __name__ == "__main__":
    unittest.main()
