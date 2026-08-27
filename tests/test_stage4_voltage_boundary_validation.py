import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage4_voltage_boundary_validation as stage4


class DynamicVoltageBoundaryTests(unittest.TestCase):
    def test_plan_is_minimal_and_counterbalanced(self):
        plan = stage4.experiment_plan()
        self.assertEqual(plan["visit_count"], 4)
        self.assertEqual(plan["scope_capture_count"], 4)
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 6500.0)
        self.assertEqual(
            [(item["mode"], item["direction"]) for item in plan["visits"]],
            [("FEEDBACK", 1), ("HYBRID", -1),
             ("HYBRID", 1), ("FEEDBACK", -1)],
        )
        self.assertEqual(plan["high_voltage_ratio"], 1.0)
        self.assertEqual(plan["low_voltage_ratio"], 0.45)

    def test_scope_summary_finds_entry_tracking_and_release(self):
        metadata = {"base_sample_hz": 40000, "divider": 10}
        samples = []
        electrical_speed = 6500.0*2.0*math.pi/60.0*10.0
        target = 24.0/math.sqrt(3.0)*0.45*0.95
        for index in range(4096):
            time_s = index*0.00025
            active = (stage4.BOUNDARY_DOWN_SEC <= time_s
                      < stage4.BOUNDARY_UP_SEC)
            id_ref = -1.2 if active else 0.0
            iq_ref = 0.5
            uq = target if active else 6.2
            samples.append((
                0.0, 0.0, iq_ref, id_ref, iq_ref, id_ref,
                uq, 0.0, 0.0, electrical_speed, electrical_speed, 1.0,
            ))
        summary = stage4.summarize_scope(
            "HYBRID", 1, metadata, samples, 24.0
        )
        summary["scope_csv"] = "synthetic.csv"
        self.assertAlmostEqual(summary["low_voltage_error_mean_v"], 0.0, places=6)
        self.assertLess(summary["low_id_reference_mean_a"], -1.0)
        self.assertAlmostEqual(summary["tail_id_reference_mean_a"], 0.0)
        self.assertAlmostEqual(summary["activation_time_ms"], 0.0)
        self.assertAlmostEqual(summary["release_time_ms"], 0.0)
        self.assertTrue(stage4.qualify([summary])["passed"])

    def test_voltage_settling_uses_window_mean_with_ripple(self):
        values = [
            (index*0.00025, 0.30 if index % 2 == 0 else -0.30)
            for index in range(120)
        ]
        self.assertEqual(
            stage4.first_window_mean(values, 0.0, lambda value: abs(value) <= 0.25),
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
