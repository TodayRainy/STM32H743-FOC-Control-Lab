import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage4_field_weakening_validation as stage4


class FieldWeakeningValidationTests(unittest.TestCase):
    def test_plan_is_counterbalanced_and_separates_main_from_extension(self):
        main = stage4.experiment_plan("main")
        extension = stage4.experiment_plan("extension")
        self.assertEqual(main["visit_count"], 12)
        self.assertEqual(main["scope_capture_count"], 12)
        self.assertEqual(main["maximum_commanded_speed_rpm"], 5000.0)
        self.assertEqual(extension["maximum_commanded_speed_rpm"], 6500.0)
        self.assertEqual(
            [item["mode"] for item in main["visits"][:6]],
            ["OFF", "OFF", "FEEDBACK", "FEEDBACK", "HYBRID", "HYBRID"],
        )
        self.assertEqual(
            [item["mode"] for item in main["visits"][6:]],
            ["HYBRID", "HYBRID", "FEEDBACK", "FEEDBACK", "OFF", "OFF"],
        )

    def test_scope_summary_extracts_voltage_and_current_coordination(self):
        metadata = {"base_sample_hz": 40000, "divider": 10}
        samples = []
        for index in range(4096):
            time_s = index*0.00025
            active = stage4.PULSE_START_SEC <= time_s < stage4.PULSE_END_SEC
            id_ref = -1.0 if active else 0.0
            iq_ref = math.sqrt(25.0-id_ref*id_ref) if active else 0.2
            target_v = 24.0/math.sqrt(3.0)*0.35*0.95
            uq = target_v if active else 4.0
            samples.append((
                0.0, 0.0, iq_ref, id_ref, iq_ref, id_ref,
                uq, 0.0, 0.0, 5000.0, 5000.0, 1.0,
            ))
        summary = stage4.summarize_scope(
            "FEEDBACK", metadata, samples, 24.0, 0.35
        )
        self.assertAlmostEqual(summary["high_voltage_error_mean_v"], 0.0, places=6)
        self.assertAlmostEqual(summary["maximum_reference_current_a"], 5.0, places=6)
        self.assertGreaterEqual(summary["minimum_q_reference_margin_a"], -1.0e-6)
        self.assertLess(summary["high_id_reference_mean_a"], -0.9)
        self.assertAlmostEqual(summary["tail_id_reference_mean_a"], 0.0)


if __name__ == "__main__":
    unittest.main()
