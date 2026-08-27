from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_position_endpoint_repeatability as d051


class Stage2PositionEndpointRepeatabilityTests(unittest.TestCase):
    def test_plan_is_small_and_does_not_flash(self):
        plan = d051.plan()
        self.assertEqual(plan["record_count"], 48)
        self.assertEqual(len(plan["centers_deg_from_start"]), 6)
        self.assertEqual(plan["capture_sec"], 2.5)
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["finally_stop_ready_and_restore_defaults"])

    def test_endpoint_qualification_uses_three_simple_limits(self):
        passing = {
            "endpoint_overshoot_deg": 0.5,
            "settling_after_reference_sec": 0.2,
            "steady_error_deg": -0.2,
        }
        self.assertTrue(d051.endpoint_qualified(passing))
        for field, value in (
            ("endpoint_overshoot_deg", 0.501),
            ("settling_after_reference_sec", 0.201),
            ("steady_error_deg", 0.201),
        ):
            record = dict(passing)
            record[field] = value
            self.assertFalse(d051.endpoint_qualified(record))

    def test_repeat_summary_requires_two_visits(self):
        base = {
            "center_offset_deg": 0.0,
            "transition": "+0_to_+10",
            "endpoint_overshoot_deg": 0.2,
            "steady_error_deg": 0.1,
            "hold_encoder_speed_ripple_rms_rpm": 2.0,
            "endpoint_qualified": True,
        }
        second = dict(base)
        second.update({
            "endpoint_overshoot_deg": 0.3,
            "steady_error_deg": -0.1,
            "hold_encoder_speed_ripple_rms_rpm": 2.5,
        })
        summary = d051.paired_repeat_summary([base, second])
        self.assertEqual(summary["pair_count"], 1)
        self.assertAlmostEqual(
            summary["maximum_endpoint_overshoot_repeat_delta_deg"], 0.1
        )
        with self.assertRaises(RuntimeError):
            d051.paired_repeat_summary([base])


if __name__ == "__main__":
    unittest.main()
