from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_position_default_ab as d053


class Stage2PositionDefaultAbTests(unittest.TestCase):
    def test_plan_is_representative_and_does_not_flash(self):
        plan = d053.plan()
        self.assertEqual(plan["centers_deg_from_start"], (0.0, 120.0, 240.0))
        self.assertEqual(plan["records_per_profile"], 24)
        self.assertEqual(plan["record_count"], 48)
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["finally_stop_ready_and_restore_defaults"])

    def test_profiles_change_only_trajectory_policy(self):
        default, candidate = d053.PROFILES
        self.assertEqual(default["track_command"], "POS TRACK RAMP 180")
        self.assertFalse(default["position_velocity_feedforward"])
        self.assertEqual(candidate["track_command"], "POS TRACK TRAP 180 900")
        self.assertTrue(candidate["position_velocity_feedforward"])
        self.assertEqual(
            candidate["center_order_deg"], tuple(reversed(default["center_order_deg"]))
        )

    def test_nearest_value_selects_nearest_timestamp(self):
        values = [(0.10, 1.0), (0.20, 2.0), (0.30, 3.0)]
        self.assertEqual(d053.nearest_value(values, 0.18), 2.0)
        with self.assertRaises(RuntimeError):
            d053.nearest_value([], 0.18)


if __name__ == "__main__":
    unittest.main()
