#!/usr/bin/env python3
"""Host regression tests for the D-043 final speed-loop A/B plan."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_final_ab_validation as validation


class Stage2SpeedFinalAbValidationTests(unittest.TestCase):
    def test_plan_has_only_default_and_fc24(self):
        plan = validation.plan()

        self.assertEqual(
            [item["name"] for item in plan["controllers"]],
            ["source_default", "fc24_pm70"],
        )
        self.assertEqual(len(plan["visits"]), 4)
        self.assertNotEqual(
            plan["controllers"][0]["kp"], plan["controllers"][1]["kp"]
        )
        self.assertEqual(plan["scope_capture_count"], 32)
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["finally_stop_and_confirm_ready"])

    def test_wide_ramp_covers_both_directions_without_full_reversal(self):
        plan = validation.plan()
        targets = plan["ramp_targets_positive_first_rpm"]

        self.assertIn(4000.0, targets)
        self.assertIn(-4000.0, targets)
        self.assertEqual(targets[0], 0.0)
        self.assertEqual(targets[-1], 0.0)
        for first, second in zip(targets, targets[1:]):
            self.assertFalse(first == 4000.0 and second == -4000.0)
            self.assertFalse(first == -4000.0 and second == 4000.0)

    def test_steps_cover_saturation_recovery_and_high_speed_local_response(self):
        transitions = set(validation.STEP_TRANSITIONS)

        self.assertTrue({
            (0.0, 1500.0), (1500.0, 0.0),
            (0.0, -1500.0), (-1500.0, 0.0),
        }.issubset(transitions))
        self.assertTrue({
            (3000.0, 3300.0), (3000.0, 2700.0),
            (-3000.0, -3300.0), (-3000.0, -2700.0),
        }.issubset(transitions))

    def test_limit_duration_ends_when_iq_reference_leaves_limit(self):
        rows = [
            {"elapsed_sec": 0.00, "iq_ref": 4.0},
            {"elapsed_sec": 0.01, "iq_ref": 4.0},
            {"elapsed_sec": 0.02, "iq_ref": 4.0},
            {"elapsed_sec": 0.03, "iq_ref": 2.0},
            {"elapsed_sec": 0.04, "iq_ref": 4.0},
            {"elapsed_sec": 0.05, "iq_ref": 0.0},
        ]

        self.assertAlmostEqual(
            validation.longest_limit_duration(rows, 4.0), 0.02
        )

    def test_speed_command_conversion_matches_motor_telemetry_units(self):
        electrical = 1500.0 * 2.0 * validation.math.pi / 60.0 * 10

        self.assertAlmostEqual(
            validation.electrical_rad_s_to_rpm(electrical, 10), 1500.0
        )


if __name__ == "__main__":
    unittest.main()
