#!/usr/bin/env python3
"""Plan checks for the guarded encoder speed-estimator hardware A/B."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import encoder_speed_estimation_validation as validation
import encoder_speed_estimation_biased_holdout as biased_holdout


class EncoderSpeedEstimatorValidationTests(unittest.TestCase):
    def test_plan_uses_two_reversed_rounds(self):
        plan = validation.plan()

        self.assertEqual(plan["visits"], 6)
        self.assertEqual(plan["round_orders"][0], tuple(reversed(plan["round_orders"][1])))
        self.assertEqual(plan["total_scope_captures"], 72)

    def test_plan_requires_explicit_hardware_and_restores_default(self):
        plan = validation.plan()

        self.assertTrue(plan["requires_24v"])
        self.assertTrue(plan["motor_will_rotate"])
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["finally_restore_default_and_stop"])

    def test_biased_holdout_rotates_below_the_safety_boundary(self):
        plan = biased_holdout.plan()

        self.assertEqual(plan["total_scope_captures"], 36)
        self.assertEqual(plan["speed_biases_rpm"], (1500.0, -1500.0))
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 1600.0)
        self.assertTrue(plan["requires_24v"])
        self.assertTrue(plan["finally_restore_default_and_stop"])


if __name__ == "__main__":
    unittest.main()
