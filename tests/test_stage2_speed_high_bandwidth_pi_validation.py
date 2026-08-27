#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_high_bandwidth_pi_validation as validation


class Stage2SpeedHighBandwidthPiValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controllers = validation.load_controllers(
            validation.DEFAULT_ANCHOR_DESIGN, validation.DEFAULT_DESIGN,
        )

    def test_plan_has_two_anchors_six_candidates_and_448_captures(self):
        plan = validation.plan(self.controllers)

        self.assertEqual(8, len(plan["controllers"]))
        self.assertEqual(2, sum(
            item["role"] == "anchor" for item in plan["controllers"]
        ))
        self.assertEqual(6, sum(
            item["role"] == "candidate" for item in plan["controllers"]
        ))
        self.assertEqual(448, plan["scope_capture_count"])
        self.assertEqual(3200.0, plan["maximum_commanded_speed_rpm"])
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["supports_checkpoint_resume"])

    def test_visit_order_covers_every_controller_and_bias_once(self):
        visits = validation.visit_order(self.controllers)
        keys = {(item["name"], bias) for item, bias in visits}

        self.assertEqual(32, len(visits))
        self.assertEqual(32, len(keys))
        self.assertEqual(
            set(validation.SPEED_BIASES_RPM),
            {bias for _, bias in visits},
        )

    def test_crossing_interpolates_frequency_phase_and_snr(self):
        records = [
            {
                "frequency_hz": 20.0,
                "open_loop_magnitude_db": 2.0,
                "open_loop_phase_deg": -110.0,
                "measured_speed_fit": {"sine_snr_db": 10.0},
            },
            {
                "frequency_hz": 40.0,
                "open_loop_magnitude_db": -2.0,
                "open_loop_phase_deg": -130.0,
                "measured_speed_fit": {"sine_snr_db": 8.0},
            },
        ]

        result = validation.crossing(records, "open_loop_magnitude_db", 0.0)

        self.assertAlmostEqual(28.284271247, result["frequency_hz"], places=6)
        self.assertAlmostEqual(60.0, result["phase_margin_deg"], places=6)
        self.assertTrue(result["measurement_valid"])


if __name__ == "__main__":
    unittest.main()
