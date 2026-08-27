#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_high_bandwidth_pi_design as design


class Stage2SpeedHighBandwidthPiDesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenarios = design.load_plant_scenarios(
            design.DEFAULT_LOW_SOURCE, design.DEFAULT_HIGH_SOURCE,
        )
        cls.report = design.build_report(
            design.DEFAULT_LOW_SOURCE, design.DEFAULT_HIGH_SOURCE,
        )

    def test_model_has_four_bias_scenarios_and_measured_high_points(self):
        self.assertEqual(4, len(self.scenarios))
        self.assertEqual(
            set(design.SCENARIO_BIASES_RPM),
            {item["bias_rpm"] for item in self.scenarios},
        )
        for scenario in self.scenarios:
            self.assertEqual(
                {40.0, 50.0, 60.0, 70.0, 80.0, 100.0},
                {frequency for frequency in scenario["points"] if frequency >= 40.0},
            )

    def test_report_generates_six_positive_finite_candidates(self):
        candidates = self.report["candidates"]
        self.assertEqual(6, len(candidates))
        self.assertEqual(
            set(design.TARGET_GAIN_CROSSOVERS_HZ),
            {item["target_gain_crossover_hz"] for item in candidates},
        )
        for candidate in candidates:
            self.assertTrue(math.isfinite(candidate["kp"]))
            self.assertTrue(math.isfinite(candidate["ki"]))
            self.assertGreater(candidate["kp"], 0.0)
            self.assertGreater(candidate["ki"], 0.0)

    def test_nominal_gain_crossover_hits_each_target(self):
        for candidate in self.report["candidates"]:
            self.assertAlmostEqual(
                candidate["target_gain_crossover_hz"],
                candidate["nominal_metrics"]["gain_crossover_hz"],
                delta=0.02,
            )

    def test_all_six_candidates_pass_the_frozen_preflight(self):
        self.assertEqual("six_candidates_preflight_passed", self.report["status"])
        self.assertTrue(all(
            item["preflight_pass"] for item in self.report["candidates"]
        ))

    def test_report_does_not_claim_hardware_or_default_change(self):
        self.assertFalse(self.report["hardware_executed"])
        self.assertFalse(self.report["default_pi_changed"])


if __name__ == "__main__":
    unittest.main()
