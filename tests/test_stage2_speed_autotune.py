#!/usr/bin/env python3
"""Host regression tests for the empirical speed-loop design core."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_autotune as autotune
import stage2_speed_autotune_validation as validation


class SpeedAutotuneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.spec = autotune.load_spec(REPO_ROOT / autotune.DEFAULT_SPEC)

    def test_frozen_bandwidth_guard_respects_both_separation_limits(self):
        control = self.spec["control"]
        theoretical = min(
            control["current_loop_robust_bandwidth_hz"]
            / control["current_to_speed_separation"],
            control["speed_loop_sample_hz"]
            / control["sample_to_bandwidth_separation"],
        )

        self.assertAlmostEqual(theoretical, 193.89639398662845)
        self.assertEqual(control["speed_bandwidth_guard_hz"], 190.0)
        self.assertLessEqual(control["speed_bandwidth_guard_hz"], theoretical)

    def test_p_scan_starts_at_half_amp_for_500_rpm_step(self):
        candidates = autotune.build_p_scan(self.spec)

        self.assertEqual(len(candidates), 6)
        expected_kp = 0.5 / autotune.rpm_to_electrical_rad_s(500.0, 10)
        self.assertAlmostEqual(candidates[0]["kp"], expected_kp)
        self.assertAlmostEqual(candidates[0]["predicted_initial_iq_a"], 0.5)
        self.assertAlmostEqual(
            candidates[-1]["kp"], expected_kp * math.pow(1.5, 5)
        )

    def test_pi_grid_contains_four_kp_and_two_integral_times(self):
        candidates = autotune.build_pi_candidates(self.spec, 0.002, 0.04)

        self.assertEqual(len(candidates), 8)
        self.assertEqual(
            {(item["kp_base_factor"], item["tau_63_factor"])
             for item in candidates},
            {(0.55, 16.0), (0.55, 24.0),
             (0.7, 16.0), (0.7, 24.0),
             (0.85, 16.0), (0.85, 24.0),
             (1.0, 16.0), (1.0, 24.0)},
        )
        for item in candidates:
            self.assertAlmostEqual(
                item["ki"], item["kp"] / item["integral_time_sec"]
            )

    def test_offline_report_never_claims_hardware_completion(self):
        report = autotune.build_report(
            REPO_ROOT / autotune.DEFAULT_SPEC, None, None
        )

        self.assertEqual(report["status"], "p_scan_ready")
        self.assertTrue(report["hardware"]["requires_24v"])
        self.assertFalse(report["hardware"]["automatic_flash"])
        self.assertTrue(report["hardware"]["finally_stop_and_confirm_ready"])

    def test_screen_plan_has_frozen_capture_count(self):
        plan = validation.plan(self.spec)

        self.assertEqual(plan["p_scope_count"], 12)
        self.assertEqual(plan["pi_scope_count"], 96)
        self.assertTrue(plan["requires_24v"])
        self.assertFalse(plan["automatic_flash"])

    def test_speed_step_metrics_recover_first_order_response(self):
        sample_period = 0.001
        pole_pairs = 10
        tau = 0.03
        target_rpm = 500.0
        samples = []
        speed_rpm = 0.0
        for index in range(1000):
            reference_rpm = 0.0 if index < 100 else target_rpm
            if index >= 100:
                speed_rpm += (reference_rpm - speed_rpm) * sample_period / tau
            elec_speed = autotune.rpm_to_electrical_rad_s(speed_rpm, pole_pairs)
            elec_reference = autotune.rpm_to_electrical_rad_s(
                reference_rpm, pole_pairs
            )
            iq_ref = 0.5 * math.exp(-max(0, index - 100) * sample_period / tau)
            samples.append((
                0.0, 0.0, iq_ref, 0.0, iq_ref, 0.0,
                0.0, 0.0, 0.0, elec_speed, elec_reference,
            ))

        result = validation.summarize_speed_step(
            samples, sample_period, pole_pairs, 1.0, 4.0
        )

        self.assertAlmostEqual(result["target_rpm"], target_rpm, places=3)
        self.assertAlmostEqual(result["tau_63_sec"], tau, delta=0.004)
        self.assertLess(result["filtered_overshoot_pct"], 0.1)
        self.assertLess(result["settling_time_sec"], 0.15)
        self.assertFalse(result["nondecaying_oscillation"])


if __name__ == "__main__":
    unittest.main()
