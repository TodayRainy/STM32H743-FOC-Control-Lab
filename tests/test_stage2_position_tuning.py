#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest
from unittest import mock


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_position_tuning as tuning


class Stage2PositionTuningTests(unittest.TestCase):
    def test_position_configuration_disables_inner_speed_ramp(self):
        with mock.patch.object(tuning.proto, "send_command") as send_command, \
             mock.patch.object(tuning, "wait_mode"), \
             mock.patch.object(tuning.time, "sleep"):
            tuning.configure_ready(
                object(), bytearray(), tuning.SELECTED_POSITION,
                tuning.POSITION_SPEED_LIMIT_RPM,
                "POS TRACK TRAP 180 900", True, True,
            )
        commands = [call.args[1] for call in send_command.call_args_list]
        self.assertIn("TRACK STEP", commands)
        self.assertLess(commands.index("TRACK STEP"),
                        commands.index("POS TRACK TRAP 180 900"))

    @staticmethod
    def frame(position_ref_deg, position_feedback_deg):
        values = [0.0] * 36
        values[26] = 1.0
        values[31] = math.radians(position_ref_deg)
        values[32] = math.radians(position_feedback_deg)
        return tuple(values)

    def test_plan_is_compact_balanced_and_bounded(self):
        plan = tuning.plan()
        self.assertEqual(len(plan["candidates"]), 5)
        self.assertEqual(len(plan["visits"]), 10)
        self.assertEqual(plan["step_record_count"], 120)
        self.assertEqual(plan["position_speed_limit_rpm"], 1500.0)
        self.assertFalse(plan["automatic_flash"])
        self.assertEqual(
            plan["trajectory_validation"]["offsets_deg"],
            tuning.TRAJECTORY_OFFSETS_DEG,
        )
        self.assertEqual(
            plan["trajectory_validation"]["trapezoid_acceleration_ratio_per_sec"],
            10.0,
        )

    def test_d047_plan_is_the_only_slower_acceleration_holdout(self):
        plan = tuning.plan(0.20)
        trajectory = plan["trajectory_validation"]
        self.assertEqual(trajectory["trapezoid_decision"], "D-047")
        self.assertEqual(trajectory["trapezoid_acceleration_time_sec"], 0.20)
        self.assertEqual(trajectory["trapezoid_acceleration_ratio_per_sec"], 5.0)
        self.assertEqual(
            [rate / 0.20 for rate in trajectory["rates_deg_per_sec"]],
            [900.0, 3600.0],
        )

    def test_p_gains_follow_kinematic_bandwidth_relation(self):
        for candidate, frequency in zip(tuning.CANDIDATES[1:], tuning.P_BANDWIDTHS_HZ):
            self.assertAlmostEqual(
                candidate["kp"], 2.0 * tuning.math.pi * 10.0 * frequency
            )
            self.assertEqual(candidate["ki"], 0.0)
            self.assertEqual(candidate["kd"], 0.0)

    def test_two_degree_steps_are_not_hard_gated(self):
        records = [{
            "step_amplitude_deg": 2.0,
            "nondecaying_oscillation": True,
            "noise_adjusted_overshoot_pct": 100.0,
            "settling_time_sec": None,
            "steady_error_deg": 1.0,
        }, {
            "step_amplitude_deg": 10.0,
            "nondecaying_oscillation": False,
            "noise_adjusted_overshoot_pct": 5.0,
            "settling_time_sec": 0.10,
            "steady_error_deg": 0.1,
        }]
        self.assertTrue(tuning.records_pass(records))

    def test_selector_prefers_lower_current_within_ten_percent_time(self):
        summaries = {
            "p4hz": {"qualified": True, "worst_settling_time_sec": 0.11,
                      "peak_abs_iq_reference_a": 0.5, "kp": 251.0},
            "p6hz": {"qualified": True, "worst_settling_time_sec": 0.10,
                      "peak_abs_iq_reference_a": 0.7, "kp": 377.0},
            "legacy": {"qualified": True, "worst_settling_time_sec": 0.05,
                        "peak_abs_iq_reference_a": 1.0, "kp": 3125.0},
        }
        self.assertEqual(tuning.select_candidate(summaries), "p4hz")

    def test_trajectory_metrics_use_reference_arrival_as_settling_origin(self):
        samples = [
            (0.000, self.frame(0.0, 0.0)),
            (0.025, self.frame(4.5, 4.0)),
            (0.050, self.frame(9.0, 8.5)),
            (0.060, self.frame(10.0, 9.8)),
            (0.080, self.frame(10.0, 10.0)),
            (0.100, self.frame(10.0, 10.0)),
        ]
        result = tuning.summarize_trajectory(
            samples, math.radians(10.0), 180.0, 0.0, 10.0
        )
        self.assertAlmostEqual(result["settling_after_reference_sec"], 0.0)
        self.assertLess(result["equivalent_tracking_lag_sec"], 0.020)
        self.assertTrue(result["qualified"])

    def test_trajectory_endpoint_overshoot_is_absolute_degrees(self):
        samples = [
            (0.000, self.frame(0.0, 0.0)),
            (0.050, self.frame(10.0, 9.0)),
            (0.075, self.frame(10.0, 10.8)),
            (0.100, self.frame(10.0, 10.0)),
        ]
        result = tuning.summarize_trajectory(
            samples, math.radians(10.0), 180.0, 0.0, 10.0
        )
        self.assertAlmostEqual(result["endpoint_overshoot_deg"], 0.8)
        self.assertFalse(result["qualified"])


if __name__ == "__main__":
    unittest.main()
