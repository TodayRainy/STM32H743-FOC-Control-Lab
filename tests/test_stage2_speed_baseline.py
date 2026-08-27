#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_baseline as baseline


class Stage2SpeedBaselineTests(unittest.TestCase):
    def test_plan_is_biased_and_bounded(self):
        plan = baseline.plan()

        self.assertEqual(plan["speed_estimator"], "win_1ms_fc150")
        self.assertEqual(plan["speed_biases_rpm"], (1500.0, -1500.0))
        self.assertEqual(plan["scope_capture_count"], 22)
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 1600.0)
        self.assertTrue(plan["requires_24v"])
        self.assertTrue(plan["finally_stop_and_confirm_ready"])

    def test_low_frequencies_receive_longer_captures(self):
        self.assertEqual(baseline.scope_divider(2.0), 100)
        self.assertEqual(baseline.scope_divider(6.0), 10)
        self.assertEqual(baseline.scope_divider(20.0), 10)

    def test_frequency_grid_resolves_expected_crossover(self):
        self.assertEqual(
            baseline.FREQUENCIES_HZ,
            (1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0, 30.0, 40.0, 60.0),
        )

    def test_bandwidth_uses_unity_gain_reference(self):
        summary = baseline.summarize_baseline([
            {"frequency_hz": 5.0, "closed_loop_magnitude_db": 1.0},
            {"frequency_hz": 10.0, "closed_loop_magnitude_db": -2.0},
            {"frequency_hz": 20.0, "closed_loop_magnitude_db": -5.0},
        ])

        self.assertGreater(summary["closed_loop_bandwidth_hz"], 10.0)
        self.assertLess(summary["closed_loop_bandwidth_hz"], 20.0)
        self.assertEqual(summary["maximum_closed_loop_peak_db"], 1.0)

    def test_direction_consistency_reports_worst_difference(self):
        records = [
            {
                "frequency_hz": frequency,
                "closed_loop_magnitude_db": 0.0,
                "closed_loop_phase_deg": 0.0,
            }
            for frequency in baseline.FREQUENCIES_HZ
        ]
        reverse = [dict(item) for item in records]
        reverse[-1]["closed_loop_magnitude_db"] = -0.5
        reverse[-1]["closed_loop_phase_deg"] = -4.0

        summary = baseline.direction_consistency([
            {"records": records}, {"records": reverse},
        ])

        self.assertEqual(summary["maximum_magnitude_difference_db"], 0.5)
        self.assertEqual(summary["maximum_phase_difference_deg"], 4.0)


if __name__ == "__main__":
    unittest.main()
