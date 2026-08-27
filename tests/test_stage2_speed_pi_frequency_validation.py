#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_pi_frequency_validation as validation


def records(bandwidth_shape: tuple[tuple[float, float], ...]) -> list[dict]:
    return [
        {
            "frequency_hz": frequency,
            "closed_loop_magnitude_db": magnitude,
            "peak_iq_reference_a": 0.8,
        }
        for frequency, magnitude in bandwidth_shape
    ]


class Stage2SpeedPiFrequencyValidationTests(unittest.TestCase):
    def setUp(self):
        self.candidates = validation.load_candidates(validation.DEFAULT_DESIGN)

    def test_plan_has_six_candidates_and_132_captures(self):
        plan = validation.plan(self.candidates)

        self.assertEqual(len(plan["design_candidates"]), 6)
        self.assertEqual(plan["scope_capture_count"], 132)
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 1600.0)
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["restore_default_speed_pi"])

    def test_visit_order_balances_direction_and_candidate_order(self):
        visits = validation.visit_order(self.candidates)

        self.assertEqual(
            [item[0]["name"] for item in visits[:6]],
            [item["name"] for item in self.candidates],
        )
        self.assertEqual(
            [item[0]["name"] for item in visits[6:]],
            [item["name"] for item in reversed(self.candidates)],
        )
        self.assertTrue(all(item[1] == 1500.0 for item in visits[:6]))
        self.assertTrue(all(item[1] == -1500.0 for item in visits[6:]))

    def test_summary_applies_only_four_simple_gates(self):
        candidate = dict(self.candidates[1])
        candidate["target_closed_loop_bandwidth_hz"] = 20.0
        response = records((
            (4.0, 0.2), (10.0, 0.0), (20.0, -3.0103), (40.0, -8.0),
        ))
        visits = [
            {"records": response},
            {"records": records((
                (4.0, 0.1), (10.0, -0.1), (19.0, -3.0103),
                (40.0, -8.2),
            ))},
        ]

        summary = validation.summarize_candidate(candidate, visits, 4.0)

        self.assertEqual(len(summary["gates"]), 4)
        self.assertTrue(summary["qualified_for_later_performance_test"])

    def test_missing_crossing_fails_bandwidth_gate(self):
        candidate = self.candidates[-1]
        flat = records(((4.0, 0.0), (20.0, -1.0), (60.0, -2.0)))

        summary = validation.summarize_candidate(
            candidate, [{"records": flat}, {"records": flat}], 4.0,
        )

        self.assertIsNone(summary["measured_bandwidths_hz"][0])
        self.assertFalse(summary["gates"]["bandwidth_prediction"])
        self.assertFalse(summary["qualified_for_later_performance_test"])


if __name__ == "__main__":
    unittest.main()
