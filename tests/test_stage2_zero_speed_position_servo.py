#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_zero_speed_position_servo as diagnostic


class Stage2ZeroSpeedPositionServoTests(unittest.TestCase):
    def test_plan_is_bounded_and_never_flashes(self):
        plan = diagnostic.plan()
        self.assertEqual(plan["decision"], "D-048")
        self.assertEqual(plan["low_speed"]["total_record_count"], 20)
        self.assertEqual(plan["low_speed"]["nonzero_record_count"], 16)
        self.assertEqual(plan["position_hold"]["record_count"], 144)
        self.assertEqual(len(plan["position_hold"]["angles_deg"]), 36)
        self.assertTrue(plan["requires_24v"])
        self.assertTrue(plan["motor_will_rotate_both_directions"])
        self.assertFalse(plan["automatic_flash"])

    def test_position_targets_cover_one_circle_from_both_directions(self):
        increasing = diagnostic.position_targets("increasing")
        decreasing = diagnostic.position_targets("decreasing")
        self.assertEqual(
            [item[0] for item in increasing],
            list(diagnostic.MECHANICAL_ANGLES_DEG),
        )
        self.assertEqual(
            sorted(item[0] for item in decreasing),
            list(diagnostic.MECHANICAL_ANGLES_DEG),
        )
        self.assertEqual(increasing[0], (0.0, 0.0))
        self.assertEqual(increasing[-1], (350.0, 350.0))
        self.assertEqual(decreasing[0], (0.0, 360))
        self.assertEqual(decreasing[-1], (10.0, 10))

    def test_low_speed_summary_requires_every_direction_and_repeat(self):
        records = []
        for speed in (5.0, 10.0, 20.0, 50.0):
            for direction in (-1.0, 1.0):
                for round_index in (0, 1):
                    target = direction * speed
                    measured = target - direction * min(1.0, 0.1 * speed)
                    records.append({
                        "round": round_index,
                        "target_rpm": target,
                        "mean_measured_speed_rpm": measured,
                        "mean_tracking_error_rpm": target - measured,
                        "speed_ripple_rms_rpm": 0.2,
                        "rms_iq_reference_a": 0.1,
                    })
        summary = diagnostic.summarize_low_speed(records)
        self.assertEqual(
            summary["minimum_bidirectional_mean_tracked_speed_rpm"], 5.0
        )
        self.assertTrue(
            all(point["record_count"] == 4 for point in summary["points"])
        )

        records[0]["mean_measured_speed_rpm"] = 1.0
        summary = diagnostic.summarize_low_speed(records)
        self.assertFalse(
            summary["points"][0]["mean_tracked_within_20_percent_or_2rpm"]
        )
        self.assertEqual(
            summary["minimum_bidirectional_mean_tracked_speed_rpm"], 10.0
        )

        incomplete = diagnostic.summarize_low_speed(records[1:])
        self.assertFalse(
            incomplete["points"][0]["complete_bidirectional_repeated_visits"]
        )
        self.assertEqual(
            incomplete["minimum_bidirectional_mean_tracked_speed_rpm"], 10.0
        )

    def test_position_map_separates_position_and_direction_terms(self):
        records = []
        for direction in ("increasing", "decreasing"):
            direction_sign = 1.0 if direction == "increasing" else -1.0
            for round_index in (0, 1):
                repeat_offset = 0.001 * round_index
                for angle in diagnostic.MECHANICAL_ANGLES_DEG:
                    periodic_iq = 0.10 * math.sin(math.radians(2.0 * angle))
                    periodic_error = 0.20 * math.sin(math.radians(2.0 * angle))
                    records.append({
                        "direction": direction,
                        "round": round_index,
                        "physical_angle_deg": angle,
                        "mean_iq_reference_a": (
                            periodic_iq + 0.02 * direction_sign + repeat_offset
                        ),
                        "mean_position_error_deg": (
                            periodic_error + 0.05 * direction_sign
                        ),
                    })

        summary = diagnostic.summarize_position_map(records)
        self.assertEqual(len(summary["angle_map"]), 36)
        self.assertTrue(summary["periodic_equivalent_current_candidate"])
        self.assertGreater(summary["minimum_iq_repeat_correlation"], 0.99)
        expected_peak_to_peak = 0.20 * math.sin(math.radians(80.0))
        self.assertAlmostEqual(
            summary["position_related_iq_peak_to_peak_a"],
            expected_peak_to_peak,
            places=6,
        )
        self.assertAlmostEqual(
            summary["direction_related_iq_peak_to_peak_a"], 0.0, places=6
        )
        self.assertAlmostEqual(
            summary["angle_map"][0]["direction_related_iq_reference_a"],
            0.02,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
