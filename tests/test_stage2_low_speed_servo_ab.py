#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest
from unittest import mock


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_low_speed_servo_ab as comparison


class Stage2LowSpeedServoAbTests(unittest.TestCase):
    @staticmethod
    def frame(electrical_angle_rad: float) -> tuple[float, ...]:
        values = [0.0] * 36
        values[12] = electrical_angle_rad
        return tuple(values)

    def test_plan_is_complete_cross_and_does_not_flash(self):
        plan = comparison.plan()
        pairs = {
            (item["estimator"], item["pi_profile"])
            for item in plan["configurations"]
        }
        self.assertEqual(len(pairs), 6)
        self.assertEqual(
            pairs,
            {
                (estimator, profile)
                for estimator in ("BASE", "WIN", "PLL")
                for profile in ("fc24", "bw20")
            },
        )
        self.assertEqual(plan["speed_record_count"], 48)
        self.assertEqual(plan["position_record_count"], 24)
        self.assertEqual(plan["start_zero_settle_sec"], 1.0)
        self.assertFalse(plan["automatic_flash"])
        self.assertTrue(plan["requires_24v"])

    def test_encoder_speed_unwraps_electrical_angle(self):
        sample_period = 0.002
        target_rpm = 10.0
        electrical_speed = (
            target_rpm * 2.0 * math.pi / 60.0 * comparison.POLE_PAIRS
        )
        samples = []
        for index in range(1000):
            timestamp = index * sample_period
            angle = (6.0 + electrical_speed * timestamp) % (2.0 * math.pi)
            samples.append((timestamp, self.frame(angle)))
        speed = comparison.encoder_speed_series(samples)
        values = [value for _, value in speed[100:]]
        self.assertAlmostEqual(sum(values) / len(values), target_rpm, places=5)
        self.assertLess(comparison.ripple_rms(values), 1.0e-5)

    def test_aggregate_uses_independent_encoder_motion(self):
        speed_records = []
        position_records = []
        for configuration, estimator, pi_profile in comparison.CONFIGURATIONS:
            scale = 1.0 if configuration == "win_fc24" else 0.5
            for target in comparison.SPEED_TARGETS_RPM:
                speed_records.append({
                    "configuration": configuration,
                    "estimator": estimator,
                    "pi_profile": pi_profile,
                    "target_rpm": target,
                    "encoder_mean_error_rpm": scale,
                    "encoder_speed_ripple_rms_rpm": 10.0 * scale,
                    "selected_speed_ripple_rms_rpm": 1.0,
                    "rms_iq_reference_a": 0.2,
                })
            for target in comparison.POSITION_TARGETS_DEG:
                position_records.append({
                    "configuration": configuration,
                    "estimator": estimator,
                    "pi_profile": pi_profile,
                    "target_offset_deg": target,
                    "mean_position_error_deg": 2.0 * scale,
                    "position_error_ripple_rms_deg": 3.0 * scale,
                    "encoder_speed_ripple_rms_rpm": 8.0 * scale,
                    "rms_iq_reference_a": 0.3,
                })
        summary = comparison.aggregate_configuration(
            speed_records, position_records
        )
        self.assertEqual(len(summary), 6)
        self.assertEqual(
            summary["win_fc24"]["relative_to_win_fc24"]
            ["nonzero_encoder_speed_ripple"],
            1.0,
        )
        self.assertEqual(
            summary["base_bw20"]["relative_to_win_fc24"]
            ["position_encoder_speed_ripple"],
            0.5,
        )

    def test_stop_confirmation_does_not_reject_frozen_speed(self):
        frame = [0.0] * 36
        frame[25] = -1000.0
        frame[27] = comparison.MODE_READY
        port = mock.Mock()
        buffer = bytearray()
        with mock.patch.object(
            comparison.proto, "send_command"
        ) as send_command, mock.patch.object(
            comparison.proto, "read_frame", return_value=(0.0, tuple(frame))
        ):
            comparison.stop_ready(port, buffer)
        send_command.assert_called_once_with(port, "STOP")


if __name__ == "__main__":
    unittest.main()
