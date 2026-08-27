#!/usr/bin/env python3
"""Host regression tests for ABZ speed-estimator replay math."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import encoder_speed_estimation as estimation


class EncoderSpeedEstimationTests(unittest.TestCase):
    def test_current_boxcar_characteristics(self):
        result = estimation.boxcar_characteristics(40000.0, 500)

        self.assertAlmostEqual(result["window_sec"], 0.0125)
        self.assertAlmostEqual(result["group_delay_sec"], 0.0062375)
        self.assertAlmostEqual(result["minus_3db_hz_approx"], 35.43568)
        self.assertAlmostEqual(result["first_notch_hz"], 80.0)

    def test_fixed_window_recovers_constant_speed_across_wrap(self):
        sample_hz = 4000.0
        sample_period = 1.0 / sample_hz
        expected_speed = 240.0
        wrapped = [
            (expected_speed * index * sample_period) % (2.0 * math.pi)
            for index in range(1000)
        ]
        unwrapped = estimation.unwrap_angles(wrapped)
        measured = estimation.fixed_window_speed(unwrapped, sample_period, 50)

        for value in measured[50:]:
            self.assertAlmostEqual(value, expected_speed, places=9)

    def test_boxcar_has_first_notch_at_80_hz(self):
        sample_hz = 4000.0
        sample_period = 1.0 / sample_hz
        frequency_hz = 80.0
        speed_amplitude = 100.0
        angle_amplitude = speed_amplitude / (2.0 * math.pi * frequency_hz)
        angle = [
            -angle_amplitude * math.cos(2.0 * math.pi * frequency_hz
                                        * index * sample_period)
            for index in range(1000)
        ]
        measured = estimation.fixed_window_speed(angle, sample_period, 50)

        self.assertLess(estimation.finite_rms(measured, 100), 1.0e-10)

    def test_tracking_pll_converges_to_constant_speed(self):
        sample_hz = 4000.0
        sample_period = 1.0 / sample_hz
        expected_speed = 200.0
        angle = [expected_speed * index * sample_period for index in range(2000)]
        measured = estimation.tracking_pll_speed(angle, sample_period, 100.0)

        steady = measured[-400:]
        self.assertLess(max(abs(value - expected_speed) for value in steady), 1.0e-6)

    def test_adaptive_count_recovers_quantized_low_speed(self):
        sample_hz = 40000.0
        sample_period = 1.0 / sample_hz
        encoder_cpr = 4096
        pole_pairs = 10
        mechanical_rpm = 10.0
        electrical_speed = (
            mechanical_rpm * 2.0 * math.pi / 60.0 * pole_pairs
        )
        count_step = 2.0 * math.pi * pole_pairs / encoder_cpr
        measured_angle = []
        for index in range(round(sample_hz * 1.0)):
            true_angle = electrical_speed * index * sample_period
            measured_angle.append(math.floor(true_angle / count_step) * count_step)
        measured = estimation.adaptive_count_speed(
            measured_angle, sample_period, count_step, 4, 0.05, 100.0,
        )
        steady = measured[round(0.2 * sample_hz):]
        mean = sum(steady) / len(steady)

        self.assertLess(abs(mean - electrical_speed) / electrical_speed, 0.01)


if __name__ == "__main__":
    unittest.main()
