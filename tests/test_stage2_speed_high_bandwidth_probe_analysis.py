#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_high_bandwidth_probe as probe
import stage2_speed_high_bandwidth_probe_analysis as analysis


def make_record(frequency_hz: float, bias_rpm: float, snr_db: float,
                magnitude_db: float, phase_deg: float) -> dict:
    return {
        "controller": analysis.CORE_CONTROLLER,
        "amplitude_rpm": analysis.CORE_AMPLITUDE_RPM,
        "frequency_hz": frequency_hz,
        "bias_rpm": bias_rpm,
        "plant_magnitude_db": magnitude_db,
        "plant_phase_deg": phase_deg,
        "maximum_possible_pi_phase_margin_deg": 180.0 + phase_deg,
        "measured_speed_fit": {"sine_snr_db": snr_db},
    }


class Stage2SpeedHighBandwidthProbeAnalysisTests(unittest.TestCase):
    def test_core_requires_all_four_biases_to_meet_snr(self):
        records = [
            make_record(40.0, bias, 10.0, 40.0, -110.0)
            for bias in probe.SPEED_BIASES_RPM
        ]
        records[-1]["measured_speed_fit"]["sine_snr_db"] = 5.9

        summary = analysis.core_frequency_summary(records)[0]

        self.assertFalse(summary["all_four_biases_snr_qualified"])
        self.assertEqual("insufficient_snr", summary["phase_region"])

    def test_core_uses_frozen_repeatability_limits(self):
        records = [
            make_record(40.0, bias, 10.0, 40.0, -110.0)
            for bias in probe.SPEED_BIASES_RPM
        ]
        records[-1]["plant_magnitude_db"] = 42.1

        summary = analysis.core_frequency_summary(records)[0]

        self.assertTrue(summary["all_four_biases_snr_qualified"])
        self.assertFalse(summary["bias_repeatable"])

    def test_phase_regions_keep_the_frozen_boundaries(self):
        self.assertEqual("green", analysis.region(65.0))
        self.assertEqual("yellow", analysis.region(50.0))
        self.assertEqual("red", analysis.region(49.9))
        self.assertEqual("insufficient_snr", analysis.region(None))


if __name__ == "__main__":
    unittest.main()
