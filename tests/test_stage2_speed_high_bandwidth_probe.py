#!/usr/bin/env python3

from __future__ import annotations

import math
import cmath
from pathlib import Path
import sys
import tempfile
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_high_bandwidth_probe as probe


class Stage2SpeedHighBandwidthProbeTests(unittest.TestCase):
    def setUp(self):
        self.controllers = probe.selected_controllers(probe.DEFAULT_DESIGN)

    def test_plan_has_balanced_128_point_probe(self):
        plan = probe.plan(self.controllers)

        self.assertEqual(plan["scope_capture_count"], 128)
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 3300.0)
        self.assertEqual(
            [item["name"] for item in plan["controllers"]],
            ["bw20_pm65", "bw40_pm65"],
        )
        self.assertTrue(plan["supports_checkpoint_resume"])
        self.assertFalse(plan["automatic_flash"])

    def test_visit_order_covers_each_controller_bias_amplitude_once(self):
        visits = probe.visit_order(self.controllers)
        keys = {
            (item["controller"]["name"], item["bias_rpm"],
             item["amplitude_rpm"])
            for item in visits
        }

        self.assertEqual(len(visits), 16)
        self.assertEqual(len(keys), 16)
        self.assertEqual(
            {item["bias_rpm"] for item in visits},
            set(probe.SPEED_BIASES_RPM),
        )

    def test_bias_pairs_cover_direction_and_speed_dependence(self):
        records = []
        for bias_rpm in probe.SPEED_BIASES_RPM:
            records.append({
                "controller": "bw20_pm65",
                "amplitude_rpm": 100.0,
                "frequency_hz": 80.0,
                "bias_rpm": bias_rpm,
                "plant_magnitude_db": -20.0,
                "plant_phase_deg": -140.0,
            })

        pairs = probe.bias_pair_differences(records)

        self.assertEqual(4, len(pairs))
        self.assertEqual(
            {"direction_1500", "direction_3000", "positive_bias", "negative_bias"},
            {item["kind"] for item in pairs},
        )

    def test_sine_fit_quality_separates_clean_and_distorted_signal(self):
        sample_period = 1.0 / 4000.0
        clean = [
            3.0 + 2.0 * math.sin(2.0 * math.pi * 50.0 * index * sample_period)
            for index in range(4096)
        ]
        distorted = [
            value + 1.0 * math.sin(
                2.0 * math.pi * 120.0 * index * sample_period
            )
            for index, value in enumerate(clean)
        ]

        clean_result = probe.sine_fit_quality(clean, 50.0, sample_period)
        distorted_result = probe.sine_fit_quality(
            distorted, 50.0, sample_period,
        )

        self.assertAlmostEqual(clean_result["amplitude"], 2.0, places=6)
        self.assertGreater(clean_result["sine_snr_db"], 100.0)
        self.assertLess(distorted_result["sine_snr_db"], 10.0)

    def test_plant_recovery_round_trip(self):
        record = {
            "frequency_hz": 60.0,
            "closed_loop_magnitude_db": -5.0,
            "closed_loop_phase_deg": -70.0,
        }

        probe.add_plant_record(record, 0.005, 0.1)

        self.assertTrue(math.isfinite(record["plant_magnitude_db"]))
        self.assertGreater(
            record["maximum_possible_pi_phase_margin_deg"], 0.0,
        )

    def test_plant_phase_does_not_wrap_past_minus_180_degrees(self):
        plant = cmath.rect(10.0, math.radians(-190.0))
        controller = 0.005
        closed_loop = controller * plant / (1.0 + controller * plant)
        record = {
            "frequency_hz": 150.0,
            "closed_loop_magnitude_db": 20.0 * math.log10(abs(closed_loop)),
            "closed_loop_phase_deg": math.degrees(cmath.phase(closed_loop)),
        }

        probe.add_plant_record(record, controller, 0.0)

        self.assertAlmostEqual(record["plant_phase_deg"], -190.0, places=6)
        self.assertAlmostEqual(
            record["maximum_possible_pi_phase_margin_deg"], -10.0, places=6,
        )

    def test_checkpoint_is_valid_json_and_replaces_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "checkpoint.json"
            probe.save_checkpoint(path, {"records": [1]})
            probe.save_checkpoint(path, {"records": [1, 2]})

            self.assertEqual(path.read_text(encoding="utf-8").count("records"), 1)
            self.assertFalse(path.with_suffix(".json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
