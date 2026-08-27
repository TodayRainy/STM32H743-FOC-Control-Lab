#!/usr/bin/env python3
"""Host regression tests for the Stage-2 current-loop autotuning design."""

from __future__ import annotations

import cmath
import math
from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_current_autotune as autotune
import stage2_current_autotune_validation as hardware_validation
import stage2_current_frequency_response as frequency_response


class CurrentAutotuneTests(unittest.TestCase):
    def test_single_point_loop_shaping_hits_requested_margin(self) -> None:
        resistance = 0.12
        inductance = 35.0e-6
        delay_s = 25.0e-6
        frequencies = [100.0, 150.0, 500.0, 1000.0, 2000.0, 4000.0]
        plant = []
        for frequency in frequencies:
            omega = 2.0 * math.pi * frequency
            value = (cmath.exp(complex(0.0, -omega * delay_s))
                     / complex(resistance, omega * inductance))
            plant.append((frequency, value))

        gains = autotune.design_from_plant(plant, 1000.0, 70.0)
        result = autotune.evaluate_frequency_trace(plant, gains)

        self.assertGreater(gains["kp"], 0.0)
        self.assertGreater(gains["ki"], 0.0)
        self.assertAlmostEqual(result["loop_crossover_hz"], 1000.0, places=6)
        self.assertAlmostEqual(result["phase_margin_deg"], 70.0, places=6)

    def test_current_baseline_generates_eight_safe_hardware_candidates(self) -> None:
        report = autotune.build_report(
            REPO_ROOT / "tests/baselines/stage1_20260824.json",
            REPO_ROOT / "tests/results/stage2_current_frf_20260824_145248.json",
            ("active_legacy",),
            2,
        )

        self.assertEqual(report["status"], "ready_for_hardware_coarse_screen")
        self.assertEqual(report["hardware_candidate_count"], 8)
        self.assertNotIn("active_legacy", report["sources"]["used_frf_profiles"])
        self.assertAlmostEqual(report["frequency_limits"]["f_upper_hz"],
                               1666.666666666667, places=6)
        self.assertAlmostEqual(report["frequency_limits"]["f_seed_hz"],
                               800.0, places=6)

        selected = [candidate for candidate in report["candidates"]
                    if candidate["selected_for_hardware_coarse_screen"]]
        self.assertEqual(len(selected), 8)
        self.assertTrue(all(
            candidate["offline_evaluation"]
            ["eligible_for_hardware_coarse_screen"]
            for candidate in selected
        ))
        self.assertTrue(any(
            candidate.get("replacement_for") == "measured_fc1500_pm65"
            for candidate in selected
        ))

    def test_hardware_plan_uses_all_eight_candidates_in_reverse_second_round(self):
        _, candidates, names = hardware_validation.load_design(
            REPO_ROOT / "tests/baselines/stage2_current_autotune_design_20260824.json"
        )
        plan = hardware_validation.build_run_plan(names, 2)

        self.assertGreaterEqual(len(candidates), 8)
        self.assertEqual(len(names), 8)
        self.assertTrue(all(name in candidates for name in names))
        self.assertNotIn("active_legacy", names)
        self.assertEqual(plan[0]["order"], names)
        self.assertEqual(plan[1]["order"], list(reversed(names)))

    def test_refined_policy_generates_seven_candidates_near_coarse_leader(self):
        report = autotune.build_report(
            REPO_ROOT / "tests/baselines/stage1_20260824.json",
            REPO_ROOT / "tests/results/stage2_current_frf_20260824_145248.json",
            ("active_legacy",),
            2,
            "refined",
        )

        self.assertEqual(report["status"], "ready_for_hardware_coarse_screen")
        self.assertEqual(report["hardware_candidate_count"], 7)
        self.assertEqual(report["hardware_candidate_names"], [
            "rl_seed",
            "measured_fc0800_pm70",
            "measured_fc0800_pm75",
            "measured_fc0900_pm70",
            "measured_fc0900_pm75",
            "measured_fc1000_pm70",
            "measured_fc1000_pm75",
        ])
        self.assertNotIn("active_legacy", report["hardware_candidate_names"])

    def test_d029_policy_spans_eight_declared_candidates(self):
        report = autotune.build_report(
            REPO_ROOT / "tests/baselines/stage1_20260824.json",
            REPO_ROOT / "tests/results/stage2_current_frf_20260824_145248.json",
            ("active_legacy",),
            2,
            "d029",
        )

        self.assertEqual(report["hardware_candidate_count"], 8)
        self.assertEqual(report["hardware_candidate_names"], [
            "rl_seed",
            "rl_1250",
            "rl_1500",
            "rl_1750",
            "rl_2000",
            "measured_fc1000_pm75",
            "measured_fc1250_pm75",
            "measured_fc1500_pm75",
        ])
        self.assertEqual(report["candidate_policy"]["name"], "d029")
        self.assertEqual(
            report["hardware_plan"]["coarse_step_amplitudes_a"],
            [0.75, 1.5, 2.25, 3.0],
        )

    def test_d032_policy_is_the_six_declared_high_margin_candidates(self):
        report = autotune.build_report(
            REPO_ROOT / "tests/baselines/stage1_20260824.json",
            REPO_ROOT / "tests/results/stage2_current_frf_20260824_145248.json",
            ("active_legacy",),
            2,
            "d032",
        )

        self.assertEqual(report["status"], "ready_for_hardware_coarse_screen")
        self.assertEqual(report["hardware_candidate_count"], 6)
        self.assertEqual(report["hardware_candidate_names"], [
            "measured_fc0850_pm80",
            "measured_fc0850_pm85",
            "measured_fc0925_pm80",
            "measured_fc0925_pm85",
            "measured_fc1000_pm80",
            "measured_fc1000_pm85",
        ])
        self.assertEqual(report["candidate_policy"]["rl_anchor_count"], 0)
        self.assertNotIn("active_legacy", report["hardware_candidate_names"])

    def test_d029_relaxes_only_normalized_settling_gate(self):
        d028 = hardware_validation.build_d028_criteria(
            (-3.0, 3.0), 4.0
        )
        d029 = hardware_validation.build_d029_criteria(
            (-3.0, 3.0), 4.0
        )

        self.assertEqual(d029["settling_bandwidth_factor"], 1.25)
        self.assertEqual(d029["excellent_settling_bandwidth_factor"], 1.0)
        for key, value in d028.items():
            if key not in ("metric_version", "settling_bandwidth_factor"):
                self.assertEqual(d029[key], value)

    def test_d029_default_hardware_plan_uses_eight_candidates(self):
        self.assertEqual(
            hardware_validation.DEFAULT_DESIGN,
            Path("tests/baselines/stage2_current_autotune_d029_design_20260824.json"),
        )
        _, candidates, names = hardware_validation.load_design(
            REPO_ROOT / hardware_validation.DEFAULT_DESIGN
        )
        visits = len(hardware_validation.build_run_plan(names, 2)) * len(names)
        captures_per_visit = (
            1 + len(hardware_validation.DEFAULT_STEPS_A)
            + len(hardware_validation.DEFAULT_D_FREQUENCIES_HZ)
            + len(hardware_validation.DEFAULT_Q_FREQUENCIES_HZ)
        )

        self.assertEqual(len(candidates), 8)
        self.assertEqual(len(names), 8)
        self.assertEqual(1 + visits * captures_per_visit, 593)

    def test_d029_holdout_uses_only_unseen_points_and_frozen_leader(self):
        _, _, available = hardware_validation.load_design(
            REPO_ROOT / hardware_validation.DEFAULT_DESIGN
        )
        names = hardware_validation.select_candidate_names(
            available, None, "holdout"
        )
        screen_steps = set(hardware_validation.DEFAULT_STEPS_A)
        screen_frequencies = set(
            hardware_validation.DEFAULT_D_FREQUENCIES_HZ
        ) | set(hardware_validation.DEFAULT_Q_FREQUENCIES_HZ)
        visits = len(hardware_validation.build_run_plan(names, 2)) * len(names)
        captures_per_visit = (
            1 + len(hardware_validation.HOLDOUT_STEPS_A)
            + 2 * len(hardware_validation.HOLDOUT_FREQUENCIES_HZ)
        )

        self.assertEqual(names, ["measured_fc1000_pm75"])
        self.assertTrue(screen_steps.isdisjoint(
            hardware_validation.HOLDOUT_STEPS_A
        ))
        self.assertTrue(screen_frequencies.isdisjoint(
            hardware_validation.HOLDOUT_FREQUENCIES_HZ
        ))
        self.assertEqual(1 + visits * captures_per_visit, 71)

    def test_d031_comparison_counterbalances_recommended_and_active_pi(self):
        _, _, available = hardware_validation.load_design(
            REPO_ROOT / hardware_validation.DEFAULT_DESIGN
        )
        available = available + [hardware_validation.ACTIVE_LEGACY_NAME]
        names = hardware_validation.select_candidate_names(
            available, None, "comparison"
        )
        plan = hardware_validation.build_run_plan(names, 2)
        captures_per_visit = (
            1 + len(hardware_validation.HOLDOUT_STEPS_A)
            + 2 * len(hardware_validation.COMPARISON_FREQUENCIES_HZ)
        )

        self.assertEqual(names, [
            "measured_fc1000_pm75", "active_legacy",
        ])
        self.assertEqual(plan[1]["order"], list(reversed(names)))
        self.assertEqual(1 + 4 * captures_per_visit, 165)
        self.assertEqual(
            hardware_validation.ACTIVE_LEGACY_AXES["d"],
            hardware_validation.ACTIVE_LEGACY_AXES["q"],
        )

    def test_d032_robustness_plan_uses_six_candidates_and_421_captures(self):
        _, candidates, names = hardware_validation.load_design(
            REPO_ROOT / hardware_validation.DEFAULT_D032_DESIGN
        )
        selected = hardware_validation.select_candidate_names(
            names, None, "robustness"
        )
        plan = hardware_validation.build_run_plan(selected, 2)
        captures_per_visit = (
            1 + len(hardware_validation.HOLDOUT_STEPS_A)
            + 2 * len(hardware_validation.HOLDOUT_FREQUENCIES_HZ)
        )

        self.assertEqual(len(candidates), 6)
        self.assertEqual(selected, names)
        self.assertEqual(plan[1]["order"], list(reversed(names)))
        self.assertEqual(1 + 12 * captures_per_visit, 421)

    def test_d032_holdout_freezes_leader_on_original_d029_points(self):
        _, _, available = hardware_validation.load_design(
            REPO_ROOT / hardware_validation.DEFAULT_D032_DESIGN
        )
        names = hardware_validation.select_candidate_names(
            available, None, "robustness_holdout"
        )
        visits = len(hardware_validation.build_run_plan(names, 2)) * len(names)
        captures_per_visit = (
            1 + len(hardware_validation.DEFAULT_STEPS_A)
            + len(hardware_validation.DEFAULT_D_FREQUENCIES_HZ)
            + len(hardware_validation.DEFAULT_Q_FREQUENCIES_HZ)
        )

        self.assertEqual(names, ["measured_fc0925_pm80"])
        self.assertEqual(1 + visits * captures_per_visit, 75)

    def test_frequency_band_rms_separates_tracking_and_sampled_ripple(self):
        sample_rate_hz = 40000.0
        samples = 800
        values = [
            0.1
            + math.cos(2.0 * math.pi * 100.0 * index / sample_rate_hz)
            + 0.5 * math.cos(2.0 * math.pi * 10000.0 * index / sample_rate_hz)
            for index in range(samples)
        ]

        in_band, out_band = hardware_validation.frequency_band_rms(
            values, sample_rate_hz, 1000.0
        )

        self.assertAlmostEqual(in_band, math.sqrt(0.1 ** 2 + 0.5), places=6)
        self.assertAlmostEqual(out_band, 0.5 / math.sqrt(2.0), places=6)

    def test_q_axis_frequency_analysis_uses_q_channels(self):
        sample_rate_hz = 40000.0
        frequency_hz = 1000.0
        reference_amplitude = 0.3
        measured_amplitude = 0.24
        measured_phase = math.radians(-20.0)
        samples = []
        for index in range(4096):
            angle = 2.0 * math.pi * frequency_hz * index / sample_rate_hz
            iq_ref = reference_amplitude * math.cos(angle)
            iq = measured_amplitude * math.cos(angle + measured_phase)
            item = [0.0] * 10
            item[2] = iq
            item[3] = 0.01 * math.cos(angle)
            item[4] = iq_ref
            item[5] = 0.0
            item[6] = 0.6 * math.cos(angle - 0.1)
            samples.append(tuple(item))

        result = frequency_response.analyze_capture(
            samples, frequency_hz, 1.0 / sample_rate_hz, "q"
        )

        self.assertEqual(result["axis"], "q")
        self.assertAlmostEqual(result["reference_amplitude_a"], 0.3, places=5)
        self.assertAlmostEqual(result["measured_amplitude_a"], 0.24, places=5)
        self.assertAlmostEqual(result["closed_loop_magnitude_db"],
                               20.0 * math.log10(0.8), places=5)
        self.assertAlmostEqual(result["closed_loop_phase_deg"], -20.0, places=4)

    def test_q_axis_low_frequency_amplitude_limits_torque_impulse(self):
        amplitude = hardware_validation.sine_amplitude

        self.assertAlmostEqual(amplitude("q", 0.3, 100.0, 300.0), 0.1)
        self.assertAlmostEqual(amplitude("q", 0.3, 150.0, 300.0), 0.15)
        self.assertAlmostEqual(amplitude("q", 0.3, 300.0, 300.0), 0.3)
        self.assertAlmostEqual(amplitude("q", 0.3, 4000.0, 300.0), 0.3)
        self.assertAlmostEqual(amplitude("d", 0.5, 100.0, 300.0), 0.5)

    def test_pwm_off_noise_is_reconstructed_from_phase_currents(self):
        samples = []
        angle = 0.7
        for index in range(400):
            alpha = 0.02 * math.cos(2.0 * math.pi * index / 20.0)
            beta = 0.04 * math.sin(2.0 * math.pi * index / 20.0)
            ia = alpha
            ib = (math.sqrt(3.0) * beta - alpha) / 2.0
            item = [0.0] * 10
            item[0] = ia
            item[1] = ib
            item[8] = angle
            samples.append(tuple(item))

        result = hardware_validation.summarize_pwm_off(samples)

        self.assertEqual(
            result["measurement"],
            "pwm_off_phase_current_noise_reconstructed_to_dq",
        )
        self.assertGreater(result["iq_noise_rms_a"], 0.0)
        self.assertGreater(result["id_noise_rms_a"], 0.0)
        self.assertGreater(result["raw_peak_current_a"], 0.0)


if __name__ == "__main__":
    unittest.main()
