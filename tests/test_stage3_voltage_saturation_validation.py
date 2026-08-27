import unittest

import stage3_voltage_saturation_validation as stage3


class Stage3VoltageSaturationValidationTests(unittest.TestCase):
    def test_plan_is_counterbalanced_and_bounded(self):
        plan = stage3.experiment_plan()
        self.assertEqual(plan["visit_count"], 8)
        self.assertEqual(plan["scope_capture_count"], 8)
        self.assertEqual(plan["modes"], ["OFF", "BOTH"])
        self.assertEqual(plan["voltage_limit_ratio"], 0.22)
        self.assertAlmostEqual(
            plan["predicted_minimum_voltage_scale"],
            0.22 / stage3.D057_PEAK_REQUESTED_UTILIZATION,
        )
        self.assertEqual(
            [(item["mode"], item["direction"]) for item in plan["visits"][:4]],
            [("OFF", 1), ("OFF", -1), ("BOTH", 1), ("BOTH", -1)],
        )
        self.assertEqual(
            [(item["mode"], item["direction"]) for item in plan["visits"][4:]],
            [("BOTH", -1), ("BOTH", 1), ("OFF", -1), ("OFF", 1)],
        )
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 3300.0)
        self.assertEqual(plan["scope"]["divider"], 10)
        self.assertAlmostEqual(plan["scope"]["duration_sec"], 1.024)
        self.assertTrue(plan["finally_stop_and_confirm_ready"])
        self.assertTrue(plan["restore_identified_model_and_ff_both"])

    def test_scope_summary_separates_limit_and_recovery(self):
        metadata = {"divider": 1, "base_sample_hz": 1000}
        samples = []
        for index in range(1024):
            voltage_scale = 0.8 if 100 <= index < 140 or index == 142 else 1.0
            iq_ref = 1.0 if 100 <= index < 140 else 0.0
            iq = iq_ref - (0.2 if 100 <= index < 140 else 0.02)
            samples.append((
                0.0, 0.0, iq, 0.01, iq_ref, 0.0,
                3.0, 0.1, 0.0, 3000.0, 3000.0, voltage_scale,
            ))
        summary = stage3.summarize_scope("OFF", metadata, samples)
        self.assertAlmostEqual(summary["minimum_voltage_scale"], 0.8)
        self.assertAlmostEqual(summary["limited_sample_fraction"], 41.0 / 1024.0)
        self.assertAlmostEqual(summary["longest_limited_duration_ms"], 40.0)
        self.assertAlmostEqual(summary["post_return_scale_recovery_ms"], 3.0)
        self.assertAlmostEqual(summary["tail_minimum_voltage_scale"], 1.0)
        self.assertAlmostEqual(summary["pre_iq_error_rms_a"], 0.02)
        self.assertAlmostEqual(summary["pulse_iq_error_rms_a"], 0.2)
        self.assertAlmostEqual(summary["tail_iq_error_rms_a"], 0.02)

    def test_both_feedforward_uses_scope_coordinates(self):
        sample = (
            0.0, 0.0, 1.25, -0.75, 0.0, 0.0,
            0.0, 0.0, 0.0, 3000.0, 0.0, 1.0,
        )
        uq, ud = stage3.feedforward("BOTH", sample)
        self.assertAlmostEqual(
            uq,
            3000.0 * (
                stage3.IDENTIFIED_FLUX_WB
                + stage3.IDENTIFIED_LD_H * -0.75
            ),
        )
        self.assertAlmostEqual(
            ud, -3000.0 * stage3.IDENTIFIED_LQ_H * 1.25,
        )
        self.assertEqual(stage3.feedforward("OFF", sample), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
