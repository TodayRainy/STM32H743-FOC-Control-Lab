import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from stage4_field_weakening_design import (
    IMPLEMENTED_KI_A_PER_V_S,
    IMPLEMENTED_KP_A_PER_V,
    build_report,
    model_id,
    voltage_target,
)


class FieldWeakeningDesignTests(unittest.TestCase):
    def test_voltage_ellipse_is_direction_symmetric(self):
        target = voltage_target(0.35)
        forward = model_id(5000.0, 1.0, target)
        reverse = model_id(-5000.0, -1.0, target)
        self.assertAlmostEqual(forward, -2.38585, places=4)
        self.assertAlmostEqual(forward, reverse, places=7)

    def test_pi_is_derived_from_nominal_voltage_sensitivity(self):
        report = build_report(".")
        design = report["pi_design"]
        self.assertLess(
            abs(IMPLEMENTED_KP_A_PER_V-design["derived_kp_a_per_v"])
            / design["derived_kp_a_per_v"],
            0.15,
        )
        self.assertLess(
            abs(IMPLEMENTED_KI_A_PER_V_S-design["derived_ki_a_per_v_s"])
            / design["derived_ki_a_per_v_s"],
            0.10,
        )

    def test_frozen_main_and_extension_points_fit_expected_current_region(self):
        report = build_report(".")
        phases = {item["name"]: item for item in report["operating_phases"]}
        main = {point["iq_a"]: point for point in phases["main"]["points"]}
        extension = {
            point["iq_a"]: point for point in phases["extension"]["points"]
        }
        self.assertTrue(main[2.0]["feasible_in_5a_circle"])
        self.assertFalse(main[3.0]["feasible_in_5a_circle"])
        self.assertTrue(extension[2.0]["feasible_in_5a_circle"])
        self.assertFalse(extension[3.0]["feasible_in_5a_circle"])
        self.assertAlmostEqual(
            phases["main"]["target_voltage_v"],
            24.0/math.sqrt(3.0)*0.35*0.95,
        )

    def test_d058_noise_supports_the_frozen_200hz_filter(self):
        noise = build_report(".")["historical_voltage_noise"]
        self.assertEqual(noise["file_count"], 8)
        self.assertGreater(noise["mean_standard_deviation_v"], 0.07)
        self.assertLess(noise["mean_standard_deviation_v"], 0.10)


if __name__ == "__main__":
    unittest.main()
