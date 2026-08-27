import math
import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

import stage3_feedforward_component_validation as stage3


class Stage3FeedforwardComponentValidationTests(unittest.TestCase):
    def test_plan_is_two_round_reversed_factorial(self):
        plan = stage3.experiment_plan()
        self.assertEqual(plan["visit_count"], 8)
        self.assertEqual(plan["steady_hold_count"], 96)
        self.assertEqual(plan["scope_capture_count"], 8)
        self.assertEqual(
            [visit["mode"] for visit in plan["visits"][:4]],
            ["OFF", "CROSS", "BEMF", "BOTH"],
        )
        self.assertEqual(
            [visit["mode"] for visit in plan["visits"][4:]],
            ["BOTH", "BEMF", "CROSS", "OFF"],
        )
        self.assertTrue(plan["finally_stop_and_confirm_ready"])
        self.assertTrue(plan["restore_identified_model_and_ff_both"])
        self.assertFalse(plan["automatic_flash"])

    def test_component_voltage_modes_and_direction(self):
        omega = 3000.0
        iq = 1.25
        id_ = -0.75
        cross = stage3.component_voltage("CROSS", omega, iq, id_)
        bemf = stage3.component_voltage("BEMF", omega, iq, id_)
        both = stage3.component_voltage("BOTH", omega, iq, id_)
        off = stage3.component_voltage("OFF", omega, iq, id_)
        self.assertEqual(off, (0.0, 0.0))
        self.assertAlmostEqual(cross[0], omega * stage3.IDENTIFIED_LD_H * id_)
        self.assertAlmostEqual(cross[1], -omega * stage3.IDENTIFIED_LQ_H * iq)
        self.assertAlmostEqual(bemf[0], omega * stage3.IDENTIFIED_FLUX_WB)
        self.assertEqual(bemf[1], 0.0)
        self.assertAlmostEqual(both[0], cross[0] + bemf[0])
        self.assertAlmostEqual(both[1], cross[1])
        reverse = stage3.component_voltage("BOTH", -omega, iq, id_)
        self.assertAlmostEqual(reverse[0], -both[0])
        self.assertAlmostEqual(reverse[1], -both[1])

    def test_factorial_summary_recovers_main_effects(self):
        voltage = {"OFF": 10.0, "CROSS": 8.0, "BEMF": 4.0, "BOTH": 2.0}
        summary = stage3.factorial_summary([
            {"mode": mode, "mean_steady_pi_voltage_rms_v": value}
            for mode, value in voltage.items()
        ])
        self.assertIsNotNone(summary)
        self.assertTrue(math.isclose(summary["both_vs_off_pi_voltage_reduction"], 0.8))
        self.assertTrue(math.isclose(summary["cross_main_effect_v"], 2.0))
        self.assertTrue(math.isclose(summary["bemf_main_effect_v"], 6.0))
        self.assertTrue(math.isclose(summary["interaction_v"], 0.0))


if __name__ == "__main__":
    unittest.main()
