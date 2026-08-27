import sys
import unittest
from pathlib import Path


TESTS_DIR = Path(__file__).resolve().parent
if str(TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_DIR))

import stage3_default_feedforward_smoke as smoke


class Stage3DefaultFeedforwardSmokeTests(unittest.TestCase):
    def test_plan_uses_reset_default_and_bounded_motion(self):
        plan = smoke.experiment_plan()
        self.assertFalse(plan["default_selection_command_sent"])
        self.assertEqual(plan["default_model_expected"], "IDENTIFIED")
        self.assertEqual(plan["default_mode_expected"], "BOTH")
        self.assertEqual(plan["speed_order_rpm"], [1500.0, -1500.0])
        self.assertEqual(plan["maximum_commanded_speed_rpm"], 1500.0)
        self.assertTrue(plan["finally_stop_and_confirm_ready"])


if __name__ == "__main__":
    unittest.main()
