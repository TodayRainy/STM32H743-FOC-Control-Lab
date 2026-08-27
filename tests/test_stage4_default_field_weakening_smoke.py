from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage4_default_field_weakening_smoke as smoke


class DefaultFieldWeakeningSmokeTests(unittest.TestCase):
    def test_plan_checks_default_without_selection_command(self):
        plan = smoke.experiment_plan()
        self.assertEqual(plan["default_mode_expected"], "HYBRID")
        self.assertFalse(plan["default_selection_command_sent"])
        self.assertEqual(plan["target_rpm"], 5000.0)
        self.assertEqual(plan["voltage_limit_ratio"], 0.35)
        self.assertTrue(plan["finally_stop_and_confirm_ready"])


if __name__ == "__main__":
    unittest.main()
