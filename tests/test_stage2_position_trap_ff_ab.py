from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_position_trap_ff_ab as d054


class Stage2PositionTrapFfAbTests(unittest.TestCase):
    def test_plan_changes_only_feedforward(self):
        plan = d054.plan()
        self.assertEqual(plan["record_count"], 48)
        off, on = plan["profiles"]
        self.assertEqual(off["track_command"], on["track_command"])
        self.assertFalse(off["position_velocity_feedforward"])
        self.assertTrue(on["position_velocity_feedforward"])
        self.assertFalse(plan["automatic_flash"])


if __name__ == "__main__":
    unittest.main()
