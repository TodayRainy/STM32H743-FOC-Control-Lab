#!/usr/bin/env python3

from __future__ import annotations

import cmath
import math
from pathlib import Path
import sys
import unittest


TESTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS_DIR))

import stage2_speed_pi_design as design


class Stage2SpeedPiDesignTests(unittest.TestCase):
    def test_closed_loop_recovery_round_trip(self):
        frequency = 20.0
        plant = design.from_db_phase(47.0, -102.0)
        loop = design.controller(
            design.CURRENT_KP, design.CURRENT_KI, frequency,
        ) * plant
        response = loop / (1.0 + loop)

        recovered = design.recover_plant(response, frequency)

        self.assertAlmostEqual(abs(recovered / plant), 1.0, places=10)
        self.assertAlmostEqual(cmath.phase(recovered / plant), 0.0, places=10)

    def test_pi_design_hits_crossover_and_phase_margin(self):
        frequency = 30.0
        plant = design.from_db_phase(44.0, -108.0)
        gains = design.design_pi(plant, frequency, 65.0)

        loop = design.controller(gains["kp"], gains["ki"], frequency) * plant

        self.assertAlmostEqual(abs(loop), 1.0, places=10)
        self.assertAlmostEqual(
            180.0 + math.degrees(cmath.phase(loop)), 65.0, places=10,
        )

    def test_pi_rejects_required_phase_lead(self):
        plant = design.from_db_phase(40.0, -130.0)

        with self.assertRaises(ValueError):
            design.design_pi(plant, 60.0, 65.0)

    def test_report_generates_six_bounded_candidates(self):
        report = design.build_report(design.DEFAULT_SOURCE)

        self.assertEqual(len(report["candidates"]), 6)
        self.assertEqual(
            tuple(
                item["target_closed_loop_bandwidth_hz"]
                for item in report["candidates"]
            ),
            design.TARGET_BANDWIDTHS_HZ,
        )
        for candidate in report["candidates"]:
            self.assertAlmostEqual(
                candidate["predicted_closed_loop_bandwidth_hz"],
                candidate["target_closed_loop_bandwidth_hz"],
                places=6,
            )
        self.assertLess(
            report["boundary_evidence"]["80hz_maximum_pi_phase_margin_deg"],
            design.TARGET_PHASE_MARGIN_DEG,
        )
        self.assertFalse(report["hardware_executed"])


if __name__ == "__main__":
    unittest.main()
