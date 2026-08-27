#!/usr/bin/env python3
"""Combine Stage-2 current step and FRF evidence into a tracked summary."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def build_summary(frf_path, step_path):
    frf = json.loads(frf_path.read_text(encoding="utf-8"))
    step = json.loads(step_path.read_text(encoding="utf-8"))
    expected_points = len(frf["frequencies_hz"])
    profiles = {}
    for profile in frf["profiles"]:
        records = [record for record in frf["records"] if record["profile"] == profile]
        closed_magnitude = [record["closed_loop_magnitude_db"] for record in records]
        closed_phase = [record["closed_loop_phase_deg"] for record in records]
        profiles[profile] = {
            "summary": frf["profile_summaries"][profile],
            "point_count": len(records),
            "complete": len(records) == expected_points,
            "max_reference_amplitude_error_a": max(
                abs(record["reference_amplitude_a"] - frf["amplitude_a"])
                for record in records
            ),
            "closed_loop_magnitude_nonincreasing": all(
                second <= first + 0.05
                for first, second in zip(closed_magnitude, closed_magnitude[1:])
            ),
            "closed_loop_phase_monotonic": all(
                second <= first + 1.0
                for first, second in zip(closed_phase, closed_phase[1:])
            ),
            "frequency_points": [{
                "frequency_hz": record["frequency_hz"],
                "reference_amplitude_a": record["reference_amplitude_a"],
                "closed_loop_magnitude_db": record["closed_loop_magnitude_db"],
                "closed_loop_phase_deg": record["closed_loop_phase_deg"],
                "open_loop_magnitude_db": record["open_loop_magnitude_db"],
                "open_loop_phase_deg": record["open_loop_phase_deg"],
                "cross_axis_ratio_db": record["cross_axis_ratio_db"],
                "voltage_amplitude_v": record["voltage_amplitude_v"],
            } for record in records],
        }

    candidates = []
    for profile, frf_profile in profiles.items():
        bandwidth_hz = frf_profile["summary"].get("loop_crossover_hz")
        step_profile = step["profiles"].get(profile)
        if step_profile is None or step_profile["bandwidth_hz"] is None:
            continue
        candidates.append({
            "profile": profile,
            "design_bandwidth_hz": step_profile["bandwidth_hz"],
            "passes_step_aggregate_screen": step_profile["passes_aggregate_screen"],
            "passes_phase_margin_55deg": frf_profile["summary"][
                "passes_phase_margin_55deg"
            ],
            "measured_loop_crossover_hz": bandwidth_hz,
            "measured_phase_margin_deg": frf_profile["summary"]["phase_margin_deg"],
        })
    passing = [candidate for candidate in candidates
               if candidate["passes_step_aggregate_screen"]
               and candidate["passes_phase_margin_55deg"]]
    selected = max(passing, key=lambda item: item["design_bandwidth_hz"], default=None)
    return {
        "frf_raw_result": frf_path.as_posix(),
        "step_grid_baseline": step_path.as_posix(),
        "method": "retain predeclared step screen, then require measured phase margin >=55 deg",
        "safety": {
            "measured_bus_voltage_v": frf["measured_bus_voltage_v"],
            "raw_peak_current_a": max(record["raw_peak_current_a"]
                                      for record in frf["records"]),
            "raw_peak_rpm": max(record["raw_peak_rpm"] for record in frf["records"]),
            "raw_peak_voltage_v": max(record["raw_peak_voltage_v"]
                                      for record in frf["records"]),
            "error": frf["error"],
            "final_mode": frf["final_mode"],
        },
        "profiles": profiles,
        "candidates": candidates,
        "selection": {
            "profile": None if selected is None else selected["profile"],
            "design_bandwidth_hz": (
                None if selected is None else selected["design_bandwidth_hz"]
            ),
            "status": (
                "combined step+FRF candidate; default PI remains unchanged pending rotating feedforward validation"
                if selected is not None else "no candidate passed both evidence gates"
            ),
        },
        "limitations": [
            "stationary no-load d-axis only",
            "gain margin not measured because open-loop phase did not reach -180 deg by 4 kHz",
            "T/(1-T) assumes the measured reference-to-current response represents the closed loop",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frf_result", type=Path)
    parser.add_argument("step_grid_baseline", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    summary = build_summary(args.frf_result, args.step_grid_baseline)
    text = json.dumps(summary, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(json.dumps({
            "output": str(args.output.resolve()),
            "selection": summary["selection"],
        }, indent=2))
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
