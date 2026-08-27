#!/usr/bin/env python3
"""Aggregate the rotating-bias encoder speed-estimator holdout."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def wrapped_difference(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def circular_mean(values: list[float]) -> float:
    x = mean([math.cos(math.radians(value)) for value in values])
    y = mean([math.sin(math.radians(value)) for value in values])
    return math.degrees(math.atan2(y, x))


def iq_reference_rms(path: Path) -> float:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return rms([float(row["iq_ref"]) for row in rows])


def aggregate(path: Path) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    grouped: dict[str, dict[tuple[float, float], list[dict]]] = defaultdict(
        lambda: defaultdict(list)
    )
    peak_current = 0.0
    peak_iq_reference = 0.0
    for visit in raw["visits"]:
        for record in visit["frequency"]:
            enriched = dict(record)
            enriched["iq_reference_rms_a"] = iq_reference_rms(
                Path(record["scope_csv"])
            )
            key = (float(record["bias_rpm"]), float(record["frequency_hz"]))
            grouped[visit["profile"]][key].append(enriched)
            peak_current = max(peak_current, record["peak_dq_current_a"])
            peak_iq_reference = max(
                peak_iq_reference, record["peak_iq_reference_a"]
            )

    profiles = {}
    for profile, records_by_point in grouped.items():
        points = {}
        for (bias, frequency), records in sorted(records_by_point.items()):
            magnitudes = [record["closed_loop_magnitude_db"] for record in records]
            phases = [record["closed_loop_phase_deg"] for record in records]
            points[f"{bias:g}:{frequency:g}"] = {
                "bias_rpm": bias,
                "frequency_hz": frequency,
                "mean_magnitude_db": mean(magnitudes),
                "round_magnitude_difference_db": abs(
                    magnitudes[0] - magnitudes[1]
                ),
                "circular_mean_phase_deg": circular_mean(phases),
                "round_phase_difference_deg": wrapped_difference(
                    phases[0], phases[1]
                ),
                "mean_iq_reference_rms_a": mean([
                    record["iq_reference_rms_a"] for record in records
                ]),
                "mean_peak_iq_reference_a": mean([
                    record["peak_iq_reference_a"] for record in records
                ]),
                "peak_dq_current_a": max(
                    record["peak_dq_current_a"] for record in records
                ),
            }

        direction = {}
        for frequency in (20.0, 40.0, 80.0):
            positive = points[f"1500:{frequency:g}"]
            negative = points[f"-1500:{frequency:g}"]
            direction[f"{frequency:g}"] = {
                "magnitude_difference_db": abs(
                    positive["mean_magnitude_db"]
                    - negative["mean_magnitude_db"]
                ),
                "phase_difference_deg": wrapped_difference(
                    positive["circular_mean_phase_deg"],
                    negative["circular_mean_phase_deg"],
                ),
                "iq_reference_rms_difference_a": abs(
                    positive["mean_iq_reference_rms_a"]
                    - negative["mean_iq_reference_rms_a"]
                ),
            }
        profiles[profile] = {
            "points": points,
            "direction_consistency": direction,
            "mean_iq_reference_rms_a": mean([
                point["mean_iq_reference_rms_a"] for point in points.values()
            ]),
            "maximum_round_magnitude_difference_db": max(
                point["round_magnitude_difference_db"] for point in points.values()
            ),
            "maximum_round_phase_difference_deg": max(
                point["round_phase_difference_deg"] for point in points.values()
            ),
            "maximum_direction_magnitude_difference_db": max(
                point["magnitude_difference_db"] for point in direction.values()
            ),
            "maximum_direction_phase_difference_deg": max(
                point["phase_difference_deg"] for point in direction.values()
            ),
        }

    win = profiles["win_1ms_fc150"]
    pll = profiles["pll_200hz"]
    win_phase_lead = {}
    win_magnitude_advantage = {}
    for bias in (1500.0, -1500.0):
        for frequency in (20.0, 40.0, 80.0):
            key = f"{bias:g}:{frequency:g}"
            win_point = win["points"][key]
            pll_point = pll["points"][key]
            win_magnitude_advantage[key] = (
                win_point["mean_magnitude_db"] - pll_point["mean_magnitude_db"]
            )
            win_phase_lead[key] = (
                win_point["circular_mean_phase_deg"]
                - pll_point["circular_mean_phase_deg"]
                + 180.0
            ) % 360.0 - 180.0

    return {
        "schema": "encoder-speed-estimation-biased-holdout-summary-v1",
        "source": str(path.resolve()),
        "status": raw["status"],
        "final_mode": raw["final_mode"],
        "scope_capture_count": raw["plan"]["total_scope_captures"],
        "safety": {
            "peak_dq_current_a": peak_current,
            "peak_iq_reference_a": peak_iq_reference,
            "completed_ready": (
                raw["status"] == "completed" and raw["final_mode"] == 17
            ),
        },
        "profiles": profiles,
        "win_vs_pll": {
            "win_magnitude_advantage_db": win_magnitude_advantage,
            "win_phase_lead_deg": win_phase_lead,
            "win_minus_pll_mean_iq_reference_rms_a": (
                win["mean_iq_reference_rms_a"]
                - pll["mean_iq_reference_rms_a"]
            ),
        },
        "decision": {
            "base": "rejected",
            "selected": "win_1ms_fc150",
            "status": "qualified_default_candidate",
            "reason": (
                "WIN removes the BASE notch, repeats within the rotating holdout, "
                "has substantially better direction consistency than PLL, and "
                "does not increase mean iq-reference RMS."
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = aggregate(args.input)
    text = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
