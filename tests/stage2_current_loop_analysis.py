#!/usr/bin/env python3
"""Recompute Stage-2 grid metrics from saved scope CSV files."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean

import scope_capture as scope
from scope_current_step import summarize


def load_scope_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    names = [name for name in scope.CHANNEL_NAMES if name in rows[0]]
    samples = [tuple(float(row[name]) for name in names) for row in rows]
    sample_period = float(rows[1]["time_s"]) - float(rows[0]["time_s"])
    return samples, sample_period


def relative_difference(a, b):
    return abs(a - b) / max(abs(a), abs(b), 1.0e-12)


def analyze_profile(profile, records, zero_records, active_zero, criteria):
    scored = [record for record in records if abs(record["target_id_a"]) >= 1.0]
    round_metrics = {}
    for round_index in sorted({record["round"] for record in scored}):
        selected = [record for record in scored if record["round"] == round_index]
        positive_rise = mean(record["rise_time_10_90_sec"] for record in selected
                             if record["target_id_a"] > 0.0)
        negative_rise = mean(record["rise_time_10_90_sec"] for record in selected
                             if record["target_id_a"] < 0.0)
        round_metrics[str(round_index)] = {
            "mean_rise_time_sec": mean(
                record["rise_time_10_90_sec"] for record in selected
            ),
            "positive_mean_rise_time_sec": positive_rise,
            "negative_mean_rise_time_sec": negative_rise,
            "aggregate_sign_rise_difference_ratio": relative_difference(
                positive_rise, negative_rise
            ),
            "mean_overshoot_pct": mean(
                record["filtered_overshoot_pct"] for record in selected
            ),
        }

    first_round, second_round = round_metrics.values()
    aggregate_repeat_rise = relative_difference(
        first_round["mean_rise_time_sec"], second_round["mean_rise_time_sec"]
    )
    aggregate_repeat_overshoot_pp = abs(
        first_round["mean_overshoot_pct"] - second_round["mean_overshoot_pct"]
    )
    profile_zero = [record for record in zero_records if record["profile"] == profile]
    mean_id_noise = mean(record["id_noise_rms_a"] for record in profile_zero)
    mean_voltage_noise = mean(record["voltage_rms_v"] for record in profile_zero)
    active_id_noise = mean(record["id_noise_rms_a"] for record in active_zero)
    active_voltage_noise = mean(record["voltage_rms_v"] for record in active_zero)
    max_sign_asymmetry = max(
        value["aggregate_sign_rise_difference_ratio"]
        for value in round_metrics.values()
    )
    max_settling = max(
        record["noise_limited_settling_time_2pct_sec"] for record in scored
    )
    checks = {
        "overshoot": max(record["filtered_overshoot_pct"] for record in scored)
                     <= criteria["max_overshoot_pct"],
        "settling": max_settling <= criteria["max_settling_sec"],
        "steady_error_rms": max(record["steady_error_rms_a"] for record in scored)
                            <= criteria["max_steady_error_rms_a"],
        "aggregate_sign_symmetry": max_sign_asymmetry
                                   <= criteria["max_sign_rise_difference"],
        "aggregate_repeat_rise": aggregate_repeat_rise
                                 <= criteria["max_repeat_rise_difference"],
        "aggregate_repeat_overshoot": aggregate_repeat_overshoot_pp
                                      <= criteria["max_repeat_overshoot_pp"],
        "mean_id_noise_vs_active": mean_id_noise
                                   <= criteria["max_noise_ratio"] * active_id_noise,
        "mean_control_voltage_noise_vs_active": mean_voltage_noise
        <= criteria["max_noise_ratio"] * active_voltage_noise,
        "peak_current": max(record["raw_peak_current_a"] for record in records)
                        < criteria["max_peak_current_a"],
        "voltage_margin": all(not record["voltage_saturated"] for record in records),
    }
    return {
        "bandwidth_hz": scored[0]["bandwidth_hz"],
        "mean_rise_time_sec": mean(
            record["rise_time_10_90_sec"] for record in scored
        ),
        "max_settling_time_sec": max_settling,
        "max_overshoot_pct": max(
            record["filtered_overshoot_pct"] for record in scored
        ),
        "max_steady_error_rms_a": max(
            record["steady_error_rms_a"] for record in scored
        ),
        "max_aggregate_sign_rise_difference_ratio": max_sign_asymmetry,
        "aggregate_repeat_rise_difference_ratio": aggregate_repeat_rise,
        "aggregate_repeat_overshoot_difference_pp": aggregate_repeat_overshoot_pp,
        "mean_zero_id_noise_rms_a": mean_id_noise,
        "mean_zero_id_noise_ratio_vs_active": mean_id_noise / active_id_noise,
        "mean_zero_voltage_rms_v": mean_voltage_noise,
        "mean_zero_voltage_ratio_vs_active": mean_voltage_noise / active_voltage_noise,
        "raw_peak_current_a": max(record["raw_peak_current_a"] for record in records),
        "raw_peak_rpm": max(record["raw_peak_rpm"] for record in records),
        "round_metrics": round_metrics,
        "checks": checks,
        "passes_aggregate_screen": all(checks.values()),
    }


def build_analysis(result_path):
    raw = json.loads(result_path.read_text(encoding="utf-8"))
    recomputed = []
    for old in raw["records"]:
        samples, sample_period = load_scope_csv(Path(old["csv"]))
        record = summarize(samples, old["target_id_a"], sample_period)
        record.update({key: value for key, value in old.items() if key not in record})
        recomputed.append(record)

    active_zero = [record for record in raw["zero_records"]
                   if record["profile"] == "active_legacy"]
    profiles = {}
    for profile in sorted({record["profile"] for record in recomputed}):
        profiles[profile] = analyze_profile(
            profile,
            [record for record in recomputed if record["profile"] == profile],
            raw["zero_records"],
            active_zero,
            raw["acceptance_criteria"],
        )

    passing = [value for value in profiles.values()
               if value["bandwidth_hz"] is not None
               and value["passes_aggregate_screen"]]
    selected = max(passing, key=lambda value: value["bandwidth_hz"], default=None)
    return {
        "raw_result": result_path.as_posix(),
        "analysis_type": "post-experiment quantization-aware aggregate screen",
        "caveat": (
            "The predeclared pointwise maximum criteria remain the primary result. "
            "This secondary screen uses interpolated crossings and aggregate sign/run "
            "means because 25 us sampling quantizes 0.1-0.2 ms rise times."
        ),
        "sample_period_sec": 25.0e-6,
        "measured_bus_voltage_v": raw["measured_bus_voltage_v"],
        "runtime_rejections": raw["runtime_rejections"],
        "skipped_profiles": raw["skipped_profiles"],
        "final_mode": raw["final_mode"],
        "criteria": raw["acceptance_criteria"],
        "profiles": profiles,
        "aggregate_screen_selection": {
            "profile": None if selected is None else next(
                name for name, value in profiles.items() if value is selected
            ),
            "bandwidth_hz": None if selected is None else selected["bandwidth_hz"],
            "status": (
                "frequency-response candidate only; not approved for PI writeback"
                if selected is not None else "no candidate passed aggregate screen"
            ),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    analysis = build_analysis(args.result)
    text = json.dumps(analysis, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(json.dumps({
            "output": str(args.output.resolve()),
            "selection": analysis["aggregate_screen_selection"],
        }, indent=2))
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
