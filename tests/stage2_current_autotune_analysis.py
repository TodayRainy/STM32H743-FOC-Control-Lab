#!/usr/bin/env python3
"""Re-evaluate a Stage-2 current autotuning hardware run offline."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

from scope_capture import CHANNEL_NAMES
from stage2_current_autotune_validation import (
    ACTIVE_LEGACY_AXES,
    ACTIVE_LEGACY_NAME,
    build_d028_criteria,
    build_d029_criteria,
    load_design,
    rank_passing_candidates,
    summarize_candidates,
    summarize_pwm_off,
)


def read_scope_csv(path: Path) -> list[tuple[float, ...]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        names = [name for name in CHANNEL_NAMES if name in (rows.fieldnames or [])]
        return [tuple(float(row[name]) for name in names) for row in rows]


def maximum(records: list[dict], key: str) -> float | None:
    values = [float(item[key]) for item in records if item.get(key) is not None]
    return max(values) if values else None


def average(records: list[dict], key: str) -> float | None:
    values = [float(item[key]) for item in records if item.get(key) is not None]
    return sum(values) / len(values) if values else None


def comparison_profile_metrics(document: dict, summary: dict,
                               name: str) -> dict:
    steps = [item for item in document["step_records"]
             if item["profile"] == name and abs(item["target_id_a"]) >= 1.5]
    zeros = [item for item in document["zero_records"]
             if item["profile"] == name]
    frf = list(summary["frf_summaries"].values())
    bandwidths = {
        axis: [item["closed_loop_bandwidth_hz"] for key, item
               in summary["frf_summaries"].items()
               if key.endswith(f"_{axis}")]
        for axis in ("d", "q")
    }
    return {
        "step_points": len(steps),
        "mean_rise_time_10_90_sec": average(steps, "rise_time_10_90_sec"),
        "max_rise_time_10_90_sec": maximum(steps, "rise_time_10_90_sec"),
        "mean_noise_limited_settling_sec": average(
            steps, "noise_limited_settling_time_2pct_sec"
        ),
        "max_noise_limited_settling_sec": maximum(
            steps, "noise_limited_settling_time_2pct_sec"
        ),
        "max_filtered_overshoot_pct": maximum(
            steps, "filtered_overshoot_pct"
        ),
        "mean_total_sampled_steady_error_rms_a": average(
            steps, "steady_error_rms_a"
        ),
        "mean_zero_id_noise_rms_a": average(zeros, "id_noise_rms_a"),
        "mean_zero_iq_noise_rms_a": average(zeros, "iq_noise_rms_a"),
        "max_step_voltage_v": maximum(steps, "raw_peak_voltage_v"),
        "d_closed_loop_bandwidths_hz": bandwidths["d"],
        "q_closed_loop_bandwidths_hz": bandwidths["q"],
        "minimum_loop_crossover_hz": min(
            item["loop_crossover_hz"] for item in frf
            if item["loop_crossover_hz"] is not None
        ),
        "minimum_phase_margin_deg": min(
            item["phase_margin_deg"] for item in frf
            if item["phase_margin_deg"] is not None
        ),
        "maximum_observed_sensitivity": max(
            item["max_observed_sensitivity"] for item in frf
        ),
        "max_sign_rise_difference_ratio": (
            summary["max_sign_rise_difference_ratio"]
        ),
        "max_repeat_rise_difference_ratio": (
            summary["max_repeat_rise_difference_ratio"]
        ),
    }


def build_report(result_path: Path, criteria_version: str = "d028") -> dict:
    document = json.loads(result_path.read_text(encoding="utf-8"))
    design_path = Path(document["design"])
    _, candidates, available_names = load_design(design_path)
    plan = document["plan"]
    if ACTIVE_LEGACY_NAME in plan.get("candidate_names", []):
        candidates[ACTIVE_LEGACY_NAME] = {
            "name": ACTIVE_LEGACY_NAME,
            "method": "comparison_reference",
            "target_crossover_hz": None,
            "target_phase_margin_deg": None,
            "axes": ACTIVE_LEGACY_AXES,
        }
        available_names = available_names + [ACTIVE_LEGACY_NAME]
    names = list(plan.get("candidate_names", available_names))
    missing = [name for name in names if name not in candidates]
    if missing:
        raise ValueError(f"result references unavailable candidates: {missing}")
    rounds = len(plan["run_plan"])
    steps = tuple(float(value) for value in plan["d_steps_a"])
    d_frequencies = tuple(float(value) for value in plan["d_frequencies_hz"])
    q_frequencies = tuple(float(value) for value in plan["q_frequencies_hz"])

    pwm_off_csv = Path(document["pwm_off_noise"]["csv"])
    pwm_off_noise = summarize_pwm_off(read_scope_csv(pwm_off_csv))
    pwm_off_noise["csv"] = str(pwm_off_csv.resolve())
    criteria_builder = (
        build_d029_criteria if criteria_version == "d029"
        else build_d028_criteria
    )
    criteria = criteria_builder(
        steps, float(plan["runtime_abort_gates"]["peak_current_a"])
    )
    summaries = summarize_candidates(
        names,
        candidates,
        rounds,
        steps,
        d_frequencies,
        q_frequencies,
        document["step_records"],
        document["zero_records"],
        document["frf_records"],
        pwm_off_noise,
        criteria,
    )

    qualification_names = list(plan.get("qualification_candidate_names", names))
    passing = [name for name in qualification_names
               if summaries[name]["passes_all_checks"]]
    ranking = rank_passing_candidates(qualification_names, summaries)
    failed_counts = Counter()
    diagnostic_rows = []
    for name in names:
        failed = [key for key, value in summaries[name]["checks"].items()
                  if not value]
        failed_counts.update(failed)
        diagnostic_rows.append({
            "name": name,
            "failed_check_count": len(failed),
            "failed_checks": failed,
            "not_a_recommendation": True,
        })
    diagnostic_rows.sort(key=lambda item: (
        item["failed_check_count"], names.index(item["name"])
    ))

    all_records = (
        document["step_records"]
        + document["zero_records"]
        + document["frf_records"]
    )
    experiment = plan.get("experiment", "screen")
    if experiment == "holdout":
        status = "independent_holdout_pass" if passing else "independent_holdout_failed"
    elif experiment == "comparison":
        status = "active_comparison_complete" if passing else "comparison_repeat_failed"
    elif experiment == "robustness":
        status = "robustness_screen_pass" if passing else "robustness_screen_failed"
    elif experiment == "robustness_holdout":
        status = (
            "robustness_holdout_pass" if passing else "robustness_holdout_failed"
        )
    else:
        status = "coarse_screen_pass" if passing else "requirement_not_met"
    report = {
        "schema": (
            f"stage2-current-autotune-hardware-analysis-{criteria_version}-v1"
        ),
        "source": str(result_path.resolve()),
        "design": str(design_path.resolve()),
        "experiment": experiment,
        "status": status,
        "complete_capture_counts": {
            "steps": len(document["step_records"]),
            "pwm_on_zero": len(document["zero_records"]),
            "frf": len(document["frf_records"]),
            "expected_total_including_pwm_off": plan["planned_scope_captures"],
        },
        "safety": {
            "measured_bus_voltage_v": document["measured_bus_voltage_v"],
            "peak_current_a": maximum(all_records, "raw_peak_current_a"),
            "peak_rpm": maximum(all_records, "raw_peak_rpm"),
            "peak_voltage_v": maximum(
                document["step_records"] + document["frf_records"],
                "raw_peak_voltage_v",
            ),
            "runtime_performance_flags": len(document["runtime_rejections"]),
            "error": document["error"],
            "final_mode": document["final_mode"],
        },
        "pwm_off_noise_corrected": pwm_off_noise,
        "acceptance_criteria": criteria,
        "candidate_summaries": summaries,
        "passing_candidates": passing,
        "coarse_ranking": ranking,
        "coarse_leader": ranking[0]["name"] if ranking else None,
        "failed_check_candidate_counts": dict(sorted(failed_counts.items())),
        "diagnostic_order": diagnostic_rows,
        "decision": (
            "The frozen leading candidate passed the independent holdout; "
            "it may now enter the separately declared active-PI comparison."
            if experiment == "holdout" and passing else
            "The frozen leading candidate failed the independent holdout; do not "
            "write gains or alter the frozen gates from this result."
            if experiment == "holdout" else
            "The active-PI comparison is complete. Use it only to decide default "
            "writeback; it does not alter the frozen no-baseline recommendation."
            if experiment == "comparison" and passing else
            "The recommended candidate did not retain its absolute qualification "
            "during comparison; do not write it as the default PI."
            if experiment == "comparison" else
            "The leading robustness candidate passed the unchanged D-029 gates; "
            "freeze it for the one declared independent holdout before writeback."
            if experiment == "robustness" and passing else
            "No robustness candidate passed every unchanged D-029 gate. Keep the "
            "active PI and close this current-loop tuning iteration."
            if experiment == "robustness" else
            "The D-032 leader passed its one frozen independent holdout and may "
            "enter the separately discussed default-writeback decision."
            if experiment == "robustness_holdout" and passing else
            "The D-032 leader failed its one frozen independent holdout. Keep the "
            "active PI and close current-loop candidate search."
            if experiment == "robustness_holdout" else
            f"No PI is recommended. Diagnose the failed {criteria_version.upper()} "
            "gates before another hardware experiment."
            if not passing else
            f"The leading candidate is a {criteria_version.upper()} coarse-screen "
            "result, not the final PI. Confirm it in the declared holdout before "
            "writing gains."
        ),
    }
    if experiment == "comparison":
        report["active_comparison"] = {
            name: comparison_profile_metrics(document, summaries[name], name)
            for name in names
        }
    return report


def main():
    parser = argparse.ArgumentParser(
        description="Re-evaluate one current-autotuning hardware result",
    )
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--criteria-version", choices=("d028", "d029"),
                        default="d028")
    args = parser.parse_args()

    report = build_report(args.result, args.criteria_version)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": report["status"],
        "passing_candidates": report["passing_candidates"],
        "failed_check_candidate_counts": report["failed_check_candidate_counts"],
        "diagnostic_order": report["diagnostic_order"],
        "safety": report["safety"],
        "output": None if args.output is None else str(args.output.resolve()),
    }, indent=2))


if __name__ == "__main__":
    main()
