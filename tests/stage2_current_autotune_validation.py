#!/usr/bin/env python3
"""Continuous hardware screening for the no-baseline current-PI autotuner.

The script consumes the 6--8 candidates produced by stage2_current_autotune.py.
It does not use the firmware's active/default PI as a candidate.  Every candidate
is visited in design order and reverse order, with d-axis steps plus d/q-axis
closed-loop frequency-response captures in one powered session.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import (
    MODE_READY,
    MODE_TORQUE_RUN,
    summarize_zero,
    wait_mode,
)
from stage2_current_frequency_response import analyze_capture, summarize_profile
from stage2_current_loop_design import build_controllers
from stage2_current_loop_validation import capture_step, set_profile


DEFAULT_DESIGN = Path(
    "tests/baselines/stage2_current_autotune_d029_design_20260824.json"
)
DEFAULT_D032_DESIGN = Path(
    "tests/baselines/stage2_current_autotune_d032_design_20260824.json"
)
DEFAULT_STEPS_A = (
    0.75, -0.75, 1.5, -1.5, 2.25, -2.25, 3.0, -3.0,
)
DEFAULT_D_FREQUENCIES_HZ = (
    100.0, 150.0, 330.0, 500.0, 680.0, 800.0, 900.0,
    1000.0, 1100.0, 1250.0, 1500.0, 2000.0, 3000.0, 4000.0,
)
DEFAULT_Q_FREQUENCIES_HZ = (
    100.0, 150.0, 330.0, 500.0, 680.0, 800.0, 900.0,
    1000.0, 1100.0, 1250.0, 1500.0, 2000.0, 3000.0, 4000.0,
)
HOLDOUT_CANDIDATE = "measured_fc1000_pm75"
D032_HOLDOUT_CANDIDATE = "measured_fc0925_pm80"
HOLDOUT_STEPS_A = (
    1.0, -1.0, 2.0, -2.0, 2.75, -2.75,
)
HOLDOUT_FREQUENCIES_HZ = (
    80.0, 120.0, 220.0, 410.0, 590.0, 740.0, 850.0,
    950.0, 1050.0, 1175.0, 1375.0, 1750.0, 2500.0, 3500.0,
)
COMPARISON_FREQUENCIES_HZ = HOLDOUT_FREQUENCIES_HZ + (
    4000.0, 4500.0, 5000.0,
)
ACTIVE_LEGACY_NAME = "active_legacy"
ACTIVE_LEGACY_AXES = build_controllers(())[ACTIVE_LEGACY_NAME]
SCOPE_SAMPLES = 4096
FOC_SAMPLE_HZ = 40000.0
POLE_PAIRS = 10.0


def load_design(path: Path) -> tuple[dict, dict[str, dict], list[str]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    names = list(document["hardware_candidate_names"])
    candidates = {item["name"]: item for item in document["candidates"]}
    if not 6 <= len(names) <= 8 or len(set(names)) != len(names):
        raise ValueError("autotuning screen requires 6 to 8 unique candidates")
    missing = [name for name in names if name not in candidates]
    if missing:
        raise ValueError(f"candidate definitions are missing: {missing}")
    for name in names:
        axes = candidates[name].get("axes", {})
        for axis in ("d", "q"):
            gains = axes.get(axis, {})
            if not all(
                math.isfinite(float(gains.get(key, 0.0)))
                and float(gains[key]) > 0.0
                for key in ("kp", "ki")
            ):
                raise ValueError(f"candidate {name} has invalid {axis}-axis gains")
    if "active_legacy" in names:
        raise ValueError("active/default PI must not enter the no-baseline candidate set")
    return document, candidates, names


def build_run_plan(names: list[str], rounds: int) -> list[dict]:
    plan = [{"round": 1, "order": list(names)}]
    if rounds == 2:
        plan.append({"round": 2, "order": list(reversed(names))})
    return plan


def select_candidate_names(available: list[str], requested: list[str] | None,
                           experiment: str) -> list[str]:
    names = list(requested) if requested else (
        [HOLDOUT_CANDIDATE] if experiment == "holdout" else
        [D032_HOLDOUT_CANDIDATE] if experiment == "robustness_holdout" else
        [HOLDOUT_CANDIDATE, ACTIVE_LEGACY_NAME]
        if experiment == "comparison" else list(available)
    )
    if len(set(names)) != len(names):
        raise ValueError("candidate selection contains duplicates")
    missing = [name for name in names if name not in available]
    if missing:
        raise ValueError(f"selected candidates are unavailable: {missing}")
    if not names:
        raise ValueError("at least one candidate must be selected")
    return names


def peak_rpm(samples) -> float:
    return max(abs(sample[9]) for sample in samples) * 60.0 / (
        2.0 * math.pi * POLE_PAIRS
    )


def sine_amplitude(axis: str, maximum_a: float, frequency_hz: float,
                   q_full_amplitude_from_hz: float) -> float:
    if axis == "d":
        return maximum_a
    # Limit low-frequency torque impulse while preserving the two points needed
    # to define the q-axis tracking plateau.  At and above the threshold the
    # full requested q-axis amplitude is used.
    return maximum_a * min(1.0, frequency_hz / q_full_amplitude_from_hz)


def capture_zero(port, timeout: float, csv_path: Path):
    scope.arm_scope(port, 1)
    time.sleep(SCOPE_SAMPLES / FOC_SAMPLE_HZ + 0.05)
    capture, samples = scope.dump_scope(port, timeout)
    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    scope.write_csv(csv_path, samples, sample_period)
    return summarize_zero(samples), samples


def summarize_pwm_off(samples) -> dict:
    # In READY the motor debug object's Iqd fields are intentionally stale/zero,
    # while the phase-current acquisition still runs.  Reconstruct dq from the
    # recorded ia/ib and encoder angle using the same amplitude-invariant
    # Clarke/Park convention as foc_math.hpp.
    q_values = []
    d_values = []
    for sample in samples:
        alpha = sample[0]
        beta = (sample[0] + 2.0 * sample[1]) / math.sqrt(3.0)
        cosine = math.cos(sample[8])
        sine = math.sin(sample[8])
        q_values.append(cosine * beta - sine * alpha)
        d_values.append(cosine * alpha + sine * beta)
    q_mean = sum(q_values) / len(q_values)
    d_mean = sum(d_values) / len(d_values)
    q_noise = math.sqrt(sum((value - q_mean) ** 2 for value in q_values)
                        / len(q_values))
    d_noise = math.sqrt(sum((value - d_mean) ** 2 for value in d_values)
                        / len(d_values))
    return {
        "measurement": "pwm_off_phase_current_noise_reconstructed_to_dq",
        "iq_mean_a": q_mean,
        "id_mean_a": d_mean,
        "iq_noise_rms_a": q_noise,
        "id_noise_rms_a": d_noise,
        "raw_peak_current_a": max(
            math.hypot(q, d) for q, d in zip(q_values, d_values)
        ),
        "source_channels": ["ia", "ib", "elec_angle"],
    }


def build_d028_criteria(steps: tuple[float, ...], max_current_a: float) -> dict:
    """Return the compact, frozen D-028 current-loop qualification criteria."""
    return {
        "metric_version": "d028",
        "max_overshoot_pct": 10.0,
        "settling_bandwidth_factor": 1.0,
        "tracking_noise_multiplier": 3.0,
        "tracking_error_fraction": 0.02,
        "ripple_noise_multiplier": 2.0,
        "ripple_current_fraction": 0.02,
        "pwm_off_noise_multiplier": 2.0,
        "zero_noise_current_fraction": 0.01,
        "maximum_validation_current_a": max(abs(value) for value in steps),
        "max_sign_rise_difference": 0.15,
        "max_repeat_rise_difference": 0.15,
        "min_phase_margin_deg": 60.0,
        "max_sensitivity": 1.7,
        "rl_bandwidth_tolerance": 0.25,
        "measured_crossover_tolerance": 0.15,
        "max_peak_current_a": max_current_a,
    }


def build_d029_criteria(steps: tuple[float, ...], max_current_a: float) -> dict:
    """Return the frozen D-029 current-loop qualification criteria."""
    criteria = build_d028_criteria(steps, max_current_a)
    criteria.update({
        "metric_version": "d029",
        "settling_bandwidth_factor": 1.25,
        "excellent_settling_bandwidth_factor": 1.0,
    })
    return criteria


def read_csv_columns(path: Path, names: tuple[str, ...]) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        result = {name: [] for name in names}
        for row in rows:
            for name in names:
                result[name].append(float(row[name]))
    return result


def frequency_band_rms(values: list[float], sample_rate_hz: float,
                       cutoff_hz: float) -> tuple[float, float]:
    """Split RMS at a DFT-bin boundary while retaining DC tracking bias."""
    count = len(values)
    if count == 0 or sample_rate_hz <= 0.0 or cutoff_hz < 0.0:
        raise ValueError("band RMS requires samples, positive rate and cutoff")
    total_mean_square = sum(value * value for value in values) / count
    maximum_bin = min(int(cutoff_hz * count / sample_rate_hz), count // 2)
    in_band_energy = 0.0
    for bin_index in range(maximum_bin + 1):
        angle_scale = -2.0 * math.pi * bin_index / count
        real = 0.0
        imag = 0.0
        for index, value in enumerate(values):
            angle = angle_scale * index
            real += value * math.cos(angle)
            imag += value * math.sin(angle)
        factor = 1.0 if bin_index == 0 or (
            count % 2 == 0 and bin_index == count // 2
        ) else 2.0
        in_band_energy += factor * (real * real + imag * imag)
    in_band_mean_square = min(
        total_mean_square, in_band_energy / (count * count)
    )
    out_band_mean_square = max(0.0, total_mean_square - in_band_mean_square)
    return math.sqrt(in_band_mean_square), math.sqrt(out_band_mean_square)


def steady_d_error_bands(csv_path: Path, bandwidth_hz: float) -> tuple[float, float]:
    columns = read_csv_columns(csv_path, ("id", "id_ref"))
    start = 3 * len(columns["id"]) // 4
    errors = [reference - measured for measured, reference in zip(
        columns["id"][start:], columns["id_ref"][start:]
    )]
    return frequency_band_rms(errors, FOC_SAMPLE_HZ, bandwidth_hz)


def pwm_off_d_bands(csv_path: Path, bandwidth_hz: float) -> tuple[float, float]:
    columns = read_csv_columns(csv_path, ("ia", "ib", "elec_angle"))
    start = 3 * len(columns["ia"]) // 4
    values = []
    for ia, ib, angle in zip(
            columns["ia"][start:], columns["ib"][start:],
            columns["elec_angle"][start:]):
        beta = (ia + 2.0 * ib) / math.sqrt(3.0)
        values.append(math.cos(angle) * ia + math.sin(angle) * beta)
    return frequency_band_rms(values, FOC_SAMPLE_HZ, bandwidth_hz)


def capture_sine(port, axis: str, amplitude_a: float, frequency_hz: float,
                 settle_cycles: float, timeout: float, csv_path: Path):
    proto.send_command(
        port,
        f"CURRENT SINE {axis.upper()} {amplitude_a:g} {frequency_hz:g}",
    )
    time.sleep(max(0.02, settle_cycles / frequency_hz))
    scope.arm_scope(port, 1)
    time.sleep(SCOPE_SAMPLES / FOC_SAMPLE_HZ + 0.03)
    proto.send_command(port, "CURRENT SINE OFF")
    capture, samples = scope.dump_scope(port, timeout)
    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    scope.write_csv(csv_path, samples, sample_period)
    return analyze_capture(samples, frequency_hz, sample_period, axis), samples


def relative_difference(first: float, second: float) -> float:
    return abs(first - second) / max(abs(first), abs(second), 1.0e-12)


def target_error_ratio(measured: float, target: float) -> float:
    return abs(measured - target) / max(abs(target), 1.0e-12)


def mean_metric(records: list[dict], metric: str) -> float | None:
    values = [item[metric] for item in records if item.get(metric) is not None]
    return sum(values) / len(values) if values else None


def aggregate_direction_difference(records: list[dict], metric: str) -> float | None:
    positive = mean_metric(
        [item for item in records if item["target_id_a"] > 0.0], metric
    )
    negative = mean_metric(
        [item for item in records if item["target_id_a"] < 0.0], metric
    )
    if positive is None or negative is None:
        return None
    return relative_difference(positive, negative)


def summarize_candidates(names: list[str], candidates: dict[str, dict],
                         rounds: int, steps: tuple[float, ...],
                         d_frequencies: tuple[float, ...],
                         q_frequencies: tuple[float, ...],
                         step_records: list[dict], zero_records: list[dict],
                         frf_records: list[dict], pwm_off_noise: dict | None,
                         criteria: dict) -> dict:
    summaries = {}
    for name in names:
        candidate = candidates[name]
        steps_here = [item for item in step_records if item["profile"] == name]
        scored_steps = [item for item in steps_here
                        if abs(item["target_id_a"]) >= 1.5]
        zeros_here = [item for item in zero_records if item["profile"] == name]
        frf_here = [item for item in frf_records if item["profile"] == name]
        frf_summaries = {}
        for round_index in range(1, rounds + 1):
            for axis in ("d", "q"):
                points = [
                    item for item in frf_here
                    if item["round"] == round_index and item["axis"] == axis
                ]
                if len(points) >= 2:
                    frf_summaries[f"round{round_index}_{axis}"] = summarize_profile(points)

        overshoots = [item["filtered_overshoot_pct"] for item in scored_steps
                      if item["filtered_overshoot_pct"] is not None]
        settling = [item["noise_limited_settling_time_2pct_sec"]
                    for item in scored_steps
                    if item["noise_limited_settling_time_2pct_sec"] is not None]
        asymmetries = [
            aggregate_direction_difference(
                [item for item in scored_steps if item["round"] == round_index],
                "rise_time_10_90_sec",
            )
            for round_index in range(1, rounds + 1)
        ]
        asymmetries = [value for value in asymmetries if value is not None]

        repeat_rise = None
        if rounds == 2:
            first_mean = mean_metric(
                [item for item in scored_steps if item["round"] == 1],
                "rise_time_10_90_sec",
            )
            second_mean = mean_metric(
                [item for item in scored_steps if item["round"] == 2],
                "rise_time_10_90_sec",
            )
            if first_mean is not None and second_mean is not None:
                repeat_rise = relative_difference(first_mean, second_mean)

        target_checks = []
        for summary in frf_summaries.values():
            if candidate["method"] == "comparison_reference":
                target_checks.append(True)
                continue
            if candidate["method"] == "ideal_rl_pole_cancellation":
                measured = summary["closed_loop_bandwidth_hz"]
                tolerance = criteria["rl_bandwidth_tolerance"]
            else:
                measured = summary["loop_crossover_hz"]
                tolerance = criteria["measured_crossover_tolerance"]
            target = float(candidate["target_crossover_hz"])
            target_checks.append(
                measured is not None
                and target_error_ratio(measured, target) <= tolerance
            )

        expected_frf = rounds * (len(d_frequencies) + len(q_frequencies))
        settling_checks = []
        normalized_settling = []
        tracking_checks = []
        ripple_checks = []
        tracking_rows = []
        pwm_off_band_cache = {}
        for item in scored_steps:
            frf_summary = frf_summaries.get(f"round{item['round']}_d")
            bandwidth = None if frf_summary is None else frf_summary[
                "closed_loop_bandwidth_hz"
            ]
            settling_time = item["noise_limited_settling_time_2pct_sec"]
            if bandwidth is not None and settling_time is not None:
                factor = settling_time * bandwidth
                normalized_settling.append(factor)
                settling_checks.append(
                    factor <= criteria["settling_bandwidth_factor"]
                )
            else:
                settling_checks.append(False)
            if (criteria.get("metric_version") in ("d028", "d029")
                    and bandwidth is not None and pwm_off_noise is not None
                    and item.get("csv") and pwm_off_noise.get("csv")):
                cache_key = round(float(bandwidth), 6)
                if cache_key not in pwm_off_band_cache:
                    pwm_off_band_cache[cache_key] = pwm_off_d_bands(
                        Path(pwm_off_noise["csv"]), bandwidth
                    )
                off_in, off_out = pwm_off_band_cache[cache_key]
                error_in, error_out = steady_d_error_bands(
                    Path(item["csv"]), bandwidth
                )
                tracking_limit = max(
                    criteria["tracking_noise_multiplier"] * off_in,
                    criteria["tracking_error_fraction"]
                    * abs(item["target_id_a"]),
                )
                ripple_limit = max(
                    criteria["ripple_noise_multiplier"] * off_out,
                    criteria["ripple_current_fraction"]
                    * criteria["maximum_validation_current_a"],
                )
                tracking_checks.append(error_in <= tracking_limit)
                ripple_checks.append(error_out <= ripple_limit)
                tracking_rows.append({
                    "round": item["round"],
                    "target_id_a": item["target_id_a"],
                    "bandwidth_hz": bandwidth,
                    "tracking_error_in_band_rms_a": error_in,
                    "tracking_limit_a": tracking_limit,
                    "sampled_high_frequency_ripple_rms_a": error_out,
                    "ripple_limit_a": ripple_limit,
                    "pwm_off_in_band_rms_a": off_in,
                    "pwm_off_out_band_rms_a": off_out,
                })
            elif criteria.get("metric_version") not in ("d028", "d029"):
                zero = next((zero for zero in zeros_here
                             if zero["round"] == item["round"]), None)
                steady_limit = None if zero is None else max(
                    3.0 * zero["id_noise_rms_a"],
                    criteria["steady_error_fraction"] * abs(item["target_id_a"]),
                )
                tracking_checks.append(
                    steady_limit is not None
                    and item["steady_error_rms_a"] <= steady_limit
                )

        zero_noise_checks = []
        if pwm_off_noise is not None:
            floor = (criteria["zero_noise_current_fraction"]
                     * criteria["maximum_validation_current_a"])
            for item in zeros_here:
                zero_noise_checks.extend((
                    item["id_noise_rms_a"] <= max(
                        criteria["pwm_off_noise_multiplier"]
                        * pwm_off_noise["id_noise_rms_a"], floor,
                    ),
                    item["iq_noise_rms_a"] <= max(
                        criteria["pwm_off_noise_multiplier"]
                        * pwm_off_noise["iq_noise_rms_a"], floor,
                    ),
                ))
        checks = {
            "all_step_points_present": len(steps_here) == rounds * len(steps),
            "all_zero_points_present": len(zeros_here) == rounds,
            "all_frf_points_present": len(frf_here) == expected_frf,
            "step_overshoot": bool(overshoots) and max(overshoots)
                              <= criteria["max_overshoot_pct"],
            "step_settling_vs_bandwidth": bool(settling_checks)
                                          and all(settling_checks),
            "tracking_error_in_band": bool(tracking_checks)
                                      and all(tracking_checks),
            "sampled_high_frequency_ripple": (
                bool(ripple_checks) and all(ripple_checks)
                if criteria.get("metric_version") in ("d028", "d029") else True
            ),
            "zero_noise_vs_pwm_off": bool(zero_noise_checks)
                                     and all(zero_noise_checks),
            "direction_and_repeat_consistency": (
                bool(asymmetries) and max(asymmetries)
                <= criteria["max_sign_rise_difference"] and (rounds == 1 or (
                    repeat_rise is not None
                    and repeat_rise <= criteria["max_repeat_rise_difference"]
                ))
            ),
            "phase_margin": len(frf_summaries) == 2 * rounds and all(
                item["phase_margin_deg"] is not None
                and item["phase_margin_deg"] >= criteria["min_phase_margin_deg"]
                for item in frf_summaries.values()
            ),
            "sensitivity": len(frf_summaries) == 2 * rounds and all(
                item["max_observed_sensitivity"] <= criteria["max_sensitivity"]
                for item in frf_summaries.values()
            ),
            "target_consistency": len(target_checks) == 2 * rounds
                                  and all(target_checks),
            "peak_current": bool(steps_here or frf_here or zeros_here) and max(
                [item["raw_peak_current_a"]
                 for item in steps_here + frf_here + zeros_here]
            ) < criteria["max_peak_current_a"],
            "voltage_margin": all(not item.get("voltage_saturated", False)
                                  for item in steps_here + frf_here),
        }
        summaries[name] = {
            "method": candidate["method"],
            "target_crossover_hz": candidate["target_crossover_hz"],
            "target_phase_margin_deg": candidate["target_phase_margin_deg"],
            "maximum_kp": max(candidate["axes"][axis]["kp"]
                              for axis in ("d", "q")),
            "maximum_ki": max(candidate["axes"][axis]["ki"]
                              for axis in ("d", "q")),
            "step_record_count": len(steps_here),
            "zero_record_count": len(zeros_here),
            "frf_record_count": len(frf_here),
            "max_filtered_overshoot_pct": max(overshoots) if overshoots else None,
            "max_noise_limited_settling_sec": max(settling) if settling else None,
            "max_normalized_settling_factor": (
                max(normalized_settling) if normalized_settling else None
            ),
            "meets_excellent_settling_target": (
                bool(normalized_settling)
                and max(normalized_settling)
                <= criteria["excellent_settling_bandwidth_factor"]
                if "excellent_settling_bandwidth_factor" in criteria else None
            ),
            "max_sign_rise_difference_ratio": max(asymmetries) if asymmetries else None,
            "max_repeat_rise_difference_ratio": repeat_rise,
            "tracking_and_ripple": tracking_rows,
            "max_tracking_limit_ratio": max(
                (item["tracking_error_in_band_rms_a"] / item["tracking_limit_a"]
                 for item in tracking_rows), default=None,
            ),
            "max_ripple_limit_ratio": max(
                (item["sampled_high_frequency_ripple_rms_a"]
                 / item["ripple_limit_a"] for item in tracking_rows), default=None,
            ),
            "frf_summaries": frf_summaries,
            "checks": checks,
            "passes_all_checks": all(checks.values()),
        }
    return summaries


def robust_bandwidth_hz(summary: dict) -> float | None:
    values = [
        item.get("closed_loop_bandwidth_hz")
        for item in summary["frf_summaries"].values()
    ]
    if not values or any(value is None for value in values):
        return None
    return min(float(value) for value in values)


def rank_passing_candidates(names: list[str], summaries: dict) -> list[dict]:
    """Rank qualified candidates by robust bandwidth, then robustness and gain."""
    rows = []
    for name in names:
        summary = summaries[name]
        if not summary["passes_all_checks"]:
            continue
        frf = list(summary["frf_summaries"].values())
        bandwidth = robust_bandwidth_hz(summary)
        rows.append({
            "name": name,
            "robust_bandwidth_hz": bandwidth,
            "minimum_phase_margin_deg": min(
                item["phase_margin_deg"] for item in frf
            ),
            "maximum_observed_sensitivity": max(
                item["max_observed_sensitivity"] for item in frf
            ),
            "maximum_kp": summary["maximum_kp"],
            "maximum_ki": summary["maximum_ki"],
            "max_ripple_limit_ratio": summary["max_ripple_limit_ratio"],
        })
    best_bandwidth = max(
        (item["robust_bandwidth_hz"] or 0.0 for item in rows), default=0.0
    )
    leading = [item for item in rows
               if (item["robust_bandwidth_hz"] or 0.0) >= 0.9 * best_bandwidth]
    trailing = [item for item in rows if item not in leading]
    leading.sort(key=lambda item: (
        -item["minimum_phase_margin_deg"],
        item["maximum_observed_sensitivity"],
        item["maximum_kp"],
        item["maximum_ki"],
        item["max_ripple_limit_ratio"] or 0.0,
        -(item["robust_bandwidth_hz"] or 0.0),
        names.index(item["name"]),
    ))
    trailing.sort(key=lambda item: (
        -(item["robust_bandwidth_hz"] or 0.0), names.index(item["name"])
    ))
    rows = leading + trailing
    for index, item in enumerate(rows, 1):
        item["rank"] = index
    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Screen or hold out no-baseline current-PI candidates",
    )
    parser.add_argument("--design", type=Path)
    parser.add_argument(
        "--experiment",
        choices=("screen", "holdout", "comparison", "robustness",
                 "robustness_holdout"),
        default="screen",
    )
    parser.add_argument("--candidate", action="append")
    parser.add_argument("--port")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rounds", type=int, choices=(1, 2), default=2)
    parser.add_argument("--steps", type=float, nargs="+")
    parser.add_argument("--d-frequencies", type=float, nargs="+",
                        default=None)
    parser.add_argument("--q-frequencies", type=float, nargs="+",
                        default=None)
    parser.add_argument("--d-amplitude", type=float, default=0.5)
    parser.add_argument("--q-amplitude", type=float, default=0.3)
    parser.add_argument("--q-full-amplitude-from-hz", type=float, default=300.0)
    parser.add_argument("--settle-cycles", type=float, default=5.0)
    parser.add_argument("--max-current", type=float, default=4.0)
    parser.add_argument("--max-calibration-current", type=float, default=5.0)
    parser.add_argument("--max-rpm", type=float, default=500.0)
    parser.add_argument("--max-voltage-utilization", type=float, default=0.98)
    parser.add_argument("--flag-overshoot-pct", type=float, default=30.0)
    parser.add_argument("--flag-settling-ms", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--results-dir", type=Path, default=Path("tests/results"))
    args = parser.parse_args()

    design_path = args.design or (
        DEFAULT_D032_DESIGN
        if args.experiment in ("robustness", "robustness_holdout")
        else DEFAULT_DESIGN
    )
    design, candidates, available_names = load_design(design_path)
    comparison = args.experiment == "comparison"
    robustness = args.experiment == "robustness"
    robustness_holdout = args.experiment == "robustness_holdout"
    if comparison:
        candidates[ACTIVE_LEGACY_NAME] = {
            "name": ACTIVE_LEGACY_NAME,
            "method": "comparison_reference",
            "target_crossover_hz": None,
            "target_phase_margin_deg": None,
            "axes": ACTIVE_LEGACY_AXES,
        }
        available_names = available_names + [ACTIVE_LEGACY_NAME]
    names = select_candidate_names(
        available_names, args.candidate, args.experiment
    )
    holdout = args.experiment == "holdout"
    steps = tuple(args.steps or (
        HOLDOUT_STEPS_A if holdout or comparison or robustness else DEFAULT_STEPS_A
    ))
    default_frequencies = (
        COMPARISON_FREQUENCIES_HZ if comparison else
        (HOLDOUT_FREQUENCIES_HZ if holdout or robustness else DEFAULT_D_FREQUENCIES_HZ)
    )
    d_frequencies = tuple(sorted(args.d_frequencies or default_frequencies))
    default_q_frequencies = (
        COMPARISON_FREQUENCIES_HZ if comparison else
        (HOLDOUT_FREQUENCIES_HZ if holdout or robustness else DEFAULT_Q_FREQUENCIES_HZ)
    )
    q_frequencies = tuple(sorted(
        args.q_frequencies or default_q_frequencies
    ))
    if not args.plan_only and not args.port:
        parser.error("--port is required unless --plan-only is used")
    if any(abs(value) < 0.1 or abs(value) > 3.0 for value in steps):
        raise ValueError("d-axis step magnitudes must stay within 0.1 to 3.0 A")
    maximum_frequency_hz = 5000.0 if comparison else 4000.0
    if any(value < 50.0 or value > maximum_frequency_hz
           for value in d_frequencies + q_frequencies):
        raise ValueError(
            f"sweep frequencies must stay within 50 to {maximum_frequency_hz:g} Hz"
        )
    if len(set(d_frequencies)) != len(d_frequencies) \
            or len(set(q_frequencies)) != len(q_frequencies):
        raise ValueError("sweep frequencies must be unique within each axis")
    if not 0.02 <= args.d_amplitude <= 0.5 \
            or not 0.02 <= args.q_amplitude <= 0.5:
        raise ValueError("d/q sine amplitudes must stay within 0.02 to 0.5 A")
    if args.q_full_amplitude_from_hz <= 0.0:
        raise ValueError("q full-amplitude frequency must be positive")

    run_plan = build_run_plan(names, args.rounds)
    criteria = build_d029_criteria(steps, args.max_current)
    visits = sum(len(item["order"]) for item in run_plan)
    captures_per_visit = (
        1 + len(steps) + len(d_frequencies) + len(q_frequencies)
    )
    plan_summary = {
        "experiment": args.experiment,
        "design": str(design_path),
        "candidate_names": names,
        "qualification_candidate_names": (
            [HOLDOUT_CANDIDATE] if comparison else names
        ),
        "run_plan": run_plan,
        "d_steps_a": steps,
        "d_frequencies_hz": d_frequencies,
        "q_frequencies_hz": q_frequencies,
        "d_sine_amplitude_a": args.d_amplitude,
        "q_sine_maximum_amplitude_a": args.q_amplitude,
        "q_sine_amplitudes_by_frequency_a": {
            f"{frequency:g}": sine_amplitude(
                "q", args.q_amplitude, frequency,
                args.q_full_amplitude_from_hz,
            )
            for frequency in q_frequencies
        },
        "q_full_amplitude_from_hz": args.q_full_amplitude_from_hz,
        "pwm_off_noise_captures": 1,
        "profile_visits": visits,
        "captures_per_profile_visit": captures_per_visit,
        "planned_scope_captures": 1 + visits * captures_per_visit,
        "raw_scope_acquisition_sec": (
            (1 + visits * captures_per_visit) * SCOPE_SAMPLES / FOC_SAMPLE_HZ
        ),
        "acceptance_criteria": criteria,
        "runtime_abort_gates": {
            "peak_current_a": args.max_current,
            "peak_rpm": args.max_rpm,
            "max_voltage_utilization": args.max_voltage_utilization,
        },
        "performance_flag_gates": {
            "overshoot_pct": args.flag_overshoot_pct,
            "noise_limited_settling_ms": args.flag_settling_ms,
        },
        "expected_motion": (
            "d-axis steps should hold rotor; zero-mean q-axis sine may cause "
            "small bidirectional oscillation, not sustained rotation"
        ),
    }
    if args.plan_only:
        print(json.dumps(plan_summary, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.results_dir.mkdir(parents=True, exist_ok=True)
    output = args.results_dir / f"stage2_current_autotune_hw_{stamp}.json"
    step_records: list[dict] = []
    zero_records: list[dict] = []
    frf_records: list[dict] = []
    pwm_off_noise = None
    runtime_rejections: list[dict] = []
    measured_bus_voltage = None
    final_mode = None
    error = None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            buffer = bytearray()
            port.reset_input_buffer()
            try:
                ready = wait_mode(
                    port, buffer, MODE_READY, args.timeout,
                    args.max_calibration_current, args.max_rpm,
                )
                measured_bus_voltage = ready[29]
                if not 18.0 <= measured_bus_voltage <= 28.0:
                    raise RuntimeError(
                        f"measured bus voltage is out of range: "
                        f"{measured_bus_voltage:.3f} V"
                    )
                voltage_limit = (
                    measured_bus_voltage / math.sqrt(3.0)
                    * args.max_voltage_utilization
                )

                off_path = args.results_dir / f"stage2_autotune_{stamp}_pwm_off.csv"
                _, off_samples = capture_zero(
                    port, args.timeout, off_path
                )
                pwm_off_noise = summarize_pwm_off(off_samples)
                pwm_off_noise.update({
                    "raw_peak_rpm": peak_rpm(off_samples),
                    "csv": str(off_path.resolve()),
                })

                proto.send_command(port, "FF OFF")
                proto.send_command(port, "START CURRENT")
                wait_mode(
                    port, buffer, MODE_TORQUE_RUN, args.timeout,
                    args.max_calibration_current, args.max_rpm,
                )
                proto.send_command(port, "CURRENT 0 0")
                time.sleep(0.25)

                for round_plan in run_plan:
                    round_index = round_plan["round"]
                    for name in round_plan["order"]:
                        candidate = candidates[name]
                        axes = candidate["axes"]
                        set_profile(port, axes)
                        proto.send_command(port, "CURRENT 0 0")
                        time.sleep(0.1)
                        stem = f"stage2_autotune_{stamp}_r{round_index}_{name}"

                        zero_path = args.results_dir / f"{stem}_zero.csv"
                        zero, samples = capture_zero(port, args.timeout, zero_path)
                        zero.update({
                            "round": round_index,
                            "profile": name,
                            "raw_peak_rpm": peak_rpm(samples),
                            "csv": str(zero_path.resolve()),
                        })
                        zero_records.append(zero)
                        if zero["raw_peak_current_a"] >= args.max_current:
                            raise RuntimeError(
                                f"{name} zero-current peak exceeded: "
                                f"{zero['raw_peak_current_a']:.3f} A"
                            )
                        if zero["raw_peak_rpm"] >= args.max_rpm:
                            raise RuntimeError(
                                f"{name} zero-current speed exceeded: "
                                f"{zero['raw_peak_rpm']:.1f} rpm"
                            )

                        for point_index, target in enumerate(steps, start=1):
                            csv_path = args.results_dir / (
                                f"{stem}_step{point_index:02d}_{target:+g}A.csv"
                            )
                            result, samples = capture_step(
                                port, target, args.timeout, csv_path
                            )
                            result.update({
                                "round": round_index,
                                "profile": name,
                                "kp_d": axes["d"]["kp"],
                                "ki_d": axes["d"]["ki"],
                                "kp_q": axes["q"]["kp"],
                                "ki_q": axes["q"]["ki"],
                                "raw_peak_rpm": peak_rpm(samples),
                                "voltage_limit_v": voltage_limit,
                                "voltage_saturated": (
                                    result["raw_peak_voltage_v"] >= voltage_limit
                                ),
                                "csv": str(csv_path.resolve()),
                            })
                            step_records.append(result)
                            if result["raw_peak_current_a"] >= args.max_current:
                                raise RuntimeError(
                                    f"{name} step current exceeded: "
                                    f"{result['raw_peak_current_a']:.3f} A"
                                )
                            if result["raw_peak_rpm"] >= args.max_rpm:
                                raise RuntimeError(
                                    f"{name} step speed exceeded: "
                                    f"{result['raw_peak_rpm']:.1f} rpm"
                                )
                            if result["voltage_saturated"]:
                                raise RuntimeError(
                                    f"{name} voltage utilization exceeded: "
                                    f"{result['raw_peak_voltage_v']:.3f} V"
                                )

                            severe = None
                            if abs(target) >= 1.5:
                                overshoot = result["filtered_overshoot_pct"]
                                settling = result[
                                    "noise_limited_settling_time_2pct_sec"
                                ]
                                if (overshoot is not None
                                        and overshoot > args.flag_overshoot_pct):
                                    severe = "runtime overshoot gate"
                                if (settling is None or settling * 1000.0
                                        > args.flag_settling_ms):
                                    severe = "runtime settling gate"
                            if severe is not None:
                                runtime_rejections.append({
                                    "round": round_index,
                                    "profile": name,
                                    "target_id_a": target,
                                    "reason": severe,
                                })
                            wait_mode(
                                port, buffer, MODE_TORQUE_RUN, 2.0,
                                args.max_current, args.max_rpm,
                            )
                            time.sleep(0.03)

                        for axis, maximum_amplitude, frequencies in (
                            ("d", args.d_amplitude, d_frequencies),
                            ("q", args.q_amplitude, q_frequencies),
                        ):
                            for frequency in frequencies:
                                amplitude = sine_amplitude(
                                    axis, maximum_amplitude, frequency,
                                    args.q_full_amplitude_from_hz,
                                )
                                csv_path = args.results_dir / (
                                    f"{stem}_{axis}_{frequency:g}hz.csv"
                                )
                                result, samples = capture_sine(
                                    port, axis, amplitude, frequency,
                                    args.settle_cycles, args.timeout, csv_path,
                                )
                                result.update({
                                    "round": round_index,
                                    "profile": name,
                                    "kp_d": axes["d"]["kp"],
                                    "ki_d": axes["d"]["ki"],
                                    "kp_q": axes["q"]["kp"],
                                    "ki_q": axes["q"]["ki"],
                                    "raw_peak_rpm": peak_rpm(samples),
                                    "voltage_limit_v": voltage_limit,
                                    "voltage_saturated": (
                                        result["raw_peak_voltage_v"] >= voltage_limit
                                    ),
                                    "csv": str(csv_path.resolve()),
                                })
                                frf_records.append(result)
                                if not 0.9 * amplitude <= result[
                                        "reference_amplitude_a"] <= 1.1 * amplitude:
                                    raise RuntimeError(
                                        f"{name} {axis}-axis sine amplitude mismatch: "
                                        f"{result['reference_amplitude_a']:.3f} A"
                                    )
                                if result["raw_peak_current_a"] >= args.max_current:
                                    raise RuntimeError(
                                        f"{name} {axis}-axis current exceeded: "
                                        f"{result['raw_peak_current_a']:.3f} A"
                                    )
                                if result["raw_peak_rpm"] >= args.max_rpm:
                                    raise RuntimeError(
                                        f"{name} {axis}-axis speed exceeded: "
                                        f"{result['raw_peak_rpm']:.1f} rpm"
                                    )
                                if result["voltage_saturated"]:
                                    raise RuntimeError(
                                        f"{name} {axis}-axis voltage utilization exceeded"
                                    )
                                wait_mode(
                                    port, buffer, MODE_TORQUE_RUN, 2.0,
                                    args.max_current, args.max_rpm,
                                )
                                proto.send_command(port, "PI CLEAR")
                                time.sleep(0.03)
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    proto.send_command(port, "CURRENT SINE OFF")
                    proto.send_command(port, "CURRENT 0 0")
                    cleanup_name = (
                        ACTIVE_LEGACY_NAME if comparison else names[0]
                    )
                    set_profile(port, candidates[cleanup_name]["axes"])
                    proto.send_command(port, "STOP")
                    final = wait_mode(
                        port, buffer, MODE_READY, 2.0,
                        args.max_calibration_current, args.max_rpm,
                    )
                    final_mode = int(round(final[27]))
                except Exception as cleanup_exc:
                    cleanup_error = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    error = cleanup_error if error is None else (
                        f"{error}; cleanup: {cleanup_error}"
                    )
    except Exception as exc:
        if error is None:
            error = f"{type(exc).__name__}: {exc}"

    summaries = summarize_candidates(
        names, candidates, args.rounds, steps, d_frequencies, q_frequencies,
        step_records, zero_records, frf_records, pwm_off_noise, criteria,
    )
    qualification_names = plan_summary["qualification_candidate_names"]
    passing = [name for name in qualification_names
               if summaries[name]["passes_all_checks"]]
    ranking = rank_passing_candidates(qualification_names, summaries)
    result_document = {
        "schema": "stage2-current-autotune-hardware-d029-v1",
        "design": str(design_path.resolve()),
        "design_schema": design.get("schema"),
        "plan": plan_summary,
        "measured_bus_voltage_v": measured_bus_voltage,
        "pwm_off_noise": pwm_off_noise,
        "step_records": step_records,
        "zero_records": zero_records,
        "frf_records": frf_records,
        "candidate_summaries": summaries,
        "passing_candidates": passing,
        "coarse_ranking": ranking,
        "coarse_leader": ranking[0]["name"] if ranking else None,
        "selection_status": (
            ("independent holdout passed" if passing
             else "independent holdout failed")
            if holdout else
            ("recommended candidate retained qualification; active comparison captured"
             if passing else "recommended candidate failed comparison repeat")
            if comparison else
            ("D-032 robustness leader passed its only independent holdout"
             if passing else "D-032 robustness leader failed its only holdout")
            if robustness_holdout else
            ("robustness candidates qualified for one frozen holdout"
             if passing else "no robustness candidate passed every D-029 criterion")
            if robustness else
            ("hardware-qualified candidates available for dense top-3 validation"
             if passing else "no candidate passed every coarse-screen criterion")
        ),
        "runtime_rejections": runtime_rejections,
        "cleanup_restored_profile": (
            ACTIVE_LEGACY_NAME if comparison else names[0]
        ),
        "error": error,
        "final_mode": final_mode,
    }
    output.write_text(
        json.dumps(result_document, indent=2) + "\n", encoding="utf-8"
    )

    if error is not None:
        raise RuntimeError(
            f"experiment failed; partial result saved to {output}: {error}"
        )
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    print(json.dumps({
        "result": str(output.resolve()),
        "step_records": len(step_records),
        "zero_records": len(zero_records),
        "frf_records": len(frf_records),
        "passing_candidates": passing,
        "selection_status": result_document["selection_status"],
        "final_mode": final_mode,
    }, indent=2))


if __name__ == "__main__":
    main()
