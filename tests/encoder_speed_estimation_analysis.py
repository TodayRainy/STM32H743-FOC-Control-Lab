#!/usr/bin/env python3
"""Aggregate the two-round encoder speed-estimator hardware A/B."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

from encoder_speed_estimation import (
    BASE_WINDOW_SEC,
    fixed_window_speed,
    tracking_pll_speed,
    unwrap_angles,
    window_iir_speed,
)
from stage2_current_frequency_response import fit_phasor, magnitude_db, phase_deg


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def wrapped_phase_difference(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def circular_mean_deg(values: list[float]) -> float:
    x = mean([math.cos(math.radians(value)) for value in values])
    y = mean([math.sin(math.radians(value)) for value in values])
    return math.degrees(math.atan2(y, x))


def replay_same_trajectory(raw: dict) -> dict:
    records: dict[str, dict[float, list[dict]]] = {
        "base": defaultdict(list),
        "win_1ms_fc150": defaultdict(list),
        "pll_200hz": defaultdict(list),
    }
    for visit in raw["visits"]:
        if visit["profile"] != "base":
            continue
        for frequency_record in visit["frequency"]:
            path = Path(frequency_record["scope_csv"])
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            time_s = [float(row["time_s"]) for row in rows]
            angle = unwrap_angles([float(row["elec_angle"]) for row in rows])
            reference = [float(row["speed_ref"]) for row in rows]
            sample_period = statistics.median(
                second - first for first, second in zip(time_s, time_s[1:])
            )
            estimates = {
                "base": fixed_window_speed(
                    angle, sample_period,
                    round(BASE_WINDOW_SEC / sample_period),
                ),
                "win_1ms_fc150": window_iir_speed(
                    angle, sample_period, 1.0e-3, 150.0,
                ),
                "pll_200hz": tracking_pll_speed(
                    angle, sample_period, 200.0,
                ),
            }
            trim = max(
                round(BASE_WINDOW_SEC / sample_period),
                round(0.025 / sample_period),
            )
            frequency_hz = float(frequency_record["frequency_hz"])
            reference_phasor, _ = fit_phasor(
                reference[trim:], frequency_hz, sample_period,
            )
            for profile, estimate in estimates.items():
                estimate_phasor, _ = fit_phasor(
                    estimate[trim:], frequency_hz, sample_period,
                )
                response = estimate_phasor / reference_phasor
                records[profile][frequency_hz].append({
                    "magnitude_db": magnitude_db(response),
                    "phase_deg": phase_deg(response),
                })

    summary = {}
    for profile, by_frequency in records.items():
        summary[profile] = {}
        for frequency_hz, samples in sorted(by_frequency.items()):
            magnitudes = [sample["magnitude_db"] for sample in samples]
            phases = [sample["phase_deg"] for sample in samples]
            summary[profile][f"{frequency_hz:g}"] = {
                "mean_magnitude_db": mean(magnitudes),
                "round_magnitude_difference_db": abs(
                    magnitudes[0] - magnitudes[1]
                ),
                "circular_mean_phase_deg": circular_mean_deg(phases),
                "round_phase_difference_deg": wrapped_phase_difference(
                    phases[0], phases[1]
                ),
            }
    return {
        "source_motion": "two BASE-controlled frequency captures",
        "scope_sample_hz": 4000.0,
        "limitation": (
            "Candidates are replayed from the same sampled electrical angle, "
            "which removes mechanical order bias but approximates the 40 kHz "
            "firmware estimators at the 4 kHz scope rate."
        ),
        "profiles": summary,
    }


def selected_estimator_replay_check(raw: dict) -> dict:
    checks: dict[str, dict[float, list[dict]]] = {
        "base": defaultdict(list),
        "win_1ms_fc150": defaultdict(list),
        "pll_200hz": defaultdict(list),
    }
    for visit in raw["visits"]:
        profile = visit["profile"]
        for frequency_record in visit["frequency"]:
            path = Path(frequency_record["scope_csv"])
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            time_s = [float(row["time_s"]) for row in rows]
            angle = unwrap_angles([float(row["elec_angle"]) for row in rows])
            recorded = [float(row["elec_speed"]) for row in rows]
            sample_period = statistics.median(
                second - first for first, second in zip(time_s, time_s[1:])
            )
            if profile == "base":
                replayed = fixed_window_speed(
                    angle, sample_period,
                    round(BASE_WINDOW_SEC / sample_period),
                )
            elif profile == "win_1ms_fc150":
                replayed = window_iir_speed(
                    angle, sample_period, 1.0e-3, 150.0,
                )
            else:
                replayed = tracking_pll_speed(
                    angle, sample_period, 200.0,
                )
            trim = max(
                round(BASE_WINDOW_SEC / sample_period),
                round(0.025 / sample_period),
            )
            frequency_hz = float(frequency_record["frequency_hz"])
            recorded_phasor, _ = fit_phasor(
                recorded[trim:], frequency_hz, sample_period,
            )
            replayed_phasor, _ = fit_phasor(
                replayed[trim:], frequency_hz, sample_period,
            )
            ratio = recorded_phasor / replayed_phasor
            checks[profile][frequency_hz].append({
                "magnitude_error_db": magnitude_db(ratio),
                "phase_error_deg": phase_deg(ratio),
            })

    summary = {}
    for profile, by_frequency in checks.items():
        summary[profile] = {}
        for frequency_hz, samples in sorted(by_frequency.items()):
            magnitudes = [sample["magnitude_error_db"] for sample in samples]
            phases = [sample["phase_error_deg"] for sample in samples]
            summary[profile][f"{frequency_hz:g}"] = {
                "mean_magnitude_error_db": mean(magnitudes),
                "maximum_absolute_magnitude_error_db": max(
                    abs(value) for value in magnitudes
                ),
                "circular_mean_phase_error_deg": circular_mean_deg(phases),
                "maximum_absolute_phase_error_deg": max(
                    abs(value) for value in phases
                ),
            }
    return {
        "comparison": "recorded 40 kHz firmware output versus 4 kHz angle replay",
        "limitation": (
            "The scope angle is decimated to 4 kHz, so small discretization and "
            "within-sample count timing differences are expected."
        ),
        "profiles": summary,
    }


def aggregate(path: Path) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    visits_by_profile: dict[str, list[dict]] = defaultdict(list)
    for visit in raw["visits"]:
        visits_by_profile[visit["profile"]].append(visit)

    profiles = {}
    global_peak_current = 0.0
    global_peak_iq_reference = 0.0
    for profile, visits in visits_by_profile.items():
        steady_by_target: dict[float, list[dict]] = defaultdict(list)
        frequency_by_hz: dict[float, list[dict]] = defaultdict(list)
        for visit in visits:
            for record in visit["steady"]:
                steady_by_target[float(record["target_rpm"])].append(record)
                global_peak_current = max(
                    global_peak_current, record["peak_dq_current_a"]
                )
                global_peak_iq_reference = max(
                    global_peak_iq_reference, record["peak_iq_reference_a"]
                )
            for record in visit["frequency"]:
                frequency_by_hz[float(record["frequency_hz"])].append(record)
                global_peak_current = max(
                    global_peak_current, record["peak_dq_current_a"]
                )
                global_peak_iq_reference = max(
                    global_peak_iq_reference, record["peak_iq_reference_a"]
                )
            global_peak_current = max(
                global_peak_current, visit["zero_speed"]["peak_dq_current_a"]
            )
            global_peak_iq_reference = max(
                global_peak_iq_reference,
                visit["zero_speed"]["peak_iq_reference_a"],
            )

        steady = {}
        for target, records in sorted(steady_by_target.items()):
            means = [record["mean_rpm"] for record in records]
            steady[f"{target:g}"] = {
                "mean_speed_rpm": mean(means),
                "mean_absolute_error_rpm": mean([
                    abs(record["mean_error_rpm"]) for record in records
                ]),
                "round_difference_rpm": abs(means[0] - means[1]),
                "mean_ripple_rms_rpm": mean([
                    record["ripple_rms_rpm"] for record in records
                ]),
                "mean_iq_reference_rms_a": mean([
                    record["iq_reference_rms_a"] for record in records
                ]),
                "peak_dq_current_a": max(
                    record["peak_dq_current_a"] for record in records
                ),
            }

        frequency = {}
        for hz, records in sorted(frequency_by_hz.items()):
            magnitudes = [record["closed_loop_magnitude_db"] for record in records]
            phases = [record["closed_loop_phase_deg"] for record in records]
            frequency[f"{hz:g}"] = {
                "mean_magnitude_db": mean(magnitudes),
                "round_magnitude_difference_db": abs(magnitudes[0] - magnitudes[1]),
                "circular_mean_phase_deg": circular_mean_deg(phases),
                "round_phase_difference_deg": wrapped_phase_difference(
                    phases[0], phases[1]
                ),
                "mean_measured_amplitude_rpm": mean([
                    record["measured_amplitude_rpm"] for record in records
                ]),
                "mean_peak_iq_reference_a": mean([
                    record["peak_iq_reference_a"] for record in records
                ]),
            }

        profiles[profile] = {
            "zero_speed": {
                "mean_ripple_rms_rpm": mean([
                    visit["zero_speed"]["ripple_rms_rpm"] for visit in visits
                ]),
                "maximum_peak_abs_rpm": max(
                    visit["zero_speed"]["peak_abs_rpm"] for visit in visits
                ),
            },
            "steady": steady,
            "frequency": frequency,
        }

    base_80hz = profiles["base"]["frequency"]["80"]["mean_magnitude_db"]
    win = profiles["win_1ms_fc150"]["frequency"]
    pll = profiles["pll_200hz"]["frequency"]
    same_trajectory = replay_same_trajectory(raw)
    selected_replay = selected_estimator_replay_check(raw)
    return {
        "schema": "encoder-speed-estimation-hardware-summary-v1",
        "source": str(path.resolve()),
        "status": raw["status"],
        "final_mode": raw["final_mode"],
        "scope_capture_count": raw["plan"]["total_scope_captures"],
        "safety": {
            "peak_dq_current_a": global_peak_current,
            "peak_iq_reference_a": global_peak_iq_reference,
            "completed_ready": (
                raw["status"] == "completed" and raw["final_mode"] == 17
            ),
        },
        "profiles": profiles,
        "same_trajectory_replay": same_trajectory,
        "selected_estimator_replay_check": selected_replay,
        "observations": {
            "base_80hz_notch_confirmed": base_80hz < -60.0,
            "win_has_higher_mean_response_than_pll_at_all_test_frequencies": all(
                win[key]["mean_magnitude_db"] > pll[key]["mean_magnitude_db"]
                for key in ("20", "40", "80")
            ),
            "frequency_response_has_material_round_dependence": any(
                profile["frequency"][key]["round_phase_difference_deg"] > 10.0
                or profile["frequency"][key]["round_magnitude_difference_db"] > 2.0
                for profile in profiles.values()
                for key in ("20", "40", "80")
            ),
        },
        "decision": {
            "base": "reject_for_outer_loop_bandwidth_study",
            "leader": "win_1ms_fc150",
            "leader_status": "provisional_not_default",
            "reason": (
                "WIN removes the BASE 80 Hz notch and preserves more closed-loop "
                "response than PLL, but the zero-mean mechanical frequency test is "
                "order dependent and neither new estimator improves all steady-speed "
                "ripple points. A same-trajectory estimator comparison is required "
                "before changing the reset default."
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    summary = aggregate(args.input)
    text = json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
