#!/usr/bin/env python3
"""Build the Stage-2 rotating PI/feedforward decision from a raw report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REFERENCE = "active_ff_identified"
CANDIDATE = "identified_2000hz_ff_identified"
FF_OFF = "active_ff_off"
ROUND_METRICS = (
    "ramp_speed_error_rms_rpm",
    "ramp_iq_error_rms_a",
    "ramp_id_error_rms_a",
    "steady_iq_error_rms_a",
    "steady_id_error_rms_a",
    "pi_voltage_rms_v",
)


def mean(values):
    return sum(values) / len(values)


def relative_change(candidate, reference):
    return candidate / reference - 1.0


def round_summary(report, profile, round_index):
    records = [
        record for record in report["transitions"]
        if record["profile"] == profile and record["round"] == round_index
    ]
    if not records:
        raise ValueError(f"missing round {round_index} for {profile}")
    result = {"round": round_index, "profile": profile, "transitions": len(records)}
    result.update({
        metric: mean([record[metric] for record in records])
        for metric in ROUND_METRICS
    })
    scope = next(
        item for item in report["scope_results"]
        if item["profile"] == profile and item["round"] == round_index
    )
    result.update({
        "scope_peak_iq_reference_a": scope["peak_iq_reference_a"],
        "scope_iq_error_rms_a": scope["excited_iq_error_rms_a"],
        "scope_id_error_rms_a": scope["excited_id_error_rms_a"],
    })
    return result


def build_analysis(report):
    profiles = {item["profile"]: item for item in report["profile_summary"]}
    for name in (FF_OFF, REFERENCE, CANDIDATE):
        if name not in profiles:
            raise ValueError(f"missing profile summary: {name}")

    reference = profiles[REFERENCE]
    candidate = profiles[CANDIDATE]
    ff_off = profiles[FF_OFF]
    rounds = []
    for round_index in (1, 2):
        ref_round = round_summary(report, REFERENCE, round_index)
        candidate_round = round_summary(report, CANDIDATE, round_index)
        rounds.append({
            "round": round_index,
            "reference": ref_round,
            "candidate": candidate_round,
            "candidate_minus_reference": {
                metric: candidate_round[metric] - ref_round[metric]
                for metric in ROUND_METRICS
            } | {
                "scope_iq_error_rms_a": (
                    candidate_round["scope_iq_error_rms_a"]
                    - ref_round["scope_iq_error_rms_a"]
                ),
                "scope_id_error_rms_a": (
                    candidate_round["scope_id_error_rms_a"]
                    - ref_round["scope_id_error_rms_a"]
                ),
            },
        })

    q_excitation_sufficient = min(
        reference["minimum_scope_peak_iq_reference_a"],
        candidate["minimum_scope_peak_iq_reference_a"],
    ) >= 0.2
    comparison = {
        "candidate_vs_same_feedforward_reference": {
            "ramp_speed_error_change": relative_change(
                candidate["mean_ramp_speed_error_rms_rpm"],
                reference["mean_ramp_speed_error_rms_rpm"],
            ),
            "ramp_iq_error_change": relative_change(
                candidate["mean_ramp_iq_error_rms_a"],
                reference["mean_ramp_iq_error_rms_a"],
            ),
            "ramp_id_error_change": relative_change(
                candidate["mean_ramp_id_error_rms_a"],
                reference["mean_ramp_id_error_rms_a"],
            ),
            "steady_iq_error_change": relative_change(
                candidate["mean_steady_iq_error_rms_a"],
                reference["mean_steady_iq_error_rms_a"],
            ),
            "steady_id_error_change": relative_change(
                candidate["mean_steady_id_error_rms_a"],
                reference["mean_steady_id_error_rms_a"],
            ),
            "scope_iq_error_change": relative_change(
                candidate["mean_scope_excited_iq_error_rms_a"],
                reference["mean_scope_excited_iq_error_rms_a"],
            ),
            "scope_id_error_change": relative_change(
                candidate["mean_scope_excited_id_error_rms_a"],
                reference["mean_scope_excited_id_error_rms_a"],
            ),
            "pi_voltage_change": relative_change(
                candidate["mean_pi_voltage_rms_v"],
                reference["mean_pi_voltage_rms_v"],
            ),
        },
        "identified_feedforward_vs_off": {
            "pi_voltage_reduction": 1.0 - (
                reference["mean_pi_voltage_rms_v"]
                / ff_off["mean_pi_voltage_rms_v"]
            ),
            "ramp_id_error_change": relative_change(
                reference["mean_ramp_id_error_rms_a"],
                ff_off["mean_ramp_id_error_rms_a"],
            ),
            "scope_id_error_change": relative_change(
                reference["mean_scope_excited_id_error_rms_a"],
                ff_off["mean_scope_excited_id_error_rms_a"],
            ),
        },
    }

    return {
        "source": report["csv"].replace(".csv", ".json"),
        "hardware_complete": report["error"] is None and report["final_mode"] == 17,
        "safety": {
            "final_mode": report["final_mode"],
            "restored_active_pi_and_ff_off": report[
                "restored_active_pi_and_ff_off"
            ],
            "peak_dq_current_a": max(
                item["peak_dq_current_a"] for item in report["profile_summary"]
            ),
            "minimum_voltage_scale": min(
                item["minimum_voltage_scale"] for item in report["profile_summary"]
            ),
        },
        "scope_q_excitation": {
            "required_peak_iq_reference_a": 0.2,
            "minimum_measured_peak_iq_reference_a": min(
                reference["minimum_scope_peak_iq_reference_a"],
                candidate["minimum_scope_peak_iq_reference_a"],
            ),
            "sufficient": q_excitation_sufficient,
        },
        "profile_summary": report["profile_summary"],
        "round_summary": rounds,
        "comparison": comparison,
        "decision": {
            "current_pi_default": "active_legacy",
            "writeback_2khz_approved": False,
            "stage2_status": "complete without PI writeback",
            "reason": (
                "The 2 kHz candidate showed no q-axis or speed advantage and "
                "repeatedly increased rotating d-axis error. Scope q excitation "
                "was below the predefined threshold, so this rejects automatic "
                "writeback but does not claim global q-axis inferiority."
            ),
            "feedforward_status": (
                "identified feedforward benefit reconfirmed; default model and "
                "enable policy remain a separate Stage-3 decision"
            ),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = json.loads(args.input.read_text(encoding="utf-8"))
    analysis = build_analysis(report)
    print(json.dumps(analysis, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(analysis, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
