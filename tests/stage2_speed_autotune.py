#!/usr/bin/env python3
"""Offline design core for the constrained empirical speed-loop autotuner."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


DEFAULT_SPEC = Path(
    "tests/baselines/stage2_speed_autotune_spec_v1_20260824.json"
)


def load_spec(path: Path) -> dict:
    spec = json.loads(path.read_text(encoding="utf-8"))
    if spec.get("schema") != "stage2-speed-autotune-spec-v1":
        raise ValueError("unsupported speed-autotuning specification")

    control = spec["control"]
    theoretical_limit = min(
        float(control["current_loop_robust_bandwidth_hz"])
        / float(control["current_to_speed_separation"]),
        float(control["speed_loop_sample_hz"])
        / float(control["sample_to_bandwidth_separation"]),
    )
    declared_limit = float(control["speed_bandwidth_guard_hz"])
    if declared_limit <= 0.0 or declared_limit > theoretical_limit:
        raise ValueError("speed bandwidth guard exceeds the frozen separation limit")
    return spec


def rpm_to_electrical_rad_s(rpm: float, pole_pairs: int) -> float:
    return rpm * 2.0 * math.pi / 60.0 * pole_pairs


def build_p_scan(spec: dict) -> list[dict]:
    scan = spec["p_scan"]
    positive_steps = [value for value in scan["step_rpm"] if value > 0.0]
    if len(positive_steps) != 1:
        raise ValueError("P scan requires exactly one positive step magnitude")

    delta_speed = rpm_to_electrical_rad_s(
        positive_steps[0], int(spec["control"]["pole_pairs"])
    )
    kp_seed = float(scan["seed_iq_a"]) / delta_speed
    multiplier = float(scan["kp_multiplier"])
    count = int(scan["maximum_candidate_count"])
    if multiplier <= 1.0 or not 1 <= count <= 8:
        raise ValueError("invalid P-scan multiplier or candidate count")

    return [
        {
            "name": f"p_scan_{index}",
            "kp": kp_seed * multiplier ** index,
            "ki": 0.0,
            "predicted_initial_iq_a": (
                kp_seed * multiplier ** index * delta_speed
            ),
        }
        for index in range(count)
    ]


def build_pi_candidates(spec: dict, kp_base: float, tau_63_sec: float) -> list[dict]:
    if not math.isfinite(kp_base) or kp_base <= 0.0:
        raise ValueError("kp_base must be positive and finite")
    if not math.isfinite(tau_63_sec) or tau_63_sec <= 0.0:
        raise ValueError("tau_63_sec must be positive and finite")

    design = spec["pi_candidates"]
    candidates = []
    for kp_factor in design["kp_base_factors"]:
        kp = float(kp_factor) * kp_base
        for ti_factor in design["integral_time_tau_factors"]:
            integral_time = float(ti_factor) * tau_63_sec
            candidates.append({
                "name": f"pi_kp{float(kp_factor):g}_ti{float(ti_factor):g}tau",
                "kp": kp,
                "ki": kp / integral_time,
                "integral_time_sec": integral_time,
                "kp_base_factor": float(kp_factor),
                "tau_63_factor": float(ti_factor),
            })

    if len(candidates) != int(design["candidate_count"]):
        raise ValueError("PI candidate count does not match the frozen specification")
    return candidates


def build_report(spec_path: Path, kp_base: float | None,
                 tau_63_sec: float | None) -> dict:
    spec = load_spec(spec_path)
    control = spec["control"]
    theoretical_limit = min(
        control["current_loop_robust_bandwidth_hz"]
        / control["current_to_speed_separation"],
        control["speed_loop_sample_hz"]
        / control["sample_to_bandwidth_separation"],
    )
    report = {
        "schema": "stage2-speed-autotune-offline-plan-v1",
        "spec": str(spec_path),
        "theoretical_bandwidth_limit_hz": theoretical_limit,
        "frozen_bandwidth_guard_hz": control["speed_bandwidth_guard_hz"],
        "p_scan_candidates": build_p_scan(spec),
        "pi_candidates": None,
        "hardware": {
            "requires_24v": True,
            "automatic_flash": False,
            "expected_maximum_speed_rpm": 4200.0,
            "finally_stop_and_confirm_ready": True,
        },
        "status": "p_scan_ready",
    }
    if (kp_base is None) != (tau_63_sec is None):
        raise ValueError("kp_base and tau_63_sec must be supplied together")
    if kp_base is not None and tau_63_sec is not None:
        report["pi_candidates"] = build_pi_candidates(
            spec, kp_base, tau_63_sec
        )
        report["status"] = "pi_preview_ready"
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the frozen empirical speed-loop tuning plan"
    )
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--kp-base", type=float)
    parser.add_argument("--tau-63", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = build_report(args.spec, args.kp_base, args.tau_63)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
