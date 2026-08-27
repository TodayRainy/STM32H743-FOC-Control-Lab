#!/usr/bin/env python3
"""Continuous Stage-2 d-axis current-loop bandwidth-grid validation."""

from __future__ import annotations

import argparse
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
    summarize,
    summarize_zero,
    wait_mode,
)
from stage2_current_loop_design import bandwidth_profile_name, build_controllers


DEFAULT_BANDWIDTHS_HZ = (750.0, 1000.0, 1250.0, 1500.0,
                         1750.0, 2000.0, 2250.0)
DEFAULT_STEPS_A = (0.5, -0.5, 1.0, -1.0, 1.5, -1.5,
                   2.0, -2.0, 2.5, -2.5, 3.0, -3.0)
SCOPE_SAMPLES = 4096
FOC_SAMPLE_HZ = 40000.0


def set_profile(port, axes):
    proto.send_command(port, f"PI IQ {axes['q']['kp']:.9g} {axes['q']['ki']:.9g}")
    proto.send_command(port, f"PI ID {axes['d']['kp']:.9g} {axes['d']['ki']:.9g}")
    proto.send_command(port, "PI CLEAR")


def build_run_plan(bandwidths_hz, rounds):
    ascending = ["active_legacy"] + [
        bandwidth_profile_name(value) for value in bandwidths_hz
    ]
    plan = [{"round": 1, "order": ascending}]
    if rounds == 2:
        plan.append({"round": 2, "order": list(reversed(ascending))})
    return plan


def relative_difference(a, b):
    denominator = max(abs(a), abs(b), 1.0e-12)
    return abs(a - b) / denominator


def pair_asymmetry(records, metric):
    differences = []
    magnitudes = sorted({abs(record["target_id_a"]) for record in records})
    for magnitude in magnitudes:
        positive = next((record.get(metric) for record in records
                         if math.isclose(record["target_id_a"], magnitude)), None)
        negative = next((record.get(metric) for record in records
                         if math.isclose(record["target_id_a"], -magnitude)), None)
        if positive is not None and negative is not None:
            differences.append(relative_difference(positive, negative))
    return max(differences) if differences else None


def summarize_profiles(records, zero_records, criteria):
    summaries = {}
    profile_names = sorted({record["profile"] for record in records})
    active_zero = [record for record in zero_records
                   if record["profile"] == "active_legacy"]
    active_zero_by_round = {record["round"]: record for record in active_zero}

    for profile in profile_names:
        selected = [record for record in records if record["profile"] == profile]
        scored = [record for record in selected if abs(record["target_id_a"]) >= 1.0]
        noises = [record for record in zero_records if record["profile"] == profile]
        rise_times = [record["rise_time_10_90_sec"] for record in scored
                      if record["rise_time_10_90_sec"] is not None]
        settling_times = [record["noise_limited_settling_time_2pct_sec"]
                          for record in scored
                          if record["noise_limited_settling_time_2pct_sec"] is not None]
        overshoots = [record["filtered_overshoot_pct"] for record in scored
                      if record["filtered_overshoot_pct"] is not None]
        round_asymmetries = []
        for round_index in sorted({record["round"] for record in scored}):
            asymmetry = pair_asymmetry(
                [record for record in scored if record["round"] == round_index],
                "rise_time_10_90_sec",
            )
            if asymmetry is not None:
                round_asymmetries.append(asymmetry)

        repeat_rise = []
        repeat_overshoot_pp = []
        if len({record["round"] for record in scored}) >= 2:
            for target in sorted({record["target_id_a"] for record in scored}):
                by_round = [record for record in scored
                            if math.isclose(record["target_id_a"], target)]
                if len(by_round) < 2:
                    continue
                first, second = by_round[0], by_round[1]
                if (first["rise_time_10_90_sec"] is not None
                        and second["rise_time_10_90_sec"] is not None):
                    repeat_rise.append(relative_difference(
                        first["rise_time_10_90_sec"], second["rise_time_10_90_sec"]
                    ))
                if (first["filtered_overshoot_pct"] is not None
                        and second["filtered_overshoot_pct"] is not None):
                    repeat_overshoot_pp.append(abs(
                        first["filtered_overshoot_pct"]
                        - second["filtered_overshoot_pct"]
                    ))

        max_id_noise = max((record["id_noise_rms_a"] for record in noises), default=None)
        max_voltage_noise = max((record["voltage_rms_v"] for record in noises),
                                default=None)
        comparable_noise_rounds = [record for record in noises
                                   if record["round"] in active_zero_by_round]
        checks = {
            "all_scored_points_present": (
                len(scored) == criteria["expected_scored_points"]
            ),
            "overshoot": bool(overshoots) and max(overshoots) <= criteria["max_overshoot_pct"],
            "settling": len(settling_times) == len(scored) and bool(scored)
                        and max(settling_times) <= criteria["max_settling_sec"],
            "steady_error_rms": bool(scored) and max(
                record["steady_error_rms_a"] for record in scored
            ) <= criteria["max_steady_error_rms_a"],
            "sign_symmetry": bool(round_asymmetries) and max(round_asymmetries)
                             <= criteria["max_sign_rise_difference"],
            "repeat_rise": (not criteria["repeat_required"])
                           or (bool(repeat_rise) and max(repeat_rise)
                               <= criteria["max_repeat_rise_difference"]),
            "repeat_overshoot": (not criteria["repeat_required"])
                                or (bool(repeat_overshoot_pp)
                                    and max(repeat_overshoot_pp)
                                    <= criteria["max_repeat_overshoot_pp"]),
            "peak_current": bool(selected) and max(
                record["raw_peak_current_a"] for record in selected
            ) < criteria["max_peak_current_a"],
            "voltage_margin": bool(selected) and all(
                not record["voltage_saturated"] for record in selected
            ),
            "id_noise_vs_active": bool(comparable_noise_rounds) and all(
                record["id_noise_rms_a"] <= criteria["max_noise_ratio"]
                * active_zero_by_round[record["round"]]["id_noise_rms_a"]
                for record in comparable_noise_rounds
            ),
            "control_voltage_noise_vs_active": (
                bool(comparable_noise_rounds) and all(
                    record["voltage_rms_v"] <= criteria["max_noise_ratio"]
                    * active_zero_by_round[record["round"]]["voltage_rms_v"]
                    for record in comparable_noise_rounds
                )
            ),
        }
        summaries[profile] = {
            "scored_points": len(scored),
            "mean_rise_time_10_90_sec": (
                sum(rise_times) / len(rise_times) if rise_times else None
            ),
            "max_noise_limited_settling_time_2pct_sec": (
                max(settling_times) if settling_times else None
            ),
            "max_filtered_overshoot_pct": max(overshoots) if overshoots else None,
            "max_steady_error_rms_a": max(
                (record["steady_error_rms_a"] for record in scored), default=None
            ),
            "max_sign_rise_difference_ratio": (
                max(round_asymmetries) if round_asymmetries else None
            ),
            "max_repeat_rise_difference_ratio": max(repeat_rise) if repeat_rise else None,
            "max_repeat_overshoot_difference_pp": (
                max(repeat_overshoot_pp) if repeat_overshoot_pp else None
            ),
            "max_zero_id_noise_rms_a": max_id_noise,
            "max_zero_voltage_rms_v": max_voltage_noise,
            "raw_peak_current_a": max(
                (record["raw_peak_current_a"] for record in selected), default=None
            ),
            "raw_peak_rpm": max(
                (record["raw_peak_rpm"] for record in selected), default=None
            ),
            "checks": checks,
            "passes_all_checks": all(checks.values()),
        }
    return summaries


def capture_zero(port, timeout, csv_path):
    scope.arm_scope(port, 1)
    time.sleep(SCOPE_SAMPLES / FOC_SAMPLE_HZ + 0.05)
    capture, samples = scope.dump_scope(port, timeout)
    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    scope.write_csv(csv_path, samples, sample_period)
    return summarize_zero(samples), samples


def capture_step(port, target_id, timeout, csv_path):
    scope.arm_scope(port, 1)
    proto.send_command(port, f"CURRENT 0 {target_id:g}")
    time.sleep(SCOPE_SAMPLES / FOC_SAMPLE_HZ + 0.05)
    proto.send_command(port, "CURRENT 0 0")
    capture, samples = scope.dump_scope(port, timeout)
    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    scope.write_csv(csv_path, samples, sample_period)
    return summarize(samples, target_id, sample_period), samples


def main():
    parser = argparse.ArgumentParser(
        description="Compare a current-PI bandwidth grid in one powered session",
    )
    parser.add_argument("--port")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--bandwidths", type=float, nargs="+",
                        default=DEFAULT_BANDWIDTHS_HZ, metavar="HZ")
    parser.add_argument("--rounds", type=int, choices=(1, 2), default=2)
    parser.add_argument("--steps", type=float, nargs="+", default=DEFAULT_STEPS_A)
    parser.add_argument("--max-step-current", type=float, default=4.0)
    parser.add_argument("--max-calibration-current", type=float, default=5.0)
    parser.add_argument("--max-rpm", type=float, default=500.0)
    parser.add_argument("--abort-overshoot-pct", type=float, default=30.0)
    parser.add_argument("--abort-settling-ms", type=float, default=2.0)
    parser.add_argument("--max-voltage-utilization", type=float, default=0.98)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--results-dir", type=Path, default=Path("tests/results"))
    args = parser.parse_args()

    bandwidths = tuple(sorted(args.bandwidths))
    if len(set(bandwidths)) != len(bandwidths):
        raise ValueError("bandwidth candidates must be unique")
    if any(value < 500.0 or value > 2500.0 for value in bandwidths):
        raise ValueError("coarse-grid bandwidths must stay within 500 to 2500 Hz")
    if any(abs(step) > 3.0 or abs(step) < 0.1 for step in args.steps):
        raise ValueError("d-axis validation steps must be within 0.1 to 3.0 A")
    if not args.plan_only and not args.port:
        parser.error("--port is required unless --plan-only is used")

    controllers = build_controllers(bandwidths)
    run_plan = build_run_plan(bandwidths, args.rounds)
    criteria = {
        "scored_min_abs_current_a": 1.0,
        "expected_scored_points": (
            sum(abs(step) >= 1.0 for step in args.steps) * args.rounds
        ),
        "repeat_required": args.rounds == 2,
        "max_overshoot_pct": 15.0,
        "max_settling_sec": 0.75e-3,
        "max_steady_error_rms_a": 0.08,
        "max_sign_rise_difference": 0.20,
        "max_repeat_rise_difference": 0.15,
        "max_repeat_overshoot_pp": 5.0,
        "max_peak_current_a": args.max_step_current,
        "max_noise_ratio": 1.20,
    }
    captures_per_visit = 1 + len(args.steps)
    planned_visits = sum(len(item["order"]) for item in run_plan)
    if args.plan_only:
        print(json.dumps({
            "bandwidths_hz": bandwidths,
            "controllers": controllers,
            "run_plan": run_plan,
            "steps_a": args.steps,
            "zero_capture_per_profile_visit": True,
            "planned_profile_visits": planned_visits,
            "planned_scope_captures": planned_visits * captures_per_visit,
            "scope_duration_per_capture_sec": SCOPE_SAMPLES / FOC_SAMPLE_HZ,
            "acceptance_criteria": criteria,
            "runtime_abort_gates": {
                "peak_current_a": args.max_step_current,
                "peak_rpm": args.max_rpm,
                "overshoot_pct": args.abort_overshoot_pct,
                "noise_limited_settling_ms": args.abort_settling_ms,
                "max_voltage_utilization": args.max_voltage_utilization,
            },
        }, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.results_dir.mkdir(parents=True, exist_ok=True)
    output = args.results_dir / f"stage2_current_loop_grid_{stamp}.json"
    records = []
    zero_records = []
    skipped = []
    rejections = []
    error = None
    final_mode = None
    measured_bus_voltage = None
    rejected_cutoff_hz = None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            buffer = bytearray()
            port.reset_input_buffer()
            try:
                ready = wait_mode(port, buffer, MODE_READY, args.timeout,
                                  args.max_calibration_current, args.max_rpm)
                measured_bus_voltage = ready[29]
                if not 18.0 <= measured_bus_voltage <= 28.0:
                    raise RuntimeError(
                        f"measured bus voltage is out of range: {measured_bus_voltage:.3f} V"
                    )
                voltage_limit = (measured_bus_voltage / math.sqrt(3.0)
                                 * args.max_voltage_utilization)
                proto.send_command(port, "FF OFF")
                proto.send_command(port, "START CURRENT")
                wait_mode(port, buffer, MODE_TORQUE_RUN, args.timeout,
                          args.max_calibration_current, args.max_rpm)
                proto.send_command(port, "CURRENT 0 0")
                time.sleep(0.25)

                for round_plan in run_plan:
                    round_index = round_plan["round"]
                    for profile_name in round_plan["order"]:
                        bandwidth_hz = next((value for value in bandwidths
                                             if bandwidth_profile_name(value)
                                             == profile_name), None)
                        if (bandwidth_hz is not None
                                and rejected_cutoff_hz is not None
                                and bandwidth_hz > rejected_cutoff_hz):
                            skipped.append({
                                "round": round_index,
                                "profile": profile_name,
                                "reason": "above runtime rejection cutoff",
                            })
                            continue

                        axes = controllers[profile_name]
                        set_profile(port, axes)
                        proto.send_command(port, "CURRENT 0 0")
                        time.sleep(0.1)
                        stem = f"stage2_grid_{stamp}_r{round_index}_{profile_name}"
                        zero_path = args.results_dir / f"{stem}_zero.csv"
                        zero_result, zero_samples = capture_zero(port, args.timeout, zero_path)
                        zero_peak_rpm = max(abs(sample[9]) for sample in zero_samples) \
                                        * 60.0 / (2.0 * math.pi * 10.0)
                        zero_result.update({
                            "round": round_index,
                            "profile": profile_name,
                            "bandwidth_hz": bandwidth_hz,
                            "raw_peak_rpm": zero_peak_rpm,
                            "csv": str(zero_path.resolve()),
                        })
                        zero_records.append(zero_result)
                        if zero_result["raw_peak_current_a"] >= args.max_step_current:
                            raise RuntimeError(
                                f"zero-current scope limit exceeded: "
                                f"{zero_result['raw_peak_current_a']:.3f} A"
                            )
                        if zero_peak_rpm >= args.max_rpm:
                            raise RuntimeError(
                                f"zero-current speed limit exceeded: {zero_peak_rpm:.1f} rpm"
                            )

                        reject_profile = False
                        for point_index, target_id in enumerate(args.steps, start=1):
                            csv_path = args.results_dir / (
                                f"{stem}_p{point_index:02d}_{target_id:+g}A.csv"
                            )
                            result, samples = capture_step(
                                port, target_id, args.timeout, csv_path
                            )
                            peak_rpm = max(abs(sample[9]) for sample in samples) \
                                       * 60.0 / (2.0 * math.pi * 10.0)
                            result.update({
                                "round": round_index,
                                "profile": profile_name,
                                "bandwidth_hz": bandwidth_hz,
                                "kp_d": axes["d"]["kp"],
                                "ki_d": axes["d"]["ki"],
                                "kp_q": axes["q"]["kp"],
                                "ki_q": axes["q"]["ki"],
                                "raw_peak_rpm": peak_rpm,
                                "voltage_limit_v": voltage_limit,
                                "voltage_saturated": (
                                    result["raw_peak_voltage_v"] >= voltage_limit
                                ),
                                "csv": str(csv_path.resolve()),
                            })
                            # Append before every gate so the failing point remains in JSON.
                            records.append(result)

                            if result["raw_peak_current_a"] >= args.max_step_current:
                                raise RuntimeError(
                                    f"scope current limit exceeded: "
                                    f"{result['raw_peak_current_a']:.3f} A"
                                )
                            if peak_rpm >= args.max_rpm:
                                raise RuntimeError(
                                    f"scope speed limit exceeded: {peak_rpm:.1f} rpm"
                                )
                            if result["voltage_saturated"]:
                                raise RuntimeError(
                                    f"voltage utilization limit exceeded: "
                                    f"{result['raw_peak_voltage_v']:.3f} V"
                                )

                            severe_reason = None
                            if abs(target_id) >= 1.0:
                                if (result["filtered_overshoot_pct"] is not None
                                        and result["filtered_overshoot_pct"]
                                        > args.abort_overshoot_pct):
                                    severe_reason = "runtime overshoot gate"
                                settling = result[
                                    "noise_limited_settling_time_2pct_sec"
                                ]
                                if (settling is None
                                        or settling * 1000.0 > args.abort_settling_ms):
                                    severe_reason = "runtime settling gate"

                            if severe_reason is not None:
                                rejection = {
                                    "round": round_index,
                                    "profile": profile_name,
                                    "bandwidth_hz": bandwidth_hz,
                                    "target_id_a": target_id,
                                    "reason": severe_reason,
                                }
                                rejections.append(rejection)
                                if profile_name == "active_legacy":
                                    raise RuntimeError(
                                        f"active baseline failed {severe_reason}"
                                    )
                                rejected_cutoff_hz = bandwidth_hz if (
                                    rejected_cutoff_hz is None
                                ) else min(rejected_cutoff_hz, bandwidth_hz)
                                reject_profile = True

                            wait_mode(port, buffer, MODE_TORQUE_RUN, 2.0,
                                      args.max_step_current, args.max_rpm)
                            time.sleep(0.05)
                            if reject_profile:
                                break
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    proto.send_command(port, "CURRENT 0 0")
                    set_profile(port, controllers["active_legacy"])
                    proto.send_command(port, "STOP")
                    final = wait_mode(port, buffer, MODE_READY, 2.0,
                                      args.max_calibration_current, args.max_rpm)
                    final_mode = int(round(final[27]))
                except Exception as cleanup_exc:
                    cleanup_error = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    error = cleanup_error if error is None else f"{error}; cleanup: {cleanup_error}"
    except Exception as exc:
        if error is None:
            error = f"{type(exc).__name__}: {exc}"

    profile_summaries = summarize_profiles(records, zero_records, criteria)
    passing_bandwidths = [value for value in bandwidths
                          if profile_summaries.get(
                              bandwidth_profile_name(value), {}
                          ).get("passes_all_checks", False)]
    step_grid_selection = {
        "highest_passing_profile": (
            bandwidth_profile_name(max(passing_bandwidths))
            if passing_bandwidths else None
        ),
        "highest_passing_bandwidth_hz": (
            max(passing_bandwidths) if passing_bandwidths else None
        ),
        "status": (
            "step-grid winner only; requires refined grid and closed-loop frequency-response validation before PI writeback"
            if passing_bandwidths
            else "no identified candidate passed every predefined step-grid check"
        ),
    }
    output.write_text(json.dumps({
        "design_baseline": "tests/baselines/stage2_current_loop_design_20260824.json",
        "bandwidths_hz": bandwidths,
        "run_plan": run_plan,
        "steps_a": args.steps,
        "acceptance_criteria": criteria,
        "runtime_abort_gates": {
            "overshoot_pct": args.abort_overshoot_pct,
            "noise_limited_settling_ms": args.abort_settling_ms,
            "max_voltage_utilization": args.max_voltage_utilization,
        },
        "measured_bus_voltage_v": measured_bus_voltage,
        "records": records,
        "zero_records": zero_records,
        "profile_summaries": profile_summaries,
        "step_grid_selection": step_grid_selection,
        "runtime_rejections": rejections,
        "skipped_profiles": skipped,
        "runtime_rejection_cutoff_hz": rejected_cutoff_hz,
        "error": error,
        "final_mode": final_mode,
    }, indent=2) + "\n", encoding="utf-8")

    if error is not None:
        raise RuntimeError(f"experiment failed; partial result saved to {output}: {error}")
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    print(json.dumps({
        "result": str(output.resolve()),
        "records": len(records),
        "zero_records": len(zero_records),
        "profile_summaries": profile_summaries,
        "step_grid_selection": step_grid_selection,
        "runtime_rejections": rejections,
        "skipped_profiles": skipped,
        "final_mode": final_mode,
    }, indent=2))


if __name__ == "__main__":
    main()
