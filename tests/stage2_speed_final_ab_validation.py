#!/usr/bin/env python3
"""Close the speed-loop study with a default-versus-fc24 wide-range A/B."""

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
from scope_current_step import wait_mode
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    capture_step,
    check_telemetry,
    open_serial_with_retry,
    step_passes,
    stop_and_confirm_ready,
    supervised_wait,
)
from stage2_speed_high_bandwidth_probe import restore_defaults, save_checkpoint


LEGACY_KP = 0.015764175 / 10.0
LEGACY_KI = 0.246037498 / 10.0
FC24_KP = 0.005365337845003614
FC24_KI = 0.2585237075491248
CONTROLLERS = (
    {"name": "source_default", "kp": LEGACY_KP, "ki": LEGACY_KI},
    {"name": "fc24_pm70", "kp": FC24_KP, "ki": FC24_KI},
)
RAMP_RATE_RPM_PER_SEC = 3000.0
RAMP_HOLD_SEC = 0.45
RAMP_TARGETS_POSITIVE_FIRST = (
    0.0, 500.0, 1500.0, 3000.0, 4000.0,
    3000.0, 1500.0, 500.0, 0.0,
    -500.0, -1500.0, -3000.0, -4000.0,
    -3000.0, -1500.0, -500.0, 0.0,
)
RAMP_TARGETS_NEGATIVE_FIRST = tuple(-value for value in RAMP_TARGETS_POSITIVE_FIRST)
STEP_TRANSITIONS = (
    (0.0, 1500.0),
    (1500.0, 0.0),
    (0.0, -1500.0),
    (-1500.0, 0.0),
    (3000.0, 3300.0),
    (3000.0, 2700.0),
    (-3000.0, -3300.0),
    (-3000.0, -2700.0),
)


def visit_order() -> list[tuple[int, dict]]:
    return (
        [(0, item) for item in CONTROLLERS]
        + [(1, item) for item in reversed(CONTROLLERS)]
    )


def plan() -> dict:
    visits = visit_order()
    return {
        "schema": "stage2-speed-final-ab-plan-v1",
        "decision": "D-043",
        "purpose": (
            "compare source-default speed PI with fc24 over the usable speed "
            "range and close the speed-loop study without further PI search"
        ),
        "controllers": list(CONTROLLERS),
        "visits": [
            {"round": round_index, "controller": item["name"]}
            for round_index, item in visits
        ],
        "ramp_rate_rpm_per_sec": RAMP_RATE_RPM_PER_SEC,
        "ramp_targets_positive_first_rpm": list(RAMP_TARGETS_POSITIVE_FIRST),
        "ramp_targets_negative_first_rpm": list(RAMP_TARGETS_NEGATIVE_FIRST),
        "step_transitions_rpm": [list(item) for item in STEP_TRANSITIONS],
        "scope_capture_count": len(visits) * len(STEP_TRANSITIONS),
        "maximum_commanded_speed_rpm": 4000.0,
        "direct_positive_to_negative_4000_reversal": False,
        "acceptance": {
            "maximum_noise_adjusted_overshoot_pct": 10.0,
            "maximum_large_step_settling_time_sec": 0.15,
            "steady_error_within_each_record_settling_band": True,
            "local_steps_use_existing_speed_v1_acceptance": True,
            "large_steps_report_iq_limit_duration_as_anti_windup_evidence": True,
            "no_new_weighted_score": True,
        },
        "safety": {
            "speed_iq_reference_limit_a": 4.0,
            "abort_actual_dq_current_a": 5.0,
            "abort_mechanical_speed_rpm": 4800.0,
            "abort_voltage_utilization": 0.98,
        },
        "requires_24v": True,
        "motor_will_rotate": True,
        "automatic_flash": False,
        "restore_default_speed_pi": True,
        "finally_stop_and_confirm_ready": True,
    }


def electrical_rad_s_to_rpm(value: float, pole_pairs: int) -> float:
    return value / pole_pairs * 60.0 / (2.0 * math.pi)


def set_and_wait_speed_command(port, buffer: bytearray, target_rpm: float,
                               spec: dict, timeout_sec: float = 1.5) -> None:
    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, f"SPEED {target_rpm:g}")
    deadline = time.monotonic() + timeout_sec
    retry_at = time.monotonic() + 0.25
    pole_pairs = int(spec["control"]["pole_pairs"])
    last_target = None
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        frame = sample[1]
        check_telemetry(frame, spec)
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode while confirming command")
        last_target = electrical_rad_s_to_rpm(frame[24], pole_pairs)
        if abs(last_target - target_rpm) <= 1.0:
            return
        if time.monotonic() >= retry_at:
            proto.send_command(port, f"SPEED {target_rpm:g}")
            retry_at = time.monotonic() + 0.25
    raise RuntimeError(
        f"speed command was not accepted: requested {target_rpm:g} rpm, "
        f"telemetry target {last_target} rpm"
    )


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def longest_limit_duration(rows: list[dict], limit_a: float) -> float:
    longest = 0.0
    started = None
    previous = None
    for row in rows:
        timestamp = float(row["elapsed_sec"])
        at_limit = abs(float(row["iq_ref"])) >= 0.99 * limit_a
        continuous = previous is None or timestamp - previous <= 0.1
        if at_limit and continuous:
            if started is None:
                started = timestamp
            longest = max(longest, timestamp - started)
        elif at_limit:
            started = timestamp
        else:
            started = None
        previous = timestamp
    return longest


def summarize_ramp_segment(rows: list[dict], target_rpm: float,
                           iq_limit_a: float) -> dict:
    if not rows:
        raise RuntimeError("no telemetry received during wide-range ramp segment")
    errors = [
        float(row["speed_ref_rpm"]) - float(row["actual_rpm"])
        for row in rows
    ]
    tail_count = max(1, len(rows) // 4)
    tail = rows[-tail_count:]
    steady_errors = [
        float(row["speed_ref_rpm"]) - float(row["actual_rpm"])
        for row in tail
    ]
    steady_speed = [float(row["actual_rpm"]) for row in tail]
    steady_mean = sum(steady_speed) / len(steady_speed)
    return {
        "target_rpm": target_rpm,
        "frames": len(rows),
        "speed_error_rms_rpm": rms(errors),
        "peak_abs_speed_error_rpm": max(abs(value) for value in errors),
        "steady_error_mean_rpm": sum(steady_errors) / len(steady_errors),
        "steady_speed_ripple_rms_rpm": rms([
            value - steady_mean for value in steady_speed
        ]),
        "peak_iq_reference_a": max(abs(float(row["iq_ref"])) for row in rows),
        "rms_iq_reference_a": rms([float(row["iq_ref"]) for row in rows]),
        "peak_actual_dq_current_a": max(
            math.hypot(float(row["iq"]), float(row["id"])) for row in rows
        ),
        "minimum_voltage_scale": min(
            float(row["voltage_scale"]) for row in rows
        ),
        "iq_limit_duration_sec": longest_limit_duration(rows, iq_limit_a),
    }


def capture_ramp_segment(port, buffer: bytearray, duration_sec: float,
                         spec: dict, started: float, profile: str,
                         phase: str, target_rpm: float,
                         all_rows: list[dict]) -> dict:
    deadline = time.monotonic() + duration_sec
    rows = []
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        timestamp, frame = sample
        check_telemetry(frame, spec)
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode during wide-range ramp")
        proto.append_record(
            rows, started, timestamp, profile, phase, target_rpm, frame
        )
    all_rows.extend(rows)
    return summarize_ramp_segment(
        rows, target_rpm,
        float(spec["safety"]["speed_iq_reference_limit_a"]),
    )


def write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def step_is_large(record: dict) -> bool:
    return abs(float(record["step_amplitude_rpm"])) > 500.0


def summarize_controller(records: list[dict], ramp_visits: list[dict],
                         spec: dict) -> dict:
    local = [item for item in records if not step_is_large(item)]
    large = [item for item in records if step_is_large(item)]
    acceptance = spec["acceptance"]
    maximum_overshoot = max(
        item["noise_adjusted_overshoot_pct"] for item in records
    )
    local_valid = all(step_passes(item, spec, True) for item in local)
    large_recovery_valid = all(
        not item["nondecaying_oscillation"]
        and item["settling_time_sec"] is not None
        and item["settling_time_sec"] <= 0.15
        and abs(item["steady_error_rpm"]) <= item["settling_band_rpm"]
        for item in large
    )
    time_to_90 = [
        item["time_to_90_sec"] for item in records
        if item["time_to_90_sec"] is not None
    ]
    settling = [
        item["settling_time_sec"] for item in records
        if item["settling_time_sec"] is not None
    ]
    return {
        "step_count": len(records),
        "ramp_visit_count": len(ramp_visits),
        "maximum_noise_adjusted_overshoot_pct": maximum_overshoot,
        "worst_time_to_90_sec": max(time_to_90) if time_to_90 else None,
        "worst_settling_time_sec": max(settling) if settling else None,
        "maximum_iq_limit_duration_sec": max(
            item["iq_limit_duration_sec"] for item in records
        ),
        "peak_iq_reference_a": max(item["peak_iq_ref_a"] for item in records),
        "peak_actual_dq_current_a": max(
            item["peak_actual_dq_current_a"] for item in records
        ),
        "mean_rms_iq_reference_a": sum(
            item["rms_iq_ref_a"] for item in records
        ) / len(records),
        "maximum_ramp_steady_error_abs_rpm": max(
            abs(stage["steady_error_mean_rpm"])
            for visit in ramp_visits for stage in visit["stages"]
        ),
        "maximum_ramp_speed_error_rms_rpm": max(
            stage["speed_error_rms_rpm"]
            for visit in ramp_visits for stage in visit["stages"]
        ),
        "minimum_ramp_voltage_scale": min(
            stage["minimum_voltage_scale"]
            for visit in ramp_visits for stage in visit["stages"]
        ),
        "local_step_performance_valid": local_valid,
        "large_step_anti_windup_recovery_valid": large_recovery_valid,
        "overshoot_valid": maximum_overshoot
        <= acceptance["maximum_noise_adjusted_overshoot_pct"],
        "closing_candidate_valid": (
            local_valid and large_recovery_valid
            and maximum_overshoot
            <= acceptance["maximum_noise_adjusted_overshoot_pct"]
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--ramps-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    experiment_plan = plan()
    if args.ramps_only:
        experiment_plan = dict(experiment_plan)
        experiment_plan["execution"] = "ramps_only"
        experiment_plan["scope_capture_count"] = 0
    if args.plan_only:
        print(json.dumps(experiment_plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / (
        f"stage2_speed_final_ab_{stamp}.json"
    )
    scope_dir = output.with_name(f"{output.stem}_scope")
    telemetry_dir = output.with_name(f"{output.stem}_telemetry")
    scope_dir.mkdir(parents=True, exist_ok=True)
    telemetry_dir.mkdir(parents=True, exist_ok=True)
    result = {
        "schema": "stage2-speed-final-ab-result-v1",
        "plan": experiment_plan,
        "ramp_visits": [],
        "step_records": [],
        "controller_summaries": None,
        "status": "running",
        "error": None,
        "final_mode": None,
        "default_pi_restored": False,
        "default_restore_error": None,
    }
    buffer = bytearray()
    started = time.monotonic()

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            for command in (
                "SPEED EST WIN",
                f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
                "FF OFF",
                f"TRACK RAMP {RAMP_RATE_RPM_PER_SEC:g}",
                "PI CLEAR",
                "START SPEED",
            ):
                proto.send_command(port, command)
            wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0)

            for round_index, controller in visit_order():
                proto.send_command(port, f"TRACK RAMP {RAMP_RATE_RPM_PER_SEC:g}")
                set_and_wait_speed_command(port, buffer, 0.0, spec)
                supervised_wait(port, buffer, 1.5, spec, MODE_SPEED_TORQUE_RUN)
                proto.send_command(
                    port,
                    f"PI SPEED {controller['kp']:.9g} {controller['ki']:.9g}",
                )
                proto.send_command(port, "PI CLEAR")

                targets = (
                    RAMP_TARGETS_POSITIVE_FIRST if round_index == 0
                    else RAMP_TARGETS_NEGATIVE_FIRST
                )
                telemetry_rows: list[dict] = []
                telemetry_path = telemetry_dir / (
                    f"r{round_index}_{controller['name']}.csv"
                )
                ramp_visit = {
                    "round": round_index,
                    "controller": controller["name"],
                    "kp": controller["kp"],
                    "ki": controller["ki"],
                    "telemetry_csv": str(telemetry_path.resolve()),
                    "stages": [],
                }
                result["ramp_visits"].append(ramp_visit)
                previous = targets[0]
                for index, target in enumerate(targets):
                    set_and_wait_speed_command(port, buffer, target, spec)
                    duration = (
                        abs(target - previous) / RAMP_RATE_RPM_PER_SEC
                        + RAMP_HOLD_SEC
                    )
                    phase = f"wide_{index:02d}_{previous:+g}_to_{target:+g}"
                    ramp_visit["stages"].append(capture_ramp_segment(
                        port, buffer, duration, spec, started,
                        controller["name"], phase, target, telemetry_rows,
                    ))
                    write_rows(telemetry_path, telemetry_rows)
                    save_checkpoint(output, result)
                    previous = target

                if not args.ramps_only:
                    transitions = (
                        STEP_TRANSITIONS if round_index == 0
                        else tuple(reversed(STEP_TRANSITIONS))
                    )
                    for step_index, (base_rpm, target_rpm) in enumerate(transitions):
                        path = scope_dir / (
                            f"r{round_index}_{controller['name']}_{step_index:02d}_"
                            f"{base_rpm:+g}_to_{target_rpm:+g}rpm.csv"
                        )
                        record = capture_step(
                            port, buffer, base_rpm, target_rpm, spec, 0.0, path,
                        )
                        record.update({
                            "round": round_index,
                            "controller": controller["name"],
                            "kp": controller["kp"],
                            "ki": controller["ki"],
                            "base_rpm": base_rpm,
                            "command_target_rpm": target_rpm,
                        })
                        result["step_records"].append(record)
                        save_checkpoint(output, result)

                proto.send_command(port, f"TRACK RAMP {RAMP_RATE_RPM_PER_SEC:g}")
                proto.send_command(port, "SPEED 0")
                supervised_wait(port, buffer, 1.7, spec, MODE_SPEED_TORQUE_RUN)

            if args.ramps_only:
                result["controller_summaries"] = {
                    controller["name"]: {
                        "ramp_visit_count": len([
                            item for item in result["ramp_visits"]
                            if item["controller"] == controller["name"]
                        ]),
                        "maximum_steady_error_abs_rpm": max(
                            abs(stage["steady_error_mean_rpm"])
                            for visit in result["ramp_visits"]
                            if visit["controller"] == controller["name"]
                            for stage in visit["stages"]
                        ),
                        "maximum_speed_error_rms_rpm": max(
                            stage["speed_error_rms_rpm"]
                            for visit in result["ramp_visits"]
                            if visit["controller"] == controller["name"]
                            for stage in visit["stages"]
                        ),
                        "peak_iq_reference_a": max(
                            stage["peak_iq_reference_a"]
                            for visit in result["ramp_visits"]
                            if visit["controller"] == controller["name"]
                            for stage in visit["stages"]
                        ),
                        "peak_actual_dq_current_a": max(
                            stage["peak_actual_dq_current_a"]
                            for visit in result["ramp_visits"]
                            if visit["controller"] == controller["name"]
                            for stage in visit["stages"]
                        ),
                        "minimum_voltage_scale": min(
                            stage["minimum_voltage_scale"]
                            for visit in result["ramp_visits"]
                            if visit["controller"] == controller["name"]
                            for stage in visit["stages"]
                        ),
                    }
                    for controller in CONTROLLERS
                }
            else:
                result["controller_summaries"] = {
                    controller["name"]: summarize_controller(
                        [item for item in result["step_records"]
                         if item["controller"] == controller["name"]],
                        [item for item in result["ramp_visits"]
                         if item["controller"] == controller["name"]],
                        spec,
                    )
                    for controller in CONTROLLERS
                }
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException, ValueError,
                KeyboardInterrupt) as error:
            result["status"] = "error"
            result["error"] = str(error) or error.__class__.__name__
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer,
            )
            try:
                port.close()
            except serial.SerialException:
                pass
            if result["final_mode"] == MODE_READY:
                restored, restore_mode, restore_error = restore_defaults(
                    args.port, args.baudrate,
                )
                result["default_pi_restored"] = restored
                result["default_restore_error"] = restore_error
                if restore_mode is not None:
                    result["final_mode"] = restore_mode
            save_checkpoint(output, result)

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["default_pi_restored"]:
        raise RuntimeError("source-default speed PI was not restored")
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "D-043 final speed A/B failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "ramp_visits": len(result["ramp_visits"]),
        "step_records": len(result["step_records"]),
        "controller_summaries": result["controller_summaries"],
        "final_mode": result["final_mode"],
        "default_pi_restored": result["default_pi_restored"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
