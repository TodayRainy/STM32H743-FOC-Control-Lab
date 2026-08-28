#!/usr/bin/env python3
"""Validate the active-flux-controlled sensorless speed range after I/f start."""

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
import stage5_sensorless_handoff_validation as handoff


SPEED_SEQUENCE_RPM = (3000.0, 5000.0, 6500.0, 3000.0, 750.0, 500.0)
SPEED_RAMP_RPM_PER_S = 3000.0


def experiment_plan() -> dict:
    return {
        "schema": "stage5-sensorless-closed-loop-plan-v1",
        "control": "active-flux observer after current-controlled I/f handoff",
        "encoder_role": "truth and test supervision only",
        "directions": [1, -1],
        "start_target_rpm": 1500.0,
        "post_handoff_speed_sequence_rpm": list(SPEED_SEQUENCE_RPM),
        "speed_ramp_rpm_per_s": SPEED_RAMP_RPM_PER_S,
        "steady_capture_sec": 0.5,
        "gates": {
            "active_flux_angle_p95_deg": 15.0,
            "observer_speed_rms_error_rpm": "max(50 rpm, 5% target)",
            "actual_speed_rms_error_rpm": "max(75 rpm, 3% target)",
        },
        "safety": {
            "maximum_dq_current_a": 5.5,
            "maximum_mechanical_speed_rpm": 6800.0,
            "bus_voltage_range_v": [18.0, 28.0],
            "minimum_voltage_scale": 0.75,
        },
        "finally_stop_and_confirm_ready": True,
    }


def capture_target(port, buffer: bytearray, target_rpm: float,
                   source_rpm: float, started: float, records: list[dict],
                   limits: dict) -> tuple[dict, float]:
    proto.send_command(port, f"SPEED {target_rpm:g}")
    deadline = time.monotonic()+abs(target_rpm-source_rpm) \
               /SPEED_RAMP_RPM_PER_S+4.0
    stable_since = None
    stable_frames = []
    visit = f"closed_{target_rpm:+g}"
    tolerance = max(75.0, 0.03*abs(target_rpm))

    while time.monotonic() < deadline:
        sample = handoff.read_checked(
            port, buffer, min(deadline, time.monotonic()+0.25), limits
        )
        if sample is None:
            continue
        timestamp, frame = sample
        handoff.append_record(records, started, timestamp, visit, frame)
        if int(round(frame[27])) != handoff.MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"left sensorless speed mode at {target_rpm:g} rpm")
        speed_ref_rpm = handoff.electrical_to_mechanical_rpm(frame[28])
        stable = (
            abs(speed_ref_rpm-target_rpm) <= 10.0
            and abs(proto.mechanical_rpm(frame)-target_rpm) <= tolerance
        )
        if stable:
            stable_since = stable_since or timestamp
            stable_frames.append(frame)
            if timestamp-stable_since >= 0.5:
                break
        else:
            stable_since = None
            stable_frames = []
    else:
        raise RuntimeError(f"sensorless speed did not settle at {target_rpm:g} rpm")

    angle_errors = [abs(handoff.circular_error(frame[11], frame[12]))
                    *180.0/math.pi for frame in stable_frames]
    observer_speed_errors = [handoff.electrical_to_mechanical_rpm(
        frame[17]-frame[18]
    ) for frame in stable_frames]
    actual_speed_errors = [proto.mechanical_rpm(frame)-target_rpm
                           for frame in stable_frames]
    summary = {
        "target_rpm": target_rpm,
        "samples": len(stable_frames),
        "active_flux_angle_p95_deg": handoff.percentile(angle_errors, 0.95),
        "active_flux_angle_max_deg": max(angle_errors),
        "observer_speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in observer_speed_errors
        )/len(observer_speed_errors)),
        "actual_speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in actual_speed_errors
        )/len(actual_speed_errors)),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in stable_frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in stable_frames),
    }
    summary["passed"] = (
        summary["active_flux_angle_p95_deg"] <= 15.0
        and summary["observer_speed_rms_error_rpm"]
            <= max(50.0, 0.05*abs(target_rpm))
        and summary["actual_speed_rms_error_rpm"]
            <= max(75.0, 0.03*abs(target_rpm))
    )
    return summary, target_rpm


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    plan = experiment_plan()
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_sensorless_closed_loop_{stamp}.json"
    )
    telemetry_path = output.with_suffix(".csv")
    limits = {
        "max_current": 5.5,
        "max_rpm": 6800.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.75,
    }
    result = {
        "schema": "stage5-sensorless-closed-loop-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "directions": [],
        "telemetry_csv": str(telemetry_path.resolve()),
        "final_mode": None,
        "defaults_restored": False,
    }
    records: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                handoff.wait_ready(port, buffer, 10.0, started, records,
                                   limits, "initial_ready")
                for command in (
                    "SPEED LIMIT 5", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1", "PI CLEAR",
                ):
                    proto.send_command(port, command)

                # Establish ABZ truth once; sensorless control remains separate.
                proto.send_command(port, "START SPEED")
                handoff.wait_mode(
                    port, buffer, handoff.MODE_SPEED_TORQUE_RUN, 20.0,
                    started, records, limits, "encoder_truth_calibration"
                )
                proto.send_command(port, "STOP")
                handoff.wait_ready(
                    port, buffer, 3.0, started, records, limits,
                    "ready_after_encoder_calibration"
                )

                for direction in (1.0, -1.0):
                    start_target = direction*1500.0
                    start_summary = handoff.run_start(
                        port, buffer, start_target, started, records, limits
                    )
                    if not start_summary["passed"]:
                        raise RuntimeError(
                            f"handoff failed at {start_target:g} rpm"
                        )
                    direction_result = {
                        "direction": int(direction),
                        "start": start_summary,
                        "holds": [],
                    }
                    source_rpm = start_target
                    for magnitude in SPEED_SEQUENCE_RPM:
                        summary, source_rpm = capture_target(
                            port, buffer, direction*magnitude, source_rpm,
                            started, records, limits,
                        )
                        direction_result["holds"].append(summary)
                        if not summary["passed"]:
                            raise RuntimeError(
                                f"closed-loop metrics failed at "
                                f"{direction*magnitude:g} rpm"
                            )
                    result["directions"].append(direction_result)
                    proto.send_command(port, "STOP")
                    handoff.wait_ready(
                        port, buffer, 3.0, started, records, limits,
                        f"ready_after_direction_{direction:+g}"
                    )
                    time.sleep(0.3)
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "STOP")
                    final = handoff.wait_ready(
                        port, buffer, 3.0, started, records, limits,
                        "final_ready"
                    )
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    handoff.wait_ready(
                        port, buffer, 2.0, started, records, limits,
                        "defaults_restored"
                    )
                    result["final_mode"] = int(round(final[27]))
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        write_csv(telemetry_path, records)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if (result["status"] != "passed"
            or result["final_mode"] != handoff.MODE_READY
            or not result["defaults_restored"]):
        raise RuntimeError(result["error"] or "sensorless range test failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "directions": len(result["directions"]),
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
