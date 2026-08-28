#!/usr/bin/env python3
"""Validate current-controlled I/f startup and active-flux handoff."""

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


MODE_SPEED_TORQUE_RUN = 12
MODE_READY = 17
POLE_PAIRS = 10.0
START_TARGETS_RPM = (1500.0, -1500.0)


def circular_error(estimate: float, reference: float) -> float:
    return math.atan2(math.sin(estimate-reference),
                      math.cos(estimate-reference))


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = fraction*(len(ordered)-1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position-lower
    return ordered[lower]*(1.0-weight)+ordered[upper]*weight


def electrical_to_mechanical_rpm(value: float) -> float:
    return value/POLE_PAIRS*60.0/(2.0*math.pi)


def experiment_plan() -> dict:
    return {
        "schema": "stage5-sensorless-handoff-plan-v1",
        "control": "current-controlled I/f to active-flux observer",
        "encoder_role": "truth and test supervision only",
        "independent_start_targets_rpm": list(START_TARGETS_RPM),
        "alignment": {"id_a": 0.8, "duration_s": 0.2},
        "open_loop": {
            "acceleration_iq_a": 1.0,
            "handoff_iq_a": 0.15,
            "current_taper_a_per_s": 5.0,
            "acceleration_rpm_per_s": 3000.0,
        },
        "handoff": {
            "speed_rpm": 1000.0,
            "phase_gate_deg": 60.0,
            "speed_gate_rpm": 100.0,
            "qualification_ms": 20.0,
            "blend_ms": 20.0,
        },
        "performance_gates": {
            "startup_time_s": 1.5,
            "steady_active_flux_angle_p95_deg": 15.0,
            "steady_speed_rms_error_rpm": 75.0,
            "maximum_iq_reference_step_a": 1.5,
        },
        "safety": {
            "maximum_dq_current_a": 5.5,
            "maximum_mechanical_speed_rpm": 1800.0,
            "bus_voltage_range_v": [18.0, 28.0],
            "minimum_voltage_scale": 0.75,
        },
        "requires_24v": True,
        "motor_will_align_then_rotate_bidirectionally": True,
        "finally_stop_and_confirm_ready": True,
    }


def append_record(records: list[dict], started: float, timestamp: float,
                  visit: str, frame: tuple[float, ...]) -> None:
    records.append({
        "elapsed_sec": timestamp-started,
        "visit": visit,
        "actual_rpm": proto.mechanical_rpm(frame),
        "virtual_angle": frame[8],
        "active_flux_angle": frame[11],
        "abz_angle": frame[12],
        "virtual_speed_rpm": electrical_to_mechanical_rpm(frame[14]),
        "active_flux_speed_rpm": electrical_to_mechanical_rpm(frame[17]),
        "abz_speed_rpm": electrical_to_mechanical_rpm(frame[18]),
        "iq": frame[6],
        "id": frame[7],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "control_speed_rpm": electrical_to_mechanical_rpm(frame[25]),
        "speed_ref_rpm": electrical_to_mechanical_rpm(frame[28]),
        "voltage_scale": frame[26],
        "motor_mode": int(round(frame[27])),
        "vbus": frame[29],
    })


def read_checked(port, buffer: bytearray, deadline: float, limits: dict):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    proto.check_frame(frame, limits["max_current"], limits["max_rpm"])
    if not limits["min_vbus"] <= frame[29] <= limits["max_vbus"]:
        raise RuntimeError(f"bus voltage out of range: {frame[29]:.3f} V")
    if (int(round(frame[27])) == MODE_SPEED_TORQUE_RUN
            and frame[26] < limits["min_voltage_scale"]):
        raise RuntimeError(f"voltage scale too low: {frame[26]:.3f}")
    return sample


def wait_ready(port, buffer: bytearray, timeout: float, started: float,
               records: list[dict], limits: dict, visit: str):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.25), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, visit, frame)
        if int(round(frame[27])) == MODE_READY:
            return frame
    raise RuntimeError(f"timeout waiting for READY during {visit}")


def wait_mode(port, buffer: bytearray, expected: int, timeout: float,
              started: float, records: list[dict], limits: dict, visit: str):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.25), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, visit, frame)
        if int(round(frame[27])) == expected:
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected} during {visit}")


def run_start(port, buffer: bytearray, target_rpm: float, started: float,
              records: list[dict], limits: dict) -> dict:
    visit = f"start_{target_rpm:+g}"
    command_time = time.monotonic()
    proto.send_command(port, f"START SENSORLESS {target_rpm:g}")
    deadline = command_time+4.0
    entered_speed_mode = False
    settled_since = None
    frames: list[tuple[float, tuple[float, ...]]] = []

    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.25), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, visit, frame)
        frames.append((timestamp, frame))
        mode = int(round(frame[27]))
        if mode == MODE_SPEED_TORQUE_RUN:
            entered_speed_mode = True
        elif entered_speed_mode:
            raise RuntimeError(f"sensorless start left speed mode at {target_rpm:g} rpm")

        speed_ref_rpm = electrical_to_mechanical_rpm(frame[28])
        steady = (
            abs(speed_ref_rpm-target_rpm) <= 10.0
            and abs(proto.mechanical_rpm(frame)-target_rpm) <= 75.0
        )
        if steady:
            settled_since = settled_since or timestamp
            if timestamp-settled_since >= 0.5:
                break
        else:
            settled_since = None
    else:
        raise RuntimeError(f"sensorless start did not settle at {target_rpm:g} rpm")

    align = [(t, f) for t, f in frames if f[23] > 0.5 and abs(f[22]) < 0.3]
    open_loop = [(t, f) for t, f in frames
                 if target_rpm*f[22] > 0.7*abs(target_rpm)
                 and abs(electrical_to_mechanical_rpm(f[28])) < 100.0]
    closed = [(t, f) for t, f in frames
              if target_rpm*electrical_to_mechanical_rpm(f[28])
              > 500.0*abs(target_rpm)]
    if not align or not open_loop or not closed:
        raise RuntimeError(
            f"missing alignment, I/f, or closed-loop segment at {target_rpm:g} rpm"
        )

    handoff_time = closed[0][0]
    steady_frames = [frame for timestamp, frame in frames
                     if timestamp >= frames[-1][0]-0.5]
    angle_errors_deg = [abs(circular_error(frame[11], frame[12]))
                        *180.0/math.pi for frame in steady_frames]
    speed_errors = [electrical_to_mechanical_rpm(frame[17]-frame[18])
                    for frame in steady_frames]
    iq_reference = [frame[22] for _, frame in frames]
    iq_steps = [abs(iq_reference[index]-iq_reference[index-1])
                for index in range(1, len(iq_reference))]

    summary = {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "startup_time_s": handoff_time-command_time,
        "alignment_samples": len(align),
        "open_loop_samples": len(open_loop),
        "closed_loop_samples": len(closed),
        "steady_actual_speed_mean_rpm": sum(
            proto.mechanical_rpm(frame) for frame in steady_frames
        )/len(steady_frames),
        "steady_speed_error_rms_rpm": math.sqrt(sum(
            value*value for value in speed_errors
        )/len(speed_errors)),
        "steady_active_flux_angle_p95_deg": percentile(angle_errors_deg, 0.95),
        "steady_active_flux_angle_max_deg": max(angle_errors_deg),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for _, frame in frames
        ),
        "peak_reference_dq_current_a": max(
            math.hypot(frame[22], frame[23]) for _, frame in frames
        ),
        "maximum_iq_reference_step_a": max(iq_steps),
        "minimum_voltage_scale": min(frame[26] for _, frame in frames),
    }
    summary["passed"] = (
        summary["startup_time_s"] <= 1.5
        and summary["steady_speed_error_rms_rpm"] <= 75.0
        and summary["steady_active_flux_angle_p95_deg"] <= 15.0
        and summary["maximum_iq_reference_step_a"] <= 1.5
    )
    return summary


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
        f"stage5_sensorless_handoff_{stamp}.json"
    )
    telemetry_path = output.with_suffix(".csv")
    limits = {
        "max_current": 5.5,
        "max_rpm": 1800.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.75,
    }
    result = {
        "schema": "stage5-sensorless-handoff-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "starts": [],
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
                wait_ready(port, buffer, 10.0, started, records, limits,
                           "initial_ready")
                for command in (
                    "SPEED LIMIT 5", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1", "PI CLEAR",
                ):
                    proto.send_command(port, command)

                # ABZ is used only as validation truth. Calibrate it before the
                # sensorless starts so observer error is referenced correctly.
                proto.send_command(port, "START SPEED")
                wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                          started, records, limits, "encoder_truth_calibration")
                proto.send_command(port, "STOP")
                wait_ready(port, buffer, 3.0, started, records, limits,
                           "ready_after_encoder_calibration")

                for target_rpm in START_TARGETS_RPM:
                    summary = run_start(port, buffer, target_rpm, started,
                                        records, limits)
                    result["starts"].append(summary)
                    if not summary["passed"]:
                        raise RuntimeError(
                            f"handoff metrics failed at {target_rpm:g} rpm"
                        )
                    proto.send_command(port, "STOP")
                    wait_ready(port, buffer, 3.0, started, records, limits,
                               f"ready_after_{target_rpm:+g}")
                    time.sleep(0.3)

                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "STOP")
                    final = wait_ready(port, buffer, 3.0, started, records,
                                       limits, "final_ready")
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    wait_ready(port, buffer, 2.0, started, records, limits,
                               "defaults_restored")
                    result["final_mode"] = int(round(final[27]))
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        write_csv(telemetry_path, records)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if (result["status"] != "passed"
            or result["final_mode"] != MODE_READY
            or not result["defaults_restored"]):
        raise RuntimeError(result["error"] or "sensorless handoff test failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "starts": result["starts"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
