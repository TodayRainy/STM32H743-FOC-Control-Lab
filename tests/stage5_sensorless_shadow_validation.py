#!/usr/bin/env python3
"""Compare eSMO and active-flux observers while ABZ remains in control."""

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
SPEED_RAMP_RPM_PER_SEC = 3000.0
TEST_SPEEDS_RPM = (500.0, 750.0, 1000.0, 1500.0, 3000.0, 5000.0, 6500.0)
OBSERVER_CHANNELS = {
    "esmo": (10, 16),
    "active_flux": (11, 17),
}


def experiment_plan(test_speeds_rpm=TEST_SPEEDS_RPM) -> dict:
    sequence = [*test_speeds_rpm, 0.0]
    sequence.extend(-speed for speed in test_speeds_rpm)
    sequence.append(0.0)
    return {
        "schema": "stage5-sensorless-shadow-plan-v1",
        "purpose": "ABZ-truth comparison of eSMO and active-flux observers",
        "control_angle": "ABZ",
        "observers_have_control_authority": False,
        "motor_model": "IDENTIFIED",
        "speed_sequence_rpm": sequence,
        "speed_ramp_rpm_per_sec": SPEED_RAMP_RPM_PER_SEC,
        "steady_capture_sec": 0.8,
        "initial_gate": {
            "angle_p95_abs_deg": 15.0,
            "speed_error_rms_rpm": "max(50 rpm, 5% of target)",
            "angle_errors_over_90_deg": 0,
        },
        "safety": {
            "maximum_dq_current_a": 5.5,
            "maximum_mechanical_speed_rpm": 6800.0,
            "bus_voltage_range_v": [18.0, 28.0],
            "minimum_voltage_scale": 0.75,
        },
        "requires_24v": True,
        "motor_will_rotate_bidirectionally": True,
        "maximum_commanded_speed_rpm": max(test_speeds_rpm),
        "finally_stop_and_confirm_ready": True,
    }


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


def observer_summary(frames: list[tuple[float, ...]], target_rpm: float,
                     angle_index: int, speed_index: int) -> dict:
    errors = [circular_error(frame[angle_index], frame[12]) for frame in frames]
    abs_errors_deg = [abs(value)*180.0/math.pi for value in errors]
    speed_errors_rpm = [
        (frame[speed_index]-frame[18])/POLE_PAIRS*60.0/(2.0*math.pi)
        for frame in frames
    ]
    mean_sine = sum(math.sin(value) for value in errors)/len(errors)
    mean_cosine = sum(math.cos(value) for value in errors)/len(errors)
    angle_rms_deg = math.sqrt(sum(value*value for value in errors)/len(errors)) \
                    *180.0/math.pi
    speed_rms_rpm = math.sqrt(
        sum(value*value for value in speed_errors_rpm)/len(speed_errors_rpm)
    )
    speed_gate = max(50.0, 0.05*abs(target_rpm))
    large_angle_count = sum(value > 90.0 for value in abs_errors_deg)
    angle_p95 = percentile(abs_errors_deg, 0.95)
    return {
        "samples": len(frames),
        "circular_angle_bias_deg": math.atan2(mean_sine, mean_cosine)
                                   *180.0/math.pi,
        "angle_error_rms_deg": angle_rms_deg,
        "angle_error_p95_abs_deg": angle_p95,
        "angle_error_max_abs_deg": max(abs_errors_deg),
        "angle_errors_over_90_deg": large_angle_count,
        "speed_error_mean_rpm": sum(speed_errors_rpm)/len(speed_errors_rpm),
        "speed_error_rms_rpm": speed_rms_rpm,
        "speed_error_gate_rpm": speed_gate,
        "passed_initial_gate": (
            angle_p95 <= 15.0
            and speed_rms_rpm <= speed_gate
            and large_angle_count == 0
        ),
    }


def summarize_hold(target_rpm: float, frames: list[tuple[float, ...]]) -> dict:
    actual_rpm = [proto.mechanical_rpm(frame) for frame in frames]
    return {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "actual_speed_mean_rpm": sum(actual_rpm)/len(actual_rpm),
        "actual_speed_rms_error_rpm": math.sqrt(sum(
            (value-target_rpm)**2 for value in actual_rpm
        )/len(actual_rpm)),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
        "observers": {
            name: observer_summary(frames, target_rpm, *indices)
            for name, indices in OBSERVER_CHANNELS.items()
        },
    }


def minimum_reliable_speeds(holds: list[dict]) -> dict:
    result = {}
    for observer in OBSERVER_CHANNELS:
        result[observer] = {}
        for direction_name, direction in (("positive", 1), ("negative", -1)):
            candidates = sorted(
                (hold for hold in holds if hold["target_rpm"]*direction > 0.0),
                key=lambda hold: abs(hold["target_rpm"]),
            )
            minimum = None
            for index, hold in enumerate(candidates):
                if all(item["observers"][observer]["passed_initial_gate"]
                       for item in candidates[index:]):
                    minimum = abs(hold["target_rpm"])
                    break
            result[observer][direction_name] = minimum
    return result


def append_record(records: list[dict], started: float, timestamp: float,
                  phase: str, target_rpm: float, frame: tuple[float, ...]) -> None:
    records.append({
        "elapsed_sec": timestamp-started,
        "phase": phase,
        "target_rpm": target_rpm,
        "actual_rpm": proto.mechanical_rpm(frame),
        "iq": frame[6],
        "id": frame[7],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "voltage_scale": frame[26],
        "motor_mode": int(round(frame[27])),
        "vbus": frame[29],
        "esmo_angle": frame[10],
        "active_flux_angle": frame[11],
        "abz_angle": frame[12],
        "esmo_speed": frame[16],
        "active_flux_speed": frame[17],
        "abz_speed": frame[18],
        "esmo_angle_error_deg": circular_error(frame[10], frame[12])
                                *180.0/math.pi,
        "active_flux_angle_error_deg": circular_error(frame[11], frame[12])
                                       *180.0/math.pi,
    })


def read_checked(port, buffer: bytearray, deadline: float,
                 limits: dict):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    proto.check_frame(frame, limits["max_current"], limits["max_rpm"])
    if not limits["min_vbus"] <= frame[29] <= limits["max_vbus"]:
        raise RuntimeError(f"bus voltage is out of range: {frame[29]:.3f} V")
    if (int(round(frame[27])) == MODE_SPEED_TORQUE_RUN
            and frame[26] < limits["min_voltage_scale"]):
        raise RuntimeError(
            f"voltage scale fell below limit: {frame[26]:.3f}"
        )
    return sample


def wait_mode(port, buffer: bytearray, expected: int, timeout: float,
              started: float, records: list[dict], limits: dict,
              phase: str) -> tuple[float, ...]:
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.25), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, phase, 0.0, frame)
        if int(round(frame[27])) == expected:
            buffer.clear()
            port.reset_input_buffer()
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected} during {phase}")


def capture_target(port, buffer: bytearray, target_rpm: float,
                   source_rpm: float, started: float, records: list[dict],
                   limits: dict) -> list[tuple[float, ...]]:
    proto.send_command(port, f"SPEED {target_rpm:g}")
    timeout = abs(target_rpm-source_rpm)/SPEED_RAMP_RPM_PER_SEC+5.0
    deadline = time.monotonic()+timeout
    stable_since = None
    stable_frames = []
    tolerance = max(75.0, 0.03*abs(target_rpm))
    phase = f"speed_{target_rpm:g}"

    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.25), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, phase, target_rpm, frame)
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {phase}")
        speed_ref_rpm = frame[28]/POLE_PAIRS*60.0/(2.0*math.pi)
        stable = (
            abs(speed_ref_rpm-target_rpm) <= 10.0
            and abs(proto.mechanical_rpm(frame)-target_rpm) <= tolerance
        )
        if stable:
            if stable_since is None:
                stable_since = timestamp
                stable_frames = []
            stable_frames.append(frame)
            if timestamp-stable_since >= 0.8:
                return stable_frames
        else:
            stable_since = None
            stable_frames = []
    raise RuntimeError(f"speed did not settle at {target_rpm:g} rpm")


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def save_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--speeds", type=float, nargs="+", default=list(TEST_SPEEDS_RPM),
        help="positive mechanical-rpm points; both directions are measured",
    )
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    plan = experiment_plan(tuple(args.speeds))
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_sensorless_shadow_{stamp}.json"
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
        "schema": "stage5-sensorless-shadow-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "holds": [],
        "minimum_reliable_speed_rpm": None,
        "telemetry_csv": str(telemetry_path.resolve()),
        "final_mode": None,
        "defaults_restored": False,
    }
    records = []
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(
            args.port, args.baudrate, timeout=0.02, write_timeout=0.2
        ) as port:
            port.reset_input_buffer()
            try:
                wait_mode(port, buffer, MODE_READY, 20.0, started, records,
                          limits, "initial_ready")
                for command in (
                    "SPEED EST WIN", "SPEED LIMIT 5",
                    "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}",
                    "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                          started, records, limits, "start_speed")

                source_rpm = 0.0
                sequence = [*args.speeds, 0.0]
                sequence.extend(-speed for speed in args.speeds)
                sequence.append(0.0)
                for target_rpm in sequence:
                    frames = capture_target(
                        port, buffer, target_rpm, source_rpm,
                        started, records, limits,
                    )
                    if target_rpm != 0.0:
                        result["holds"].append(
                            summarize_hold(target_rpm, frames)
                        )
                    source_rpm = target_rpm

                result["minimum_reliable_speed_rpm"] = minimum_reliable_speeds(
                    result["holds"]
                )
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED 0")
                    try:
                        capture_target(
                            port, buffer, 0.0, 0.0, started, records, limits
                        )
                    except RuntimeError:
                        pass
                    proto.send_command(port, "STOP")
                    final = wait_mode(
                        port, buffer, MODE_READY, 3.0, started, records,
                        limits, "final_ready",
                    )
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    wait_mode(port, buffer, MODE_READY, 2.0, started, records,
                              limits, "defaults_restored")
                    result["final_mode"] = int(round(final[27]))
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        write_csv(telemetry_path, records)
        save_json(output, result)

    if (result["status"] != "passed"
            or result["final_mode"] != MODE_READY
            or not result["defaults_restored"]):
        raise RuntimeError(result["error"] or "sensorless shadow sweep failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "minimum_reliable_speed_rpm": result["minimum_reliable_speed_rpm"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
