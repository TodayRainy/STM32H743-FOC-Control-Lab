#!/usr/bin/env python3
"""Validate low-speed HFI as a shadow observer while ABZ controls the motor."""

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
import stage5_sensorless_shadow_validation as shadow


MODE_SPEED_TORQUE_RUN = 12
MODE_READY = 17
POLE_PAIRS = 10.0
AMPLITUDES_V = (0.5,)
SEED_OFFSETS_DEG = (-45.0, 45.0)
SPEED_SEQUENCE_RPM = (100.0, 250.0, 500.0, 750.0,
                      500.0, 250.0, 100.0,
                      -100.0, -250.0, -500.0, -750.0)
SPEED_RAMP_RPM_PER_S = 500.0


def experiment_plan() -> dict:
    return {
        "schema": "stage5-hfi-shadow-plan-v1",
        "control_angle": "ABZ",
        "observer_control_authority": False,
        "algorithm": "estimated-d-axis square-wave injection, synchronous demodulation, double-angle PLL",
        "injection_frequency_hz": 10000.0,
        "candidate_injection_voltage_v": list(AMPLITUDES_V),
        "standstill_seed_offsets_deg": list(SEED_OFFSETS_DEG),
        "speed_sequence_rpm": list(SPEED_SEQUENCE_RPM),
        "speed_ramp_rpm_per_s": SPEED_RAMP_RPM_PER_S,
        "gates": {
            "standstill_angle_p95_deg": 15.0,
            "running_angle_p95_deg": 20.0,
            "running_speed_rms_error_rpm": "max(50 rpm, 8% target)",
        },
        "safety": {
            "maximum_hfi_dq_current_a": 3.5,
            "maximum_calibration_phase_current_a": 5.5,
            "maximum_hfi_mechanical_speed_rpm": 900.0,
            "maximum_calibration_speed_rpm": 1800.0,
            "bus_voltage_range_v": [18.0, 28.0],
            "minimum_voltage_scale": 0.9,
        },
        "requires_24v": True,
        "motor_may_align_during_encoder_calibration_then_rotate_bidirectionally": True,
        "finally_stop_and_confirm_ready": True,
    }


def circular_error(estimate: float, reference: float) -> float:
    return math.atan2(math.sin(estimate-reference),
                      math.cos(estimate-reference))


def percentile(values: list[float], fraction: float) -> float:
    return shadow.percentile(values, fraction)


def electrical_to_mechanical_rpm(value: float) -> float:
    return value/POLE_PAIRS*60.0/(2.0*math.pi)


def read_checked(port, buffer: bytearray, deadline: float,
                 limits: dict):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    proto.check_frame(frame, limits["max_current"], limits["max_rpm"])
    if not limits["min_vbus"] <= frame[29] <= limits["max_vbus"]:
        raise RuntimeError(f"bus voltage out of range: {frame[29]:.3f} V")
    if (int(round(frame[27])) == MODE_SPEED_TORQUE_RUN
            and frame[26] < limits["min_voltage_scale"]):
        raise RuntimeError(
            f"voltage scale fell below limit: {frame[26]:.3f}"
        )
    if (limits["hfi_active"]
            and math.hypot(frame[6], frame[7])
                > limits["max_hfi_dq_current"]):
        raise RuntimeError(
            f"HFI dq current exceeded limit: "
            f"{math.hypot(frame[6], frame[7]):.3f} A"
        )
    if (limits["hfi_active"]
            and abs(proto.mechanical_rpm(frame))
                > limits["max_hfi_rpm"]):
        raise RuntimeError(
            f"HFI mechanical speed exceeded limit: "
            f"{proto.mechanical_rpm(frame):.1f} rpm"
        )
    return sample


def append_record(records: list[dict], started: float, timestamp: float,
                  phase: str, target_rpm: float,
                  frame: tuple[float, ...]) -> None:
    records.append({
        "elapsed_sec": timestamp-started,
        "phase": phase,
        "target_rpm": target_rpm,
        "speed_ref_rpm": electrical_to_mechanical_rpm(frame[28]),
        "actual_rpm": proto.mechanical_rpm(frame),
        "iq": frame[6],
        "id": frame[7],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "hfi_angle": frame[9],
        "abz_angle": frame[12],
        "hfi_speed": frame[15],
        "abz_speed": frame[18],
        "hfi_angle_error_deg": circular_error(frame[9], frame[12])
                               *180.0/math.pi,
        "voltage_scale": frame[26],
        "motor_mode": int(round(frame[27])),
        "vbus": frame[29],
    })


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
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected} during {phase}")


def capture_for(port, buffer: bytearray, duration: float, phase: str,
                target_rpm: float, started: float, records: list[dict],
                limits: dict) -> list[tuple[float, ...]]:
    deadline = time.monotonic()+duration
    frames = []
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.1), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {phase}")
        append_record(records, started, timestamp, phase, target_rpm, frame)
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no telemetry during {phase}")
    return frames


def summarize_frames(frames: list[tuple[float, ...]], target_rpm: float,
                     angle_gate_deg: float) -> dict:
    angle_errors = [abs(circular_error(frame[9], frame[12]))
                    *180.0/math.pi for frame in frames]
    speed_errors = [electrical_to_mechanical_rpm(frame[15]-frame[18])
                    for frame in frames]
    actual_errors = [proto.mechanical_rpm(frame)-target_rpm
                     for frame in frames]
    speed_gate = 15.0 if target_rpm == 0.0 \
        else max(50.0, 0.08*abs(target_rpm))
    result = {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "angle_p95_deg": percentile(angle_errors, 0.95),
        "angle_max_deg": max(angle_errors),
        "speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in speed_errors
        )/len(speed_errors)),
        "actual_speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in actual_errors
        )/len(actual_errors)),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
    }
    result["passed"] = (
        result["angle_p95_deg"] <= angle_gate_deg
        and result["speed_rms_error_rpm"] <= speed_gate
    )
    return result


def wait_speed(port, buffer: bytearray, target_rpm: float,
               source_rpm: float, started: float, records: list[dict],
               limits: dict) -> list[tuple[float, ...]]:
    proto.send_command(port, f"SPEED {target_rpm:g}")
    deadline = time.monotonic()+abs(target_rpm-source_rpm) \
               /SPEED_RAMP_RPM_PER_S+4.0
    stable_since = None
    stable_frames = []
    recent_actual_rpm = []
    tolerance = max(30.0, 0.04*abs(target_rpm))
    phase = f"speed_{target_rpm:+g}"
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer,
                              min(deadline, time.monotonic()+0.1), limits)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, phase, target_rpm, frame)
        speed_ref_rpm = electrical_to_mechanical_rpm(frame[28])
        recent_actual_rpm.append(proto.mechanical_rpm(frame))
        if len(recent_actual_rpm) > 50:
            del recent_actual_rpm[0]
        mean_actual_rpm = sum(recent_actual_rpm)/len(recent_actual_rpm)
        stable = (
            len(recent_actual_rpm) >= 25
            and abs(speed_ref_rpm-target_rpm) <= 5.0
            and abs(mean_actual_rpm-target_rpm) <= tolerance
        )
        if stable:
            stable_since = stable_since or timestamp
            stable_frames.append(frame)
            if timestamp-stable_since >= 0.5:
                return stable_frames
        else:
            stable_since = None
            stable_frames = []
    raise RuntimeError(f"speed did not settle at {target_rpm:g} rpm")


def summarize_scope(metadata: dict,
                    samples: list[tuple[float, ...]]) -> dict:
    iq = [sample[2] for sample in samples]
    id_ = [sample[3] for sample in samples]
    return {
        "sample_rate_hz": metadata["base_sample_hz"]/metadata["divider"],
        "samples": len(samples),
        "peak_actual_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "iq_peak_to_peak_a": max(iq)-min(iq),
        "id_peak_to_peak_a": max(id_)-min(id_),
        "uq_peak_to_peak_v": max(sample[6] for sample in samples)
                             -min(sample[6] for sample in samples),
        "ud_peak_to_peak_v": max(sample[7] for sample in samples)
                             -min(sample[7] for sample in samples),
        "minimum_voltage_scale": min(sample[11] for sample in samples),
        "max_capture_time_us": metadata["max_capture_cycles"]
                               /metadata["cpu_hz"]*1.0e6,
    }


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
    parser.add_argument("--standstill-only", action="store_true")
    parser.add_argument("--characterize", action="store_true")
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
        f"stage5_hfi_shadow_{stamp}.json"
    )
    telemetry_path = output.with_suffix(".csv")
    scope_path = output.with_name(output.stem+"_scope.csv")
    limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 3.5,
        "max_rpm": 1800.0,
        "max_hfi_rpm": 900.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.9,
        "hfi_active": False,
    }
    result = {
        "schema": "stage5-hfi-shadow-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "standstill": [],
        "selected_injection_voltage_v": None,
        "running": [],
        "scope": None,
        "telemetry_csv": str(telemetry_path.resolve()),
        "scope_csv": str(scope_path.resolve()),
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
                wait_mode(port, buffer, MODE_READY, 10.0, started, records,
                          limits, "initial_ready")
                for command in (
                    "SPEED LIMIT 3", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}", "PI CLEAR",
                    "START SPEED",
                ):
                    proto.send_command(port, command)
                wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                          started, records, limits, "encoder_calibration")
                wait_speed(port, buffer, 0.0, 0.0, started, records, limits)

                for amplitude in AMPLITUDES_V:
                    for offset in SEED_OFFSETS_DEG:
                        proto.send_command(port, "HFI OFF")
                        limits["hfi_active"] = False
                        time.sleep(0.05)
                        proto.send_command(
                            port, f"HFI SHADOW {amplitude:g} {offset:g}"
                        )
                        limits["hfi_active"] = True
                        frames = capture_for(
                            port, buffer, 1.0,
                            f"standstill_{amplitude:g}V_{offset:+g}deg",
                            0.0, started, records, limits,
                        )
                        summary = summarize_frames(frames[len(frames)//2:],
                                                   0.0, 15.0)
                        summary.update({
                            "injection_voltage_v": amplitude,
                            "initial_offset_deg": offset,
                        })
                        result["standstill"].append(summary)

                # Always retain one 40 kHz raw-current capture for carrier
                # phase and saliency analysis, even when the first PLL gains
                # or sign convention do not pass the angle gate.
                proto.send_command(port, "HFI OFF")
                limits["hfi_active"] = False
                proto.send_command(port, "HFI FIXED 0.5 0")
                limits["hfi_active"] = True
                time.sleep(0.05)
                metadata, samples = scope.capture_scope(port, 1, 15.0)
                scope.write_csv(
                    scope_path, samples,
                    metadata["divider"]/metadata["base_sample_hz"],
                )
                result["scope"] = summarize_scope(metadata, samples)
                proto.send_command(port, "HFI OFF")
                limits["hfi_active"] = False
                buffer.clear()
                port.reset_input_buffer()

                passing = [amplitude for amplitude in AMPLITUDES_V if all(
                    item["passed"] for item in result["standstill"]
                    if item["injection_voltage_v"] == amplitude
                )]
                if not passing and not args.characterize:
                    raise RuntimeError("no HFI injection candidate passed standstill")
                selected = min(passing) if passing else AMPLITUDES_V[0]
                result["selected_injection_voltage_v"] = selected

                if not args.standstill_only:
                    proto.send_command(port, f"HFI SHADOW {selected:g} 0")
                    limits["hfi_active"] = True
                    time.sleep(0.2)

                    source_rpm = 0.0
                    for target_rpm in SPEED_SEQUENCE_RPM:
                        if args.characterize:
                            proto.send_command(port, f"SPEED {target_rpm:g}")
                            duration = abs(target_rpm-source_rpm) \
                                       /SPEED_RAMP_RPM_PER_S+2.0
                            frames = capture_for(
                                port, buffer, duration,
                                f"speed_{target_rpm:+g}", target_rpm,
                                started, records, limits,
                            )
                            frames = frames[len(frames)//2:]
                        else:
                            frames = wait_speed(
                                port, buffer, target_rpm, source_rpm,
                                started, records, limits,
                            )
                        summary = summarize_frames(frames, target_rpm, 20.0)
                        result["running"].append(summary)
                        if not summary["passed"] and not args.characterize:
                            raise RuntimeError(
                                f"HFI metrics failed at {target_rpm:g} rpm"
                            )
                        source_rpm = target_rpm
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "HFI OFF")
                    limits["hfi_active"] = False
                    proto.send_command(port, "SPEED 0")
                    time.sleep(0.5)
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
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if (result["status"] != "passed"
            or result["final_mode"] != MODE_READY
            or not result["defaults_restored"]):
        raise RuntimeError(result["error"] or "HFI shadow validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "selected_injection_voltage_v": result["selected_injection_voltage_v"],
        "running_points": len(result["running"]),
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
