#!/usr/bin/env python3
"""Validate HFI as the low-speed control angle with ABZ as reference only."""

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
import stage5_hfi_shadow_validation as hfi


SPEED_SEQUENCE_RPM = (100.0, 250.0, 500.0, 250.0, 100.0,
                      -100.0, -250.0, -500.0, -250.0, -100.0)
SPEED_RAMP_RPM_PER_S = 500.0
DEFAULT_SPEED_KP = 0.00536533785
DEFAULT_SPEED_KI = 0.258523708


def abz_rpm(frame: tuple[float, ...]) -> float:
    return hfi.electrical_to_mechanical_rpm(frame[18])


def read_control_frame(port, buffer: bytearray, deadline: float):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    if int(round(frame[27])) != hfi.MODE_SPEED_TORQUE_RUN:
        raise RuntimeError("motor left HFI speed-control mode")
    current = math.hypot(frame[6], frame[7])
    if current > 4.5:
        raise RuntimeError(
            f"HFI control current {current:.3f} A, "
            f"angle error {math.degrees(hfi.circular_error(frame[9], frame[12])):.2f} deg, "
            f"HFI/ABZ speed {hfi.electrical_to_mechanical_rpm(frame[15]):.1f}/"
            f"{abz_rpm(frame):.1f} rpm"
        )
    if not 18.0 <= frame[29] <= 28.0:
        raise RuntimeError(f"bus voltage out of range: {frame[29]:.3f} V")
    if frame[26] < 0.9:
        raise RuntimeError(f"voltage scale fell below 0.9: {frame[26]:.3f}")
    if abs(abz_rpm(frame)) > 650.0:
        raise RuntimeError("ABZ reference speed exceeded 650 rpm")
    if abs(hfi.electrical_to_mechanical_rpm(frame[15])) > 700.0:
        raise RuntimeError("HFI estimated speed exceeded 700 rpm")
    return sample


def capture_control(port, buffer: bytearray, duration: float,
                    phase: str, target_rpm: float,
                    started: float, records: list[dict]) -> list[tuple[float, ...]]:
    deadline = time.monotonic()+duration
    frames = []
    while time.monotonic() < deadline:
        sample = read_control_frame(
            port, buffer, min(deadline, time.monotonic()+0.1)
        )
        if sample is None:
            continue
        timestamp, frame = sample
        records.append({
            "elapsed_sec": timestamp-started,
            "phase": phase,
            "target_rpm": target_rpm,
            "abz_rpm": abz_rpm(frame),
            "hfi_rpm": hfi.electrical_to_mechanical_rpm(frame[15]),
            "control_rpm": hfi.electrical_to_mechanical_rpm(frame[25]),
            "angle_error_deg": math.degrees(
                hfi.circular_error(frame[9], frame[12])
            ),
            "iq": frame[6],
            "id": frame[7],
            "voltage_scale": frame[26],
            "vbus": frame[29],
        })
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no telemetry during {phase}")
    return frames


def summarize(frames: list[tuple[float, ...]], target_rpm: float) -> dict:
    angle = sorted(abs(math.degrees(hfi.circular_error(frame[9], frame[12])))
                   for frame in frames)
    hfi_speed_error = [hfi.electrical_to_mechanical_rpm(frame[15])-target_rpm
                       for frame in frames]
    abz_speed_error = [abz_rpm(frame)-target_rpm for frame in frames]
    control_source_error = [abs(frame[25]-frame[15]) for frame in frames]
    result = {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "angle_p95_deg": hfi.percentile(angle, 0.95),
        "hfi_speed_rms_error_rpm": math.sqrt(sum(value*value
            for value in hfi_speed_error)/len(frames)),
        "abz_speed_rms_error_rpm": math.sqrt(sum(value*value
            for value in abz_speed_error)/len(frames)),
        "control_source_max_error_rad_s": max(control_source_error),
        "peak_dq_current_a": max(math.hypot(frame[6], frame[7])
                                 for frame in frames),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
    }
    angle_gate = 15.0 if target_rpm == 0.0 else 22.0
    speed_gate = max(50.0, 0.10*abs(target_rpm))
    result["passed"] = (
        result["angle_p95_deg"] <= angle_gate
        and result["hfi_speed_rms_error_rpm"] <= speed_gate
        and result["abz_speed_rms_error_rpm"] <= speed_gate
        and result["control_source_max_error_rad_s"] <= 1.0e-5
        and result["peak_dq_current_a"] <= 2.0
    )
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--standstill-only", action="store_true")
    parser.add_argument("--speeds", type=float, nargs="*")
    parser.add_argument("--speed-pi-scale", type=float, default=1.0)
    parser.add_argument("--voltage", type=float, default=0.5)
    args = parser.parse_args()

    speed_sequence = tuple(args.speeds) if args.speeds is not None \
                     else SPEED_SEQUENCE_RPM
    speed_kp = DEFAULT_SPEED_KP*args.speed_pi_scale
    speed_ki = DEFAULT_SPEED_KI*args.speed_pi_scale

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_control_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    result = {
        "schema": "stage5-hfi-control-result-v1",
        "status": "running",
        "error": None,
        "control_angle": "HFI",
        "reference_only": "ABZ",
        "injection_voltage_v": args.voltage,
        "speed_sequence_rpm": list(speed_sequence),
        "speed_pi": {"kp": speed_kp, "ki": speed_ki,
                     "scale": args.speed_pi_scale},
        "points": [],
        "telemetry_csv": str(telemetry.resolve()),
        "final_mode": None,
    }
    shadow_limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 3.5,
        "max_rpm": 1800.0,
        "max_hfi_rpm": 900.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.9,
        "hfi_active": False,
    }
    buffer = bytearray()
    shadow_records: list[dict] = []
    records: list[dict] = []
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                hfi.wait_mode(port, buffer, hfi.MODE_READY, 10.0,
                              started, shadow_records, shadow_limits,
                              "initial_ready")
                for command in (
                    "SPEED LIMIT 3", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1", "TRACK RAMP 500",
                    f"PI SPEED {speed_kp:.9g} {speed_ki:.9g}",
                    "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, shadow_records, shadow_limits,
                              "encoder_calibration")
                hfi.wait_speed(port, buffer, 0.0, 0.0,
                               started, shadow_records, shadow_limits)
                proto.send_command(port, f"HFI SHADOW {args.voltage:g} 0")
                shadow_limits["hfi_active"] = True
                shadow = hfi.capture_for(
                    port, buffer, 0.8, "hfi_calibration", 0.0,
                    started, shadow_records, shadow_limits,
                )
                shadow_summary = hfi.summarize_frames(
                    shadow[len(shadow)//2:], 0.0, 15.0
                )
                if not shadow_summary["passed"]:
                    raise RuntimeError("HFI calibration shadow gate failed")

                proto.send_command(port, "HFI CONTROL")
                buffer.clear()
                port.reset_input_buffer()
                frames = capture_control(
                    port, buffer, 0.25, "control_0", 0.0, started, records
                )
                point = summarize(frames[len(frames)//2:], 0.0)
                result["points"].append(point)
                if not point["passed"]:
                    raise RuntimeError("HFI standstill control gate failed")

                if not args.standstill_only:
                    source_rpm = 0.0
                    for target_rpm in speed_sequence:
                        proto.send_command(port, f"SPEED {target_rpm:g}")
                        duration = abs(target_rpm-source_rpm) \
                                   /SPEED_RAMP_RPM_PER_S+1.5
                        frames = capture_control(
                            port, buffer, duration,
                            f"control_{target_rpm:+g}", target_rpm,
                            started, records,
                        )
                        point = summarize(frames[len(frames)//2:], target_rpm)
                        result["points"].append(point)
                        if not point["passed"]:
                            raise RuntimeError(
                                f"HFI control gate failed at {target_rpm:g} rpm"
                            )
                        source_rpm = target_rpm
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "STOP")
                    shadow_limits["hfi_active"] = False
                    final = hfi.wait_mode(
                        port, buffer, hfi.MODE_READY, 3.0,
                        started, shadow_records, shadow_limits, "final_ready",
                    )
                    result["final_mode"] = int(round(final[27]))
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH",
                        f"PI SPEED {DEFAULT_SPEED_KP:.9g} "
                        f"{DEFAULT_SPEED_KI:.9g}",
                        "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        write_csv(telemetry, records)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] != "passed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "HFI control validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "points": len(result["points"]),
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
