#!/usr/bin/env python3
"""Validate independent HFI axis acquisition and IPD against ABZ truth."""

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
import stage5_hfi_shadow_validation as hfi


IPD_CURRENT_A = 1.0
REPOSITION_SPEED_RPM = 100.0


def record(rows: list[dict], started: float, timestamp: float,
           visit: int, phase: str, frame: tuple[float, ...]) -> None:
    rows.append({
        "elapsed_sec": timestamp-started,
        "visit": visit,
        "phase": phase,
        "motor_mode": int(round(frame[27])),
        "hfi_angle_deg": math.degrees(frame[9]),
        "abz_angle_deg": math.degrees(frame[12]),
        "angle_error_deg": math.degrees(
            hfi.circular_error(frame[9], frame[12])
        ),
        "hfi_rpm": hfi.electrical_to_mechanical_rpm(frame[15]),
        "abz_rpm": hfi.electrical_to_mechanical_rpm(frame[18]),
        "iq_a": frame[6],
        "id_a": frame[7],
        "iq_ref_a": frame[22],
        "id_ref_a": frame[23],
        "voltage_scale": frame[26],
        "vbus_v": frame[29],
    })


def capture(port, buffer: bytearray, duration: float, visit: int,
            phase: str, started: float, rows: list[dict]) \
            -> list[tuple[float, ...]]:
    frames = []
    deadline = time.monotonic()+duration
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic()+0.1)
        )
        if sample is None:
            continue
        timestamp, frame = sample
        mode = int(round(frame[27]))
        if mode in (proto.MODE_TORQUE_RUN, proto.MODE_SPEED_TORQUE_RUN,
                    proto.MODE_POSITION_SPEED_TORQUE_RUN):
            if math.hypot(frame[6], frame[7]) > 5.5:
                raise RuntimeError("dq current exceeded 5.5 A")
        elif max(abs(frame[3]-12.0), abs(frame[4]-12.0),
                 abs(frame[5]-12.0)) > 5.5:
            raise RuntimeError("phase current exceeded 5.5 A")
        if not 18.0 <= frame[29] <= 28.0:
            raise RuntimeError(f"bus voltage out of range: {frame[29]:.3f} V")
        if (int(round(frame[27])) == hfi.MODE_SPEED_TORQUE_RUN
                and frame[26] < 0.9):
            raise RuntimeError(
                f"voltage scale fell below limit: {frame[26]:.3f}"
            )
        record(rows, started, timestamp, visit, phase, frame)
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no telemetry during {phase}")
    return frames


def wait_mode(port, buffer: bytearray, expected: int, timeout: float,
              visit: int, phase: str, started: float,
              rows: list[dict]) -> tuple[float, ...]:
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        frames = capture(
            port, buffer, min(0.1, deadline-time.monotonic()),
            visit, phase, started, rows,
        )
        for frame in frames:
            if int(round(frame[27])) == expected:
                return frame
    raise RuntimeError(f"timeout waiting for mode {expected} during {phase}")


def summarize_visit(frames: list[tuple[float, ...]], visit: int) -> dict:
    running = [frame for frame in frames
               if int(round(frame[27])) == hfi.MODE_SPEED_TORQUE_RUN]
    if len(running) < 50:
        raise RuntimeError(f"visit {visit}: insufficient running telemetry")
    tail = running[-max(30, len(running)//4):]
    angle_errors = sorted(abs(math.degrees(
        hfi.circular_error(frame[9], frame[12])
    )) for frame in tail)
    unwrapped_abz = [running[0][12]]
    for previous, current in zip(running, running[1:]):
        unwrapped_abz.append(
            unwrapped_abz[-1]+hfi.circular_error(current[12], previous[12])
        )
    result = {
        "visit": visit,
        "samples": len(running),
        "saw_positive_ipd_current": any(
            frame[23] > 0.7*IPD_CURRENT_A for frame in running
        ),
        "saw_negative_ipd_current": any(
            frame[23] < -0.7*IPD_CURRENT_A for frame in running
        ),
        "initial_abz_electrical_angle_deg": math.degrees(running[0][12]),
        "final_angle_error_mean_deg": sum(
            math.degrees(hfi.circular_error(frame[9], frame[12]))
            for frame in tail
        )/len(tail),
        "final_angle_error_p95_deg": hfi.percentile(angle_errors, 0.95),
        "final_hfi_speed_rms_rpm": math.sqrt(sum(
            hfi.electrical_to_mechanical_rpm(frame[15])**2
            for frame in tail
        )/len(tail)),
        "final_abz_speed_rms_rpm": math.sqrt(sum(
            hfi.electrical_to_mechanical_rpm(frame[18])**2
            for frame in tail
        )/len(tail)),
        "peak_dq_current_a": max(math.hypot(frame[6], frame[7])
                                 for frame in running),
        "peak_abs_speed_rpm": max(abs(
            hfi.electrical_to_mechanical_rpm(frame[18])
        ) for frame in running),
        "rotor_excursion_mechanical_deg": math.degrees(
            max(unwrapped_abz)-min(unwrapped_abz)
        )/hfi.POLE_PAIRS,
        "minimum_voltage_scale": min(frame[26] for frame in running),
    }
    result["passed"] = (
        result["saw_positive_ipd_current"]
        and result["saw_negative_ipd_current"]
        and result["final_angle_error_p95_deg"] <= 20.0
        and result["peak_dq_current_a"] <= 2.5
        and result["final_abz_speed_rms_rpm"] <= 10.0
        and result["rotor_excursion_mechanical_deg"] <= 10.0
        and result["minimum_voltage_scale"] >= 0.9
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
    parser.add_argument("--visits", type=int, default=4)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.visits <= 8:
        parser.error("visits must be in [1,8]")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_ipd_validation_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    result = {
        "schema": "stage5-hfi-ipd-validation-result-v1",
        "status": "running",
        "error": None,
        "method": "automatic four-axis acquisition, +/-1.0 A saturation IPD, estimated-d-axis tracking",
        "visits": [],
        "scopes": [],
        "telemetry_csv": str(telemetry.resolve()),
        "final_mode": None,
        "defaults_restored": False,
    }
    rows: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                wait_mode(port, buffer, hfi.MODE_READY, 10.0, -1,
                          "initial_ready", started, rows)

                # Calibrate ABZ once so it remains an independent truth source.
                for command in (
                    "SPEED LIMIT 2", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE OFF", "VOLT LIMIT 1", "TRACK STEP", "PI CLEAR",
                    "START SPEED",
                ):
                    proto.send_command(port, command)
                wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0, -1,
                          "abz_calibration", started, rows)
                proto.send_command(port, "SPEED 0")
                capture(port, buffer, 0.4, -1, "abz_zero", started, rows)
                proto.send_command(port, "STOP")
                wait_mode(port, buffer, hfi.MODE_READY, 3.0, -1,
                          "abz_ready", started, rows)

                for visit in range(args.visits):
                    scope.arm_scope(port, 10)
                    proto.send_command(port, "START HFI")
                    frames = capture(
                        port, buffer, 1.15, visit, "ipd", started, rows
                    )
                    metadata, samples = scope.dump_scope(port, 15.0)
                    scope_path = output.with_name(
                        f"{output.stem}_visit_{visit:02d}_scope.csv"
                    )
                    scope.write_csv(
                        scope_path, samples,
                        metadata["divider"]/metadata["base_sample_hz"],
                    )
                    result["scopes"].append({
                        "visit": visit,
                        "path": str(scope_path.resolve()),
                        "samples": metadata["sample_count"],
                        "sample_rate_hz": metadata["base_sample_hz"]
                                          /metadata["divider"],
                    })
                    buffer.clear()
                    port.reset_input_buffer()
                    point = summarize_visit(frames, visit)
                    result["visits"].append(point)
                    proto.send_command(port, "STOP")
                    wait_mode(port, buffer, hfi.MODE_READY, 3.0, visit,
                              "ipd_ready", started, rows)

                    if visit+1 < args.visits:
                        direction = 1.0 if (visit & 1) == 0 else -1.0
                        proto.send_command(port, "START SPEED")
                        wait_mode(
                            port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 3.0,
                            visit, "reposition_start", started, rows,
                        )
                        proto.send_command(
                            port, f"SPEED {direction*REPOSITION_SPEED_RPM:g}"
                        )
                        capture(port, buffer, 0.12+0.04*visit, visit,
                                "reposition", started, rows)
                        proto.send_command(port, "SPEED 0")
                        capture(port, buffer, 0.35, visit,
                                "reposition_zero", started, rows)
                        proto.send_command(port, "STOP")
                        wait_mode(port, buffer, hfi.MODE_READY, 3.0, visit,
                                  "reposition_ready", started, rows)

                result["status"] = "passed" if all(
                    point["passed"] for point in result["visits"]
                ) else "characterized"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED 0")
                    proto.send_command(port, "STOP")
                    final = wait_mode(port, buffer, hfi.MODE_READY, 3.0, -1,
                                      "final_ready", started, rows)
                    result["final_mode"] = int(round(final[27]))
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        write_csv(telemetry, rows)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if (result["status"] == "failed"
            or result["final_mode"] != hfi.MODE_READY):
        raise RuntimeError(result["error"] or "HFI IPD validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "visits": result["visits"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
