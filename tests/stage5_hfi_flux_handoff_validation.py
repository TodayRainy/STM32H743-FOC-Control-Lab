#!/usr/bin/env python3
"""Validate the bidirectional HFI/Active-Flux overlap in one continuous run."""

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


SPEED_RAMP_RPM_PER_S = 300.0
SPEED_SEQUENCE_RPM = (
    350.0, 500.0, 600.0, 750.0, 500.0, 350.0, 0.0,
    -350.0, -500.0, -600.0, -750.0, -500.0, -350.0, 0.0,
)


def parse_speed_sequence(text: str) -> tuple[float, ...]:
    values = tuple(float(value.strip()) for value in text.split(",")
                   if value.strip())
    if not values:
        raise argparse.ArgumentTypeError("speed sequence must not be empty")
    return values


def source_name(frame: tuple[float, ...]) -> str:
    return "ACTIVE_FLUX" if int(round(frame[35])) == 1 else "HFI"


def read_checked(port, buffer: bytearray, deadline: float,
                 hfi_overspeed: list[int]):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    proto.check_frame(frame, 6.5, 1.0e9)
    if not 18.0 <= frame[29] <= 28.0:
        raise RuntimeError(f"bus voltage out of range: {frame[29]:.3f} V")
    if int(round(frame[27])) == hfi.MODE_SPEED_TORQUE_RUN:
        if frame[26] < 0.90:
            raise RuntimeError(
                f"voltage scale fell below 0.90: {frame[26]:.3f}"
            )
        actual_rpm = hfi.electrical_to_mechanical_rpm(frame[18])
        hfi_overspeed[1] = hfi_overspeed[1]+1 \
            if abs(actual_rpm) > 900.0 else 0
        if hfi_overspeed[1] >= 10:
            raise RuntimeError(
                f"mechanical speed remained above limit: {actual_rpm:.1f} rpm"
            )
        hfi_overspeed[0] = hfi_overspeed[0]+1 \
            if source_name(frame) == "HFI" and abs(actual_rpm) > 720.0 else 0
        if hfi_overspeed[0] >= 10:
            raise RuntimeError(
                f"HFI remained above its validity ceiling: {actual_rpm:.1f} rpm"
            )
    return sample


def append_record(records: list[dict], started: float, timestamp: float,
                  phase: str, target_rpm: float,
                  frame: tuple[float, ...]) -> None:
    source = source_name(frame)
    selected_angle = frame[11] if source == "ACTIVE_FLUX" else frame[9]
    records.append({
        "elapsed_sec": timestamp-started,
        "phase": phase,
        "target_rpm": target_rpm,
        "source": source,
        "actual_rpm": hfi.electrical_to_mechanical_rpm(frame[18]),
        "control_rpm": hfi.electrical_to_mechanical_rpm(frame[25]),
        "hfi_rpm": hfi.electrical_to_mechanical_rpm(frame[15]),
        "flux_rpm": hfi.electrical_to_mechanical_rpm(frame[17]),
        "selected_angle_error_deg": math.degrees(
            hfi.circular_error(selected_angle, frame[12])
        ),
        "hfi_angle_error_deg": math.degrees(
            hfi.circular_error(frame[9], frame[12])
        ),
        "flux_angle_error_deg": math.degrees(
            hfi.circular_error(frame[11], frame[12])
        ),
        "iq": frame[6],
        "id": frame[7],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "voltage_scale": frame[26],
        "feedforward_mode": int(round(frame[35])),
        "motor_mode": int(round(frame[27])),
        "vbus": frame[29],
    })


def capture_for(port, buffer: bytearray, duration: float, phase: str,
                target_rpm: float, started: float, records: list[dict],
                hfi_overspeed: list[int]) -> list[tuple[float, ...]]:
    deadline = time.monotonic()+duration
    frames = []
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic()+0.1), hfi_overspeed
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != hfi.MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {phase}")
        append_record(records, started, timestamp, phase, target_rpm, frame)
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no telemetry during {phase}")
    return frames


def wait_mode(port, buffer: bytearray, expected: int, timeout: float,
              phase: str, started: float, records: list[dict],
              hfi_overspeed: list[int]) -> tuple[float, ...]:
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic()+0.1), hfi_overspeed
        )
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, phase, 0.0, frame)
        if int(round(frame[27])) == expected:
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected} during {phase}")


def summarize(frames: list[tuple[float, ...]], target_rpm: float) -> dict:
    tail = frames[-max(100, len(frames)//3):]
    sources = [source_name(frame) for frame in tail]
    source = max(set(sources), key=sources.count)
    selected_index = 11 if source == "ACTIVE_FLUX" else 9
    angle_errors = [abs(math.degrees(
        hfi.circular_error(frame[selected_index], frame[12])
    )) for frame in tail]
    actual = [hfi.electrical_to_mechanical_rpm(frame[18]) for frame in tail]
    control = [hfi.electrical_to_mechanical_rpm(frame[25]) for frame in tail]
    result = {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "source": source,
        "angle_p95_deg": hfi.percentile(angle_errors, 0.95),
        "tail_mean_actual_rpm": sum(actual)/len(actual),
        "actual_speed_rms_error_rpm": math.sqrt(sum(
            (value-target_rpm)**2 for value in actual
        )/len(actual)),
        "control_to_abz_rms_rpm": math.sqrt(sum(
            (selected-reference)**2
            for selected, reference in zip(control, actual)
        )/len(actual)),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
    }
    expected_source = "ACTIVE_FLUX" if abs(target_rpm) >= 600.0 else "HFI"
    angle_gate = 25.0 if source == "ACTIVE_FLUX" else 35.0
    result["passed"] = (
        source == expected_source
        and result["angle_p95_deg"] <= angle_gate
        and abs(result["tail_mean_actual_rpm"]-target_rpm) <= 25.0
        and result["actual_speed_rms_error_rpm"] <= 55.0
        and result["control_to_abz_rms_rpm"] <= 55.0
        and result["minimum_voltage_scale"] >= 0.90
    )
    return result


def write_csv(path: Path, records: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--speeds", type=parse_speed_sequence,
                        default=SPEED_SEQUENCE_RPM)
    parser.add_argument("--speed-limit", type=float, default=2.0)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_flux_handoff_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    result = {
        "schema": "stage5-hfi-flux-handoff-result-v1",
        "status": "running",
        "error": None,
        "speed_sequence_rpm": list(args.speeds),
        "speed_limit_a": args.speed_limit,
        "speed_ramp_rpm_per_s": SPEED_RAMP_RPM_PER_S,
        "running": [],
        "source_transitions": [],
        "telemetry_csv": str(telemetry.resolve()),
        "final_mode": None,
    }
    records: list[dict] = []
    buffer = bytearray()
    hfi_overspeed = [0, 0]
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                wait_mode(port, buffer, hfi.MODE_READY, 10.0,
                          "initial_ready", started, records, hfi_overspeed)

                # ABZ remains an independent truth source only after its
                # normal index calibration; it is not used by HFI control.
                for command in (
                    f"SPEED LIMIT {args.speed_limit:g}",
                    "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE OFF", "VOLT LIMIT 1", "TRACK STEP",
                    "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                          "abz_calibration", started, records, hfi_overspeed)
                capture_for(port, buffer, 0.4, "abz_zero", 0.0,
                            started, records, hfi_overspeed)
                proto.send_command(port, "STOP")
                wait_mode(port, buffer, hfi.MODE_READY, 3.0,
                          "abz_ready", started, records, hfi_overspeed)

                for command in (
                    "FW MODE HYBRID", "VOLT LIMIT 1", "TRACK RAMP 300",
                    "PI CLEAR", "START HFI",
                ):
                    proto.send_command(port, command)
                wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 10.0,
                          "hfi_start", started, records, hfi_overspeed)
                capture_for(port, buffer, 1.15, "ipd_and_track", 0.0,
                            started, records, hfi_overspeed)
                capture_for(port, buffer, 0.25, "control_zero", 0.0,
                            started, records, hfi_overspeed)

                source_rpm = 0.0
                for target_rpm in args.speeds:
                    proto.send_command(port, f"SPEED {target_rpm:g}")
                    duration = abs(target_rpm-source_rpm) \
                               /SPEED_RAMP_RPM_PER_S+1.2
                    frames = capture_for(
                        port, buffer, duration,
                        f"closed_loop_{target_rpm:+g}", target_rpm,
                        started, records, hfi_overspeed,
                    )
                    summary = summarize(frames, target_rpm)
                    result["running"].append(summary)
                    print(json.dumps({
                        "phase": f"closed_loop_{target_rpm:+g}",
                        "summary": summary,
                    }), flush=True)
                    if not summary["passed"]:
                        raise RuntimeError(
                            f"handoff matrix failed at {target_rpm:g} rpm"
                        )
                    source_rpm = target_rpm

                previous = None
                for record in records:
                    if not record["phase"].startswith("closed_loop_"):
                        continue
                    if record["source"] != previous:
                        if previous is not None:
                            result["source_transitions"].append({
                                "elapsed_sec": record["elapsed_sec"],
                                "from": previous,
                                "to": record["source"],
                                "actual_rpm": record["actual_rpm"],
                                "control_rpm": record["control_rpm"],
                            })
                        previous = record["source"]
                expected_sources = [
                    "ACTIVE_FLUX" if abs(speed) >= 600.0 else "HFI"
                    for speed in args.speeds
                ]
                expected_transitions = sum(
                    current != previous
                    for previous, current in zip(
                        expected_sources, expected_sources[1:]
                    )
                )
                if len(result["source_transitions"]) != expected_transitions:
                    raise RuntimeError(
                        "unexpected HFI/Active-Flux source transition count: "
                        f"expected {expected_transitions}, got "
                        f"{len(result['source_transitions'])}"
                    )
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED 0")
                    proto.send_command(port, "STOP")
                    time.sleep(0.05)
                    port.reset_input_buffer()
                    buffer.clear()
                    final = wait_mode(
                        port, buffer, hfi.MODE_READY, 3.0,
                        "final_ready", started, records, hfi_overspeed,
                    )
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 4",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    wait_mode(
                        port, buffer, hfi.MODE_READY, 2.0,
                        "defaults_restored", started, records, hfi_overspeed,
                    )
                    result["final_mode"] = int(round(final[27]))
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        if records:
            write_csv(telemetry, records)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] != "passed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "handoff validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "transitions": result["source_transitions"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
