#!/usr/bin/env python3
"""Measure the selected speed PI closed-loop frequency response."""

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
from scope_current_step import wait_mode
from stage2_current_frequency_response import fit_phasor, magnitude_db, phase_deg
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    SCOPE_DIVIDER,
    SCOPE_DURATION_SEC,
    check_telemetry,
    open_serial_with_retry,
    stop_and_confirm_ready,
    supervised_wait,
)
from stage2_speed_autotune import DEFAULT_SPEC, load_spec


DEFAULT_FREQUENCIES_HZ = (2.0, 5.0, 10.0, 20.0, 40.0, 80.0, 120.0, 160.0)
DEFAULT_KP = 0.00410918168695387
DEFAULT_KI = 0.0140681012016855


def interpolate_log_crossing(first: dict, second: dict, target_db: float) -> float:
    y0 = first["closed_loop_magnitude_db"]
    y1 = second["closed_loop_magnitude_db"]
    fraction = 0.0 if y1 == y0 else (target_db - y0) / (y1 - y0)
    fraction = max(0.0, min(1.0, fraction))
    return math.exp(
        math.log(first["frequency_hz"])
        + fraction * math.log(second["frequency_hz"] / first["frequency_hz"])
    )


def analyze_capture(samples, frequency_hz: float, sample_period: float,
                    pole_pairs: int) -> dict:
    measured, _ = fit_phasor(
        [sample[9] for sample in samples], frequency_hz, sample_period,
    )
    reference, _ = fit_phasor(
        [sample[10] for sample in samples], frequency_hz, sample_period,
    )
    iq_reference, _ = fit_phasor(
        [sample[4] for sample in samples], frequency_hz, sample_period,
    )
    if abs(reference) < 1.0:
        raise RuntimeError("recorded speed sine reference is too small")
    closed_loop = measured / reference
    rpm_scale = 60.0 / (2.0 * math.pi * pole_pairs)
    return {
        "frequency_hz": frequency_hz,
        "reference_amplitude_rpm": abs(reference) * rpm_scale,
        "measured_amplitude_rpm": abs(measured) * rpm_scale,
        "closed_loop_magnitude_db": magnitude_db(closed_loop),
        "closed_loop_phase_deg": phase_deg(closed_loop),
        "iq_reference_amplitude_a": abs(iq_reference),
        "raw_peak_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "raw_peak_speed_rpm": max(abs(sample[9]) * rpm_scale for sample in samples),
    }


def summarize(points: list[dict], bandwidth_guard_hz: float,
              maximum_peak_db: float) -> dict:
    ordered = sorted(points, key=lambda point: point["frequency_hz"])
    plateau_db = sum(
        point["closed_loop_magnitude_db"] for point in ordered[:2]
    ) / min(2, len(ordered))
    target_db = plateau_db - 3.0103
    bandwidth_hz = None
    for first, second in zip(ordered, ordered[1:]):
        if ((first["closed_loop_magnitude_db"] - target_db)
                * (second["closed_loop_magnitude_db"] - target_db) <= 0.0):
            bandwidth_hz = interpolate_log_crossing(first, second, target_db)
            break
    relative_peak_db = max(
        point["closed_loop_magnitude_db"] - plateau_db for point in ordered
    )
    return {
        "low_frequency_gain_db": plateau_db,
        "minus_3db_target_db": target_db,
        "closed_loop_bandwidth_hz": bandwidth_hz,
        "maximum_relative_peak_db": relative_peak_db,
        "passes_bandwidth_guard": (
            bandwidth_hz is not None and bandwidth_hz <= bandwidth_guard_hz
        ),
        "passes_peak_guard": relative_peak_db <= maximum_peak_db,
        "qualified": (
            bandwidth_hz is not None
            and bandwidth_hz <= bandwidth_guard_hz
            and relative_peak_db <= maximum_peak_db
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--ki", type=float, default=DEFAULT_KI)
    parser.add_argument("--amplitude-rpm", type=float, default=300.0)
    parser.add_argument("--frequencies", type=float, nargs="+",
                        default=DEFAULT_FREQUENCIES_HZ)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / f"stage2_speed_frf_{stamp}.json"
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    result = {
        "schema": "stage2-speed-frequency-response-v1",
        "spec": str(args.spec),
        "kp": args.kp,
        "ki": args.ki,
        "amplitude_rpm": args.amplitude_rpm,
        "frequencies_hz": list(args.frequencies),
        "records": [],
        "summary": None,
        "error": None,
        "final_mode": None,
    }

    with open_serial_with_retry(args.port, args.baudrate) as setup_port:
        setup_port.reset_input_buffer()
        wait_mode(setup_port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        proto.send_command(
            setup_port,
            f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
        )
        proto.send_command(setup_port, "FF OFF")
        proto.send_command(setup_port, "TRACK STEP")
        proto.send_command(setup_port, f"PI SPEED {args.kp:.9g} {args.ki:.9g}")
        proto.send_command(setup_port, "PI CLEAR")
        proto.send_command(setup_port, "START SPEED")

    time.sleep(1.0)
    buffer.clear()
    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        try:
            running = wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0,
            )
            check_telemetry(running, spec)
            for frequency_hz in args.frequencies:
                proto.send_command(
                    port, f"SPEED SINE {args.amplitude_rpm:g} {frequency_hz:g}",
                )
                supervised_wait(
                    port, buffer, max(0.5, 3.0 / frequency_hz),
                    spec, MODE_SPEED_TORQUE_RUN,
                )
                scope.arm_scope(port, SCOPE_DIVIDER)
                supervised_wait(
                    port, buffer, SCOPE_DURATION_SEC + 0.08,
                    spec, MODE_SPEED_TORQUE_RUN,
                )
                capture, samples = scope.dump_scope(port, 25.0)
                sample_period = capture["divider"] / capture["base_sample_hz"]
                path = scope_dir / f"speed_sine_{frequency_hz:g}hz.csv"
                scope.write_csv(path, samples, sample_period)
                record = analyze_capture(
                    samples, frequency_hz, sample_period,
                    int(spec["control"]["pole_pairs"]),
                )
                record["scope_csv"] = str(path.resolve())
                result["records"].append(record)
                proto.send_command(port, "SPEED SINE OFF")
                supervised_wait(port, buffer, 0.3, spec, MODE_SPEED_TORQUE_RUN)
            guard = spec["leader_bandwidth_guard"]
            result["summary"] = summarize(
                result["records"], guard["maximum_bandwidth_hz"],
                guard["maximum_closed_loop_peak_db"],
            )
        except (RuntimeError, serial.SerialException) as error:
            result["error"] = str(error)
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer,
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if result["error"] is not None:
        raise RuntimeError(result["error"])
    print(json.dumps({
        "result": str(output.resolve()),
        "summary": result["summary"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
