#!/usr/bin/env python3
"""Measure the current speed-loop baseline with the frozen WIN estimator."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import wait_mode
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    open_serial_with_retry,
    stop_and_confirm_ready,
    supervised_wait,
)
from stage2_speed_frequency_response import analyze_capture, interpolate_log_crossing


DEFAULT_KP = 0.005365337845003614
DEFAULT_KI = 0.2585237075491248
SPEED_BIASES_RPM = (1500.0, -1500.0)
SINE_AMPLITUDE_RPM = 100.0
FREQUENCIES_HZ = (
    1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0, 30.0, 40.0, 60.0,
)


def scope_divider(frequency_hz: float) -> int:
    if frequency_hz <= 4.0:
        return 100
    return 10


def plan() -> dict:
    return {
        "schema": "stage2-speed-baseline-plan-v2",
        "speed_estimator": "win_1ms_fc150",
        "kp": DEFAULT_KP,
        "ki": DEFAULT_KI,
        "speed_biases_rpm": SPEED_BIASES_RPM,
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "frequencies_hz": FREQUENCIES_HZ,
        "scope_dividers": {
            str(frequency): scope_divider(frequency)
            for frequency in FREQUENCIES_HZ
        },
        "scope_capture_count": len(SPEED_BIASES_RPM) * len(FREQUENCIES_HZ),
        "maximum_commanded_speed_rpm": (
            max(abs(value) for value in SPEED_BIASES_RPM) + SINE_AMPLITUDE_RPM
        ),
        "requires_24v": True,
        "motor_will_rotate": True,
        "finally_stop_and_confirm_ready": True,
    }


def add_control_cost(record: dict, samples: list, pole_pairs: int) -> None:
    rpm_scale = 60.0 / (2.0 * math.pi * pole_pairs)
    record.update({
        "mean_measured_speed_rpm": (
            sum(sample[9] for sample in samples) / len(samples) * rpm_scale
        ),
        "mean_reference_speed_rpm": (
            sum(sample[10] for sample in samples) / len(samples) * rpm_scale
        ),
        "iq_reference_rms_a": math.sqrt(
            sum(sample[4] ** 2 for sample in samples) / len(samples)
        ),
        "peak_iq_reference_a": max(abs(sample[4]) for sample in samples),
    })


def direction_consistency(visits: list[dict]) -> dict:
    positive = {
        item["frequency_hz"]: item
        for item in visits[0]["records"]
    }
    negative = {
        item["frequency_hz"]: item
        for item in visits[1]["records"]
    }
    points = []
    for frequency in FREQUENCIES_HZ:
        first = positive[frequency]
        second = negative[frequency]
        phase_difference = abs(
            (first["closed_loop_phase_deg"] - second["closed_loop_phase_deg"]
             + 180.0) % 360.0 - 180.0
        )
        points.append({
            "frequency_hz": frequency,
            "magnitude_difference_db": abs(
                first["closed_loop_magnitude_db"]
                - second["closed_loop_magnitude_db"]
            ),
            "phase_difference_deg": phase_difference,
        })
    return {
        "points": points,
        "maximum_magnitude_difference_db": max(
            item["magnitude_difference_db"] for item in points
        ),
        "maximum_phase_difference_deg": max(
            item["phase_difference_deg"] for item in points
        ),
    }


def summarize_baseline(records: list[dict]) -> dict:
    ordered = sorted(records, key=lambda item: item["frequency_hz"])
    target_db = -3.0103
    bandwidth_hz = None
    for first, second in zip(ordered, ordered[1:]):
        if ((first["closed_loop_magnitude_db"] - target_db)
                * (second["closed_loop_magnitude_db"] - target_db) <= 0.0):
            bandwidth_hz = interpolate_log_crossing(first, second, target_db)
            break
    peak = max(ordered, key=lambda item: item["closed_loop_magnitude_db"])
    return {
        "bandwidth_definition": "first -3.0103 dB crossing relative to unity DC gain",
        "closed_loop_bandwidth_hz": bandwidth_hz,
        "maximum_closed_loop_peak_db": peak["closed_loop_magnitude_db"],
        "peak_frequency_hz": peak["frequency_hz"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.plan_only:
        print(json.dumps(plan(), indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or (
        Path("tests/results") / f"stage2_speed_baseline_{stamp}.json"
    )
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    pole_pairs = int(spec["control"]["pole_pairs"])
    buffer = bytearray()
    result = {
        "schema": "stage2-speed-baseline-result-v2",
        "plan": plan(),
        "visits": [],
        "summary": None,
        "status": "running",
        "error": None,
        "final_mode": None,
    }

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            proto.send_command(port, "SPEED EST WIN")
            proto.send_command(
                port,
                f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
            )
            proto.send_command(port, "FF OFF")
            proto.send_command(port, "TRACK RAMP 3000")
            proto.send_command(port, f"PI SPEED {DEFAULT_KP:.9g} {DEFAULT_KI:.9g}")
            proto.send_command(port, "PI CLEAR")
            proto.send_command(port, "START SPEED")
            wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0,
            )

            for bias_rpm in SPEED_BIASES_RPM:
                proto.send_command(port, "TRACK RAMP 3000")
                proto.send_command(port, "SPEED 0")
                supervised_wait(port, buffer, 0.8, spec, MODE_SPEED_TORQUE_RUN)
                proto.send_command(port, f"SPEED {bias_rpm:g}")
                supervised_wait(port, buffer, 1.2, spec, MODE_SPEED_TORQUE_RUN)
                visit = {"bias_rpm": bias_rpm, "records": []}

                for frequency_hz in FREQUENCIES_HZ:
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(port, buffer, 0.3, spec, MODE_SPEED_TORQUE_RUN)
                    proto.send_command(
                        port,
                        f"SPEED SINE {SINE_AMPLITUDE_RPM:g} {frequency_hz:g}",
                    )
                    supervised_wait(
                        port, buffer, max(0.5, 3.0 / frequency_hz),
                        spec, MODE_SPEED_TORQUE_RUN,
                    )
                    divider = scope_divider(frequency_hz)
                    scope.arm_scope(port, divider)
                    capture_duration = 4096.0 * divider / 40000.0
                    supervised_wait(
                        port, buffer, capture_duration + 0.08,
                        spec, MODE_SPEED_TORQUE_RUN,
                    )
                    capture, samples = scope.dump_scope(port, 25.0)
                    if capture["divider"] != divider:
                        raise RuntimeError(
                            "scope divider mismatch: "
                            f"requested {divider}, received {capture['divider']}"
                        )
                    sample_period = capture["divider"] / capture["base_sample_hz"]
                    path = scope_dir / (
                        f"{bias_rpm:+g}rpm_{frequency_hz:g}hz.csv"
                    )
                    scope.write_csv(path, samples, sample_period)
                    record = analyze_capture(
                        samples, frequency_hz, sample_period, pole_pairs,
                    )
                    add_control_cost(record, samples, pole_pairs)
                    record["scope_divider"] = divider
                    record["scope_csv"] = str(path.resolve())
                    visit["records"].append(record)
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.3, spec, MODE_SPEED_TORQUE_RUN,
                    )

                visit["frequency_summary"] = summarize_baseline(visit["records"])
                result["visits"].append(visit)

            result["summary"] = {
                "bandwidths_hz": [
                    visit["frequency_summary"]["closed_loop_bandwidth_hz"]
                    for visit in result["visits"]
                ],
                "direction_consistency": direction_consistency(result["visits"]),
                "peak_dq_current_a": max(
                    record["raw_peak_dq_current_a"]
                    for visit in result["visits"]
                    for record in visit["records"]
                ),
                "peak_iq_reference_a": max(
                    record["peak_iq_reference_a"]
                    for visit in result["visits"]
                    for record in visit["records"]
                ),
            }
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException) as error:
            result["status"] = "error"
            result["error"] = str(error)
            raise
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer,
            )
            if result["final_mode"] == MODE_READY:
                try:
                    proto.send_command(port, "SPEED EST WIN")
                except serial.SerialException:
                    pass
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )

    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "summary": result["summary"],
        "final_mode": result["final_mode"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
