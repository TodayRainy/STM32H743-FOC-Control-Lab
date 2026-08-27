#!/usr/bin/env python3
"""Verify reset-default FW HYBRID without sending an FW MODE command."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
from stage2_current_combination_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    capture_transition,
    wait_mode,
)
from stage3_feedforward_component_validation import save_report, write_csv


TARGET_RPM = 5000.0
VOLTAGE_LIMIT_RATIO = 0.35
TARGET_UTILIZATION = 0.95
SPEED_RAMP_RPM_PER_SEC = 3000.0


def experiment_plan() -> dict:
    return {
        "schema": "stage4-default-field-weakening-smoke-plan-v1",
        "purpose": "verify reset-default HYBRID without sending FW MODE",
        "default_mode_expected": "HYBRID",
        "default_selection_command_sent": False,
        "target_rpm": TARGET_RPM,
        "voltage_limit_ratio": VOLTAGE_LIMIT_RATIO,
        "total_dq_current_limit_a": 5.0,
        "requires_24v": True,
        "motor_will_rotate": True,
        "maximum_commanded_speed_rpm": TARGET_RPM,
        "finally_stop_and_confirm_ready": True,
    }


def summarize(samples) -> dict:
    tail_start = max(0.0, samples[-1][0]-0.3)
    tail = [frame for elapsed, frame in samples if elapsed >= tail_start]
    raw_voltage = [
        math.hypot(frame[20], frame[21])/max(frame[26], 1.0e-9)
        for frame in tail
    ]
    target_voltage = (
        sum(frame[29] for frame in tail)/len(tail)
        / math.sqrt(3.0)*VOLTAGE_LIMIT_RATIO*TARGET_UTILIZATION
    )
    return {
        "mean_speed_rpm": sum(proto.mechanical_rpm(frame) for frame in tail)/len(tail),
        "mean_id_reference_a": sum(frame[23] for frame in tail)/len(tail),
        "mean_voltage_request_v": sum(raw_voltage)/len(raw_voltage),
        "target_voltage_v": target_voltage,
        "mean_voltage_error_v": sum(raw_voltage)/len(raw_voltage)-target_voltage,
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for _, frame in samples
        ),
        "minimum_voltage_scale": min(frame[26] for _, frame in samples),
        "feedforward_modes": sorted({int(round(frame[35])) for _, frame in samples}),
    }


def main():
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
        f"stage4_default_field_weakening_smoke_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    limits = {
        "max_current": 5.5,
        "max_rpm": 5350.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.85,
    }
    result = {
        "schema": "stage4-default-field-weakening-smoke-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "summary": None,
        "final_mode": None,
        "defaults_restored": False,
        "telemetry_csv": str(telemetry.resolve()),
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
                wait_mode(
                    port, buffer, MODE_READY, 20.0, started, records,
                    0, "default_hybrid", "initial_ready", limits,
                )
                # Intentionally no FW MODE command.
                for command in (
                    "SPEED EST WIN", "SPEED LIMIT 5",
                    "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    f"VOLT LIMIT {VOLTAGE_LIMIT_RATIO:g}",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}",
                    "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                wait_mode(
                    port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                    started, records, 0, "default_hybrid",
                    "start_speed", limits,
                )
                proto.send_command(port, f"SPEED {TARGET_RPM:g}")
                samples = capture_transition(
                    port, buffer, TARGET_RPM,
                    TARGET_RPM/SPEED_RAMP_RPM_PER_SEC+4.0,
                    0.6, started, records, 0, "default_hybrid", limits,
                    stable_tolerance_rpm=120.0,
                )
                result["summary"] = summarize(samples)
                summary = result["summary"]
                if summary["mean_id_reference_a"] > -0.5:
                    raise RuntimeError("reset-default HYBRID did not generate negative id")
                if abs(summary["mean_voltage_error_v"]) > 0.25:
                    raise RuntimeError("reset-default HYBRID did not track voltage target")
                if summary["feedforward_modes"] != [1]:
                    raise RuntimeError("IDENTIFIED+BOTH current feedforward changed")
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}")
                    proto.send_command(port, "SPEED 0")
                    capture_transition(
                        port, buffer, 0.0, 6.0, 0.4,
                        started, records, 0, "cleanup", limits,
                        stable_tolerance_rpm=120.0,
                    )
                    proto.send_command(port, "STOP")
                    final = wait_mode(
                        port, buffer, MODE_READY, 3.0, started, records,
                        0, "cleanup", "final_ready", limits,
                    )
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "FF MODEL IDENTIFIED", "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    wait_mode(
                        port, buffer, MODE_READY, 2.0, started, records,
                        0, "cleanup", "defaults_restored", limits,
                    )
                    result["final_mode"] = int(round(final[27]))
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        if records:
            write_csv(telemetry, records)
        save_report(output, result)

    if result["status"] != "passed" or result["final_mode"] != MODE_READY:
        raise RuntimeError(result["error"] or "default HYBRID smoke failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "summary": result["summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
