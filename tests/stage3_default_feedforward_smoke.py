#!/usr/bin/env python3
"""Verify the reset-default IDENTIFIED+BOTH feedforward without selecting it."""

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
from stage2_speed_autotune_validation import open_serial_with_retry
from stage3_feedforward_component_validation import (
    check_mode,
    save_report,
    stop_and_restore,
    write_csv,
)


SPEEDS_RPM = (1500.0, -1500.0)
SPEED_RAMP_RPM_PER_SEC = 3000.0
TEST_SPEED_IQ_LIMIT_A = 4.0


def experiment_plan() -> dict:
    return {
        "schema": "stage3-default-feedforward-smoke-plan-v1",
        "purpose": "verify reset-default BOTH without sending FF model/mode commands",
        "default_model_expected": "IDENTIFIED",
        "default_mode_expected": "BOTH",
        "speed_order_rpm": list(SPEEDS_RPM),
        "speed_ramp_rpm_per_sec": SPEED_RAMP_RPM_PER_SEC,
        "default_selection_command_sent": False,
        "requires_24v": True,
        "motor_will_rotate": True,
        "maximum_commanded_speed_rpm": max(abs(value) for value in SPEEDS_RPM),
        "finally_stop_and_confirm_ready": True,
    }


def summarize_hold(target_rpm: float,
                   samples: list[tuple[float, tuple[float, ...]]]) -> dict:
    tail_start = max(0.0, samples[-1][0] - 0.2)
    tail = [frame for elapsed, frame in samples if elapsed >= tail_start]
    modes = sorted({int(round(frame[35])) for _, frame in samples})
    return {
        "target_rpm": target_rpm,
        "samples": len(samples),
        "feedforward_modes": modes,
        "mean_tail_speed_rpm": sum(proto.mechanical_rpm(frame) for frame in tail) / len(tail),
        "mean_tail_uq_feedforward_v": sum(frame[33] for frame in tail) / len(tail),
        "mean_tail_ud_feedforward_v": sum(frame[34] for frame in tail) / len(tail),
        "peak_dq_current_a": max(math.hypot(frame[6], frame[7]) for _, frame in samples),
        "minimum_voltage_scale": min(frame[26] for _, frame in samples),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--telemetry", type=Path)
    args = parser.parse_args()

    plan = experiment_plan()
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / f"stage3_default_feedforward_smoke_{stamp}.json"
    telemetry = args.telemetry or output.with_suffix(".csv")
    limits = {
        "max_current": 5.0,
        "max_rpm": 1800.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.90,
    }
    result = {
        "schema": "stage3-default-feedforward-smoke-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "initial_mode": None,
        "holds": [],
        "final_mode": None,
        "defaults_restored": False,
        "telemetry_csv": str(telemetry.resolve()),
    }
    records: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()

    port = open_serial_with_retry(args.port, args.baudrate)
    try:
        port.reset_input_buffer()
        initial = wait_mode(
            port, buffer, MODE_READY, 20.0, started, records,
            0, "default_smoke", "initial_ready", limits,
        )
        check_mode(initial, "BOTH")
        result["initial_mode"] = int(round(initial[35]))
        try:
            for command in (
                f"SPEED LIMIT {TEST_SPEED_IQ_LIMIT_A:g}",
                f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}",
                "PI CLEAR",
                "START SPEED",
            ):
                proto.send_command(port, command)
            running = wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, started, records,
                0, "default_smoke", "start_speed", limits,
            )
            check_mode(running, "BOTH")

            for target_rpm in SPEEDS_RPM:
                proto.send_command(port, f"SPEED {target_rpm:g}")
                samples = capture_transition(
                    port, buffer, target_rpm,
                    abs(target_rpm) * 2.0 / SPEED_RAMP_RPM_PER_SEC + 4.0,
                    0.4, started, records, 0, "default_smoke", limits,
                    stable_tolerance_rpm=90.0,
                )
                for _, frame in samples:
                    check_mode(frame, "BOTH")
                hold = summarize_hold(target_rpm, samples)
                if hold["feedforward_modes"] != [1]:
                    raise RuntimeError("feedforward mode changed during default smoke")
                if math.copysign(1.0, hold["mean_tail_uq_feedforward_v"]) != math.copysign(1.0, target_rpm):
                    raise RuntimeError("q-axis feedforward sign does not follow speed direction")
                if abs(hold["mean_tail_uq_feedforward_v"]) < 0.5:
                    raise RuntimeError("q-axis feedforward magnitude is unexpectedly small")
                result["holds"].append(hold)
            result["status"] = "passed"
        except (RuntimeError, serial.SerialException, KeyboardInterrupt) as error:
            result["status"] = "failed"
            result["error"] = str(error) or error.__class__.__name__
        finally:
            final_mode, restored, cleanup_error = stop_and_restore(
                port, args.port, args.baudrate, buffer,
                started, records, limits,
            )
            result["final_mode"] = final_mode
            result["defaults_restored"] = restored
            if cleanup_error is not None:
                result["error"] = result["error"] or cleanup_error
    finally:
        port.close()
        write_csv(telemetry, records)
        save_report(output, result)

    if result["status"] != "passed" or result["final_mode"] != MODE_READY:
        raise RuntimeError(result["error"] or "D-059 default smoke failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "initial_mode": result["initial_mode"],
        "holds": result["holds"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
