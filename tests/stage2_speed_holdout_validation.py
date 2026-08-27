#!/usr/bin/env python3
"""Run the selected speed PI against the frozen high-speed holdout set."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
from scope_current_step import wait_mode
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    capture_step,
    open_serial_with_retry,
    stop_and_confirm_ready,
    summarize_candidate,
    supervised_wait,
)


DEFAULT_KP = 0.00410918168695387
DEFAULT_KI = 0.0140681012016855


def build_holdout_transitions(spec: dict) -> list[tuple[float, float]]:
    holdout = spec["holdout"]
    return [
        (0.0, holdout["step_rpm"][0]),
        (0.0, holdout["step_rpm"][1]),
        (-2200.0, -2200.0 - holdout["local_step_at_2200_rpm"]),
        (-2200.0, -2200.0 + holdout["local_step_at_2200_rpm"]),
        (2200.0, 2200.0 - holdout["local_step_at_2200_rpm"]),
        (2200.0, 2200.0 + holdout["local_step_at_2200_rpm"]),
        (-4200.0, -4200.0 - holdout["local_step_at_4200_rpm"]),
        (-4200.0, -4200.0 + holdout["local_step_at_4200_rpm"]),
        (4200.0, 4200.0 - holdout["local_step_at_4200_rpm"]),
        (4200.0, 4200.0 + holdout["local_step_at_4200_rpm"]),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--ki", type=float, default=DEFAULT_KI)
    parser.add_argument("--high-speed-only", action="store_true")
    parser.add_argument("--startup-only", action="store_true")
    parser.add_argument("--edge-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / f"stage2_speed_holdout_{stamp}.json"
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    result = {
        "schema": "stage2-speed-holdout-result-v1",
        "spec": str(args.spec),
        "kp": args.kp,
        "ki": args.ki,
        "records": [],
        "summary": None,
        "error": None,
        "final_mode": None,
    }

    with open_serial_with_retry(args.port, args.baudrate) as setup_port:
        setup_port.reset_input_buffer()
        ready = wait_mode(setup_port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
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
            wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0)
            transitions = build_holdout_transitions(spec)
            if args.high_speed_only:
                transitions = [
                    transition for transition in transitions
                    if abs(transition[0]) >= 4000.0
                ]
            if args.startup_only:
                transitions = [
                    transition for transition in transitions
                    if transition[0] == 0.0
                ]
            if args.edge_only:
                transitions = [
                    transition for transition in transitions
                    if transition[0] == 0.0 or abs(transition[0]) >= 4000.0
                ]
            visits = int(spec["holdout"]["visits"])
            for visit in range(visits):
                ordered = transitions if visit == 0 else list(reversed(transitions))
                previous_sign = 0
                for base_rpm, target_rpm in ordered:
                    sign = -1 if base_rpm < 0.0 else (1 if base_rpm > 0.0 else 0)
                    if previous_sign and sign and sign != previous_sign:
                        proto.send_command(
                            port,
                            f"TRACK RAMP {spec['screen']['operating_point_ramp_rpm_per_sec']:g}",
                        )
                        proto.send_command(port, "SPEED 0")
                        supervised_wait(
                            port, buffer, 1.7, spec, MODE_SPEED_TORQUE_RUN,
                        )
                    previous_sign = sign or previous_sign
                    path = scope_dir / (
                        f"v{visit}_{base_rpm:+g}_to_{target_rpm:+g}rpm.csv"
                    )
                    record = capture_step(
                        port, buffer, base_rpm, target_rpm, spec, 0.0, path,
                    )
                    record.update({
                        "profile": "selected_speed_pi",
                        "round": visit,
                        "kp": args.kp,
                        "ki": args.ki,
                    })
                    result["records"].append(record)
            result["summary"] = summarize_candidate(result["records"], spec)
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
