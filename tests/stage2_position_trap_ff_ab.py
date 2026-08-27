#!/usr/bin/env python3
"""D-054 isolate position velocity feedforward on the frozen TRAP profile."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import serial

import stage2_low_speed_servo_ab as d049
import stage2_position_default_ab as d053
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


DECISION = "D-054"
PROFILES = (
    {
        "name": "trap_ff_off",
        "track_command": "POS TRACK TRAP 180 900",
        "position_velocity_feedforward": False,
        "center_order_deg": d053.CENTERS_DEG,
    },
    {
        "name": "trap_ff_on",
        "track_command": "POS TRACK TRAP 180 900",
        "position_velocity_feedforward": True,
        "center_order_deg": tuple(reversed(d053.CENTERS_DEG)),
    },
)


def plan() -> dict:
    result = d053.plan()
    result.update({
        "schema": "stage2-position-trap-ff-ab-plan-v1",
        "decision": DECISION,
        "purpose": "isolate full trajectory velocity feedforward",
        "profiles": PROFILES,
        "records_per_profile": 24,
        "record_count": 48,
    })
    return result


def run_experiment(port, buffer: bytearray, output: Path,
                   result: dict, rows: list[dict]) -> None:
    for profile in PROFILES:
        d053.run_profile(port, buffer, output, result, rows, profile)
    result["summary"] = {
        profile["name"]: d053.aggregate_profile([
            record for record in result["records"]
            if record["profile"] == profile["name"]
        ])
        for profile in PROFILES
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM30")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()

    test_plan = plan()
    if args.plan_only:
        print(json.dumps(test_plan, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path("tests/results") / f"stage2_position_trap_ff_ab_{stamp}.json"
    csv_output = output.with_suffix(".csv")
    result = {
        "schema": "stage2-position-trap-ff-ab-result-v1",
        "plan": test_plan,
        "records": [],
        "summary": None,
        "raw_csv": str(csv_output.resolve()),
        "status": "running",
        "error": None,
        "final_mode": None,
        "defaults_restored": False,
    }
    rows = []
    buffer = bytearray()
    port = open_serial_with_retry(args.port, args.baudrate)
    try:
        port.reset_input_buffer()
        d049.wait_mode_without_frozen_speed(port, buffer, d049.MODE_READY, 20.0)
        try:
            run_experiment(port, buffer, output, result, rows)
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException) as error:
            result["status"] = "error"
            result["error"] = str(error)
        finally:
            try:
                d049.restore_defaults(port, buffer)
                result["final_mode"] = d049.MODE_READY
                result["defaults_restored"] = True
            except (RuntimeError, serial.SerialException):
                result["final_mode"] = None
    finally:
        port.close()
        d053.d051.write_csv(csv_output, rows)
        save_checkpoint(output, result)

    print(json.dumps({
        "result": str(output.resolve()),
        "raw_csv": str(csv_output.resolve()),
        "status": result["status"],
        "summary": result["summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
        "error": result["error"],
    }, indent=2))
    if result["status"] != "completed" or result["final_mode"] != d049.MODE_READY:
        raise RuntimeError(result["error"] or "D-054 did not close safely")


if __name__ == "__main__":
    main()
