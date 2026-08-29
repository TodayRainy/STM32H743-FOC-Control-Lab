#!/usr/bin/env python3
"""Compare tensor and estimated-d-axis HFI tracking with ABZ in control."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
import stage5_hfi_shadow_validation as hfi


SPEED_SEQUENCE_RPM = (100.0, 250.0, 500.0, 750.0,
                      500.0, 250.0, 100.0,
                      -100.0, -250.0, -500.0, -750.0)
SPEED_RAMP_RPM_PER_S = 500.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--injection", type=float, default=0.5)
    parser.add_argument("--standstill-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_d_axis_tracking_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    raw_scope = output.with_name(output.stem+"_scope.csv")
    result = {
        "schema": "stage5-hfi-d-axis-tracking-result-v1",
        "status": "running",
        "error": None,
        "control_angle": "ABZ",
        "tensor": None,
        "d_axis": None,
        "running": [],
        "scope": None,
        "telemetry_csv": str(telemetry.resolve()),
        "scope_csv": str(raw_scope.resolve()),
        "final_mode": None,
    }
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
    records: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                hfi.wait_mode(port, buffer, hfi.MODE_READY, 10.0,
                              started, records, limits, "initial_ready")
                for command in (
                    "SPEED LIMIT 2", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE OFF", "VOLT LIMIT 1",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}", "PI CLEAR",
                    "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, records, limits, "abz_calibration")
                hfi.wait_speed(port, buffer, 0.0, 0.0,
                               started, records, limits)

                proto.send_command(port, f"HFI SHADOW {args.injection:g} 0")
                limits["hfi_active"] = True
                frames = hfi.capture_for(
                    port, buffer, 0.9, "tensor", 0.0,
                    started, records, limits,
                )
                result["tensor"] = hfi.summarize_frames(
                    frames[len(frames)//2:], 0.0, 15.0
                )

                proto.send_command(port, "HFI DTRACK")
                frames = hfi.capture_for(
                    port, buffer, 0.9, "d_axis", 0.0,
                    started, records, limits,
                )
                result["d_axis"] = hfi.summarize_frames(
                    frames[len(frames)//2:], 0.0, 20.0
                )
                metadata, samples = scope.capture_scope(port, 1, 15.0)
                scope.write_csv(
                    raw_scope, samples,
                    metadata["divider"]/metadata["base_sample_hz"],
                )
                result["scope"] = hfi.summarize_scope(metadata, samples)

                if not args.standstill_only:
                    source_rpm = 0.0
                    for target_rpm in SPEED_SEQUENCE_RPM:
                        frames = hfi.wait_speed(
                            port, buffer, target_rpm, source_rpm,
                            started, records, limits,
                        )
                        summary = hfi.summarize_frames(
                            frames, target_rpm, 25.0
                        )
                        result["running"].append(summary)
                        source_rpm = target_rpm

                result["status"] = "passed" if (
                    result["tensor"]["passed"]
                    and result["d_axis"]["passed"]
                    and all(point["passed"] for point in result["running"])
                ) else "characterized"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "HFI OFF")
                    limits["hfi_active"] = False
                    proto.send_command(port, "SPEED 0")
                    time.sleep(0.3)
                    proto.send_command(port, "STOP")
                    final = hfi.wait_mode(
                        port, buffer, hfi.MODE_READY, 3.0,
                        started, records, limits, "final_ready",
                    )
                    result["final_mode"] = int(round(final[27]))
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        hfi.write_csv(telemetry, records)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] == "failed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "d-axis HFI validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "tensor": result["tensor"],
        "d_axis": result["d_axis"],
        "running": result["running"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
