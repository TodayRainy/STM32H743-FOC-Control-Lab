#!/usr/bin/env python3
"""Capture a frozen-angle four-state HFI tensor at standstill."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial

import motor_baseline as proto
import scope_capture as scope
import stage5_hfi_carrier_analysis as carrier
import stage5_hfi_shadow_validation as hfi


def summarize_tensor(matrix: list[list[float]]) -> dict:
    response = np.asarray(matrix, dtype=float)
    symmetric = 0.5*(response+response.T)
    cross = symmetric[0, 1]
    diagonal = symmetric[1, 1]-symmetric[0, 0]
    return {
        "symmetric_matrix_a_per_v_per_sample": symmetric.tolist(),
        "antisymmetric_cross_a_per_v_per_sample":
            float(0.5*(response[0, 1]-response[1, 0])),
        "saliency_axis_deg": math.degrees(0.5*math.atan2(2.0*cross,
                                                          diagonal)),
        "diagonal_difference_a_per_v_per_sample": float(diagonal),
        "symmetric_cross_a_per_v_per_sample": float(cross),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--voltage", type=float, default=0.5)
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--tracking", action="store_true")
    parser.add_argument("--control-transition", action="store_true")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_tensor_capture_{stamp}.json"
    )
    scope_path = output.with_name(output.stem+"_scope.csv")
    result = {
        "schema": "stage5-hfi-tensor-capture-v1",
        "status": "running",
        "error": None,
        "injection_voltage_v": args.voltage,
        "carrier_angle_offset_deg": args.offset,
        "control_angle": "ABZ",
        "hfi_tracking": args.tracking,
        "scope_csv": str(scope_path.resolve()),
        "scope": None,
        "best_carrier_fit": None,
        "tensor": None,
        "board_demodulation": None,
        "tracking_summary": None,
        "final_mode": None,
    }
    limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 3.5,
        "max_rpm": 1800.0,
        "max_hfi_rpm": 100.0,
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
                    "SPEED LIMIT 3", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE HYBRID", "VOLT LIMIT 1", "TRACK RAMP 500",
                    "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, records, limits,
                              "encoder_calibration")
                hfi.wait_speed(port, buffer, 0.0, 0.0,
                               started, records, limits)

                mode = "SHADOW" if args.tracking else "TENSOR"
                proto.send_command(port, f"HFI {mode} "
                                         f"{args.voltage:g} {args.offset:g}")
                limits["hfi_active"] = True
                time.sleep(1.0 if args.tracking else 0.05)
                if args.control_transition:
                    if not args.tracking:
                        raise RuntimeError(
                            "--control-transition requires --tracking"
                        )
                    scope.arm_scope(port, 1)
                    proto.send_command(port, "HFI CONTROL")
                    time.sleep(0.13)
                    proto.send_command(port, "STOP")
                    metadata, samples = scope.dump_scope(port, 15.0)
                else:
                    metadata, samples = scope.capture_scope(port, 1, 15.0)
                scope.write_csv(
                    scope_path, samples,
                    metadata["divider"]/metadata["base_sample_hz"],
                )
                peak_current = max(math.hypot(sample[2], sample[3])
                                   for sample in samples)
                minimum_scale = min(sample[11] for sample in samples)
                current_limit = 4.5 if args.control_transition \
                    else limits["max_hfi_dq_current"]
                if peak_current > current_limit:
                    raise RuntimeError(
                        f"HFI dq current exceeded limit: {peak_current:.3f} A"
                    )
                if minimum_scale < limits["min_voltage_scale"]:
                    raise RuntimeError(
                        f"voltage scale fell below limit: {minimum_scale:.3f}"
                    )
                analysis = carrier.analyze(scope_path)
                best = analysis["best_fit"]
                result["scope"] = {
                    **metadata,
                    "peak_dq_current_a": peak_current,
                    "minimum_voltage_scale": minimum_scale,
                }
                result["best_carrier_fit"] = best
                result["tensor"] = summarize_tensor(
                    best["matrix_a_per_v_per_sample"]
                )
                steady = samples[len(samples)//2:]
                phase = [sample[12] for sample in steady
                         if abs(sample[12]) > 1.0e-8]
                result["board_demodulation"] = {
                    "phase_updates": len(phase),
                    "phase_error_mean_deg": math.degrees(sum(phase)/len(phase)),
                    "phase_error_min_deg": math.degrees(min(phase)),
                    "phase_error_max_deg": math.degrees(max(phase)),
                    "d_response_mean_a": sum(sample[13] for sample in steady)
                                           /len(steady),
                    "q_response_mean_a": sum(sample[14] for sample in steady)
                                           /len(steady),
                }
                if args.tracking and not args.control_transition:
                    buffer.clear()
                    port.reset_input_buffer()
                    frames = hfi.capture_for(
                        port, buffer, 0.25, "tracking_scope_followup",
                        0.0, started, records, limits,
                    )
                    result["tracking_summary"] = hfi.summarize_frames(
                        frames, 0.0, 180.0
                    )
                result["status"] = "passed"
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
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] != "passed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "HFI tensor capture failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "best_carrier_fit": result["best_carrier_fit"],
        "tensor": result["tensor"],
        "board_demodulation": result["board_demodulation"],
        "tracking_summary": result["tracking_summary"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
