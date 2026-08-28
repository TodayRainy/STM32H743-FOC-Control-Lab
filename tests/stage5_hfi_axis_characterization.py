#!/usr/bin/env python3
"""Characterize the standstill HFI saliency axis with fixed injection angles."""

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
import stage5_hfi_shadow_validation as hfi


OFFSETS_DEG = (-60.0, -45.0, -30.0, 0.0, 30.0, 45.0, 60.0)
INJECTION_VOLTAGE_V = 0.5


def experiment_plan() -> dict:
    return {
        "schema": "stage5-hfi-axis-characterization-plan-v1",
        "control_angle": "ABZ",
        "hfi_pll": "disabled",
        "injection_voltage_v": INJECTION_VOLTAGE_V,
        "injection_frequency_hz": 10000.0,
        "fixed_axis_offsets_deg": list(OFFSETS_DEG),
        "fit": "q_response = a*cos(2*offset)+b*sin(2*offset)+c",
        "requires_24v": True,
        "motor_target_speed_rpm": 0.0,
        "finally_stop_and_confirm_ready": True,
    }


def synchronous_response(samples: list[tuple[float, ...]],
                         offset_deg: float) -> dict:
    offset = math.radians(offset_deg)
    sine = math.sin(offset)
    cosine = math.cos(offset)
    responses_q = []
    responses_d = []
    carrier = [sample[6]*sine+sample[7]*cosine for sample in samples]

    for index in range(1, len(samples)):
        previous_sign = 1.0 if carrier[index-1] >= 0.0 else -1.0
        current_sign = 1.0 if carrier[index] >= 0.0 else -1.0
        if current_sign != previous_sign:
            continue
        delta_q_abz = samples[index][2]-samples[index-1][2]
        delta_d_abz = samples[index][3]-samples[index-1][3]
        delta_q_est = cosine*delta_q_abz-sine*delta_d_abz
        delta_d_est = sine*delta_q_abz+cosine*delta_d_abz
        responses_q.append(-current_sign*delta_q_est)
        responses_d.append(-current_sign*delta_d_est)

    if len(responses_q) < 100:
        raise RuntimeError("insufficient complete HFI half-period pairs")
    q = np.asarray(responses_q)
    d = np.asarray(responses_d)
    return {
        "offset_deg": offset_deg,
        "carrier_pairs": len(q),
        "q_response_mean_a": float(q.mean()),
        "q_response_std_a": float(q.std()),
        "d_response_mean_a": float(d.mean()),
        "d_response_std_a": float(d.std()),
        "peak_actual_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "minimum_voltage_scale": min(sample[11] for sample in samples),
    }


def fit_axis(points: list[dict]) -> dict:
    offsets = np.radians([point["offset_deg"] for point in points])
    response = np.asarray([point["q_response_mean_a"] for point in points])
    design = np.column_stack((np.cos(2.0*offsets),
                              np.sin(2.0*offsets),
                              np.ones(len(offsets))))
    coefficients, _, _, _ = np.linalg.lstsq(design, response, rcond=None)
    predicted = design@coefficients
    residual = response-predicted
    a, b, bias = coefficients
    amplitude = math.hypot(a, b)
    phase = math.atan2(a, -b)
    axis_offset = 0.5*phase
    while axis_offset > math.pi/2.0:
        axis_offset -= math.pi
    while axis_offset <= -math.pi/2.0:
        axis_offset += math.pi
    return {
        "cos_coefficient_a": float(a),
        "sin_coefficient_a": float(b),
        "bias_a": float(bias),
        "amplitude_a": amplitude,
        "saliency_axis_offset_deg": math.degrees(axis_offset),
        "residual_rms_a": float(math.sqrt(np.mean(residual*residual))),
        "predicted_q_response_a": predicted.tolist(),
    }


def main() -> None:
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
        f"stage5_hfi_axis_characterization_{stamp}.json"
    )
    scope_dir = output.with_suffix("")
    result = {
        "schema": "stage5-hfi-axis-characterization-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "points": [],
        "fit": None,
        "scope_directory": str(scope_dir.resolve()),
        "final_mode": None,
        "defaults_restored": False,
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

                scope_dir.mkdir(parents=True, exist_ok=True)
                for offset in OFFSETS_DEG:
                    proto.send_command(port, "HFI OFF")
                    limits["hfi_active"] = False
                    time.sleep(0.05)
                    proto.send_command(
                        port, f"HFI FIXED {INJECTION_VOLTAGE_V:g} {offset:g}"
                    )
                    limits["hfi_active"] = True
                    time.sleep(0.05)
                    metadata, samples = scope.capture_scope(port, 1, 15.0)
                    path = scope_dir/f"fixed_{offset:+g}deg.csv"
                    scope.write_csv(
                        path, samples,
                        metadata["divider"]/metadata["base_sample_hz"],
                    )
                    point = synchronous_response(samples, offset)
                    point["scope_csv"] = str(path.resolve())
                    result["points"].append(point)
                    if point["peak_actual_dq_current_a"] > 3.5:
                        raise RuntimeError(
                            f"HFI current exceeded limit at {offset:g} deg"
                        )
                result["fit"] = fit_axis(result["points"])
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
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if (result["status"] != "passed"
            or result["final_mode"] != hfi.MODE_READY):
        raise RuntimeError(result["error"] or "HFI axis characterization failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "fit": result["fit"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
