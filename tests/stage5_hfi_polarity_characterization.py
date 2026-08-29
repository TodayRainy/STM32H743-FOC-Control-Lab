#!/usr/bin/env python3
"""Measure HFI saturation polarity contrast while ABZ remains in control."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
import stage5_hfi_shadow_validation as hfi


DEFAULT_BIASES_A = (0.5, 1.0, 1.5)


def experiment_plan(biases: tuple[float, ...], visits: int,
                    injection_voltage: float) -> dict:
    return {
        "schema": "stage5-hfi-polarity-characterization-plan-v1",
        "control_angle": "ABZ",
        "hfi_role": "estimated-axis shadow observer",
        "method": "compare HFI incremental response at positive and negative d-axis magnetization",
        "injection_voltage_v": injection_voltage,
        "d_axis_bias_a": list(biases),
        "rotor_position_visits": visits,
        "reposition_speed_rpm": 100.0,
        "requires_24v": True,
        "motor_may_reposition_bidirectionally_below_150_rpm": True,
        "finally_stop_and_confirm_ready": True,
    }


def circular_mean(values: list[float]) -> float:
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    return math.atan2(sine, cosine) % (2.0*math.pi)


def percentile(values: list[float], fraction: float) -> float:
    return hfi.percentile(sorted(values), fraction)


def summarize_scope(samples: list[tuple[float, ...]], target_id: float,
                    visit: int, path: Path) -> dict:
    steady = samples[len(samples)//4:]
    d_response = [sample[13] for sample in steady]
    q_response = [sample[14] for sample in steady]
    phase_error = [abs(sample[12]) for sample in steady]
    return {
        "visit": visit,
        "target_id_a": target_id,
        "actual_id_mean_a": statistics.fmean(sample[3] for sample in steady),
        "actual_iq_mean_a": statistics.fmean(sample[2] for sample in steady),
        "id_reference_mean_a": statistics.fmean(sample[5] for sample in steady),
        "rotor_electrical_angle_deg": math.degrees(circular_mean(
            [sample[8] for sample in steady]
        )),
        "mechanical_speed_rms_rpm": math.sqrt(statistics.fmean(
            (hfi.electrical_to_mechanical_rpm(sample[9]))**2
            for sample in steady
        )),
        "d_response_mean": statistics.fmean(d_response),
        "d_response_std": statistics.pstdev(d_response),
        "q_response_mean": statistics.fmean(q_response),
        "phase_error_p95_deg": math.degrees(percentile(phase_error, 0.95)),
        "peak_dq_current_a": max(math.hypot(sample[2], sample[3])
                                 for sample in samples),
        "minimum_voltage_scale": min(sample[11] for sample in samples),
        "scope_csv": str(path.resolve()),
    }


def pair_summaries(points: list[dict], biases: tuple[float, ...]) -> list[dict]:
    pairs = []
    visits = sorted({point["visit"] for point in points})
    for visit in visits:
        visit_points = {point["target_id_a"]: point for point in points
                        if point["visit"] == visit}
        for bias in biases:
            positive = visit_points[bias]
            negative = visit_points[-bias]
            mean_response = 0.5*(positive["d_response_mean"]
                                 +negative["d_response_mean"])
            contrast = positive["d_response_mean"]-negative["d_response_mean"]
            pairs.append({
                "visit": visit,
                "bias_a": bias,
                "rotor_electrical_angle_deg":
                    positive["rotor_electrical_angle_deg"],
                "positive_response": positive["d_response_mean"],
                "negative_response": negative["d_response_mean"],
                "contrast": contrast,
                "normalized_contrast": contrast/mean_response
                    if abs(mean_response) > 1.0e-6 else None,
            })
    return pairs


def aggregate_pairs(pairs: list[dict], biases: tuple[float, ...]) -> list[dict]:
    result = []
    for bias in biases:
        selected = [pair for pair in pairs if pair["bias_a"] == bias]
        contrasts = [pair["contrast"] for pair in selected]
        normalized = [pair["normalized_contrast"] for pair in selected
                      if pair["normalized_contrast"] is not None]
        positive = sum(value > 0.0 for value in contrasts)
        negative = sum(value < 0.0 for value in contrasts)
        result.append({
            "bias_a": bias,
            "visits": len(selected),
            "contrast_mean": statistics.fmean(contrasts),
            "contrast_min": min(contrasts),
            "contrast_max": max(contrasts),
            "normalized_contrast_mean": statistics.fmean(normalized),
            "sign_agreement": max(positive, negative)/len(contrasts),
            "observed_sign": 1 if positive > negative else -1,
        })
    return result


def capture_point(port, target_id: float, visit: int, sequence_index: int,
                  output_dir: Path, limits: dict) -> dict:
    proto.send_command(port, f"SPEED ID {target_id:g}")
    time.sleep(0.12)
    metadata, samples = scope.capture_scope(port, 1, 15.0)
    path = output_dir/(
        f"visit_{visit:02d}_point_{sequence_index:02d}_id_{target_id:+g}a.csv"
    )
    scope.write_csv(path, samples,
                    metadata["divider"]/metadata["base_sample_hz"])
    point = summarize_scope(samples, target_id, visit, path)
    if point["peak_dq_current_a"] > limits["max_hfi_dq_current"]:
        raise RuntimeError(
            f"HFI dq current exceeded limit: {point['peak_dq_current_a']:.3f} A"
        )
    if point["minimum_voltage_scale"] < limits["min_voltage_scale"]:
        raise RuntimeError(
            f"voltage scale fell below limit: {point['minimum_voltage_scale']:.3f}"
        )
    if abs(point["id_reference_mean_a"]-target_id) > 0.05:
        raise RuntimeError(
            f"d-axis reference was not applied: requested {target_id:.3f} A, "
            f"observed {point['id_reference_mean_a']:.3f} A"
        )
    if point["mechanical_speed_rms_rpm"] > limits["max_hfi_rpm"]:
        raise RuntimeError(
            f"standstill speed exceeded limit: {point['mechanical_speed_rms_rpm']:.1f} rpm"
        )
    return point


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--biases", type=float, nargs="+",
                        default=DEFAULT_BIASES_A)
    parser.add_argument("--visits", type=int, default=4)
    parser.add_argument("--voltage", type=float, default=0.5)
    args = parser.parse_args()

    biases = tuple(sorted(set(abs(value) for value in args.biases)))
    if (not biases or biases[-1] > 1.5 or biases[0] <= 0.0
            or args.visits < 1 or args.visits > 8
            or not 0.1 <= args.voltage <= 1.0):
        parser.error("biases must be in (0,1.5] A, visits in [1,8], voltage in [0.1,1.0] V")
    plan = experiment_plan(biases, args.visits, args.voltage)
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_polarity_characterization_{stamp}.json"
    )
    output_dir = output.with_suffix("")
    result = {
        "schema": "stage5-hfi-polarity-characterization-result-v1",
        "plan": plan,
        "status": "running",
        "error": None,
        "points": [],
        "pairs": [],
        "aggregate": [],
        "final_mode": None,
        "defaults_restored": False,
    }
    limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 3.5,
        "max_rpm": 1800.0,
        "max_hfi_rpm": 30.0,
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
                    "FW MODE OFF", "VOLT LIMIT 1", "TRACK STEP", "PI CLEAR",
                    "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, records, limits,
                              "encoder_calibration")
                hfi.wait_speed(port, buffer, 0.0, 0.0,
                               started, records, limits)
                proto.send_command(port, f"HFI SHADOW {args.voltage:g} 0")
                limits["hfi_active"] = True
                time.sleep(0.8)
                output_dir.mkdir(parents=True, exist_ok=True)

                for visit in range(args.visits):
                    if visit > 0:
                        direction = 1.0 if (visit & 1) else -1.0
                        proto.send_command(port, f"SPEED {direction*100.0:g}")
                        time.sleep(0.18+0.035*visit)
                        proto.send_command(port, "SPEED 0")
                        time.sleep(0.8)

                    ordered = list(biases)
                    if visit & 1:
                        ordered.reverse()
                    targets = []
                    for bias in ordered:
                        if (visit+ordered.index(bias)) & 1:
                            targets.extend((-bias, bias))
                        else:
                            targets.extend((bias, -bias))
                    zero = capture_point(
                        port, 0.0, visit, 0, output_dir, limits
                    )
                    zero["sequence_index"] = 0
                    result["points"].append(zero)
                    for point_index, target_id in enumerate(targets, start=1):
                        point = capture_point(
                            port, target_id, visit, point_index,
                            output_dir, limits
                        )
                        point["sequence_index"] = point_index
                        result["points"].append(point)
                        proto.send_command(port, "SPEED ID 0")
                        time.sleep(0.12)

                nonzero = [point for point in result["points"]
                           if point["target_id_a"] != 0.0]
                result["pairs"] = pair_summaries(nonzero, biases)
                result["aggregate"] = aggregate_pairs(result["pairs"], biases)
                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED ID 0")
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
                        "SPEED EST WIN", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                        "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] != "passed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "HFI polarity characterization failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "aggregate": result["aggregate"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
