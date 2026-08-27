#!/usr/bin/env python3
"""Validate the six D-039 measured-plant speed PI candidates on hardware."""

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
from stage2_speed_baseline import (
    DEFAULT_KI,
    DEFAULT_KP,
    add_control_cost,
    summarize_baseline,
)
from stage2_speed_frequency_response import analyze_capture


DEFAULT_DESIGN = Path(
    "tests/baselines/stage2_speed_pi_design_d039_20260825.json"
)
SPEED_BIASES_RPM = (1500.0, -1500.0)
SINE_AMPLITUDE_RPM = 100.0
FREQUENCIES_HZ = (
    4.0, 8.0, 10.0, 15.0, 20.0, 25.0,
    30.0, 35.0, 40.0, 50.0, 60.0,
)
SCOPE_DIVIDER = 10
MAXIMUM_BANDWIDTH_ERROR_FRACTION = 0.20
MAXIMUM_CLOSED_LOOP_PEAK_DB = 2.0
MAXIMUM_DIRECTION_BANDWIDTH_DIFFERENCE_FRACTION = 0.15
IQ_SATURATION_FRACTION = 0.95


def load_candidates(path: Path) -> list[dict]:
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("schema") != "stage2-speed-pi-measured-plant-design-v1":
        raise ValueError("unsupported D-039 speed PI design schema")
    candidates = report.get("candidates", [])
    if len(candidates) != 6:
        raise ValueError("D-039 validation requires exactly six candidates")
    targets = [
        float(item["target_closed_loop_bandwidth_hz"])
        for item in candidates
    ]
    if targets != sorted(targets) or len(set(targets)) != len(targets):
        raise ValueError("candidate bandwidth targets must be unique and ascending")
    for item in candidates:
        for field in ("kp", "ki"):
            value = float(item[field])
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"candidate {item['name']} has invalid {field}")
    return candidates


def visit_order(candidates: list[dict]) -> list[tuple[dict, float]]:
    """Balance slow drift: positive ascending, negative descending."""
    return (
        [(candidate, SPEED_BIASES_RPM[0]) for candidate in candidates]
        + [(candidate, SPEED_BIASES_RPM[1]) for candidate in reversed(candidates)]
    )


def plan(candidates: list[dict]) -> dict:
    visits = visit_order(candidates)
    return {
        "schema": "stage2-speed-pi-frequency-validation-plan-v1",
        "design_candidates": [
            {
                "name": item["name"],
                "kp": item["kp"],
                "ki": item["ki"],
                "target_closed_loop_bandwidth_hz": (
                    item["target_closed_loop_bandwidth_hz"]
                ),
            }
            for item in candidates
        ],
        "visit_order": [
            {"candidate": candidate["name"], "bias_rpm": bias}
            for candidate, bias in visits
        ],
        "speed_estimator": "win_1ms_fc150",
        "feedforward": "off",
        "speed_biases_rpm": SPEED_BIASES_RPM,
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "frequencies_hz": FREQUENCIES_HZ,
        "scope_divider": SCOPE_DIVIDER,
        "scope_capture_count": len(visits) * len(FREQUENCIES_HZ),
        "maximum_commanded_speed_rpm": (
            max(abs(value) for value in SPEED_BIASES_RPM) + SINE_AMPLITUDE_RPM
        ),
        "acceptance": {
            "maximum_bandwidth_error_fraction": (
                MAXIMUM_BANDWIDTH_ERROR_FRACTION
            ),
            "maximum_closed_loop_peak_db": MAXIMUM_CLOSED_LOOP_PEAK_DB,
            "maximum_direction_bandwidth_difference_fraction": (
                MAXIMUM_DIRECTION_BANDWIDTH_DIFFERENCE_FRACTION
            ),
            "maximum_iq_reference_fraction_of_limit": IQ_SATURATION_FRACTION,
        },
        "purpose": "validate D-039 frequency prediction; do not rank or write default PI",
        "requires_24v": True,
        "motor_will_rotate": True,
        "automatic_flash": False,
        "restore_default_speed_pi": True,
        "finally_stop_and_confirm_ready": True,
    }


def summarize_candidate(candidate: dict, visits: list[dict],
                        iq_limit_a: float) -> dict:
    if len(visits) != 2:
        raise ValueError("each candidate requires positive and negative visits")
    frequency_summaries = [
        summarize_baseline(visit["records"]) for visit in visits
    ]
    bandwidths = [
        item["closed_loop_bandwidth_hz"] for item in frequency_summaries
    ]
    target = float(candidate["target_closed_loop_bandwidth_hz"])
    bandwidth_errors = [
        None if value is None else abs(value - target) / target
        for value in bandwidths
    ]
    if any(value is None for value in bandwidths):
        direction_difference = None
    else:
        direction_difference = abs(bandwidths[0] - bandwidths[1]) / (
            0.5 * (bandwidths[0] + bandwidths[1])
        )
    peak_db = max(
        item["maximum_closed_loop_peak_db"] for item in frequency_summaries
    )
    all_records = [
        record for visit in visits for record in visit["records"]
    ]
    peak_iq_reference_a = max(
        record["peak_iq_reference_a"] for record in all_records
    )
    gates = {
        "bandwidth_prediction": all(
            value is not None
            and value <= MAXIMUM_BANDWIDTH_ERROR_FRACTION
            for value in bandwidth_errors
        ),
        "closed_loop_peak": peak_db <= MAXIMUM_CLOSED_LOOP_PEAK_DB,
        "direction_consistency": (
            direction_difference is not None
            and direction_difference
            <= MAXIMUM_DIRECTION_BANDWIDTH_DIFFERENCE_FRACTION
        ),
        "iq_not_saturated": (
            peak_iq_reference_a < IQ_SATURATION_FRACTION * iq_limit_a
        ),
    }
    return {
        "target_closed_loop_bandwidth_hz": target,
        "measured_bandwidths_hz": bandwidths,
        "bandwidth_error_fractions": bandwidth_errors,
        "direction_bandwidth_difference_fraction": direction_difference,
        "maximum_closed_loop_peak_db": peak_db,
        "peak_iq_reference_a": peak_iq_reference_a,
        "gates": gates,
        "qualified_for_later_performance_test": all(gates.values()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--design", type=Path, default=DEFAULT_DESIGN)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    candidates = load_candidates(args.design)
    experiment_plan = plan(candidates)
    if args.plan_only:
        print(json.dumps(experiment_plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or (
        Path("tests/results")
        / f"stage2_speed_pi_frequency_validation_{stamp}.json"
    )
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    pole_pairs = int(spec["control"]["pole_pairs"])
    iq_limit_a = float(spec["safety"]["speed_iq_reference_limit_a"])
    buffer = bytearray()
    result = {
        "schema": "stage2-speed-pi-frequency-validation-result-v1",
        "design": str(args.design.resolve()),
        "plan": experiment_plan,
        "visits": [],
        "candidate_summaries": {},
        "status": "running",
        "error": None,
        "final_mode": None,
        "default_pi_restored": False,
    }

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            proto.send_command(port, "SPEED EST WIN")
            proto.send_command(port, f"SPEED LIMIT {iq_limit_a:g}")
            proto.send_command(port, "FF OFF")
            proto.send_command(port, "TRACK RAMP 3000")
            proto.send_command(port, "PI CLEAR")
            proto.send_command(port, "START SPEED")
            wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0,
            )

            for candidate, bias_rpm in visit_order(candidates):
                proto.send_command(port, "TRACK RAMP 3000")
                proto.send_command(port, "SPEED 0")
                supervised_wait(
                    port, buffer, 0.8, spec, MODE_SPEED_TORQUE_RUN,
                )
                proto.send_command(
                    port,
                    f"PI SPEED {candidate['kp']:.9g} {candidate['ki']:.9g}",
                )
                proto.send_command(port, "PI CLEAR")
                proto.send_command(port, f"SPEED {bias_rpm:g}")
                supervised_wait(
                    port, buffer, 1.2, spec, MODE_SPEED_TORQUE_RUN,
                )
                visit = {
                    "candidate": candidate["name"],
                    "kp": candidate["kp"],
                    "ki": candidate["ki"],
                    "target_closed_loop_bandwidth_hz": (
                        candidate["target_closed_loop_bandwidth_hz"]
                    ),
                    "bias_rpm": bias_rpm,
                    "records": [],
                }

                for frequency_hz in FREQUENCIES_HZ:
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN,
                    )
                    proto.send_command(
                        port,
                        f"SPEED SINE {SINE_AMPLITUDE_RPM:g} {frequency_hz:g}",
                    )
                    supervised_wait(
                        port, buffer, max(0.5, 3.0 / frequency_hz),
                        spec, MODE_SPEED_TORQUE_RUN,
                    )
                    scope.arm_scope(port, SCOPE_DIVIDER)
                    capture_duration = 4096.0 * SCOPE_DIVIDER / 40000.0
                    supervised_wait(
                        port, buffer, capture_duration + 0.08,
                        spec, MODE_SPEED_TORQUE_RUN,
                    )
                    capture, samples = scope.dump_scope(port, 25.0)
                    if capture["divider"] != SCOPE_DIVIDER:
                        raise RuntimeError(
                            "scope divider mismatch: "
                            f"requested {SCOPE_DIVIDER}, "
                            f"received {capture['divider']}"
                        )
                    sample_period = (
                        capture["divider"] / capture["base_sample_hz"]
                    )
                    path = scope_dir / (
                        f"{candidate['name']}_{bias_rpm:+g}rpm_"
                        f"{frequency_hz:g}hz.csv"
                    )
                    scope.write_csv(path, samples, sample_period)
                    record = analyze_capture(
                        samples, frequency_hz, sample_period, pole_pairs,
                    )
                    add_control_cost(record, samples, pole_pairs)
                    record["scope_divider"] = capture["divider"]
                    record["scope_csv"] = str(path.resolve())
                    visit["records"].append(record)
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN,
                    )

                visit["frequency_summary"] = summarize_baseline(
                    visit["records"]
                )
                result["visits"].append(visit)

            for candidate in candidates:
                visits = [
                    visit for visit in result["visits"]
                    if visit["candidate"] == candidate["name"]
                ]
                result["candidate_summaries"][candidate["name"]] = (
                    summarize_candidate(candidate, visits, iq_limit_a)
                )
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException, ValueError) as error:
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
                    proto.send_command(
                        port, f"PI SPEED {DEFAULT_KP:.9g} {DEFAULT_KI:.9g}",
                    )
                    proto.send_command(port, "PI CLEAR")
                    result["default_pi_restored"] = True
                except serial.SerialException:
                    pass
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["default_pi_restored"]:
        raise RuntimeError("source-default speed PI was not restored")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "qualified_candidates": [
            name for name, summary in result["candidate_summaries"].items()
            if summary["qualified_for_later_performance_test"]
        ],
        "final_mode": result["final_mode"],
        "default_pi_restored": result["default_pi_restored"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
