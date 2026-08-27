#!/usr/bin/env python3
"""Qualify the six D-042 high-bandwidth speed PI candidates on hardware."""

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
from stage2_speed_baseline import DEFAULT_KI, DEFAULT_KP, add_control_cost
from stage2_speed_frequency_response import analyze_capture
from stage2_speed_high_bandwidth_probe import (
    add_plant_record,
    record_key,
    restore_defaults,
    save_checkpoint,
    sine_fit_quality,
)
from stage2_speed_high_bandwidth_pi_design import DEFAULT_OUTPUT as DEFAULT_DESIGN
from stage2_speed_pi_frequency_validation import (
    DEFAULT_DESIGN as DEFAULT_ANCHOR_DESIGN,
    load_candidates as load_anchor_candidates,
)


ANCHOR_NAMES = ("bw20_pm65", "bw40_pm65")
SPEED_BIASES_RPM = (1500.0, 3000.0, -3000.0, -1500.0)
SINE_AMPLITUDE_RPM = 200.0
FREQUENCIES_HZ = (
    4.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0,
    35.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0,
)
SCOPE_DIVIDER = 10
REFERENCE_AMPLITUDE_TOLERANCE_FRACTION = 0.02
MINIMUM_CROSSING_POINT_SNR_DB = 6.0
MAXIMUM_CROSSOVER_TARGET_ERROR_FRACTION = 0.20
MINIMUM_PHASE_MARGIN_DEG = 60.0
MAXIMUM_CLOSED_LOOP_PEAK_DB = 2.0
MAXIMUM_BANDWIDTH_CONSISTENCY_FRACTION = 0.15
IQ_SATURATION_FRACTION = 0.95


def load_controllers(anchor_path: Path, design_path: Path) -> list[dict]:
    anchors_by_name = {
        item["name"]: item for item in load_anchor_candidates(anchor_path)
    }
    anchors = [
        {
            "name": name,
            "role": "anchor",
            "kp": float(anchors_by_name[name]["kp"]),
            "ki": float(anchors_by_name[name]["ki"]),
            "target_gain_crossover_hz": None,
        }
        for name in ANCHOR_NAMES
    ]
    design = json.loads(design_path.read_text(encoding="utf-8"))
    if design.get("schema") != "stage2-speed-high-bandwidth-pi-design-v1":
        raise ValueError("unsupported D-042 PI design schema")
    if design.get("status") != "six_candidates_preflight_passed":
        raise ValueError("D-042 candidates did not pass offline preflight")
    candidates = []
    for item in design["candidates"]:
        if not item.get("preflight_pass"):
            raise ValueError(f"candidate {item['name']} failed preflight")
        candidates.append({
            "name": item["name"],
            "role": "candidate",
            "kp": float(item["kp"]),
            "ki": float(item["ki"]),
            "target_gain_crossover_hz": float(
                item["target_gain_crossover_hz"]
            ),
        })
    if len(candidates) != 6:
        raise ValueError("D-042 requires six preflight candidates")
    return anchors + candidates


def visit_order(controllers: list[dict]) -> list[tuple[dict, float]]:
    forward = list(controllers)
    reverse = list(reversed(controllers))
    rotated = forward[4:] + forward[:4]
    return (
        [(item, 1500.0) for item in forward]
        + [(item, 3000.0) for item in reverse]
        + [(item, -3000.0) for item in rotated]
        + [(item, -1500.0) for item in reversed(rotated)]
    )


def plan(controllers: list[dict]) -> dict:
    visits = visit_order(controllers)
    return {
        "schema": "stage2-speed-high-bandwidth-pi-validation-plan-v1",
        "purpose": (
            "measure actual crossover, phase margin and closed-loop bandwidth; "
            "do not trust the non-monotonic model to name the final bandwidth"
        ),
        "controllers": controllers,
        "visits": [
            {"controller": item["name"], "bias_rpm": bias}
            for item, bias in visits
        ],
        "speed_biases_rpm": list(SPEED_BIASES_RPM),
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "frequencies_hz": list(FREQUENCIES_HZ),
        "scope_divider": SCOPE_DIVIDER,
        "scope_capture_count": len(visits) * len(FREQUENCIES_HZ),
        "maximum_commanded_speed_rpm": (
            max(abs(value) for value in SPEED_BIASES_RPM)
            + SINE_AMPLITUDE_RPM
        ),
        "qualification": {
            "maximum_crossover_target_error_fraction": (
                MAXIMUM_CROSSOVER_TARGET_ERROR_FRACTION
            ),
            "minimum_phase_margin_deg": MINIMUM_PHASE_MARGIN_DEG,
            "maximum_closed_loop_peak_db": MAXIMUM_CLOSED_LOOP_PEAK_DB,
            "maximum_bandwidth_consistency_fraction": (
                MAXIMUM_BANDWIDTH_CONSISTENCY_FRACTION
            ),
        },
        "measurement_validity": {
            "reference_amplitude_tolerance_fraction": (
                REFERENCE_AMPLITUDE_TOLERANCE_FRACTION
            ),
            "minimum_crossing_point_snr_db": MINIMUM_CROSSING_POINT_SNR_DB,
            "maximum_iq_reference_fraction_of_limit": IQ_SATURATION_FRACTION,
        },
        "speed_estimator": "win_1ms_fc150",
        "feedforward": "off",
        "requires_24v": True,
        "motor_will_rotate": True,
        "automatic_flash": False,
        "supports_checkpoint_resume": True,
        "restore_default_speed_pi": True,
        "finally_stop_and_confirm_ready": True,
    }


def crossing(records: list[dict], magnitude_field: str,
             threshold_db: float) -> dict | None:
    ordered = sorted(records, key=lambda item: item["frequency_hz"])
    previous = ordered[0]
    for current in ordered[1:]:
        if (
            previous[magnitude_field] > threshold_db
            and current[magnitude_field] <= threshold_db
        ):
            denominator = current[magnitude_field] - previous[magnitude_field]
            fraction = (threshold_db - previous[magnitude_field]) / denominator
            frequency_hz = math.exp(
                math.log(previous["frequency_hz"])
                + fraction
                * math.log(current["frequency_hz"] / previous["frequency_hz"])
            )
            minimum_snr = min(
                previous["measured_speed_fit"]["sine_snr_db"],
                current["measured_speed_fit"]["sine_snr_db"],
            )
            result = {
                "frequency_hz": frequency_hz,
                "bracket_hz": [
                    previous["frequency_hz"], current["frequency_hz"]
                ],
                "minimum_bracket_snr_db": minimum_snr,
                "measurement_valid": minimum_snr >= MINIMUM_CROSSING_POINT_SNR_DB,
            }
            if magnitude_field == "open_loop_magnitude_db":
                phases = []
                for item in (previous, current):
                    phase = float(item["open_loop_phase_deg"])
                    phases.append(phase - 360.0 if phase > 0.0 else phase)
                loop_phase = (1.0 - fraction) * phases[0] + fraction * phases[1]
                result["phase_margin_deg"] = 180.0 + loop_phase
            return result
        previous = current
    return None


def relative_range(values: list[float]) -> float:
    mean = sum(values) / len(values)
    return (max(values) - min(values)) / mean


def summarize_controller(controller_info: dict, records: list[dict],
                         iq_limit_a: float) -> dict:
    visits = []
    for bias_rpm in SPEED_BIASES_RPM:
        points = [
            item for item in records
            if item["controller"] == controller_info["name"]
            and item["bias_rpm"] == bias_rpm
        ]
        gain = crossing(points, "open_loop_magnitude_db", 0.0)
        bandwidth = crossing(points, "closed_loop_magnitude_db", -3.0103)
        visits.append({
            "bias_rpm": bias_rpm,
            "record_count": len(points),
            "gain_crossover": gain,
            "closed_loop_bandwidth": bandwidth,
            "maximum_closed_loop_peak_db": max(
                item["closed_loop_magnitude_db"] for item in points
            ),
            "maximum_reference_amplitude_error_fraction": max(
                abs(item["reference_amplitude_rpm"] - SINE_AMPLITUDE_RPM)
                / SINE_AMPLITUDE_RPM
                for item in points
            ),
            "peak_iq_reference_a": max(
                item["peak_iq_reference_a"] for item in points
            ),
        })
    gains = [item["gain_crossover"] for item in visits]
    bandwidths = [item["closed_loop_bandwidth"] for item in visits]
    measurement_valid = (
        all(item["record_count"] == len(FREQUENCIES_HZ) for item in visits)
        and all(item is not None and item["measurement_valid"] for item in gains)
        and all(
            item is not None and item["measurement_valid"]
            for item in bandwidths
        )
        and max(
            item["maximum_reference_amplitude_error_fraction"]
            for item in visits
        ) <= REFERENCE_AMPLITUDE_TOLERANCE_FRACTION
        and max(item["peak_iq_reference_a"] for item in visits)
        < IQ_SATURATION_FRACTION * iq_limit_a
    )
    summary = {
        "name": controller_info["name"],
        "role": controller_info["role"],
        "kp": controller_info["kp"],
        "ki": controller_info["ki"],
        "target_gain_crossover_hz": controller_info[
            "target_gain_crossover_hz"
        ],
        "visits": visits,
        "measurement_valid": measurement_valid,
        "peak_iq_reference_a": max(
            item["peak_iq_reference_a"] for item in visits
        ),
    }
    if not measurement_valid:
        summary.update({
            "minimum_phase_margin_deg": None,
            "maximum_closed_loop_peak_db": None,
            "bandwidth_consistency_fraction": None,
            "gates": None,
            "qualified": False,
        })
        return summary
    gain_frequencies = [item["frequency_hz"] for item in gains]
    bandwidth_frequencies = [item["frequency_hz"] for item in bandwidths]
    minimum_pm = min(item["phase_margin_deg"] for item in gains)
    maximum_peak = max(
        item["maximum_closed_loop_peak_db"] for item in visits
    )
    consistency = relative_range(bandwidth_frequencies)
    if controller_info["role"] == "candidate":
        target = controller_info["target_gain_crossover_hz"]
        crossover_gate = max(
            abs(value - target) / target for value in gain_frequencies
        ) <= MAXIMUM_CROSSOVER_TARGET_ERROR_FRACTION
    else:
        crossover_gate = True
    gates = {
        "gain_crossover_target": crossover_gate,
        "phase_margin": minimum_pm >= MINIMUM_PHASE_MARGIN_DEG,
        "closed_loop_peak": maximum_peak <= MAXIMUM_CLOSED_LOOP_PEAK_DB,
        "bandwidth_consistency": consistency
        <= MAXIMUM_BANDWIDTH_CONSISTENCY_FRACTION,
    }
    summary.update({
        "measured_gain_crossovers_hz": gain_frequencies,
        "measured_closed_loop_bandwidths_hz": bandwidth_frequencies,
        "minimum_measured_closed_loop_bandwidth_hz": min(
            bandwidth_frequencies
        ),
        "maximum_measured_closed_loop_bandwidth_hz": max(
            bandwidth_frequencies
        ),
        "minimum_phase_margin_deg": minimum_pm,
        "maximum_closed_loop_peak_db": maximum_peak,
        "bandwidth_consistency_fraction": consistency,
        "gates": gates,
        "qualified": controller_info["role"] == "candidate" and all(gates.values()),
    })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--anchor-design", type=Path, default=DEFAULT_ANCHOR_DESIGN)
    parser.add_argument("--design", type=Path, default=DEFAULT_DESIGN)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()

    if args.output and args.resume:
        parser.error("--output and --resume are mutually exclusive")
    controllers = load_controllers(args.anchor_design, args.design)
    experiment_plan = plan(controllers)
    if args.plan_only:
        print(json.dumps(experiment_plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.resume or args.output or (
        Path("tests/results")
        / f"stage2_speed_high_bandwidth_pi_validation_{stamp}.json"
    )
    if args.resume:
        if not output.exists():
            parser.error(f"resume result does not exist: {output}")
        result = json.loads(output.read_text(encoding="utf-8"))
        if result.get("plan") != experiment_plan:
            parser.error("resume result plan does not match D-042")
        result.update({
            "status": "running", "error": None, "final_mode": None,
            "default_pi_restored": False, "default_restore_error": None,
            "controller_summaries": None, "qualified_ranking": None,
            "safety_summary": None,
        })
    else:
        result = {
            "schema": "stage2-speed-high-bandwidth-pi-validation-result-v1",
            "plan": experiment_plan,
            "records": [],
            "controller_summaries": None,
            "qualified_ranking": None,
            "safety_summary": None,
            "status": "running",
            "error": None,
            "final_mode": None,
            "default_pi_restored": False,
            "default_restore_error": None,
        }
    completed = {item["record_key"] for item in result["records"]}
    if len(completed) != len(result["records"]):
        parser.error("resume result contains duplicate record keys")
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    pole_pairs = int(spec["control"]["pole_pairs"])
    iq_limit_a = float(spec["safety"]["speed_iq_reference_limit_a"])
    buffer = bytearray()

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            for command in (
                "SPEED EST WIN", f"SPEED LIMIT {iq_limit_a:g}",
                "FF OFF", "TRACK RAMP 3000", "PI CLEAR", "START SPEED",
            ):
                proto.send_command(port, command)
            wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0)
            for controller_info, bias_rpm in visit_order(controllers):
                pending = [
                    frequency_hz for frequency_hz in FREQUENCIES_HZ
                    if record_key(
                        controller_info["name"], bias_rpm,
                        SINE_AMPLITUDE_RPM, frequency_hz,
                    ) not in completed
                ]
                if not pending:
                    continue
                proto.send_command(port, "TRACK RAMP 3000")
                proto.send_command(port, "SPEED 0")
                supervised_wait(port, buffer, 1.2, spec, MODE_SPEED_TORQUE_RUN)
                proto.send_command(
                    port,
                    f"PI SPEED {controller_info['kp']:.9g} "
                    f"{controller_info['ki']:.9g}",
                )
                proto.send_command(port, "PI CLEAR")
                proto.send_command(port, f"SPEED {bias_rpm:g}")
                supervised_wait(port, buffer, 1.5, spec, MODE_SPEED_TORQUE_RUN)
                for frequency_hz in pending:
                    key = record_key(
                        controller_info["name"], bias_rpm,
                        SINE_AMPLITUDE_RPM, frequency_hz,
                    )
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN)
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
                        raise RuntimeError("scope divider mismatch")
                    sample_period = capture["divider"] / capture["base_sample_hz"]
                    path = scope_dir / (
                        f"{controller_info['name']}_{bias_rpm:+g}rpm_"
                        f"a{SINE_AMPLITUDE_RPM:g}_{frequency_hz:g}hz.csv"
                    )
                    scope.write_csv(path, samples, sample_period)
                    record = analyze_capture(
                        samples, frequency_hz, sample_period, pole_pairs,
                    )
                    add_control_cost(record, samples, pole_pairs)
                    record.update({
                        "record_key": key,
                        "controller": controller_info["name"],
                        "role": controller_info["role"],
                        "kp": controller_info["kp"],
                        "ki": controller_info["ki"],
                        "bias_rpm": bias_rpm,
                        "amplitude_rpm": SINE_AMPLITUDE_RPM,
                        "scope_divider": capture["divider"],
                        "scope_csv": str(path.resolve()),
                        "reference_fit": sine_fit_quality(
                            [sample[10] for sample in samples],
                            frequency_hz, sample_period,
                        ),
                        "measured_speed_fit": sine_fit_quality(
                            [sample[9] for sample in samples],
                            frequency_hz, sample_period,
                        ),
                    })
                    add_plant_record(
                        record, controller_info["kp"], controller_info["ki"],
                    )
                    result["records"].append(record)
                    completed.add(key)
                    save_checkpoint(output, result)
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN,
                    )
            if len(result["records"]) != experiment_plan["scope_capture_count"]:
                raise RuntimeError("D-042 ended without all planned records")
            result["controller_summaries"] = [
                summarize_controller(item, result["records"], iq_limit_a)
                for item in controllers
            ]
            result["qualified_ranking"] = [
                item["name"] for item in sorted(
                    (
                        item for item in result["controller_summaries"]
                        if item["qualified"]
                    ),
                    key=lambda item: item[
                        "minimum_measured_closed_loop_bandwidth_hz"
                    ],
                    reverse=True,
                )
            ]
            result["safety_summary"] = {
                "peak_iq_reference_a": max(
                    item["peak_iq_reference_a"] for item in result["records"]
                ),
                "peak_actual_dq_current_a": max(
                    item["raw_peak_dq_current_a"] for item in result["records"]
                ),
                "peak_raw_speed_rpm": max(
                    item["raw_peak_speed_rpm"] for item in result["records"]
                ),
            }
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException, ValueError,
                KeyboardInterrupt) as error:
            result["status"] = "error"
            result["error"] = str(error) or error.__class__.__name__
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer,
            )
            try:
                port.close()
            except serial.SerialException:
                pass
            if result["final_mode"] == MODE_READY:
                restored, restore_mode, restore_error = restore_defaults(
                    args.port, args.baudrate,
                )
                result["default_pi_restored"] = restored
                result["default_restore_error"] = restore_error
                if restore_mode is not None:
                    result["final_mode"] = restore_mode
            save_checkpoint(output, result)

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["default_pi_restored"]:
        raise RuntimeError("source-default speed PI was not restored")
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "D-042 hardware validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "records": len(result["records"]),
        "controller_summaries": result["controller_summaries"],
        "qualified_ranking": result["qualified_ranking"],
        "safety_summary": result["safety_summary"],
        "final_mode": result["final_mode"],
        "default_pi_restored": result["default_pi_restored"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
