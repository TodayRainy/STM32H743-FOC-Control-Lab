#!/usr/bin/env python3
"""Probe the 40-150 Hz speed plant with two already-stable PI controllers."""

from __future__ import annotations

import argparse
import cmath
import json
import math
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import wait_mode
from stage2_current_frequency_response import fit_phasor
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
from stage2_speed_pi_design import from_db_phase
from stage2_speed_pi_frequency_validation import (
    DEFAULT_DESIGN,
    load_candidates,
)


CONTROLLER_NAMES = ("bw20_pm65", "bw40_pm65")
SPEED_BIASES_RPM = (1500.0, 3000.0, -3000.0, -1500.0)
SINE_AMPLITUDES_RPM = (100.0, 300.0)
FREQUENCIES_HZ = (40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 120.0, 150.0)
SCOPE_DIVIDER = 10
REFERENCE_AMPLITUDE_TOLERANCE_FRACTION = 0.02
MINIMUM_MEASURED_SINE_SNR_DB = 6.0
MAXIMUM_PLANT_MAGNITUDE_PAIR_DIFFERENCE_DB = 2.0
MAXIMUM_PLANT_PHASE_PAIR_DIFFERENCE_DEG = 10.0
GREEN_MAXIMUM_PI_MARGIN_DEG = 65.0
YELLOW_MAXIMUM_PI_MARGIN_DEG = 50.0


def selected_controllers(design_path: Path) -> list[dict]:
    by_name = {
        item["name"]: item for item in load_candidates(design_path)
    }
    missing = [name for name in CONTROLLER_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"D-041 controller candidates are missing: {missing}")
    return [by_name[name] for name in CONTROLLER_NAMES]


def visit_order(controllers: list[dict]) -> list[dict]:
    by_name = {item["name"]: item for item in controllers}
    forward = SPEED_BIASES_RPM
    reverse = tuple(reversed(SPEED_BIASES_RPM))
    visits = []
    for name, biases in (
        (CONTROLLER_NAMES[0], forward),
        (CONTROLLER_NAMES[1], reverse),
    ):
        for bias in biases:
            visits.append({
                "controller": by_name[name],
                "bias_rpm": bias,
                "amplitude_rpm": SINE_AMPLITUDES_RPM[0],
            })
    for name, biases in (
        (CONTROLLER_NAMES[1], forward),
        (CONTROLLER_NAMES[0], reverse),
    ):
        for bias in biases:
            visits.append({
                "controller": by_name[name],
                "bias_rpm": bias,
                "amplitude_rpm": SINE_AMPLITUDES_RPM[1],
            })
    return visits


def record_key(controller: str, bias_rpm: float, amplitude_rpm: float,
               frequency_hz: float) -> str:
    return (
        f"{controller}|{bias_rpm:+g}|{amplitude_rpm:g}|{frequency_hz:g}"
    )


def plan(controllers: list[dict]) -> dict:
    visits = visit_order(controllers)
    return {
        "schema": "stage2-speed-high-bandwidth-probe-plan-v1",
        "purpose": (
            "measure the 40-150 Hz plant boundary with stable controllers; "
            "do not test an unqualified 100/150 Hz PI"
        ),
        "controllers": [
            {"name": item["name"], "kp": item["kp"], "ki": item["ki"]}
            for item in controllers
        ],
        "visits": [
            {
                "controller": item["controller"]["name"],
                "bias_rpm": item["bias_rpm"],
                "amplitude_rpm": item["amplitude_rpm"],
            }
            for item in visits
        ],
        "speed_biases_rpm": list(SPEED_BIASES_RPM),
        "sine_amplitudes_rpm": list(SINE_AMPLITUDES_RPM),
        "frequencies_hz": list(FREQUENCIES_HZ),
        "scope_divider": SCOPE_DIVIDER,
        "scope_capture_count": len(visits) * len(FREQUENCIES_HZ),
        "maximum_commanded_speed_rpm": (
            max(abs(value) for value in SPEED_BIASES_RPM)
            + max(SINE_AMPLITUDES_RPM)
        ),
        "measurement_checks": {
            "reference_amplitude_tolerance_fraction": (
                REFERENCE_AMPLITUDE_TOLERANCE_FRACTION
            ),
            "minimum_measured_sine_snr_db": MINIMUM_MEASURED_SINE_SNR_DB,
            "maximum_plant_pair_magnitude_difference_db": (
                MAXIMUM_PLANT_MAGNITUDE_PAIR_DIFFERENCE_DB
            ),
            "maximum_plant_pair_phase_difference_deg": (
                MAXIMUM_PLANT_PHASE_PAIR_DIFFERENCE_DEG
            ),
        },
        "feasibility_interpretation": {
            "green": (
                "maximum PI margin >=65 deg; about 55 deg remains if the PI "
                "adds 10 deg lag"
            ),
            "yellow": "maximum PI margin 50-65 deg; boundary or lead compensation",
            "red": "maximum PI margin <50 deg; ordinary PI is not the next step",
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


def phase_difference_deg(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def negative_feedback_phase_deg(value: complex) -> float:
    phase_deg = math.degrees(cmath.phase(value))
    if phase_deg > 0.0:
        phase_deg -= 360.0
    return phase_deg


def sine_fit_quality(values: list[float], frequency_hz: float,
                     sample_period: float) -> dict:
    phasor, offset = fit_phasor(values, frequency_hz, sample_period)
    residual_sum = 0.0
    for index, value in enumerate(values):
        angle = 2.0 * math.pi * frequency_hz * index * sample_period
        fitted = (
            phasor.real * math.cos(angle)
            - phasor.imag * math.sin(angle)
            + offset
        )
        residual_sum += (value - fitted) ** 2
    residual_rms = math.sqrt(residual_sum / len(values))
    sine_rms = abs(phasor) / math.sqrt(2.0)
    snr_db = 20.0 * math.log10(
        max(sine_rms, 1.0e-12) / max(residual_rms, 1.0e-12)
    )
    return {
        "amplitude": abs(phasor),
        "offset": offset,
        "residual_rms": residual_rms,
        "sine_snr_db": snr_db,
    }


def add_plant_record(record: dict, kp: float, ki: float) -> None:
    frequency_hz = float(record["frequency_hz"])
    response = from_db_phase(
        float(record["closed_loop_magnitude_db"]),
        float(record["closed_loop_phase_deg"]),
    )
    if abs(1.0 - response) < 1.0e-6:
        raise RuntimeError("closed-loop response is too close to unity for inversion")
    loop = response / (1.0 - response)
    controller = kp + ki / (1j * 2.0 * math.pi * frequency_hz)
    plant = loop / controller
    phase_deg = negative_feedback_phase_deg(plant)
    record.update({
        "open_loop_magnitude_db": 20.0 * math.log10(abs(loop)),
        "open_loop_phase_deg": math.degrees(cmath.phase(loop)),
        "plant_magnitude_db": 20.0 * math.log10(abs(plant)),
        "plant_phase_deg": phase_deg,
        "maximum_possible_pi_phase_margin_deg": 180.0 + phase_deg,
    })


def pair_differences(records: list[dict], varying_field: str,
                     fixed_fields: tuple[str, ...]) -> list[dict]:
    groups: dict[tuple, list[dict]] = {}
    for record in records:
        key = tuple(record[field] for field in fixed_fields)
        groups.setdefault(key, []).append(record)
    pairs = []
    for key, items in groups.items():
        by_value = {item[varying_field]: item for item in items}
        if len(by_value) != 2:
            continue
        first, second = [by_value[value] for value in sorted(by_value)]
        pairs.append({
            "fixed": dict(zip(fixed_fields, key)),
            "first": first[varying_field],
            "second": second[varying_field],
            "plant_magnitude_difference_db": abs(
                first["plant_magnitude_db"] - second["plant_magnitude_db"]
            ),
            "plant_phase_difference_deg": phase_difference_deg(
                first["plant_phase_deg"], second["plant_phase_deg"]
            ),
        })
    return pairs


def bias_pair_differences(records: list[dict]) -> list[dict]:
    pairs = []
    by_fixed: dict[tuple, dict[float, dict]] = {}
    for record in records:
        key = (
            record["controller"], record["amplitude_rpm"],
            record["frequency_hz"],
        )
        by_fixed.setdefault(key, {})[record["bias_rpm"]] = record
    comparisons = (
        (1500.0, -1500.0, "direction_1500"),
        (3000.0, -3000.0, "direction_3000"),
        (1500.0, 3000.0, "positive_bias"),
        (-1500.0, -3000.0, "negative_bias"),
    )
    for key, by_bias in by_fixed.items():
        for first_bias, second_bias, kind in comparisons:
            if first_bias not in by_bias or second_bias not in by_bias:
                continue
            first = by_bias[first_bias]
            second = by_bias[second_bias]
            pairs.append({
                "fixed": {
                    "controller": key[0],
                    "amplitude_rpm": key[1],
                    "frequency_hz": key[2],
                },
                "kind": kind,
                "first_bias_rpm": first_bias,
                "second_bias_rpm": second_bias,
                "plant_magnitude_difference_db": abs(
                    first["plant_magnitude_db"]
                    - second["plant_magnitude_db"]
                ),
                "plant_phase_difference_deg": phase_difference_deg(
                    first["plant_phase_deg"], second["plant_phase_deg"]
                ),
            })
    return pairs


def summarize(records: list[dict]) -> dict:
    if not records:
        return {}
    amplitude_pairs = pair_differences(
        records,
        "amplitude_rpm",
        ("controller", "bias_rpm", "frequency_hz"),
    )
    controller_pairs = pair_differences(
        records,
        "controller",
        ("bias_rpm", "amplitude_rpm", "frequency_hz"),
    )
    bias_pairs = bias_pair_differences(records)
    frequencies = []
    for frequency_hz in FREQUENCIES_HZ:
        points = [
            item for item in records if item["frequency_hz"] == frequency_hz
        ]
        amp_pairs = [
            item for item in amplitude_pairs
            if item["fixed"]["frequency_hz"] == frequency_hz
        ]
        ctrl_pairs = [
            item for item in controller_pairs
            if item["fixed"]["frequency_hz"] == frequency_hz
        ]
        speed_pairs = [
            item for item in bias_pairs
            if item["fixed"]["frequency_hz"] == frequency_hz
        ]
        maximum_reference_error = max(
            abs(item["reference_amplitude_rpm"] - item["amplitude_rpm"])
            / item["amplitude_rpm"]
            for item in points
        )
        minimum_snr = min(
            item["measured_speed_fit"]["sine_snr_db"] for item in points
        )
        maximum_pair_magnitude = max(
            [item["plant_magnitude_difference_db"]
             for item in amp_pairs + ctrl_pairs + speed_pairs],
            default=math.inf,
        )
        maximum_pair_phase = max(
            [item["plant_phase_difference_deg"]
             for item in amp_pairs + ctrl_pairs + speed_pairs],
            default=math.inf,
        )
        worst_maximum_pi_margin = min(
            item["maximum_possible_pi_phase_margin_deg"] for item in points
        )
        measurement_valid = (
            maximum_reference_error <= REFERENCE_AMPLITUDE_TOLERANCE_FRACTION
            and minimum_snr >= MINIMUM_MEASURED_SINE_SNR_DB
        )
        plant_repeatable = (
            maximum_pair_magnitude
            <= MAXIMUM_PLANT_MAGNITUDE_PAIR_DIFFERENCE_DB
            and maximum_pair_phase
            <= MAXIMUM_PLANT_PHASE_PAIR_DIFFERENCE_DEG
        )
        if not measurement_valid:
            region = "measurement_invalid"
        elif not plant_repeatable:
            region = "plant_not_repeatable"
        elif worst_maximum_pi_margin >= GREEN_MAXIMUM_PI_MARGIN_DEG:
            region = "green"
        elif worst_maximum_pi_margin >= YELLOW_MAXIMUM_PI_MARGIN_DEG:
            region = "yellow"
        else:
            region = "red"
        frequencies.append({
            "frequency_hz": frequency_hz,
            "record_count": len(points),
            "maximum_reference_amplitude_error_fraction": (
                maximum_reference_error
            ),
            "minimum_measured_speed_sine_snr_db": minimum_snr,
            "plant_magnitude_range_db": [
                min(item["plant_magnitude_db"] for item in points),
                max(item["plant_magnitude_db"] for item in points),
            ],
            "plant_phase_range_deg": [
                min(item["plant_phase_deg"] for item in points),
                max(item["plant_phase_deg"] for item in points),
            ],
            "maximum_cross_condition_plant_magnitude_difference_db": (
                maximum_pair_magnitude
            ),
            "maximum_cross_condition_plant_phase_difference_deg": (
                maximum_pair_phase
            ),
            "worst_maximum_possible_pi_phase_margin_deg": (
                worst_maximum_pi_margin
            ),
            "measurement_valid": measurement_valid,
            "plant_repeatable": plant_repeatable,
            "region": region,
        })
    return {
        "frequency_summary": frequencies,
        "amplitude_pair_count": len(amplitude_pairs),
        "controller_pair_count": len(controller_pairs),
        "bias_pair_count": len(bias_pairs),
        "peak_actual_dq_current_a": max(
            item["raw_peak_dq_current_a"] for item in records
        ),
        "peak_iq_reference_a": max(
            item["peak_iq_reference_a"] for item in records
        ),
        "peak_raw_speed_rpm": max(
            item["raw_peak_speed_rpm"] for item in records
        ),
    }


def save_checkpoint(path: Path, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def restore_defaults(port_name: str, baudrate: int) -> tuple[bool, int | None, str | None]:
    buffer = bytearray()
    try:
        with open_serial_with_retry(port_name, baudrate) as port:
            port.reset_input_buffer()
            wait_mode(port, buffer, MODE_READY, 10.0, 5.0, 4800.0)
            proto.send_command(port, "SPEED EST WIN")
            proto.send_command(
                port, f"PI SPEED {DEFAULT_KP:.9g} {DEFAULT_KI:.9g}",
            )
            proto.send_command(port, "PI CLEAR")
            frame = wait_mode(port, buffer, MODE_READY, 5.0, 5.0, 4800.0)
            return True, int(round(frame[27])), None
    except (RuntimeError, serial.SerialException) as error:
        return False, None, str(error)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--design", type=Path, default=DEFAULT_DESIGN)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()

    if args.output and args.resume:
        parser.error("--output and --resume are mutually exclusive")
    controllers = selected_controllers(args.design)
    experiment_plan = plan(controllers)
    if args.plan_only:
        print(json.dumps(experiment_plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.resume or args.output or (
        Path("tests/results") / f"stage2_speed_high_bandwidth_probe_{stamp}.json"
    )
    if args.resume:
        if not output.exists():
            parser.error(f"resume result does not exist: {output}")
        result = json.loads(output.read_text(encoding="utf-8"))
        if result.get("schema") != "stage2-speed-high-bandwidth-probe-result-v1":
            parser.error("resume result schema does not match D-041")
        if result.get("plan") != experiment_plan:
            parser.error("resume result plan does not match the current D-041 plan")
        result["status"] = "running"
        result["error"] = None
        result["final_mode"] = None
        result["default_pi_restored"] = False
        result["default_restore_error"] = None
        result["summary"] = None
    else:
        result = {
            "schema": "stage2-speed-high-bandwidth-probe-result-v1",
            "design": str(args.design.resolve()),
            "plan": experiment_plan,
            "records": [],
            "summary": None,
            "status": "running",
            "error": None,
            "final_mode": None,
            "default_pi_restored": False,
            "default_restore_error": None,
        }
    scope_dir = output.with_name(f"{output.stem}_scope")
    scope_dir.mkdir(parents=True, exist_ok=True)
    completed = {
        item["record_key"] for item in result["records"]
    }
    if len(completed) != len(result["records"]):
        parser.error("resume result contains duplicate record keys")
    pole_pairs = int(spec["control"]["pole_pairs"])
    iq_limit_a = float(spec["safety"]["speed_iq_reference_limit_a"])
    buffer = bytearray()

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

            for visit in visit_order(controllers):
                controller = visit["controller"]
                bias_rpm = visit["bias_rpm"]
                amplitude_rpm = visit["amplitude_rpm"]
                pending = [
                    frequency for frequency in FREQUENCIES_HZ
                    if record_key(
                        controller["name"], bias_rpm, amplitude_rpm, frequency,
                    ) not in completed
                ]
                if not pending:
                    continue
                proto.send_command(port, "TRACK RAMP 3000")
                proto.send_command(port, "SPEED 0")
                supervised_wait(
                    port, buffer, 1.2, spec, MODE_SPEED_TORQUE_RUN,
                )
                proto.send_command(
                    port,
                    f"PI SPEED {controller['kp']:.9g} {controller['ki']:.9g}",
                )
                proto.send_command(port, "PI CLEAR")
                proto.send_command(port, f"SPEED {bias_rpm:g}")
                supervised_wait(
                    port, buffer, 1.5, spec, MODE_SPEED_TORQUE_RUN,
                )

                for frequency_hz in pending:
                    key = record_key(
                        controller["name"], bias_rpm,
                        amplitude_rpm, frequency_hz,
                    )
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN,
                    )
                    proto.send_command(
                        port,
                        f"SPEED SINE {amplitude_rpm:g} {frequency_hz:g}",
                    )
                    supervised_wait(
                        port, buffer, 0.5, spec, MODE_SPEED_TORQUE_RUN,
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
                        f"{controller['name']}_{bias_rpm:+g}rpm_"
                        f"a{amplitude_rpm:g}_{frequency_hz:g}hz.csv"
                    )
                    scope.write_csv(path, samples, sample_period)
                    record = analyze_capture(
                        samples, frequency_hz, sample_period, pole_pairs,
                    )
                    add_control_cost(record, samples, pole_pairs)
                    record.update({
                        "record_key": key,
                        "controller": controller["name"],
                        "kp": controller["kp"],
                        "ki": controller["ki"],
                        "bias_rpm": bias_rpm,
                        "amplitude_rpm": amplitude_rpm,
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
                    add_plant_record(record, controller["kp"], controller["ki"])
                    result["records"].append(record)
                    completed.add(key)
                    save_checkpoint(output, result)
                    proto.send_command(port, f"SPEED {bias_rpm:g}")
                    supervised_wait(
                        port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN,
                    )

            if len(result["records"]) != experiment_plan["scope_capture_count"]:
                raise RuntimeError(
                    "D-041 ended without all planned records: "
                    f"{len(result['records'])}/"
                    f"{experiment_plan['scope_capture_count']}"
                )
            result["summary"] = summarize(result["records"])
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
        raise RuntimeError(
            "source-default speed PI was not restored: "
            f"{result['default_restore_error']}"
        )
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "D-041 hardware probe failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "records": len(result["records"]),
        "frequency_summary": result["summary"]["frequency_summary"],
        "final_mode": result["final_mode"],
        "default_pi_restored": result["default_pi_restored"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
