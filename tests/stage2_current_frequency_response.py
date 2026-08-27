#!/usr/bin/env python3
"""Closed-loop d/q-axis current frequency-response experiment for Stage 2."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import MODE_READY, MODE_TORQUE_RUN, wait_mode
from stage2_current_loop_design import bandwidth_profile_name, build_controllers


DEFAULT_BANDWIDTHS_HZ = (1500.0, 1750.0, 2000.0, 2250.0)
DEFAULT_FREQUENCIES_HZ = (
    100.0, 150.0, 220.0, 330.0, 470.0, 680.0, 1000.0, 1250.0,
    1500.0, 1750.0, 2000.0, 2250.0, 2500.0, 3000.0, 3500.0, 4000.0,
)
SCOPE_SAMPLES = 4096
FOC_SAMPLE_HZ = 40000.0


def set_profile(port, axes):
    proto.send_command(port, f"PI IQ {axes['q']['kp']:.9g} {axes['q']['ki']:.9g}")
    proto.send_command(port, f"PI ID {axes['d']['kp']:.9g} {axes['d']['ki']:.9g}")
    proto.send_command(port, "PI CLEAR")


def solve_three_by_three(matrix, vector):
    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-12:
            raise RuntimeError("singular sinusoid fit")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            scale = augmented[row][column]
            augmented[row] = [
                value - scale * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][3] for row in range(3)]


def fit_phasor(values, frequency_hz, sample_period):
    cosines = []
    sines = []
    for index in range(len(values)):
        angle = 2.0 * math.pi * frequency_hz * index * sample_period
        cosines.append(math.cos(angle))
        sines.append(math.sin(angle))

    matrix = (
        (sum(value * value for value in cosines),
         sum(cosine * sine for cosine, sine in zip(cosines, sines)),
         sum(cosines)),
        (sum(cosine * sine for cosine, sine in zip(cosines, sines)),
         sum(value * value for value in sines),
         sum(sines)),
        (sum(cosines), sum(sines), float(len(values))),
    )
    vector = (
        sum(value * cosine for value, cosine in zip(values, cosines)),
        sum(value * sine for value, sine in zip(values, sines)),
        sum(values),
    )
    cosine_coefficient, sine_coefficient, offset = solve_three_by_three(
        matrix, vector
    )
    return complex(cosine_coefficient, -sine_coefficient), offset


def magnitude_db(value):
    return 20.0 * math.log10(max(abs(value), 1.0e-12))


def phase_deg(value):
    return math.degrees(math.atan2(value.imag, value.real))


def analyze_capture(samples, frequency_hz, sample_period, axis="d"):
    if axis == "d":
        measured_values = [sample[3] for sample in samples]
        reference_values = [sample[5] for sample in samples]
        cross_values = [sample[2] for sample in samples]
        voltage_values = [sample[7] for sample in samples]
    elif axis == "q":
        measured_values = [sample[2] for sample in samples]
        reference_values = [sample[4] for sample in samples]
        cross_values = [sample[3] for sample in samples]
        voltage_values = [sample[6] for sample in samples]
    else:
        raise ValueError(f"unsupported current axis: {axis}")
    reference, reference_offset = fit_phasor(
        reference_values, frequency_hz, sample_period
    )
    measured, measured_offset = fit_phasor(
        measured_values, frequency_hz, sample_period
    )
    cross_axis, _ = fit_phasor(cross_values, frequency_hz, sample_period)
    voltage, voltage_offset = fit_phasor(
        voltage_values, frequency_hz, sample_period
    )
    if abs(reference) < 0.02:
        raise RuntimeError(f"recorded {axis}-axis sine reference is too small")

    closed_loop = measured / reference
    sensitivity = 1.0 - closed_loop
    denominator = 1.0 - closed_loop
    open_loop = closed_loop / denominator if abs(denominator) > 1.0e-6 else None
    plant = measured / voltage if abs(voltage) > 1.0e-6 else None
    return {
        "axis": axis,
        "frequency_hz": frequency_hz,
        "reference_amplitude_a": abs(reference),
        "reference_offset_a": reference_offset,
        "measured_amplitude_a": abs(measured),
        "measured_offset_a": measured_offset,
        "closed_loop_magnitude_db": magnitude_db(closed_loop),
        "closed_loop_phase_deg": phase_deg(closed_loop),
        "sensitivity_magnitude": abs(sensitivity),
        "sensitivity_magnitude_db": magnitude_db(sensitivity),
        "open_loop_magnitude_db": None if open_loop is None else magnitude_db(open_loop),
        "open_loop_phase_deg": None if open_loop is None else phase_deg(open_loop),
        "cross_axis_ratio_db": magnitude_db(cross_axis / reference),
        "voltage_amplitude_v": abs(voltage),
        "voltage_offset_v": voltage_offset,
        "plant_magnitude_db_a_per_v": None if plant is None else magnitude_db(plant),
        "plant_phase_deg": None if plant is None else phase_deg(plant),
        "raw_peak_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "raw_peak_voltage_v": max(math.hypot(sample[6], sample[7]) for sample in samples),
    }


def unwrap_degrees(values):
    unwrapped = []
    for value in values:
        if not unwrapped:
            unwrapped.append(value)
            continue
        while value - unwrapped[-1] > 180.0:
            value -= 360.0
        while value - unwrapped[-1] < -180.0:
            value += 360.0
        unwrapped.append(value)
    return unwrapped


def interpolate_log_frequency(first, second, first_value, second_value, target):
    if second_value == first_value:
        fraction = 0.0
    else:
        fraction = (target - first_value) / (second_value - first_value)
    fraction = max(0.0, min(1.0, fraction))
    return math.exp(math.log(first) + fraction * (math.log(second) - math.log(first)))


def crossing(points, value_key, target):
    for first, second in zip(points, points[1:]):
        first_value = first[value_key]
        second_value = second[value_key]
        if first_value is None or second_value is None:
            continue
        if (first_value - target) * (second_value - target) <= 0.0:
            frequency = interpolate_log_frequency(
                first["frequency_hz"], second["frequency_hz"],
                first_value, second_value, target,
            )
            fraction = ((math.log(frequency) - math.log(first["frequency_hz"]))
                        / (math.log(second["frequency_hz"])
                           - math.log(first["frequency_hz"])))
            return first, second, frequency, fraction
    return None


def summarize_profile(points):
    points = sorted(points, key=lambda point: point["frequency_hz"])
    loop_phases = unwrap_degrees([point["open_loop_phase_deg"] for point in points])
    for point, unwrapped in zip(points, loop_phases):
        point["open_loop_phase_unwrapped_deg"] = unwrapped

    loop_crossing = crossing(points, "open_loop_magnitude_db", 0.0)
    phase_crossing = crossing(points, "open_loop_phase_unwrapped_deg", -180.0)

    phase_margin = None
    loop_crossover_hz = None
    if loop_crossing is not None:
        first, second, loop_crossover_hz, fraction = loop_crossing
        loop_phase = (first["open_loop_phase_unwrapped_deg"]
                      + fraction * (second["open_loop_phase_unwrapped_deg"]
                                    - first["open_loop_phase_unwrapped_deg"]))
        phase_margin = 180.0 + loop_phase

    # Define -3 dB relative to the measured low-frequency tracking plateau.
    # Points below one quarter of crossover are safely inside the pass band;
    # keep at least two points so one noisy sample cannot define the plateau.
    plateau_points = []
    if loop_crossover_hz is not None:
        plateau_points = [
            point for point in points
            if point["frequency_hz"] <= 0.25 * loop_crossover_hz
        ]
    if len(plateau_points) < 2:
        plateau_points = points[:2]
    low_gain_db = sum(
        point["closed_loop_magnitude_db"] for point in plateau_points
    ) / len(plateau_points)
    bandwidth_crossing = crossing(
        points, "closed_loop_magnitude_db", low_gain_db - 3.0103
    )

    gain_margin_db = None
    phase_crossover_hz = None
    if phase_crossing is not None:
        first, second, phase_crossover_hz, fraction = phase_crossing
        loop_gain_db = (first["open_loop_magnitude_db"]
                        + fraction * (second["open_loop_magnitude_db"]
                                      - first["open_loop_magnitude_db"]))
        gain_margin_db = -loop_gain_db

    return {
        "low_frequency_closed_loop_gain_db": low_gain_db,
        "low_frequency_plateau_points_hz": [
            point["frequency_hz"] for point in plateau_points
        ],
        "closed_loop_bandwidth_hz": (
            None if bandwidth_crossing is None else bandwidth_crossing[2]
        ),
        "loop_crossover_hz": loop_crossover_hz,
        "phase_margin_deg": phase_margin,
        "phase_crossover_hz": phase_crossover_hz,
        "gain_margin_db": gain_margin_db,
        "max_cross_axis_ratio_db": max(point["cross_axis_ratio_db"] for point in points),
        "raw_peak_current_a": max(point["raw_peak_current_a"] for point in points),
        "raw_peak_rpm": max(point["raw_peak_rpm"] for point in points),
        "raw_peak_voltage_v": max(point["raw_peak_voltage_v"] for point in points),
        "max_observed_sensitivity": max(
            point["sensitivity_magnitude"] for point in points
        ),
        "passes_phase_margin_60deg": phase_margin is not None and phase_margin >= 60.0,
        "gain_margin_status": (
            "measured" if gain_margin_db is not None
            else "phase did not reach -180 deg within the measured sweep"
        ),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Measure d/q-axis closed-loop current frequency response",
    )
    parser.add_argument("--port")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--bandwidths", type=float, nargs="+",
                        default=DEFAULT_BANDWIDTHS_HZ, metavar="HZ")
    parser.add_argument("--exclude-active", action="store_true")
    parser.add_argument("--frequencies", type=float, nargs="+",
                        default=DEFAULT_FREQUENCIES_HZ, metavar="HZ")
    parser.add_argument("--amplitude", type=float, default=0.5)
    parser.add_argument("--axis", choices=("d", "q"), default="d")
    parser.add_argument("--settle-cycles", type=float, default=5.0)
    parser.add_argument("--max-current", type=float, default=2.0)
    parser.add_argument("--max-rpm", type=float, default=500.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--results-dir", type=Path, default=Path("tests/results"))
    args = parser.parse_args()

    bandwidths = tuple(sorted(args.bandwidths))
    frequencies = tuple(sorted(args.frequencies))
    if len(set(bandwidths)) != len(bandwidths):
        raise ValueError("bandwidth candidates must be unique")
    if any(value < 500.0 or value > 2500.0 for value in bandwidths):
        raise ValueError("candidate bandwidths must stay within 500 to 2500 Hz")
    if len(set(frequencies)) != len(frequencies):
        raise ValueError("sweep frequencies must be unique")
    if any(value < 50.0 or value > 4000.0 for value in frequencies):
        raise ValueError("sweep frequencies must stay within 50 to 4000 Hz")
    if args.amplitude <= 0.0 or args.amplitude > 0.5:
        raise ValueError("current sine amplitude must be within 0 to 0.5 A")
    if not args.plan_only and not args.port:
        parser.error("--port is required unless --plan-only is used")

    controllers = build_controllers(bandwidths)
    profiles = [bandwidth_profile_name(value) for value in bandwidths]
    if not args.exclude_active:
        profiles.insert(0, "active_legacy")
    if args.plan_only:
        print(json.dumps({
            "profiles": profiles,
            "controllers": {name: controllers[name] for name in profiles},
            "frequencies_hz": frequencies,
            "axis": args.axis,
            "amplitude_a": args.amplitude,
            "scope_captures": len(profiles) * len(frequencies),
            "scope_duration_sec": SCOPE_SAMPLES / FOC_SAMPLE_HZ,
            "method": ("least-squares synchronous phasor fit; "
                       f"T=I{args.axis}/I{args.axis}_ref; L=T/(1-T)"),
            "selection": ("relative -3 dB tracking bandwidth; phase margin >=60 deg; "
                          "gain margin reported only if -180 deg is observed"),
        }, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.results_dir.mkdir(parents=True, exist_ok=True)
    output = args.results_dir / f"stage2_current_frf_{stamp}.json"
    records = []
    final_mode = None
    measured_bus_voltage = None
    error = None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            buffer = bytearray()
            port.reset_input_buffer()
            try:
                ready = wait_mode(port, buffer, MODE_READY, args.timeout, 5.0, args.max_rpm)
                measured_bus_voltage = ready[29]
                if not 18.0 <= measured_bus_voltage <= 28.0:
                    raise RuntimeError(
                        f"measured bus voltage is out of range: {measured_bus_voltage:.3f} V"
                    )
                proto.send_command(port, "FF OFF")
                proto.send_command(port, "START CURRENT")
                wait_mode(port, buffer, MODE_TORQUE_RUN, args.timeout, 5.0, args.max_rpm)
                proto.send_command(port, "CURRENT 0 0")
                time.sleep(0.25)

                for profile in profiles:
                    axes = controllers[profile]
                    set_profile(port, axes)
                    proto.send_command(port, "CURRENT 0 0")
                    time.sleep(0.1)
                    for frequency_hz in frequencies:
                        proto.send_command(
                            port, f"CURRENT SINE {args.axis.upper()} "
                                  f"{args.amplitude:g} {frequency_hz:g}"
                        )
                        time.sleep(max(0.02, args.settle_cycles / frequency_hz))
                        scope.arm_scope(port, 1)
                        time.sleep(SCOPE_SAMPLES / FOC_SAMPLE_HZ + 0.03)
                        proto.send_command(port, "CURRENT SINE OFF")
                        capture, samples = scope.dump_scope(port, args.timeout)
                        sample_period = capture["divider"] / float(capture["base_sample_hz"])
                        csv_path = args.results_dir / (
                            f"stage2_frf_{stamp}_{args.axis}_{profile}_"
                            f"{frequency_hz:g}hz.csv"
                        )
                        scope.write_csv(csv_path, samples, sample_period)
                        result = analyze_capture(
                            samples, frequency_hz, sample_period, args.axis
                        )
                        peak_rpm = max(abs(sample[9]) for sample in samples) \
                                   * 60.0 / (2.0 * math.pi * 10.0)
                        result.update({
                            "profile": profile,
                            "bandwidth_hz": next((value for value in bandwidths
                                                  if bandwidth_profile_name(value)
                                                  == profile), None),
                            "raw_peak_rpm": peak_rpm,
                            "csv": str(csv_path.resolve()),
                        })
                        records.append(result)
                        if not 0.9 * args.amplitude <= result["reference_amplitude_a"] \
                                <= 1.1 * args.amplitude:
                            raise RuntimeError(
                                f"recorded sine amplitude mismatch: "
                                f"{result['reference_amplitude_a']:.3f} A"
                            )
                        if result["raw_peak_current_a"] >= args.max_current:
                            raise RuntimeError(
                                f"scope current limit exceeded: "
                                f"{result['raw_peak_current_a']:.3f} A"
                            )
                        if peak_rpm >= args.max_rpm:
                            raise RuntimeError(
                                f"scope speed limit exceeded: {peak_rpm:.1f} rpm"
                            )
                        wait_mode(port, buffer, MODE_TORQUE_RUN, 2.0,
                                  args.max_current, args.max_rpm)
                        proto.send_command(port, "PI CLEAR")
                        time.sleep(0.03)
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    proto.send_command(port, "CURRENT SINE OFF")
                    proto.send_command(port, "CURRENT 0 0")
                    set_profile(port, controllers["active_legacy"])
                    proto.send_command(port, "STOP")
                    final = wait_mode(port, buffer, MODE_READY, 2.0, 5.0, args.max_rpm)
                    final_mode = int(round(final[27]))
                except Exception as cleanup_exc:
                    cleanup_error = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    error = cleanup_error if error is None else f"{error}; cleanup: {cleanup_error}"
    except Exception as exc:
        if error is None:
            error = f"{type(exc).__name__}: {exc}"

    profile_summaries = {
        profile: summarize_profile([record for record in records
                                    if record["profile"] == profile])
        for profile in profiles
        if len([record for record in records if record["profile"] == profile]) >= 2
    }
    output.write_text(json.dumps({
        "profiles": profiles,
        "frequencies_hz": frequencies,
        "amplitude_a": args.amplitude,
        "axis": args.axis,
        "measured_bus_voltage_v": measured_bus_voltage,
        "method": ("least-squares synchronous phasor fit; "
                   f"T=I{args.axis}/I{args.axis}_ref; L=T/(1-T)"),
        "records": records,
        "profile_summaries": profile_summaries,
        "error": error,
        "final_mode": final_mode,
    }, indent=2) + "\n", encoding="utf-8")

    if error is not None:
        raise RuntimeError(f"experiment failed; partial result saved to {output}: {error}")
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    print(json.dumps({
        "result": str(output.resolve()),
        "records": len(records),
        "profile_summaries": profile_summaries,
        "final_mode": final_mode,
    }, indent=2))


if __name__ == "__main__":
    main()
