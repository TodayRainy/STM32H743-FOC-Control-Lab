#!/usr/bin/env python3
"""Short q-axis pulse A/B validation for a freely rotating motor."""

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
from scope_current_step import MODE_READY, MODE_TORQUE_RUN, first_crossing, median_filter, wait_mode
from stage2_current_loop_design import build_report
from stage2_current_loop_validation import DEFAULT_PROFILES, set_profile


DEFAULT_PULSES_A = (0.5, -0.5, 1.0, -1.0, 2.0, -2.0)


def summarize_q_pulse(samples, target_iq, sample_period):
    iq = [sample[2] for sample in samples]
    iq_filtered = median_filter(iq)
    iq_ref = [sample[4] for sample in samples]
    tolerance = max(0.005, abs(target_iq) * 0.05)
    start = next((index for index, value in enumerate(iq_ref)
                  if abs(value - target_iq) <= tolerance), None)
    if start is None:
        raise RuntimeError("q-axis pulse command is not present in scope data")
    stop = next((index for index in range(start + 1, len(iq_ref))
                 if abs(iq_ref[index]) <= 0.02), None)
    if stop is None or stop - start < 8:
        raise RuntimeError("q-axis pulse was too short or did not return to zero")

    baseline_values = iq_filtered[:start] or [iq_filtered[0]]
    baseline = sum(baseline_values) / len(baseline_values)
    amplitude = target_iq - baseline
    direction = 1.0 if amplitude >= 0.0 else -1.0
    index_10 = first_crossing(iq_filtered, start, baseline + 0.1 * amplitude, direction)
    index_90 = first_crossing(iq_filtered, start, baseline + 0.9 * amplitude, direction)
    if index_10 is not None and index_10 >= stop:
        index_10 = None
    if index_90 is not None and index_90 >= stop:
        index_90 = None
    rise_time = None
    if index_10 is not None and index_90 is not None and index_90 >= index_10:
        rise_time = (index_90 - index_10) * sample_period

    directed_peak = max(direction * (value - baseline)
                        for value in iq_filtered[start:stop])
    overshoot = max(0.0, (directed_peak - abs(amplitude)) / abs(amplitude) * 100.0)
    plateau_count = max(8, round(0.001 / sample_period))
    plateau = iq[max(start, stop - plateau_count):stop]
    peak_rpm = max(abs(sample[9]) for sample in samples) \
               * 60.0 / (2.0 * math.pi * 10.0)

    return {
        "target_iq_a": target_iq,
        "pulse_duration_sec": (stop - start) * sample_period,
        "rise_time_10_90_sec": rise_time,
        "filtered_overshoot_pct": overshoot,
        "plateau_iq_mean_a": sum(plateau) / len(plateau),
        "plateau_error_rms_a": math.sqrt(
            sum((target_iq - value) ** 2 for value in plateau) / len(plateau)
        ),
        "raw_peak_current_a": max(math.hypot(sample[2], sample[3]) for sample in samples),
        "raw_peak_rpm": peak_rpm,
    }


def main():
    parser = argparse.ArgumentParser(description="Compare PI profiles with short q-axis pulses")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--profile", action="append", choices=DEFAULT_PROFILES)
    parser.add_argument("--pulses", type=float, nargs="+", default=DEFAULT_PULSES_A)
    parser.add_argument("--pulse-ms", type=float, default=5.0)
    parser.add_argument("--max-step-current", type=float, default=4.0)
    parser.add_argument("--max-calibration-current", type=float, default=5.0)
    parser.add_argument("--max-rpm", type=float, default=500.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--results-dir", type=Path, default=Path("tests/results"))
    args = parser.parse_args()

    if any(abs(pulse) > 2.0 or abs(pulse) < 0.1 for pulse in args.pulses):
        raise ValueError("q-axis validation pulses must be within 0.1 to 2.0 A")
    if not 2.0 <= args.pulse_ms <= 10.0:
        raise ValueError("q-axis pulse duration must be between 2 and 10 ms")

    controllers = build_report()["controllers"]
    profile_names = args.profile or list(DEFAULT_PROFILES)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.results_dir.mkdir(parents=True, exist_ok=True)
    records = []
    buffer = bytearray()
    final_mode = None

    with serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        ready = wait_mode(port, buffer, MODE_READY, args.timeout,
                          args.max_calibration_current, args.max_rpm)
        if not 18.0 <= ready[29] <= 28.0:
            raise RuntimeError(f"measured bus voltage is out of range: {ready[29]:.3f} V")

        try:
            proto.send_command(port, "FF OFF")
            proto.send_command(port, "START CURRENT")
            wait_mode(port, buffer, MODE_TORQUE_RUN, args.timeout,
                      args.max_calibration_current, args.max_rpm)
            proto.send_command(port, "CURRENT 0 0")
            time.sleep(0.25)

            for profile_name in profile_names:
                axes = controllers[profile_name]
                set_profile(port, axes)
                time.sleep(0.1)

                for target_iq in args.pulses:
                    scope.arm_scope(port, 1)
                    proto.send_command(port, f"CURRENT {target_iq:g} 0")
                    time.sleep(args.pulse_ms / 1000.0)
                    proto.send_command(port, "CURRENT 0 0")
                    time.sleep(0.11)
                    capture, samples = scope.dump_scope(port, args.timeout)
                    sample_period = capture["divider"] / float(capture["base_sample_hz"])
                    result = summarize_q_pulse(samples, target_iq, sample_period)
                    csv_path = args.results_dir / (
                        f"stage2_q_pulse_{stamp}_{profile_name}_{target_iq:+g}A.csv"
                    )
                    scope.write_csv(csv_path, samples, sample_period)
                    result.update({"profile": profile_name, "csv": str(csv_path.resolve())})
                    if result["raw_peak_current_a"] > args.max_step_current:
                        raise RuntimeError(
                            f"scope current limit exceeded: {result['raw_peak_current_a']:.3f} A"
                        )
                    if result["raw_peak_rpm"] > args.max_rpm:
                        raise RuntimeError(f"scope speed limit exceeded: {result['raw_peak_rpm']:.1f} rpm")

                    records.append(result)
                    wait_mode(port, buffer, MODE_TORQUE_RUN, 2.0,
                              args.max_step_current, args.max_rpm)
                    time.sleep(0.05)
        finally:
            proto.send_command(port, "CURRENT 0 0")
            set_profile(port, controllers["active_legacy"])
            proto.send_command(port, "STOP")
            try:
                final = wait_mode(port, buffer, MODE_READY, 2.0,
                                  args.max_calibration_current, args.max_rpm)
                final_mode = int(round(final[27]))
            except RuntimeError:
                final_mode = None

    summaries = {}
    for profile_name in profile_names:
        selected = [record for record in records if record["profile"] == profile_name]
        rises = [record["rise_time_10_90_sec"] for record in selected
                 if record["rise_time_10_90_sec"] is not None]
        summaries[profile_name] = {
            "mean_rise_time_10_90_sec": sum(rises) / len(rises) if rises else None,
            "max_filtered_overshoot_pct": max(
                record["filtered_overshoot_pct"] for record in selected
            ),
            "mean_plateau_error_rms_a": sum(
                record["plateau_error_rms_a"] for record in selected
            ) / len(selected),
            "raw_peak_current_a": max(record["raw_peak_current_a"] for record in selected),
            "raw_peak_rpm": max(record["raw_peak_rpm"] for record in selected),
        }

    output = args.results_dir / f"stage2_q_current_pulse_{stamp}.json"
    output.write_text(json.dumps({"profiles": profile_names, "pulses_a": args.pulses,
                                  "records": records, "profile_summaries": summaries,
                                  "final_mode": final_mode}, indent=2) + "\n", encoding="utf-8")
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    print(json.dumps({"result": str(output.resolve()), "records": len(records),
                      "profile_summaries": summaries, "final_mode": final_mode}, indent=2))


if __name__ == "__main__":
    main()
