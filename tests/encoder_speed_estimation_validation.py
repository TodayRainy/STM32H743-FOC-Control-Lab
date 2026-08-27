#!/usr/bin/env python3
"""Run guarded BASE/WIN-IIR/PLL encoder-speed hardware A/B."""

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
from scope_current_step import wait_mode
from stage2_current_frequency_response import fit_phasor, magnitude_db, phase_deg
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    check_telemetry,
    open_serial_with_retry,
    stop_and_confirm_ready,
    supervised_wait,
)


SCOPE_DIVIDER = 10
STEADY_SPEEDS_RPM = (
    50.0, 500.0, 1500.0, 3000.0,
    -50.0, -500.0, -1500.0, -3000.0,
)
SINE_FREQUENCIES_HZ = (20.0, 40.0, 80.0)
SINE_AMPLITUDE_RPM = 100.0
SINE_SETTLE_SEC = 0.5
MODE_COMMANDS = {
    "base": "SPEED EST BASE",
    "win_1ms_fc150": "SPEED EST WIN",
    "pll_200hz": "SPEED EST PLL",
}
ROUND_ORDERS = (
    ("base", "win_1ms_fc150", "pll_200hz"),
    ("pll_200hz", "win_1ms_fc150", "base"),
)


def plan() -> dict:
    captures_per_visit = 1 + len(STEADY_SPEEDS_RPM) + len(SINE_FREQUENCIES_HZ)
    return {
        "schema": "encoder-speed-estimation-hardware-plan-v2",
        "round_orders": ROUND_ORDERS,
        "visits": sum(len(order) for order in ROUND_ORDERS),
        "steady_speeds_rpm": STEADY_SPEEDS_RPM,
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "sine_frequencies_hz": SINE_FREQUENCIES_HZ,
        "captures_per_visit": captures_per_visit,
        "total_scope_captures": captures_per_visit * sum(
            len(order) for order in ROUND_ORDERS
        ),
        "requires_24v": True,
        "motor_will_rotate": True,
        "automatic_flash": False,
        "finally_restore_default_and_stop": True,
    }


def capture_running_scope(port, buffer: bytearray, spec: dict,
                          duration_sec: float, path: Path) -> tuple[list, float]:
    scope.arm_scope(port, SCOPE_DIVIDER)
    supervised_wait(port, buffer, duration_sec, spec, MODE_SPEED_TORQUE_RUN)
    capture, samples = scope.dump_scope(port, 25.0)
    sample_period = capture["divider"] / capture["base_sample_hz"]
    scope.write_csv(path, samples, sample_period)
    return samples, sample_period


def speed_stats(samples: list, pole_pairs: int) -> dict:
    rpm_scale = 60.0 / (2.0 * math.pi * pole_pairs)
    tail = samples[len(samples) // 2:]
    selected_speed = [sample[9] * rpm_scale for sample in tail]
    mean = sum(selected_speed) / len(selected_speed)
    ripple = math.sqrt(
        sum((value - mean) ** 2 for value in selected_speed)
        / len(selected_speed)
    )
    return {
        "mean_rpm": mean,
        "ripple_rms_rpm": ripple,
        "peak_abs_rpm": max(abs(value) for value in selected_speed),
        "iq_reference_rms_a": math.sqrt(
            sum(sample[4] ** 2 for sample in tail) / len(tail)
        ),
        "peak_iq_reference_a": max(abs(sample[4]) for sample in tail),
        "peak_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in tail
        ),
    }


def sine_stats(samples: list, sample_period: float,
               pole_pairs: int, frequency_hz: float) -> dict:
    measured, _ = fit_phasor(
        [sample[9] for sample in samples], frequency_hz, sample_period,
    )
    reference, _ = fit_phasor(
        [sample[10] for sample in samples], frequency_hz, sample_period,
    )
    response = measured / reference
    rpm_scale = 60.0 / (2.0 * math.pi * pole_pairs)
    return {
        "frequency_hz": frequency_hz,
        "reference_amplitude_rpm": abs(reference) * rpm_scale,
        "measured_amplitude_rpm": abs(measured) * rpm_scale,
        "closed_loop_magnitude_db": magnitude_db(response),
        "closed_loop_phase_deg": phase_deg(response),
        "peak_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "peak_iq_reference_a": max(abs(sample[4]) for sample in samples),
    }


def enter_speed_mode(port, buffer: bytearray, spec: dict, profile: str) -> None:
    proto.send_command(port, "STOP")
    wait_mode(port, buffer, MODE_READY, 5.0, 5.0, 4800.0)
    proto.send_command(port, MODE_COMMANDS[profile])
    proto.send_command(
        port, f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
    )
    proto.send_command(port, "TRACK RAMP 3000")
    proto.send_command(port, "PI CLEAR")
    proto.send_command(port, "START SPEED")
    wait_mode(port, buffer, MODE_SPEED_TORQUE_RUN, 10.0, 5.0, 4800.0)
    proto.send_command(port, "SPEED 0")
    supervised_wait(port, buffer, 0.5, spec, MODE_SPEED_TORQUE_RUN)


def leave_speed_mode(port, buffer: bytearray, spec: dict) -> None:
    proto.send_command(port, "SPEED SINE OFF")
    proto.send_command(port, "TRACK RAMP 3000")
    proto.send_command(port, "SPEED 0")
    supervised_wait(port, buffer, 0.6, spec, MODE_SPEED_TORQUE_RUN)
    proto.send_command(port, "STOP")
    wait_mode(port, buffer, MODE_READY, 5.0, 5.0, 4800.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scope-dir", type=Path)
    args = parser.parse_args()

    if args.plan_only:
        print(json.dumps(plan(), indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / f"encoder_speed_ab_{stamp}.json"
    scope_dir = args.scope_dir or Path("tests/results") / f"encoder_speed_ab_{stamp}_scope"
    scope_dir.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    result = {
        "schema": "encoder-speed-estimation-hardware-result-v1",
        "plan": plan(),
        "visits": [],
        "status": "running",
        "final_mode": None,
    }

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            for round_index, order in enumerate(ROUND_ORDERS):
                for profile in order:
                    enter_speed_mode(port, buffer, spec, profile)
                    visit = {
                        "round": round_index,
                        "profile": profile,
                        "steady": [],
                        "frequency": [],
                    }

                    noise_path = scope_dir / f"r{round_index}_{profile}_zero.csv"
                    samples, sample_period = capture_running_scope(
                        port, buffer, spec, 1.15, noise_path,
                    )
                    noise = speed_stats(samples, int(spec["control"]["pole_pairs"]))
                    noise["scope_csv"] = str(noise_path.resolve())
                    visit["zero_speed"] = noise

                    for target_rpm in STEADY_SPEEDS_RPM:
                        proto.send_command(port, "TRACK RAMP 3000")
                        proto.send_command(port, "SPEED 0")
                        supervised_wait(port, buffer, 1.2, spec, MODE_SPEED_TORQUE_RUN)
                        proto.send_command(port, f"SPEED {target_rpm:g}")
                        supervised_wait(
                            port, buffer, abs(target_rpm) / 3000.0 + 0.5,
                            spec, MODE_SPEED_TORQUE_RUN,
                        )
                        path = scope_dir / f"r{round_index}_{profile}_{target_rpm:+g}rpm.csv"
                        samples, sample_period = capture_running_scope(
                            port, buffer, spec, 1.15, path,
                        )
                        summary = speed_stats(
                            samples, int(spec["control"]["pole_pairs"]),
                        )
                        summary.update({
                            "target_rpm": target_rpm,
                            "mean_error_rpm": summary["mean_rpm"] - target_rpm,
                            "scope_csv": str(path.resolve()),
                        })
                        visit["steady"].append(summary)

                    proto.send_command(port, "TRACK RAMP 3000")
                    proto.send_command(port, "SPEED 0")
                    supervised_wait(port, buffer, 0.8, spec, MODE_SPEED_TORQUE_RUN)
                    for frequency_hz in SINE_FREQUENCIES_HZ:
                        path = scope_dir / f"r{round_index}_{profile}_{frequency_hz:g}hz.csv"
                        proto.send_command(
                            port, f"SPEED SINE {SINE_AMPLITUDE_RPM:g} {frequency_hz:g}",
                        )
                        supervised_wait(
                            port, buffer, SINE_SETTLE_SEC,
                            spec, MODE_SPEED_TORQUE_RUN,
                        )
                        scope.arm_scope(port, SCOPE_DIVIDER)
                        supervised_wait(port, buffer, 1.15, spec, MODE_SPEED_TORQUE_RUN)
                        capture, samples = scope.dump_scope(port, 25.0)
                        sample_period = capture["divider"] / capture["base_sample_hz"]
                        scope.write_csv(path, samples, sample_period)
                        summary = sine_stats(
                            samples, sample_period,
                            int(spec["control"]["pole_pairs"]), frequency_hz,
                        )
                        summary["scope_csv"] = str(path.resolve())
                        visit["frequency"].append(summary)
                        proto.send_command(port, "SPEED SINE OFF")
                        supervised_wait(port, buffer, 0.25, spec, MODE_SPEED_TORQUE_RUN)

                    result["visits"].append(visit)
                    leave_speed_mode(port, buffer, spec)

            result["status"] = "completed"
        except Exception as error:
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
                except serial.SerialException:
                    with open_serial_with_retry(args.port, args.baudrate) as recovery_port:
                        proto.send_command(recovery_port, "SPEED EST WIN")
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
