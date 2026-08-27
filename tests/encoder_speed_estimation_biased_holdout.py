#!/usr/bin/env python3
"""Run the rotating-bias holdout for BASE/WIN/PLL speed estimators."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import wait_mode
from encoder_speed_estimation_validation import (
    ROUND_ORDERS,
    SCOPE_DIVIDER,
    SINE_AMPLITUDE_RPM,
    SINE_FREQUENCIES_HZ,
    SINE_SETTLE_SEC,
    enter_speed_mode,
    leave_speed_mode,
    sine_stats,
)
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    open_serial_with_retry,
    stop_and_confirm_ready,
    supervised_wait,
)


SPEED_BIASES_RPM = (1500.0, -1500.0)


def plan() -> dict:
    visits = sum(len(order) for order in ROUND_ORDERS)
    captures_per_visit = len(SPEED_BIASES_RPM) * len(SINE_FREQUENCIES_HZ)
    return {
        "schema": "encoder-speed-estimation-biased-holdout-plan-v2",
        "round_orders": ROUND_ORDERS,
        "visits": visits,
        "speed_biases_rpm": SPEED_BIASES_RPM,
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "sine_frequencies_hz": SINE_FREQUENCIES_HZ,
        "captures_per_visit": captures_per_visit,
        "total_scope_captures": visits * captures_per_visit,
        "requires_24v": True,
        "motor_will_rotate": True,
        "maximum_commanded_speed_rpm": (
            max(abs(value) for value in SPEED_BIASES_RPM) + SINE_AMPLITUDE_RPM
        ),
        "automatic_flash": False,
        "finally_restore_default_and_stop": True,
    }


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
    output = args.output or Path("tests/results") / f"encoder_speed_bias_{stamp}.json"
    scope_dir = args.scope_dir or (
        Path("tests/results") / f"encoder_speed_bias_{stamp}_scope"
    )
    scope_dir.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    result = {
        "schema": "encoder-speed-estimation-biased-holdout-result-v1",
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
                        "frequency": [],
                    }
                    for bias_rpm in SPEED_BIASES_RPM:
                        for frequency_hz in SINE_FREQUENCIES_HZ:
                            proto.send_command(port, "TRACK RAMP 3000")
                            proto.send_command(port, f"SPEED {bias_rpm:g}")
                            supervised_wait(
                                port, buffer, abs(bias_rpm) / 3000.0 + 0.5,
                                spec, MODE_SPEED_TORQUE_RUN,
                            )
                            proto.send_command(
                                port,
                                f"SPEED SINE {SINE_AMPLITUDE_RPM:g} {frequency_hz:g}",
                            )
                            supervised_wait(
                                port, buffer, SINE_SETTLE_SEC,
                                spec, MODE_SPEED_TORQUE_RUN,
                            )
                            path = scope_dir / (
                                f"r{round_index}_{profile}_{bias_rpm:+g}rpm_"
                                f"{frequency_hz:g}hz.csv"
                            )
                            scope.arm_scope(port, SCOPE_DIVIDER)
                            supervised_wait(
                                port, buffer, 1.15, spec, MODE_SPEED_TORQUE_RUN,
                            )
                            capture, samples = scope.dump_scope(port, 25.0)
                            sample_period = (
                                capture["divider"] / capture["base_sample_hz"]
                            )
                            scope.write_csv(path, samples, sample_period)
                            summary = sine_stats(
                                samples, sample_period,
                                int(spec["control"]["pole_pairs"]), frequency_hz,
                            )
                            summary.update({
                                "bias_rpm": bias_rpm,
                                "scope_csv": str(path.resolve()),
                            })
                            visit["frequency"].append(summary)
                            proto.send_command(port, "SPEED SINE OFF")
                            supervised_wait(
                                port, buffer, 0.3, spec, MODE_SPEED_TORQUE_RUN,
                            )
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
                    with open_serial_with_retry(
                        args.port, args.baudrate,
                    ) as recovery_port:
                        proto.send_command(recovery_port, "SPEED EST WIN")
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
