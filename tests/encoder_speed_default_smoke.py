#!/usr/bin/env python3
"""Verify the reset-default ABZ speed estimator without selecting it at runtime."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import motor_baseline as proto
import scope_capture as scope
from encoder_speed_estimation_validation import SCOPE_DIVIDER, sine_stats
from scope_current_step import wait_mode
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    open_serial_with_retry,
    stop_and_confirm_ready,
    supervised_wait,
)


BIAS_RPM = 1500.0
AMPLITUDE_RPM = 100.0
FREQUENCY_HZ = 80.0
MINIMUM_RESPONSE_DB = -35.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scope-output", type=Path)
    args = parser.parse_args()

    spec = load_spec(args.spec)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or (
        Path("tests/results") / f"encoder_speed_default_smoke_{stamp}.json"
    )
    scope_output = args.scope_output or (
        Path("tests/results") / f"encoder_speed_default_smoke_{stamp}.csv"
    )
    buffer = bytearray()
    result = {
        "schema": "encoder-speed-default-smoke-v1",
        "default_selection_command_sent": False,
        "bias_rpm": BIAS_RPM,
        "amplitude_rpm": AMPLITUDE_RPM,
        "frequency_hz": FREQUENCY_HZ,
        "minimum_response_db": MINIMUM_RESPONSE_DB,
        "status": "running",
        "final_mode": None,
    }

    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        try:
            proto.send_command(
                port,
                f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
            )
            proto.send_command(port, "TRACK RAMP 3000")
            proto.send_command(port, "PI CLEAR")
            proto.send_command(port, "START SPEED")
            wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 10.0, 5.0, 4800.0,
            )
            proto.send_command(port, f"SPEED {BIAS_RPM:g}")
            supervised_wait(port, buffer, 1.1, spec, MODE_SPEED_TORQUE_RUN)
            proto.send_command(
                port, f"SPEED SINE {AMPLITUDE_RPM:g} {FREQUENCY_HZ:g}",
            )
            supervised_wait(port, buffer, 0.5, spec, MODE_SPEED_TORQUE_RUN)
            scope.arm_scope(port, SCOPE_DIVIDER)
            supervised_wait(port, buffer, 1.15, spec, MODE_SPEED_TORQUE_RUN)
            capture, samples = scope.dump_scope(port, 25.0)
            sample_period = capture["divider"] / capture["base_sample_hz"]
            scope.write_csv(scope_output, samples, sample_period)
            result["measurement"] = sine_stats(
                samples,
                sample_period,
                int(spec["control"]["pole_pairs"]),
                FREQUENCY_HZ,
            )
            result["scope_csv"] = str(scope_output.resolve())
            if result["measurement"]["closed_loop_magnitude_db"] < MINIMUM_RESPONSE_DB:
                raise RuntimeError(
                    "80 Hz response matches the rejected BASE notch, not WIN"
                )
            result["status"] = "passed"
        except Exception as error:
            result["status"] = "failed"
            result["error"] = str(error)
            raise
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer,
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
