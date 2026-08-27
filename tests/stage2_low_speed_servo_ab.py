#!/usr/bin/env python3
"""D-049 low-speed estimator x speed-PI cross comparison."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import stage2_zero_speed_position_servo as d048
from scope_current_step import wait_mode
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


POLE_PAIRS = 10
MODE_SPEED = 12
MODE_POSITION = 15
MODE_READY = 17
POSITION_KP = 376.991118
PI_PROFILES = {
    "fc24": (0.00536533785, 0.258523708),
    "bw20": (0.00290921105, 0.0765091919),
}
CONFIGURATIONS = (
    ("win_fc24", "WIN", "fc24"),
    ("base_fc24", "BASE", "fc24"),
    ("pll_fc24", "PLL", "fc24"),
    ("pll_bw20", "PLL", "bw20"),
    ("base_bw20", "BASE", "bw20"),
    ("win_bw20", "WIN", "bw20"),
)
SPEED_TARGETS_RPM = (5.0, 10.0, 20.0, 0.0, -5.0, -10.0, -20.0, 0.0)
POSITION_TARGETS_DEG = (10.0, 0.0, -10.0, 0.0)
SPEED_CAPTURE_SEC = 2.0
SPEED_TAIL_SEC = 1.0
POSITION_CAPTURE_SEC = 2.5
POSITION_TAIL_SEC = 1.0
START_ZERO_SETTLE_SEC = 1.0
DERIVATIVE_WINDOW_SEC = 0.020


def plan() -> dict:
    return {
        "schema": "stage2-low-speed-servo-ab-plan-v1",
        "decision": "D-049",
        "configurations": [
            {
                "name": name,
                "estimator": estimator,
                "pi_profile": pi_profile,
                "kp": PI_PROFILES[pi_profile][0],
                "ki": PI_PROFILES[pi_profile][1],
            }
            for name, estimator, pi_profile in CONFIGURATIONS
        ],
        "speed_targets_rpm": SPEED_TARGETS_RPM,
        "speed_record_count": len(CONFIGURATIONS) * len(SPEED_TARGETS_RPM),
        "speed_capture_sec": SPEED_CAPTURE_SEC,
        "position_targets_deg": POSITION_TARGETS_DEG,
        "position_record_count": len(CONFIGURATIONS) * len(POSITION_TARGETS_DEG),
        "position_capture_sec": POSITION_CAPTURE_SEC,
        "start_zero_settle_sec": START_ZERO_SETTLE_SEC,
        "independent_motion_measure": "ABZ electrical angle, unwrapped, 20 ms difference",
        "automatic_flash": False,
        "requires_24v": True,
        "motor_will_move_both_directions": True,
        "finally_stop_ready_and_restore_defaults": True,
    }


def unwrap_electrical_position(
        samples: list[tuple[float, tuple[float, ...]]]) -> list[tuple[float, float]]:
    if not samples:
        return []
    previous = samples[0][1][12]
    unwrapped = previous
    result = [(samples[0][0], unwrapped)]
    for timestamp, frame in samples[1:]:
        angle = frame[12]
        delta = angle - previous
        if delta > math.pi:
            delta -= 2.0 * math.pi
        elif delta < -math.pi:
            delta += 2.0 * math.pi
        unwrapped += delta
        result.append((timestamp, unwrapped))
        previous = angle
    return result


def encoder_speed_series(
        samples: list[tuple[float, tuple[float, ...]]],
        window_sec: float = DERIVATIVE_WINDOW_SEC) -> list[tuple[float, float]]:
    position = unwrap_electrical_position(samples)
    result = []
    start_index = 0
    for index in range(1, len(position)):
        target_time = position[index][0] - window_sec
        while (start_index + 1 < index
               and position[start_index + 1][0] <= target_time):
            start_index += 1
        elapsed = position[index][0] - position[start_index][0]
        if elapsed < 0.5 * window_sec:
            continue
        electrical_delta = position[index][1] - position[start_index][1]
        mechanical_rpm = (
            electrical_delta / POLE_PAIRS / elapsed * 60.0 / (2.0 * math.pi)
        )
        result.append((position[index][0], mechanical_rpm))
    return result


def tail_values(values: list[tuple[float, float]], tail_sec: float) -> list[float]:
    if not values:
        raise RuntimeError("no derived encoder-speed samples")
    threshold = values[-1][0] - tail_sec
    tail = [value for timestamp, value in values if timestamp >= threshold]
    if len(tail) < 10:
        raise RuntimeError("insufficient derived encoder-speed tail")
    return tail


def ripple_rms(values: list[float]) -> float:
    value_mean = d048.mean(values)
    return d048.rms([value - value_mean for value in values])


def summarize_speed(samples: list[tuple[float, tuple]], target_rpm: float,
                    configuration: str, estimator: str,
                    pi_profile: str) -> dict:
    frames = d048.tail_frames(samples, SPEED_TAIL_SEC)
    selected_speed = [proto.mechanical_rpm(frame) for frame in frames]
    iq_reference = [frame[22] for frame in frames]
    actual_speed = tail_values(encoder_speed_series(samples), SPEED_TAIL_SEC)
    return {
        "configuration": configuration,
        "estimator": estimator,
        "pi_profile": pi_profile,
        "target_rpm": target_rpm,
        "mean_selected_speed_rpm": d048.mean(selected_speed),
        "selected_speed_ripple_rms_rpm": ripple_rms(selected_speed),
        "mean_encoder_speed_rpm": d048.mean(actual_speed),
        "encoder_speed_ripple_rms_rpm": ripple_rms(actual_speed),
        "encoder_mean_error_rpm": target_rpm - d048.mean(actual_speed),
        "rms_iq_reference_a": d048.rms(iq_reference),
        "iq_reference_ripple_rms_a": ripple_rms(iq_reference),
        "sample_count": len(frames),
        "derived_sample_count": len(actual_speed),
    }


def summarize_position(samples: list[tuple[float, tuple]], expected_rad: float,
                       target_deg: float, configuration: str,
                       estimator: str, pi_profile: str) -> dict:
    frames = d048.tail_frames(samples, POSITION_TAIL_SEC)
    feedback = [frame[32] for frame in frames]
    position_error = [math.degrees(expected_rad - value) for value in feedback]
    selected_speed = [proto.mechanical_rpm(frame) for frame in frames]
    actual_speed = tail_values(encoder_speed_series(samples), POSITION_TAIL_SEC)
    iq_reference = [frame[22] for frame in frames]
    reference_error = [math.degrees(frame[30] - frame[31]) for frame in frames]
    return {
        "configuration": configuration,
        "estimator": estimator,
        "pi_profile": pi_profile,
        "target_offset_deg": target_deg,
        "mean_position_error_deg": d048.mean(position_error),
        "position_error_ripple_rms_deg": ripple_rms(position_error),
        "mean_selected_speed_rpm": d048.mean(selected_speed),
        "selected_speed_ripple_rms_rpm": ripple_rms(selected_speed),
        "mean_encoder_speed_rpm": d048.mean(actual_speed),
        "encoder_speed_ripple_rms_rpm": ripple_rms(actual_speed),
        "maximum_tail_target_reference_error_deg": max(map(abs, reference_error)),
        "rms_iq_reference_a": d048.rms(iq_reference),
        "iq_reference_ripple_rms_a": ripple_rms(iq_reference),
        "sample_count": len(frames),
        "derived_sample_count": len(actual_speed),
    }


def aggregate_configuration(speed_records: list[dict],
                            position_records: list[dict]) -> dict:
    names = {record["configuration"] for record in speed_records + position_records}
    summary = {}
    for name in sorted(names):
        speed = [record for record in speed_records
                 if record["configuration"] == name and record["target_rpm"] != 0.0]
        zero = [record for record in speed_records
                if record["configuration"] == name and record["target_rpm"] == 0.0]
        position = [record for record in position_records
                    if record["configuration"] == name]
        first = (speed or zero or position)[0]
        summary[name] = {
            "estimator": first["estimator"],
            "pi_profile": first["pi_profile"],
            "maximum_absolute_encoder_mean_error_rpm": max(
                abs(record["encoder_mean_error_rpm"]) for record in speed
            ),
            "maximum_nonzero_encoder_speed_ripple_rms_rpm": max(
                record["encoder_speed_ripple_rms_rpm"] for record in speed
            ),
            "maximum_zero_encoder_speed_ripple_rms_rpm": max(
                record["encoder_speed_ripple_rms_rpm"] for record in zero
            ),
            "maximum_selected_speed_ripple_rms_rpm": max(
                record["selected_speed_ripple_rms_rpm"] for record in speed
            ),
            "maximum_rms_iq_reference_a": max(
                record["rms_iq_reference_a"] for record in speed + zero + position
            ),
            "median_absolute_position_error_deg": statistics.median(
                abs(record["mean_position_error_deg"]) for record in position
            ),
            "maximum_position_error_ripple_rms_deg": max(
                record["position_error_ripple_rms_deg"] for record in position
            ),
            "maximum_position_encoder_speed_ripple_rms_rpm": max(
                record["encoder_speed_ripple_rms_rpm"] for record in position
            ),
        }
    baseline = summary.get("win_fc24")
    if baseline:
        for metrics in summary.values():
            metrics["relative_to_win_fc24"] = {
                "nonzero_encoder_speed_ripple": (
                    metrics["maximum_nonzero_encoder_speed_ripple_rms_rpm"]
                    / baseline["maximum_nonzero_encoder_speed_ripple_rms_rpm"]
                ),
                "position_encoder_speed_ripple": (
                    metrics["maximum_position_encoder_speed_ripple_rms_rpm"]
                    / baseline["maximum_position_encoder_speed_ripple_rms_rpm"]
                ),
                "median_absolute_position_error": (
                    metrics["median_absolute_position_error_deg"]
                    / baseline["median_absolute_position_error_deg"]
                ),
            }
    return summary


def telemetry_row(samples_phase: str, elapsed: float, frame: tuple,
                  configuration: str, estimator: str, pi_profile: str,
                  target: float) -> dict:
    row = d048.telemetry_row(
        samples_phase, elapsed, frame,
        experiment=samples_phase, round="", direction="", target=target,
    )
    row.update({
        "configuration": configuration,
        "estimator": estimator,
        "pi_profile": pi_profile,
        "encoder_electrical_angle_rad": frame[12],
    })
    return row


def append_rows(rows: list[dict], samples: list[tuple], phase: str,
                configuration: str, estimator: str,
                pi_profile: str, target: float) -> None:
    rows.extend(
        telemetry_row(
            phase, elapsed, frame, configuration, estimator, pi_profile, target
        )
        for elapsed, frame in samples
    )


def wait_mode_without_frozen_speed(port, buffer: bytearray,
                                   expected_mode: int, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        frame = sample[1]
        if math.hypot(frame[6], frame[7]) > d048.ACTUAL_DQ_ABORT_A:
            raise RuntimeError("actual dq current exceeded limit while waiting for mode")
        if int(round(frame[27])) == expected_mode:
            buffer.clear()
            port.reset_input_buffer()
            return
    raise RuntimeError(f"timeout waiting for motor mode {expected_mode}")


def stop_ready(port, buffer: bytearray) -> None:
    proto.send_command(port, "STOP")
    wait_mode_without_frozen_speed(port, buffer, MODE_READY, 5.0)


def restore_defaults(port, buffer: bytearray) -> None:
    stop_ready(port, buffer)
    d048.send_commands(port, (
        "SPEED EST WIN", "FF OFF", "SPEED LIMIT 7",
        f"PI SPEED {PI_PROFILES['fc24'][0]:.9g} {PI_PROFILES['fc24'][1]:.9g}",
        f"PI POSITION {POSITION_KP:.9g} 0 0",
        "POS FF OFF", "POS SPEED LIMIT 5000",
        "POS TRACK RAMP 180", "PI CLEAR",
    ))


def configure(port, estimator: str, pi_profile: str,
              position_mode: bool) -> None:
    kp, ki = PI_PROFILES[pi_profile]
    commands = [
        f"SPEED EST {estimator}", "FF OFF", "SPEED LIMIT 4",
        f"PI SPEED {kp:.9g} {ki:.9g}",
        f"PI POSITION {POSITION_KP:.9g} 0 0", "PI CLEAR",
    ]
    if position_mode:
        commands.extend((
            "TRACK STEP", "POS SPEED LIMIT 1500",
            "POS TRACK TRAP 180 900", "POS FF ON",
        ))
    else:
        commands.append("TRACK RAMP 100")
    d048.send_commands(port, tuple(commands))


def run_speed_configuration(port, buffer: bytearray, output: Path,
                            result: dict, rows: list[dict],
                            configuration: str, estimator: str,
                            pi_profile: str) -> None:
    configure(port, estimator, pi_profile, False)
    proto.send_command(port, "START SPEED")
    wait_mode(port, buffer, MODE_SPEED, 20.0, d048.ACTUAL_DQ_ABORT_A,
              d048.MECHANICAL_SPEED_ABORT_RPM)
    d048.capture(port, buffer, START_ZERO_SETTLE_SEC, MODE_SPEED)
    for index, target in enumerate(SPEED_TARGETS_RPM):
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"SPEED {target:g}")
        samples = d048.capture(port, buffer, SPEED_CAPTURE_SEC, MODE_SPEED)
        result["speed_records"].append(
            summarize_speed(samples, target, configuration, estimator, pi_profile)
        )
        append_rows(
            rows, samples, f"speed_{index:02d}", configuration,
            estimator, pi_profile, target,
        )
        save_checkpoint(output, result)
    stop_ready(port, buffer)


def run_position_configuration(port, buffer: bytearray, output: Path,
                               result: dict, rows: list[dict],
                               configuration: str, estimator: str,
                               pi_profile: str) -> None:
    configure(port, estimator, pi_profile, True)
    proto.send_command(port, "START POSITION")
    wait_mode(port, buffer, MODE_POSITION, 20.0, d048.ACTUAL_DQ_ABORT_A,
              d048.MECHANICAL_SPEED_ABORT_RPM)
    initial = d048.capture(port, buffer, START_ZERO_SETTLE_SEC, MODE_POSITION)
    origin_rad = initial[-1][1][30]
    for index, target in enumerate(POSITION_TARGETS_DEG):
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {target:g}")
        samples = d048.capture(port, buffer, POSITION_CAPTURE_SEC, MODE_POSITION)
        result["position_records"].append(
            summarize_position(
                samples, origin_rad + math.radians(target), target,
                configuration, estimator, pi_profile,
            )
        )
        append_rows(
            rows, samples, f"position_{index:02d}", configuration,
            estimator, pi_profile, target,
        )
        save_checkpoint(output, result)
    stop_ready(port, buffer)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM30")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()

    test_plan = plan()
    if args.plan_only:
        print(json.dumps(test_plan, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path("tests/results") / f"stage2_low_speed_servo_ab_{stamp}.json"
    csv_output = output.with_suffix(".csv")
    result = {
        "schema": "stage2-low-speed-servo-ab-result-v1",
        "plan": test_plan,
        "speed_records": [],
        "position_records": [],
        "configuration_summary": None,
        "raw_csv": str(csv_output.resolve()),
        "status": "running",
        "error": None,
        "final_mode": None,
        "defaults_restored": False,
    }
    rows = []
    buffer = bytearray()
    port = open_serial_with_retry(args.port, args.baudrate)
    try:
        port.reset_input_buffer()
        wait_mode_without_frozen_speed(port, buffer, MODE_READY, 20.0)
        try:
            for configuration, estimator, pi_profile in CONFIGURATIONS:
                run_speed_configuration(
                    port, buffer, output, result, rows,
                    configuration, estimator, pi_profile,
                )
                run_position_configuration(
                    port, buffer, output, result, rows,
                    configuration, estimator, pi_profile,
                )
            result["configuration_summary"] = aggregate_configuration(
                result["speed_records"], result["position_records"]
            )
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException) as error:
            result["status"] = "error"
            result["error"] = str(error)
        finally:
            try:
                restore_defaults(port, buffer)
                result["final_mode"] = MODE_READY
                result["defaults_restored"] = True
            except (RuntimeError, serial.SerialException):
                result["final_mode"] = None
    finally:
        port.close()
        write_csv(csv_output, rows)
        save_checkpoint(output, result)

    print(json.dumps({
        "result": str(output.resolve()),
        "raw_csv": str(csv_output.resolve()),
        "status": result["status"],
        "configuration_summary": result["configuration_summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
        "error": result["error"],
    }, indent=2))
    if result["status"] != "completed" or result["final_mode"] != MODE_READY:
        raise RuntimeError(result["error"] or "D-049 did not close safely")


if __name__ == "__main__":
    main()
