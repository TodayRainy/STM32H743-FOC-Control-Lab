#!/usr/bin/env python3
"""D-048 low-speed tracking and multi-position hold diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
from scope_current_step import wait_mode
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


MODE_SPEED = 12
MODE_POSITION = 15
MODE_READY = 17
POLE_PAIRS = 10
SPEED_KP = 0.00536533785
SPEED_KI = 0.258523708
POSITION_KP = 376.991118
LOW_SPEED_ORDERS = (
    (5.0, 10.0, 20.0, 50.0, 0.0, -5.0, -10.0, -20.0, -50.0, 0.0),
    (-5.0, -10.0, -20.0, -50.0, 0.0, 5.0, 10.0, 20.0, 50.0, 0.0),
)
MECHANICAL_ANGLES_DEG = tuple(float(value) for value in range(0, 360, 10))
POSITION_VISITS = (
    (0, "increasing"), (0, "decreasing"),
    (1, "decreasing"), (1, "increasing"),
)
SPEED_CAPTURE_SEC = 2.0
POSITION_CAPTURE_SEC = 0.60
POSITION_TAIL_SEC = 0.25
ACTUAL_DQ_ABORT_A = 5.0
IQ_REFERENCE_LIMIT_A = 4.0
MECHANICAL_SPEED_ABORT_RPM = 250.0
VOLTAGE_SCALE_ABORT = 0.98


def plan() -> dict:
    return {
        "schema": "stage2-zero-speed-position-servo-plan-v1",
        "decision": "D-048",
        "low_speed": {
            "orders_rpm": LOW_SPEED_ORDERS,
            "nonzero_record_count": 16,
            "total_record_count": sum(len(order) for order in LOW_SPEED_ORDERS),
            "capture_sec": SPEED_CAPTURE_SEC,
            "tail_fraction": 0.5,
        },
        "position_hold": {
            "angles_deg": MECHANICAL_ANGLES_DEG,
            "spacing_deg": 10.0,
            "visits": POSITION_VISITS,
            "record_count": len(MECHANICAL_ANGLES_DEG) * len(POSITION_VISITS),
            "trajectory": "TRAP 180 deg/s, 900 deg/s2, velocity FF ON",
            "capture_sec": POSITION_CAPTURE_SEC,
            "tail_sec": POSITION_TAIL_SEC,
        },
        "safety": {
            "iq_reference_limit_a": IQ_REFERENCE_LIMIT_A,
            "actual_dq_abort_a": ACTUAL_DQ_ABORT_A,
            "mechanical_speed_abort_rpm": MECHANICAL_SPEED_ABORT_RPM,
            "minimum_voltage_scale": VOLTAGE_SCALE_ABORT,
        },
        "requires_24v": True,
        "motor_will_rotate_both_directions": True,
        "automatic_flash": False,
        "finally_stop_ready_and_restore_defaults": True,
    }


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def standard_deviation(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def rmse(first: list[float], second: list[float]) -> float:
    return math.sqrt(mean([(a - b) ** 2 for a, b in zip(first, second)]))


def correlation(first: list[float], second: list[float]) -> float:
    first_mean = mean(first)
    second_mean = mean(second)
    first_centered = [value - first_mean for value in first]
    second_centered = [value - second_mean for value in second]
    denominator = math.sqrt(
        sum(value * value for value in first_centered)
        * sum(value * value for value in second_centered)
    )
    if denominator <= 1.0e-12:
        return 1.0 if rmse(first, second) <= 1.0e-6 else 0.0
    return sum(a * b for a, b in zip(first_centered, second_centered)) / denominator


def check_frame(frame: tuple[float, ...], required_mode: int) -> None:
    if int(round(frame[27])) != required_mode:
        raise RuntimeError(f"motor left required mode {required_mode}")
    actual_dq = math.hypot(frame[6], frame[7])
    if actual_dq > ACTUAL_DQ_ABORT_A:
        raise RuntimeError(f"actual dq current exceeded {ACTUAL_DQ_ABORT_A:g} A")
    if abs(frame[22]) > IQ_REFERENCE_LIMIT_A + 0.05:
        raise RuntimeError("speed-loop q-current reference exceeded limit")
    if abs(proto.mechanical_rpm(frame)) > MECHANICAL_SPEED_ABORT_RPM:
        raise RuntimeError("mechanical speed exceeded zero-speed diagnostic limit")
    if frame[26] < VOLTAGE_SCALE_ABORT:
        raise RuntimeError("voltage utilization entered the abort region")


def capture(port, buffer: bytearray, duration_sec: float,
            required_mode: int) -> list[tuple[float, tuple[float, ...]]]:
    started = time.monotonic()
    deadline = started + duration_sec
    samples = []
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        check_frame(sample[1], required_mode)
        samples.append((sample[0] - started, sample[1]))
    if len(samples) < 20:
        raise RuntimeError("insufficient telemetry for zero-speed diagnostic")
    return samples


def telemetry_row(phase: str, elapsed: float, frame: tuple[float, ...],
                  **labels) -> dict:
    return {
        "phase": phase,
        **labels,
        "elapsed_sec": elapsed,
        "mode": int(round(frame[27])),
        "target_speed_rpm": frame[24] / POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "speed_reference_rpm": frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "measured_speed_rpm": proto.mechanical_rpm(frame),
        "target_position_deg": math.degrees(frame[30]),
        "position_reference_deg": math.degrees(frame[31]),
        "position_feedback_deg": math.degrees(frame[32]),
        "iq_reference_a": frame[22],
        "iq_a": frame[6],
        "id_a": frame[7],
        "voltage_scale": frame[26],
    }


def append_rows(rows: list[dict], samples: list[tuple[float, tuple]],
                phase: str, **labels) -> None:
    rows.extend(
        telemetry_row(phase, elapsed, frame, **labels)
        for elapsed, frame in samples
    )


def tail_frames(samples: list[tuple[float, tuple]],
                duration_sec: float) -> list[tuple[float, ...]]:
    final_time = samples[-1][0]
    frames = [frame for elapsed, frame in samples
              if elapsed >= final_time - duration_sec]
    if len(frames) < 10:
        raise RuntimeError("insufficient steady-state telemetry")
    return frames


def summarize_speed(samples: list[tuple[float, tuple]], target_rpm: float,
                    round_index: int, order_index: int) -> dict:
    frames = tail_frames(samples, SPEED_CAPTURE_SEC * 0.5)
    speed = [proto.mechanical_rpm(frame) for frame in frames]
    speed_ref = [frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi)
                 for frame in frames]
    iq_ref = [frame[22] for frame in frames]
    iq = [frame[6] for frame in frames]
    return {
        "round": round_index,
        "order_index": order_index,
        "target_rpm": target_rpm,
        "mean_speed_reference_rpm": mean(speed_ref),
        "mean_measured_speed_rpm": mean(speed),
        "mean_tracking_error_rpm": mean(speed_ref) - mean(speed),
        "speed_ripple_rms_rpm": rms([value - mean(speed) for value in speed]),
        "speed_standard_deviation_rpm": standard_deviation(speed),
        "mean_iq_reference_a": mean(iq_ref),
        "rms_iq_reference_a": rms(iq_ref),
        "iq_reference_ripple_rms_a": rms(
            [value - mean(iq_ref) for value in iq_ref]
        ),
        "rms_iq_a": rms(iq),
        "sample_count": len(frames),
    }


def summarize_low_speed(records: list[dict]) -> dict:
    by_speed = defaultdict(list)
    for record in records:
        if record["target_rpm"] != 0.0:
            by_speed[abs(record["target_rpm"])].append(record)
    points = []
    for speed in sorted(by_speed):
        items = by_speed[speed]
        errors = [abs(item["mean_tracking_error_rpm"]) for item in items]
        complete_visits = (
            len(items) == 4
            and {math.copysign(1.0, item["target_rpm"]) for item in items}
            == {-1.0, 1.0}
            and {item["round"] for item in items} == {0, 1}
        )
        sign_correct = all(
            item["mean_measured_speed_rpm"] * item["target_rpm"] > 0.0
            for item in items
        )
        mean_tracked = (
            complete_visits
            and sign_correct
            and max(errors) <= max(2.0, 0.20 * speed)
        )
        points.append({
            "absolute_target_rpm": speed,
            "record_count": len(items),
            "complete_bidirectional_repeated_visits": complete_visits,
            "maximum_absolute_mean_error_rpm": max(errors),
            "maximum_speed_ripple_rms_rpm": max(
                item["speed_ripple_rms_rpm"] for item in items
            ),
            "maximum_rms_iq_reference_a": max(
                item["rms_iq_reference_a"] for item in items
            ),
            "sign_correct_in_all_visits": sign_correct,
            "mean_tracked_within_20_percent_or_2rpm": mean_tracked,
        })
    tracked = [item["absolute_target_rpm"] for item in points
               if item["mean_tracked_within_20_percent_or_2rpm"]]
    return {
        "points": points,
        "minimum_bidirectional_mean_tracked_speed_rpm": min(tracked) if tracked else None,
    }


def summarize_hold(samples: list[tuple[float, tuple]], expected_position_rad: float,
                   physical_angle_deg: float, command_offset_deg: float,
                   direction: str, round_index: int) -> dict:
    frames = tail_frames(samples, POSITION_TAIL_SEC)
    feedback = [frame[32] for frame in frames]
    speed = [proto.mechanical_rpm(frame) for frame in frames]
    iq_ref = [frame[22] for frame in frames]
    iq = [frame[6] for frame in frames]
    position_error = [math.degrees(expected_position_rad - value)
                      for value in feedback]
    return {
        "round": round_index,
        "direction": direction,
        "physical_angle_deg": physical_angle_deg,
        "command_offset_deg": command_offset_deg,
        "mean_position_error_deg": mean(position_error),
        "position_error_ripple_rms_deg": rms(
            [value - mean(position_error) for value in position_error]
        ),
        "mean_speed_rpm": mean(speed),
        "speed_ripple_rms_rpm": rms([value - mean(speed) for value in speed]),
        "mean_iq_reference_a": mean(iq_ref),
        "iq_reference_ripple_rms_a": rms(
            [value - mean(iq_ref) for value in iq_ref]
        ),
        "mean_iq_a": mean(iq),
        "iq_ripple_rms_a": rms([value - mean(iq) for value in iq]),
        "sample_count": len(frames),
    }


def summarize_position_map(records: list[dict]) -> dict:
    lookup = {
        (record["direction"], record["round"], record["physical_angle_deg"]): record
        for record in records
    }
    repeatability = {}
    for direction in ("increasing", "decreasing"):
        first_iq = [lookup[(direction, 0, angle)]["mean_iq_reference_a"]
                    for angle in MECHANICAL_ANGLES_DEG]
        second_iq = [lookup[(direction, 1, angle)]["mean_iq_reference_a"]
                     for angle in MECHANICAL_ANGLES_DEG]
        first_error = [lookup[(direction, 0, angle)]["mean_position_error_deg"]
                       for angle in MECHANICAL_ANGLES_DEG]
        second_error = [lookup[(direction, 1, angle)]["mean_position_error_deg"]
                        for angle in MECHANICAL_ANGLES_DEG]
        repeatability[direction] = {
            "iq_reference_correlation": correlation(first_iq, second_iq),
            "iq_reference_rmse_a": rmse(first_iq, second_iq),
            "position_error_correlation": correlation(first_error, second_error),
            "position_error_rmse_deg": rmse(first_error, second_error),
        }

    angle_map = []
    for angle in MECHANICAL_ANGLES_DEG:
        direction_values = {}
        for direction in ("increasing", "decreasing"):
            items = [lookup[(direction, round_index, angle)]
                     for round_index in (0, 1)]
            direction_values[direction] = {
                "mean_iq_reference_a": mean(
                    [item["mean_iq_reference_a"] for item in items]
                ),
                "mean_position_error_deg": mean(
                    [item["mean_position_error_deg"] for item in items]
                ),
            }
        increasing = direction_values["increasing"]
        decreasing = direction_values["decreasing"]
        angle_map.append({
            "physical_angle_deg": angle,
            **direction_values,
            "position_related_iq_reference_a": 0.5 * (
                increasing["mean_iq_reference_a"]
                + decreasing["mean_iq_reference_a"]
            ),
            "direction_related_iq_reference_a": 0.5 * (
                increasing["mean_iq_reference_a"]
                - decreasing["mean_iq_reference_a"]
            ),
            "position_related_error_deg": 0.5 * (
                increasing["mean_position_error_deg"]
                + decreasing["mean_position_error_deg"]
            ),
            "direction_related_error_deg": 0.5 * (
                increasing["mean_position_error_deg"]
                - decreasing["mean_position_error_deg"]
            ),
        })

    position_iq = [item["position_related_iq_reference_a"] for item in angle_map]
    direction_iq = [item["direction_related_iq_reference_a"] for item in angle_map]
    maximum_repeat_rmse = max(
        item["iq_reference_rmse_a"] for item in repeatability.values()
    )
    minimum_repeat_correlation = min(
        item["iq_reference_correlation"] for item in repeatability.values()
    )
    position_peak_to_peak = max(position_iq) - min(position_iq)
    periodic_candidate = (
        minimum_repeat_correlation >= 0.70
        and position_peak_to_peak >= 0.02
        and position_peak_to_peak >= 2.0 * maximum_repeat_rmse
    )
    return {
        "repeatability": repeatability,
        "angle_map": angle_map,
        "position_related_iq_peak_to_peak_a": position_peak_to_peak,
        "direction_related_iq_peak_to_peak_a": max(direction_iq) - min(direction_iq),
        "maximum_iq_repeat_rmse_a": maximum_repeat_rmse,
        "minimum_iq_repeat_correlation": minimum_repeat_correlation,
        "periodic_equivalent_current_candidate": periodic_candidate,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def send_commands(port, commands: tuple[str, ...]) -> None:
    for command in commands:
        proto.send_command(port, command)
        time.sleep(0.06)


def stop_ready(port, buffer: bytearray) -> None:
    proto.send_command(port, "SPEED 0")
    proto.send_command(port, "STOP")
    wait_mode(port, buffer, MODE_READY, 5.0, ACTUAL_DQ_ABORT_A,
              MECHANICAL_SPEED_ABORT_RPM)


def restore_defaults(port, buffer: bytearray) -> None:
    stop_ready(port, buffer)
    send_commands(port, (
        "SPEED EST WIN", "FF OFF", "SPEED LIMIT 7",
        f"PI SPEED {SPEED_KP:.9g} {SPEED_KI:.9g}",
        f"PI POSITION {POSITION_KP:.9g} 0 0",
        "POS FF OFF", "POS SPEED LIMIT 5000",
        "POS TRACK RAMP 180", "PI CLEAR",
    ))


def run_low_speed(port, buffer: bytearray, output: Path,
                  result: dict, rows: list[dict]) -> None:
    send_commands(port, (
        "SPEED EST WIN", "FF OFF", "SPEED LIMIT 4", "TRACK RAMP 100",
        f"PI SPEED {SPEED_KP:.9g} {SPEED_KI:.9g}", "PI CLEAR", "START SPEED",
    ))
    wait_mode(port, buffer, MODE_SPEED, 20.0, ACTUAL_DQ_ABORT_A,
              MECHANICAL_SPEED_ABORT_RPM)
    for round_index, order in enumerate(LOW_SPEED_ORDERS):
        for order_index, target_rpm in enumerate(order):
            buffer.clear()
            port.reset_input_buffer()
            proto.send_command(port, f"SPEED {target_rpm:g}")
            samples = capture(port, buffer, SPEED_CAPTURE_SEC, MODE_SPEED)
            result["low_speed_records"].append(
                summarize_speed(samples, target_rpm, round_index, order_index)
            )
            append_rows(
                rows, samples, f"speed_r{round_index}_{order_index:02d}",
                experiment="low_speed", round=round_index,
                direction="", target=target_rpm,
            )
            save_checkpoint(output, result)
    result["low_speed_summary"] = summarize_low_speed(
        result["low_speed_records"]
    )
    stop_ready(port, buffer)


def position_targets(direction: str) -> tuple[tuple[float, float], ...]:
    if direction == "increasing":
        return tuple((angle, angle) for angle in MECHANICAL_ANGLES_DEG)
    return tuple((offset % 360.0, offset)
                 for offset in range(360, 0, -10))


def run_position_map(port, buffer: bytearray, output: Path,
                     result: dict, rows: list[dict]) -> None:
    send_commands(port, (
        "SPEED EST WIN", "FF OFF", "SPEED LIMIT 4", "POS SPEED LIMIT 1500",
        "TRACK STEP",
        f"PI SPEED {SPEED_KP:.9g} {SPEED_KI:.9g}",
        f"PI POSITION {POSITION_KP:.9g} 0 0",
        "POS TRACK TRAP 180 900", "POS FF ON", "PI CLEAR", "START POSITION",
    ))
    wait_mode(port, buffer, MODE_POSITION, 20.0, ACTUAL_DQ_ABORT_A,
              MECHANICAL_SPEED_ABORT_RPM)
    initial = capture(port, buffer, 0.5, MODE_POSITION)
    origin_rad = initial[-1][1][30]
    last_offset = 0.0

    for round_index, direction in POSITION_VISITS:
        preposition = -10.0 if direction == "increasing" else 370.0
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {preposition:g}")
        preposition_duration = abs(preposition - last_offset) / 180.0 + 0.55
        samples = capture(port, buffer, preposition_duration, MODE_POSITION)
        append_rows(
            rows, samples, f"position_r{round_index}_{direction}_pre",
            experiment="position_preposition", round=round_index,
            direction=direction, target=preposition,
        )
        last_offset = preposition

        for point_index, (physical_angle, command_offset) in enumerate(
                position_targets(direction)):
            buffer.clear()
            port.reset_input_buffer()
            proto.send_command(port, f"POSITION {command_offset:g}")
            samples = capture(port, buffer, POSITION_CAPTURE_SEC, MODE_POSITION)
            expected = origin_rad + math.radians(command_offset)
            result["position_hold_records"].append(
                summarize_hold(
                    samples, expected, physical_angle, command_offset,
                    direction, round_index,
                )
            )
            append_rows(
                rows, samples,
                f"position_r{round_index}_{direction}_{point_index:02d}",
                experiment="position_hold", round=round_index,
                direction=direction, target=command_offset,
            )
            save_checkpoint(output, result)
            last_offset = command_offset

    result["position_map_summary"] = summarize_position_map(
        result["position_hold_records"]
    )
    stop_ready(port, buffer)


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
    output = Path("tests/results") / f"stage2_zero_speed_servo_{stamp}.json"
    csv_output = output.with_suffix(".csv")
    result = {
        "schema": "stage2-zero-speed-position-servo-result-v1",
        "plan": test_plan,
        "low_speed_records": [],
        "low_speed_summary": None,
        "position_hold_records": [],
        "position_map_summary": None,
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
        wait_mode(port, buffer, MODE_READY, 20.0, ACTUAL_DQ_ABORT_A,
                  MECHANICAL_SPEED_ABORT_RPM)
        try:
            run_low_speed(port, buffer, output, result, rows)
            run_position_map(port, buffer, output, result, rows)
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
        "low_speed_summary": result["low_speed_summary"],
        "position_map_summary": result["position_map_summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
        "error": result["error"],
    }, indent=2))
    if result["status"] != "completed" or result["final_mode"] != MODE_READY:
        raise RuntimeError(result["error"] or "D-048 did not close safely")


if __name__ == "__main__":
    main()
