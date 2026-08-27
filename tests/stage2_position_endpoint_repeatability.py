#!/usr/bin/env python3
"""D-051 small-position endpoint repeatability across mechanical regions."""

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
import stage2_low_speed_servo_ab as d049
import stage2_position_tuning as position
import stage2_zero_speed_position_servo as d048
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


DECISION = "D-051"
CENTERS_DEG = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
LOCAL_ORDERS_DEG = (
    (10.0, 0.0, -10.0, 0.0),
    (-10.0, 0.0, 10.0, 0.0),
)
CAPTURE_SEC = 2.5
TAIL_SEC = 0.40
RATE_DEG_PER_SEC = 180.0
ACCELERATION_DEG_PER_SEC2 = 900.0
MAX_ENDPOINT_OVERSHOOT_DEG = 0.5
MAX_SETTLING_SEC = 0.2
MAX_STEADY_ERROR_DEG = 0.2


def plan() -> dict:
    return {
        "schema": "stage2-position-endpoint-repeatability-plan-v1",
        "decision": DECISION,
        "centers_deg_from_start": CENTERS_DEG,
        "local_orders_deg": LOCAL_ORDERS_DEG,
        "record_count": (
            len(CENTERS_DEG) * sum(len(order) for order in LOCAL_ORDERS_DEG)
        ),
        "capture_sec": CAPTURE_SEC,
        "fixed_configuration": {
            "speed_estimator": "WIN",
            "speed_pi": "D-043 fc24",
            "position_controller": "p6",
            "trajectory": "TRAP 180 deg/s, 900 deg/s2",
            "position_velocity_feedforward": True,
        },
        "acceptance": {
            "maximum_endpoint_overshoot_deg": MAX_ENDPOINT_OVERSHOOT_DEG,
            "maximum_settling_after_reference_sec": MAX_SETTLING_SEC,
            "maximum_absolute_steady_error_deg": MAX_STEADY_ERROR_DEG,
        },
        "independent_hold_motion": "ABZ electrical angle, 20 ms difference",
        "automatic_flash": False,
        "requires_24v": True,
        "motor_will_move_both_directions": True,
        "finally_stop_ready_and_restore_defaults": True,
    }


def endpoint_qualified(record: dict) -> bool:
    settling = record["settling_after_reference_sec"]
    return (
        record["endpoint_overshoot_deg"] <= MAX_ENDPOINT_OVERSHOOT_DEG
        and settling is not None
        and settling <= MAX_SETTLING_SEC
        and abs(record["steady_error_deg"]) <= MAX_STEADY_ERROR_DEG
    )


def transition_name(from_local_deg: float, to_local_deg: float) -> str:
    return f"{from_local_deg:+g}_to_{to_local_deg:+g}"


def ripple_rms(values: list[float]) -> float:
    value_mean = statistics.fmean(values)
    return math.sqrt(statistics.fmean(
        (value - value_mean) ** 2 for value in values
    ))


def add_hold_metrics(record: dict,
                     samples: list[tuple[float, tuple[float, ...]]]) -> None:
    actual_speed = d049.tail_values(d049.encoder_speed_series(samples), TAIL_SEC)
    frames = d048.tail_frames(samples, TAIL_SEC)
    selected_speed = [proto.mechanical_rpm(frame) for frame in frames]
    position_feedback = [math.degrees(frame[32]) for frame in frames]
    iq_reference = [frame[22] for frame in frames]
    record.update({
        "hold_encoder_speed_ripple_rms_rpm": ripple_rms(actual_speed),
        "hold_selected_speed_ripple_rms_rpm": ripple_rms(selected_speed),
        "hold_position_ripple_rms_deg": ripple_rms(position_feedback),
        "hold_iq_reference_rms_a": math.sqrt(statistics.fmean(
            value * value for value in iq_reference
        )),
    })


def paired_repeat_summary(records: list[dict]) -> dict:
    grouped = defaultdict(list)
    for record in records:
        grouped[(record["center_offset_deg"], record["transition"])].append(record)
    pairs = []
    for (center, transition), values in sorted(grouped.items()):
        if len(values) != 2:
            raise RuntimeError("each center/transition must have exactly two repeats")
        first, second = values
        pairs.append({
            "center_offset_deg": center,
            "transition": transition,
            "endpoint_overshoot_repeat_delta_deg": abs(
                first["endpoint_overshoot_deg"] - second["endpoint_overshoot_deg"]
            ),
            "steady_error_repeat_delta_deg": abs(
                first["steady_error_deg"] - second["steady_error_deg"]
            ),
            "hold_speed_ripple_repeat_delta_rpm": abs(
                first["hold_encoder_speed_ripple_rms_rpm"]
                - second["hold_encoder_speed_ripple_rms_rpm"]
            ),
            "both_qualified": first["endpoint_qualified"]
                              and second["endpoint_qualified"],
        })
    return {
        "pair_count": len(pairs),
        "pairs": pairs,
        "maximum_endpoint_overshoot_repeat_delta_deg": max(
            pair["endpoint_overshoot_repeat_delta_deg"] for pair in pairs
        ),
        "maximum_steady_error_repeat_delta_deg": max(
            pair["steady_error_repeat_delta_deg"] for pair in pairs
        ),
        "maximum_hold_speed_ripple_repeat_delta_rpm": max(
            pair["hold_speed_ripple_repeat_delta_rpm"] for pair in pairs
        ),
    }


def aggregate(records: list[dict]) -> dict:
    by_center = {}
    for center in CENTERS_DEG:
        values = [record for record in records
                  if record["center_offset_deg"] == center]
        by_center[f"{center:g}"] = {
            "record_count": len(values),
            "qualified_count": sum(record["endpoint_qualified"] for record in values),
            "maximum_endpoint_overshoot_deg": max(
                record["endpoint_overshoot_deg"] for record in values
            ),
            "worst_settling_after_reference_sec": max(
                (record["settling_after_reference_sec"] for record in values
                 if record["settling_after_reference_sec"] is not None),
                default=None,
            ),
            "maximum_absolute_steady_error_deg": max(
                abs(record["steady_error_deg"]) for record in values
            ),
            "maximum_hold_encoder_speed_ripple_rms_rpm": max(
                record["hold_encoder_speed_ripple_rms_rpm"] for record in values
            ),
        }
    return {
        "record_count": len(records),
        "qualified_count": sum(record["endpoint_qualified"] for record in records),
        "by_center": by_center,
        "repeatability": paired_repeat_summary(records),
        "safety": {
            "peak_abs_iq_reference_a": max(
                record["peak_abs_iq_reference_a"] for record in records
            ),
            "peak_actual_dq_current_a": max(
                record["peak_actual_dq_current_a"] for record in records
            ),
            "peak_abs_speed_rpm": max(
                record["peak_abs_speed_rpm"] for record in records
            ),
            "minimum_voltage_scale": min(
                record["minimum_voltage_scale"] for record in records
            ),
        },
    }


def telemetry_row(samples_phase: str, elapsed: float, frame: tuple,
                  center: float, round_index: int,
                  from_local: float, to_local: float) -> dict:
    row = d048.telemetry_row(
        samples_phase, elapsed, frame, experiment="position_endpoint",
        round=round_index, direction=transition_name(from_local, to_local),
        target=center + to_local,
    )
    row.update({
        "center_offset_deg": center,
        "from_local_deg": from_local,
        "to_local_deg": to_local,
        "encoder_electrical_angle_rad": frame[12],
    })
    return row


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_experiment(port, buffer: bytearray, output: Path,
                   result: dict, rows: list[dict]) -> None:
    position.configure_ready(
        port, buffer, position.SELECTED_POSITION,
        position.POSITION_SPEED_LIMIT_RPM,
        f"POS TRACK TRAP {RATE_DEG_PER_SEC:g} {ACCELERATION_DEG_PER_SEC2:g}",
        True, True,
    )
    initial = d048.capture(port, buffer, 1.0, d048.MODE_POSITION)
    origin_rad = initial[-1][1][30]
    previous_absolute = 0.0

    for center in CENTERS_DEG:
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {center:g}")
        preposition_sec = abs(center - previous_absolute) / RATE_DEG_PER_SEC + 2.5
        d048.capture(port, buffer, preposition_sec, d048.MODE_POSITION)
        previous_absolute = center

        for round_index, order in enumerate(LOCAL_ORDERS_DEG):
            from_local = 0.0
            for order_index, to_local in enumerate(order):
                target = center + to_local
                buffer.clear()
                port.reset_input_buffer()
                proto.send_command(port, f"POSITION {target:g}")
                samples = d048.capture(
                    port, buffer, CAPTURE_SEC, d048.MODE_POSITION
                )
                record = position.summarize_trajectory(
                    samples, origin_rad + math.radians(target),
                    RATE_DEG_PER_SEC, center + from_local, target,
                )
                record.update({
                    "center_offset_deg": center,
                    "round": round_index,
                    "order_index": order_index,
                    "transition": transition_name(from_local, to_local),
                    "from_local_deg": from_local,
                    "to_local_deg": to_local,
                })
                add_hold_metrics(record, samples)
                record["endpoint_qualified"] = endpoint_qualified(record)
                result["records"].append(record)
                phase = f"c{center:g}_r{round_index}_{order_index}"
                rows.extend(telemetry_row(
                    phase, elapsed, frame, center, round_index,
                    from_local, to_local,
                ) for elapsed, frame in samples)
                save_checkpoint(output, result)
                from_local = to_local
                previous_absolute = target
    result["summary"] = aggregate(result["records"])


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
    output = Path("tests/results") / f"stage2_position_endpoint_repeatability_{stamp}.json"
    csv_output = output.with_suffix(".csv")
    result = {
        "schema": "stage2-position-endpoint-repeatability-result-v1",
        "plan": test_plan,
        "records": [],
        "summary": None,
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
        d049.wait_mode_without_frozen_speed(port, buffer, d049.MODE_READY, 20.0)
        try:
            run_experiment(port, buffer, output, result, rows)
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException) as error:
            result["status"] = "error"
            result["error"] = str(error)
        finally:
            try:
                d049.restore_defaults(port, buffer)
                result["final_mode"] = d049.MODE_READY
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
        "summary": result["summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
        "error": result["error"],
    }, indent=2))
    if result["status"] != "completed" or result["final_mode"] != d049.MODE_READY:
        raise RuntimeError(result["error"] or "D-051 did not close safely")


if __name__ == "__main__":
    main()
