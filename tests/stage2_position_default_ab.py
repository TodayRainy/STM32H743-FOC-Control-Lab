#!/usr/bin/env python3
"""D-053 corrected position-default A/B at representative regions."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import stage2_low_speed_servo_ab as d049
import stage2_position_endpoint_repeatability as d051
import stage2_position_tuning as position
import stage2_zero_speed_position_servo as d048
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


DECISION = "D-053"
CENTERS_DEG = (0.0, 120.0, 240.0)
LOCAL_ORDERS_DEG = d051.LOCAL_ORDERS_DEG
CAPTURE_SEC = d051.CAPTURE_SEC
RATE_DEG_PER_SEC = 180.0
PROFILES = (
    {
        "name": "default_ramp_ff_off",
        "track_command": "POS TRACK RAMP 180",
        "position_velocity_feedforward": False,
        "center_order_deg": CENTERS_DEG,
    },
    {
        "name": "candidate_trap_ff_on",
        "track_command": "POS TRACK TRAP 180 900",
        "position_velocity_feedforward": True,
        "center_order_deg": tuple(reversed(CENTERS_DEG)),
    },
)


def plan() -> dict:
    records_per_profile = len(CENTERS_DEG) * sum(
        len(order) for order in LOCAL_ORDERS_DEG
    )
    return {
        "schema": "stage2-position-default-ab-plan-v1",
        "decision": DECISION,
        "purpose": "select the source-default position trajectory after D-052",
        "centers_deg_from_start": CENTERS_DEG,
        "center_roles": {
            "0": "dynamic braking-lag case",
            "120": "near-zero corrective-torque case",
            "240": "good control region",
        },
        "local_orders_deg": LOCAL_ORDERS_DEG,
        "profiles": PROFILES,
        "records_per_profile": records_per_profile,
        "record_count": len(PROFILES) * records_per_profile,
        "capture_sec": CAPTURE_SEC,
        "fixed_configuration": {
            "speed_estimator": "WIN",
            "speed_pi": "D-043 fc24",
            "position_controller": "p6",
            "inner_speed_track": "STEP/NONE",
        },
        "acceptance": {
            "maximum_endpoint_overshoot_deg": d051.MAX_ENDPOINT_OVERSHOOT_DEG,
            "maximum_settling_after_reference_sec": d051.MAX_SETTLING_SEC,
            "maximum_absolute_steady_error_deg": d051.MAX_STEADY_ERROR_DEG,
        },
        "diagnostic_only": (
            "speed command, selected/ABZ speed and iq at trajectory arrival"
        ),
        "automatic_flash": False,
        "requires_24v": True,
        "motor_will_move_both_directions": True,
        "finally_stop_ready_and_restore_defaults": True,
    }


def nearest_value(values: list[tuple[float, float]], timestamp: float) -> float:
    if not values:
        raise RuntimeError("no derived encoder-speed samples")
    return min(values, key=lambda item: abs(item[0] - timestamp))[1]


def add_arrival_metrics(
        record: dict,
        samples: list[tuple[float, tuple[float, ...]]],
        expected_target_rad: float,
        from_offset_deg: float,
        to_offset_deg: float) -> None:
    arrival_index = next(
        (index for index, (_, frame) in enumerate(samples)
         if abs(math.degrees(frame[31] - expected_target_rad)) <= 0.05),
        None,
    )
    if arrival_index is None:
        raise RuntimeError("position trajectory reference did not reach its target")
    timestamp, frame = samples[arrival_index]
    direction = 1.0 if to_offset_deg >= from_offset_deg else -1.0
    selected_speed = proto.mechanical_rpm(frame)
    encoder_speed = nearest_value(d049.encoder_speed_series(samples), timestamp)
    speed_command = frame[24] / d048.POLE_PAIRS * 60.0 / (2.0 * math.pi)
    speed_reference = frame[28] / d048.POLE_PAIRS * 60.0 / (2.0 * math.pi)
    record.update({
        "reference_arrival_sec": timestamp,
        "arrival_position_error_deg": math.degrees(
            expected_target_rad - frame[32]
        ),
        "arrival_speed_command_rpm": speed_command,
        "arrival_speed_reference_rpm": speed_reference,
        "arrival_selected_speed_rpm": selected_speed,
        "arrival_encoder_speed_rpm": encoder_speed,
        "arrival_iq_reference_a": frame[22],
        "arrival_directed_encoder_speed_rpm": direction * encoder_speed,
    })


def aggregate_profile(records: list[dict]) -> dict:
    if not records:
        raise RuntimeError("profile has no records")
    by_center = {}
    for center in CENTERS_DEG:
        values = [record for record in records
                  if record["center_offset_deg"] == center]
        by_center[f"{center:g}"] = {
            "record_count": len(values),
            "qualified_count": sum(
                record["endpoint_qualified"] for record in values
            ),
            "maximum_endpoint_overshoot_deg": max(
                record["endpoint_overshoot_deg"] for record in values
            ),
            "maximum_absolute_steady_error_deg": max(
                abs(record["steady_error_deg"]) for record in values
            ),
            "maximum_directed_encoder_speed_at_arrival_rpm": max(
                record["arrival_directed_encoder_speed_rpm"]
                for record in values
            ),
        }
    return {
        "record_count": len(records),
        "qualified_count": sum(
            record["endpoint_qualified"] for record in records
        ),
        "overshoot_pass_count": sum(
            record["endpoint_overshoot_deg"]
            <= d051.MAX_ENDPOINT_OVERSHOOT_DEG for record in records
        ),
        "settling_pass_count": sum(
            record["settling_after_reference_sec"] is not None
            and record["settling_after_reference_sec"] <= d051.MAX_SETTLING_SEC
            for record in records
        ),
        "steady_error_pass_count": sum(
            abs(record["steady_error_deg"]) <= d051.MAX_STEADY_ERROR_DEG
            for record in records
        ),
        "maximum_endpoint_overshoot_deg": max(
            record["endpoint_overshoot_deg"] for record in records
        ),
        "maximum_absolute_steady_error_deg": max(
            abs(record["steady_error_deg"]) for record in records
        ),
        "maximum_hold_encoder_speed_ripple_rms_rpm": max(
            record["hold_encoder_speed_ripple_rms_rpm"] for record in records
        ),
        "repeatability": d051.paired_repeat_summary(records),
        "by_center": by_center,
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


def append_record(
        port, buffer: bytearray, result: dict, rows: list[dict],
        profile: dict, origin_rad: float, center: float, round_index: int,
        order_index: int, from_local: float, to_local: float) -> None:
    target = center + to_local
    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, f"POSITION {target:g}")
    samples = d048.capture(port, buffer, CAPTURE_SEC, d048.MODE_POSITION)
    expected_target = origin_rad + math.radians(target)
    record = position.summarize_trajectory(
        samples, expected_target, RATE_DEG_PER_SEC,
        center + from_local, target,
    )
    record.update({
        "profile": profile["name"],
        "center_offset_deg": center,
        "round": round_index,
        "order_index": order_index,
        "transition": d051.transition_name(from_local, to_local),
        "from_local_deg": from_local,
        "to_local_deg": to_local,
    })
    d051.add_hold_metrics(record, samples)
    add_arrival_metrics(
        record, samples, expected_target,
        center + from_local, target,
    )
    record["endpoint_qualified"] = d051.endpoint_qualified(record)
    result["records"].append(record)
    phase = (
        f"{profile['name']}_c{center:g}_r{round_index}_{order_index}"
    )
    for elapsed, frame in samples:
        row = d051.telemetry_row(
            phase, elapsed, frame, center, round_index,
            from_local, to_local,
        )
        row["profile"] = profile["name"]
        rows.append(row)


def run_profile(port, buffer: bytearray, output: Path, result: dict,
                rows: list[dict], profile: dict) -> None:
    position.configure_ready(
        port, buffer, position.SELECTED_POSITION,
        position.POSITION_SPEED_LIMIT_RPM,
        profile["track_command"], True,
        profile["position_velocity_feedforward"],
    )
    initial = d048.capture(port, buffer, 1.0, d048.MODE_POSITION)
    origin_rad = initial[-1][1][30]
    previous_absolute = 0.0

    for center in profile["center_order_deg"]:
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {center:g}")
        preposition_sec = abs(center - previous_absolute) / RATE_DEG_PER_SEC + 2.5
        d048.capture(port, buffer, preposition_sec, d048.MODE_POSITION)
        previous_absolute = center

        for round_index, order in enumerate(LOCAL_ORDERS_DEG):
            from_local = 0.0
            for order_index, to_local in enumerate(order):
                append_record(
                    port, buffer, result, rows, profile, origin_rad,
                    center, round_index, order_index, from_local, to_local,
                )
                save_checkpoint(output, result)
                from_local = to_local
                previous_absolute = center + to_local

    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, "POSITION 0")
    return_sec = abs(previous_absolute) / RATE_DEG_PER_SEC + 2.5
    d048.capture(port, buffer, return_sec, d048.MODE_POSITION)
    position.stop_ready(port, buffer)


def run_experiment(port, buffer: bytearray, output: Path,
                   result: dict, rows: list[dict]) -> None:
    for profile in PROFILES:
        run_profile(port, buffer, output, result, rows, profile)
    result["summary"] = {
        profile["name"]: aggregate_profile([
            record for record in result["records"]
            if record["profile"] == profile["name"]
        ])
        for profile in PROFILES
    }


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
    output = Path("tests/results") / f"stage2_position_default_ab_{stamp}.json"
    csv_output = output.with_suffix(".csv")
    result = {
        "schema": "stage2-position-default-ab-result-v1",
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
        d051.write_csv(csv_output, rows)
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
        raise RuntimeError(result["error"] or "D-053 did not close safely")


if __name__ == "__main__":
    main()
