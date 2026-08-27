#!/usr/bin/env python3
"""D-044 supervised position-loop tuning over a compact P-only grid."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
from scope_current_step import wait_mode
from stage2_speed_autotune_validation import open_serial_with_retry
from stage2_speed_high_bandwidth_probe import save_checkpoint


MODE_READY = 17
MODE_POSITION = 15
POLE_PAIRS = 10
LEGACY_POSITION = {"name": "legacy", "kp": 3125.0, "ki": 0.0, "kd": 25.0}
P_BANDWIDTHS_HZ = (4.0, 6.0, 8.0, 10.0)
CANDIDATES = (LEGACY_POSITION,) + tuple({
    "name": f"p{frequency:g}hz",
    "target_bandwidth_hz": frequency,
    "kp": 2.0 * math.pi * POLE_PAIRS * frequency,
    "ki": 0.0,
    "kd": 0.0,
} for frequency in P_BANDWIDTHS_HZ)
OFFSETS_DEG = (2.0, 0.0, -2.0, 0.0, 10.0, 0.0,
               -10.0, 0.0, 30.0, 0.0, -30.0, 0.0)
DEFAULT_SMOKE_OFFSETS_DEG = (10.0, 0.0, -10.0, 0.0, 30.0, 0.0, -30.0, 0.0)
SELECTED_POSITION = CANDIDATES[2]
POSITION_SPEED_LIMIT_RPM = 1500.0
SPEED_IQ_LIMIT_A = 4.0
ACTUAL_DQ_ABORT_A = 5.0
VOLTAGE_SCALE_ABORT = 0.98
STEP_DURATION_SEC = 0.60
SETTLING_BAND_DEG = 0.20
MAX_SETTLING_SEC = 0.20
MAX_OVERSHOOT_PCT = 10.0
MAX_STEADY_ERROR_DEG = 0.20
TRAJECTORY_RATES_DEG_PER_SEC = (180.0, 720.0)
TRAPEZOID_ACCELERATION_TIMES_SEC = (0.10, 0.20)
DEFAULT_TRAPEZOID_ACCELERATION_TIME_SEC = TRAPEZOID_ACCELERATION_TIMES_SEC[0]
TRAJECTORY_OFFSETS_DEG = (10.0, 0.0, -10.0, 0.0, 30.0, 0.0,
                          -30.0, 0.0, 180.0, 0.0, -180.0, 0.0)
MAX_TRAJECTORY_ENDPOINT_OVERSHOOT_DEG = 0.50
MAX_TRAJECTORY_SETTLING_SEC = 0.20
MAX_EQUIVALENT_TRACKING_LAG_SEC = 0.020


def visit_order() -> list[tuple[int, dict]]:
    return ([(0, candidate) for candidate in CANDIDATES]
            + [(1, candidate) for candidate in reversed(CANDIDATES)])


def plan(trapezoid_acceleration_time_sec: float =
         DEFAULT_TRAPEZOID_ACCELERATION_TIME_SEC) -> dict:
    trapezoid_acceleration_ratio = 1.0 / trapezoid_acceleration_time_sec
    return {
        "schema": "stage2-position-tuning-plan-v1",
        "decision": "D-044",
        "candidates": list(CANDIDATES),
        "visits": [
            {"round": round_index, "controller": candidate["name"]}
            for round_index, candidate in visit_order()
        ],
        "offsets_deg": OFFSETS_DEG,
        "step_record_count": len(visit_order()) * len(OFFSETS_DEG),
        "position_speed_limit_rpm": POSITION_SPEED_LIMIT_RPM,
        "speed_iq_limit_a": SPEED_IQ_LIMIT_A,
        "acceptance": {
            "maximum_noise_adjusted_overshoot_pct": MAX_OVERSHOOT_PCT,
            "maximum_settling_time_sec": MAX_SETTLING_SEC,
            "settling_and_steady_band_deg": SETTLING_BAND_DEG,
            "two_degree_step_is_report_only": True,
            "nondecaying_oscillation": False,
        },
        "trajectory_smoke_deg_per_sec": (180.0, 720.0),
        "trajectory_validation": {
            "trapezoid_decision": (
                "D-046" if trapezoid_acceleration_time_sec == 0.10 else "D-047"
            ),
            "rates_deg_per_sec": TRAJECTORY_RATES_DEG_PER_SEC,
            "trapezoid_acceleration_time_sec": trapezoid_acceleration_time_sec,
            "trapezoid_acceleration_ratio_per_sec": trapezoid_acceleration_ratio,
            "offsets_deg": TRAJECTORY_OFFSETS_DEG,
            "maximum_endpoint_overshoot_deg": MAX_TRAJECTORY_ENDPOINT_OVERSHOOT_DEG,
            "maximum_settling_after_reference_sec": MAX_TRAJECTORY_SETTLING_SEC,
            "maximum_equivalent_tracking_lag_sec": MAX_EQUIVALENT_TRACKING_LAG_SEC,
        },
        "requires_24v": True,
        "motor_will_move": True,
        "automatic_flash": False,
        "finally_stop_ready_and_restore_defaults": True,
    }


def check_frame(frame) -> None:
    if int(round(frame[27])) != MODE_POSITION:
        raise RuntimeError("motor left position mode")
    actual_dq = math.hypot(frame[6], frame[7])
    if actual_dq > ACTUAL_DQ_ABORT_A:
        raise RuntimeError(f"actual dq current exceeded {ACTUAL_DQ_ABORT_A:g} A")
    if abs(frame[22]) > SPEED_IQ_LIMIT_A + 0.05:
        raise RuntimeError("speed-loop q-current reference exceeded limit")
    if abs(proto.mechanical_rpm(frame)) > POSITION_SPEED_LIMIT_RPM + 25.0:
        raise RuntimeError("position-loop mechanical speed exceeded limit")
    if frame[26] < VOLTAGE_SCALE_ABORT:
        raise RuntimeError("voltage utilization entered the abort region")


def capture(port, buffer: bytearray, duration_sec: float) -> list[tuple[float, tuple]]:
    started = time.monotonic()
    deadline = started + duration_sec
    samples = []
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        check_frame(sample[1])
        samples.append((sample[0] - started, sample[1]))
    if len(samples) < 20:
        raise RuntimeError("insufficient position telemetry")
    return samples


def summarize_step(samples: list[tuple[float, tuple]], expected_target_rad: float,
                   command_offset_deg: float) -> dict:
    times = [sample[0] for sample in samples]
    frames = [sample[1] for sample in samples]
    feedback = [math.degrees(frame[32]) for frame in frames]
    target_deg = math.degrees(expected_target_rad)
    initial_deg = feedback[0]
    amplitude = target_deg - initial_deg
    direction = 1.0 if amplitude >= 0.0 else -1.0
    directed = [direction * (value - initial_deg) for value in feedback]
    amplitude_abs = abs(amplitude)
    peak = max(directed)
    tail_count = max(10, len(frames) // 5)
    tail = feedback[-tail_count:]
    steady_mean = sum(tail) / len(tail)
    steady_noise_rms = math.sqrt(
        sum((value - steady_mean) ** 2 for value in tail) / len(tail)
    )
    raw_overshoot_deg = max(0.0, peak - amplitude_abs)
    adjusted_overshoot_deg = max(0.0, raw_overshoot_deg - 3.0 * steady_noise_rms)
    adjusted_overshoot_pct = (
        100.0 * adjusted_overshoot_deg / amplitude_abs
        if amplitude_abs >= math.radians(0.05) * 180.0 / math.pi else 0.0
    )
    settling_time = None
    for index, value in enumerate(feedback):
        if all(abs(later - target_deg) <= SETTLING_BAND_DEG
               for later in feedback[index:]):
            settling_time = times[index]
            break
    middle = feedback[len(feedback) // 2:3 * len(feedback) // 4]
    middle_rms = math.sqrt(sum((value - target_deg) ** 2 for value in middle) / len(middle))
    tail_rms = math.sqrt(sum((value - target_deg) ** 2 for value in tail) / len(tail))
    nondecaying = tail_rms > max(SETTLING_BAND_DEG, 1.15 * middle_rms)
    reported_target = frames[-1][30]
    if abs(reported_target - expected_target_rad) > math.radians(0.5):
        raise RuntimeError("position command target was not confirmed by telemetry")
    return {
        "command_offset_deg": command_offset_deg,
        "step_amplitude_deg": amplitude,
        "noise_adjusted_overshoot_pct": adjusted_overshoot_pct,
        "settling_time_sec": settling_time,
        "steady_error_deg": target_deg - steady_mean,
        "steady_noise_rms_deg": steady_noise_rms,
        "nondecaying_oscillation": nondecaying,
        "peak_abs_speed_rpm": max(abs(proto.mechanical_rpm(frame)) for frame in frames),
        "peak_abs_iq_reference_a": max(abs(frame[22]) for frame in frames),
        "peak_actual_dq_current_a": max(math.hypot(frame[6], frame[7]) for frame in frames),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
        "sample_count": len(frames),
    }


def records_pass(records: list[dict]) -> bool:
    gated = [record for record in records
             if abs(record["step_amplitude_deg"]) >= 5.0]
    return bool(gated) and all(
        not record["nondecaying_oscillation"]
        and record["noise_adjusted_overshoot_pct"] <= MAX_OVERSHOOT_PCT
        and record["settling_time_sec"] is not None
        and record["settling_time_sec"] <= MAX_SETTLING_SEC
        and abs(record["steady_error_deg"]) <= MAX_STEADY_ERROR_DEG
        for record in gated
    )


def summarize_candidate(candidate: dict, records: list[dict]) -> dict:
    gated = [record for record in records
             if abs(record["step_amplitude_deg"]) >= 5.0]
    settling = [record["settling_time_sec"] for record in gated
                if record["settling_time_sec"] is not None]
    return {
        "kp": candidate["kp"], "ki": candidate["ki"], "kd": candidate["kd"],
        "record_count": len(records),
        "maximum_noise_adjusted_overshoot_pct": max(
            record["noise_adjusted_overshoot_pct"] for record in gated
        ),
        "worst_settling_time_sec": max(settling) if settling else None,
        "maximum_absolute_steady_error_deg": max(
            abs(record["steady_error_deg"]) for record in gated
        ),
        "peak_abs_speed_rpm": max(record["peak_abs_speed_rpm"] for record in records),
        "peak_abs_iq_reference_a": max(
            record["peak_abs_iq_reference_a"] for record in records
        ),
        "peak_actual_dq_current_a": max(
            record["peak_actual_dq_current_a"] for record in records
        ),
        "qualified": records_pass(records),
    }


def select_candidate(summaries: dict) -> str | None:
    qualified = [(name, summary) for name, summary in summaries.items()
                 if summary["qualified"] and name != "legacy"]
    if not qualified:
        return None
    best_settling = min(item[1]["worst_settling_time_sec"] for item in qualified)
    close = [item for item in qualified
             if item[1]["worst_settling_time_sec"] <= 1.10 * best_settling]
    return min(close, key=lambda item: (
        item[1]["peak_abs_iq_reference_a"], item[1]["kp"]
    ))[0]


def append_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def append_trace_rows(rows: list[dict], samples: list[tuple[float, tuple]],
                      phase: str, offset_deg: float) -> None:
    for elapsed, frame in samples:
        rows.append({
            "phase": phase,
            "target_offset_deg": offset_deg,
            "elapsed_sec": elapsed,
            "target_position_deg": math.degrees(frame[30]),
            "position_ref_deg": math.degrees(frame[31]),
            "position_feedback_deg": math.degrees(frame[32]),
            "position_speed_command_rpm": (
                frame[24] / POLE_PAIRS * 60.0 / (2.0 * math.pi)
            ),
            "speed_loop_reference_rpm": (
                frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi)
            ),
            "measured_speed_rpm": proto.mechanical_rpm(frame),
            "iq_reference_a": frame[22],
            "iq_a": frame[6],
            "voltage_scale": frame[26],
        })


def configure_ready(port, buffer: bytearray, candidate: dict,
                    position_speed_limit: float, track_command: str,
                    apply_position_pi: bool = True,
                    position_feedforward: bool | None = None) -> None:
    commands = [
        "SPEED EST WIN", "FF OFF", f"SPEED LIMIT {SPEED_IQ_LIMIT_A:g}",
        f"POS SPEED LIMIT {position_speed_limit:g}", "TRACK STEP",
    ]
    if apply_position_pi:
        commands.append(
            f"PI POSITION {candidate['kp']:.9g} {candidate['ki']:.9g} {candidate['kd']:.9g}"
        )
    if position_feedforward is not None:
        commands.append("POS FF ON" if position_feedforward else "POS FF OFF")
    commands.extend((track_command, "PI CLEAR"))
    for command in commands:
        for _ in range(2):
            proto.send_command(port, command)
            time.sleep(0.08)
    proto.send_command(port, "START POSITION")
    wait_mode(port, buffer, MODE_POSITION, 20.0,
              ACTUAL_DQ_ABORT_A, 1600.0)


def stop_ready(port, buffer: bytearray) -> None:
    proto.send_command(port, "STOP")
    wait_mode(port, buffer, MODE_READY, 4.0, ACTUAL_DQ_ABORT_A, 1600.0)


def run_steps(port, buffer: bytearray, candidate: dict,
              round_index: int, output: Path, result: dict,
              offsets=OFFSETS_DEG, apply_position_pi: bool = True) -> None:
    configure_ready(
        port, buffer, candidate, POSITION_SPEED_LIMIT_RPM, "POS TRACK STEP",
        apply_position_pi,
    )
    initial = capture(port, buffer, 0.5)
    origin_rad = initial[-1][1][30]
    previous = 0.0
    for index, offset in enumerate(offsets):
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {offset:g}")
        samples = capture(port, buffer, STEP_DURATION_SEC)
        record = summarize_step(
            samples, origin_rad + math.radians(offset), offset
        )
        record.update({
            "round": round_index, "controller": candidate["name"],
            "from_offset_deg": previous, "to_offset_deg": offset,
            "kp": candidate["kp"], "ki": candidate["ki"], "kd": candidate["kd"],
        })
        result["step_records"].append(record)
        save_checkpoint(output, result)
        previous = offset
    stop_ready(port, buffer)


def run_trajectory_smoke(port, buffer: bytearray, candidate: dict,
                         output: Path, result: dict,
                         apply_position_pi: bool = True) -> None:
    for rate in (180.0, 720.0):
        configure_ready(
            port, buffer, candidate, POSITION_SPEED_LIMIT_RPM,
            f"POS TRACK RAMP {rate:g}", apply_position_pi,
        )
        initial = capture(port, buffer, 0.4)
        origin_rad = initial[-1][1][30]
        for offset in (30.0, 0.0, -30.0, 0.0):
            buffer.clear()
            port.reset_input_buffer()
            proto.send_command(port, f"POSITION {offset:g}")
            duration = abs(30.0) / rate + 0.45
            samples = capture(port, buffer, duration)
            expected = origin_rad + math.radians(offset)
            frames = [sample[1] for sample in samples]
            result["trajectory_records"].append({
                "controller": candidate["name"], "rate_deg_per_sec": rate,
                "target_offset_deg": offset,
                "tracking_error_rms_deg": math.sqrt(sum(
                    (math.degrees(frame[31] - frame[32])) ** 2 for frame in frames
                ) / len(frames)),
                "peak_abs_speed_rpm": max(abs(proto.mechanical_rpm(frame)) for frame in frames),
                "peak_abs_iq_reference_a": max(abs(frame[22]) for frame in frames),
                "peak_actual_dq_current_a": max(math.hypot(frame[6], frame[7]) for frame in frames),
            })
            save_checkpoint(output, result)
        stop_ready(port, buffer)


def run_trace_diagnostic(port, buffer: bytearray, output: Path,
                         result: dict) -> None:
    candidate = SELECTED_POSITION
    configure_ready(
        port, buffer, candidate, POSITION_SPEED_LIMIT_RPM, "POS TRACK STEP", True
    )
    initial = capture(port, buffer, 0.5)
    origin_rad = initial[-1][1][30]
    rows = []
    previous = 0.0
    for index, offset in enumerate((10.0, 0.0, -10.0, 0.0) * 2):
        pre = capture(port, buffer, 0.10)
        append_trace_rows(rows, pre, f"{index:02d}_pre", offset)
        buffer.clear()
        port.reset_input_buffer()
        proto.send_command(port, f"POSITION {offset:g}")
        post = capture(port, buffer, STEP_DURATION_SEC)
        append_trace_rows(rows, post, f"{index:02d}_post", offset)
        samples = [(0.0, pre[-1][1])] + [
            (timestamp + 0.001, frame) for timestamp, frame in post
        ]
        record = summarize_step(samples, origin_rad + math.radians(offset), offset)
        record.update({
            "round": index // 4, "controller": candidate["name"],
            "from_offset_deg": previous, "to_offset_deg": offset,
            "kp": candidate["kp"], "ki": candidate["ki"], "kd": candidate["kd"],
        })
        result["step_records"].append(record)
        previous = offset
    append_csv(output.with_suffix(".csv"), rows)
    stop_ready(port, buffer)


def summarize_trajectory(samples: list[tuple[float, tuple]],
                         expected_target_rad: float, rate_deg_per_sec: float,
                         from_offset_deg: float, to_offset_deg: float) -> dict:
    target_deg = math.degrees(expected_target_rad)
    direction = 1.0 if to_offset_deg >= from_offset_deg else -1.0
    arrival = next((index for index, (_, frame) in enumerate(samples)
                    if abs(math.degrees(frame[31]) - target_deg) <= 0.05), None)
    if arrival is None:
        raise RuntimeError("position trajectory reference did not reach its target")
    active = samples[:max(1, arrival)]
    endpoint = samples[arrival:]
    tracking_error = [math.degrees(frame[31] - frame[32]) for _, frame in active]
    tracking_rms = math.sqrt(sum(value * value for value in tracking_error)
                             / len(tracking_error))
    directed_overshoot = [
        direction * (math.degrees(frame[32]) - target_deg)
        for _, frame in endpoint
    ]
    endpoint_overshoot = max(0.0, max(directed_overshoot))
    arrival_time = samples[arrival][0]
    settling_time = None
    for index in range(arrival, len(samples)):
        if all(abs(math.degrees(frame[32]) - target_deg) <= SETTLING_BAND_DEG
               for _, frame in samples[index:]):
            settling_time = samples[index][0] - arrival_time
            break
    tail_count = max(10, len(endpoint) // 5)
    tail = endpoint[-tail_count:]
    steady_error = target_deg - sum(
        math.degrees(frame[32]) for _, frame in tail
    ) / len(tail)
    equivalent_lag = tracking_rms / rate_deg_per_sec
    qualified = (
        endpoint_overshoot <= MAX_TRAJECTORY_ENDPOINT_OVERSHOOT_DEG
        and settling_time is not None
        and settling_time <= MAX_TRAJECTORY_SETTLING_SEC
        and abs(steady_error) <= MAX_STEADY_ERROR_DEG
        and equivalent_lag <= MAX_EQUIVALENT_TRACKING_LAG_SEC
    )
    frames = [frame for _, frame in samples]
    return {
        "rate_deg_per_sec": rate_deg_per_sec,
        "from_offset_deg": from_offset_deg,
        "to_offset_deg": to_offset_deg,
        "tracking_error_rms_deg": tracking_rms,
        "equivalent_tracking_lag_sec": equivalent_lag,
        "endpoint_overshoot_deg": endpoint_overshoot,
        "settling_after_reference_sec": settling_time,
        "steady_error_deg": steady_error,
        "peak_abs_speed_rpm": max(abs(proto.mechanical_rpm(frame)) for frame in frames),
        "peak_abs_iq_reference_a": max(abs(frame[22]) for frame in frames),
        "peak_actual_dq_current_a": max(math.hypot(frame[6], frame[7]) for frame in frames),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
        "qualified": qualified,
        "sample_count": len(frames),
    }


def summarize_trajectory_records(records: list[dict]) -> dict:
    return {
        "record_count": len(records),
        "qualified_count": sum(record["qualified"] for record in records),
        "qualified": all(record["qualified"] for record in records),
        "maximum_endpoint_overshoot_deg": max(
            record["endpoint_overshoot_deg"] for record in records
        ),
        "worst_settling_after_reference_sec": max(
            (record["settling_after_reference_sec"] for record in records
             if record["settling_after_reference_sec"] is not None),
            default=None,
        ),
        "maximum_equivalent_tracking_lag_sec": max(
            record["equivalent_tracking_lag_sec"] for record in records
        ),
        "maximum_absolute_steady_error_deg": max(
            abs(record["steady_error_deg"]) for record in records
        ),
        "peak_abs_iq_reference_a": max(
            record["peak_abs_iq_reference_a"] for record in records
        ),
        "peak_actual_dq_current_a": max(
            record["peak_actual_dq_current_a"] for record in records
        ),
    }


def run_trajectory_validation(port, buffer: bytearray, output: Path,
                              result: dict, feedforward_enabled: bool = False,
                              rates=TRAJECTORY_RATES_DEG_PER_SEC,
                              trajectory_mode: str = "ramp",
                              trapezoid_acceleration_time_sec: float =
                              DEFAULT_TRAPEZOID_ACCELERATION_TIME_SEC) -> None:
    candidate = SELECTED_POSITION
    rows = []
    first_record = len(result["trajectory_records"])
    profile = (
        f"ff_{'on' if feedforward_enabled else 'off'}"
        if trajectory_mode == "ramp"
        else f"{trajectory_mode}_ff_{'on' if feedforward_enabled else 'off'}"
    )
    for rate in rates:
        acceleration = (
            rate / trapezoid_acceleration_time_sec
            if trajectory_mode == "trapezoid" else None
        )
        track_command = (
            f"POS TRACK TRAP {rate:g} {acceleration:g}"
            if acceleration is not None else f"POS TRACK RAMP {rate:g}"
        )
        configure_ready(
            port, buffer, candidate, POSITION_SPEED_LIMIT_RPM,
            track_command, True, feedforward_enabled,
        )
        initial = capture(port, buffer, 0.5)
        origin_rad = initial[-1][1][30]
        previous = 0.0
        for index, offset in enumerate(TRAJECTORY_OFFSETS_DEG):
            buffer.clear()
            port.reset_input_buffer()
            proto.send_command(port, f"POSITION {offset:g}")
            duration = abs(offset - previous) / rate + 0.50
            if acceleration is not None:
                duration += 2.0 * rate / acceleration
            samples = capture(port, buffer, duration)
            record = summarize_trajectory(
                samples, origin_rad + math.radians(offset), rate, previous, offset
            )
            record.update({
                "controller": candidate["name"],
                "position_feedforward": feedforward_enabled,
                "profile": profile,
                "trajectory_mode": trajectory_mode,
                "acceleration_deg_per_sec2": acceleration,
            })
            result["trajectory_records"].append(record)
            append_trace_rows(
                rows, samples, f"rate_{rate:g}_{index:02d}", offset
            )
            save_checkpoint(output, result)
            previous = offset
        stop_ready(port, buffer)
    trace_output = output.with_name(f"{output.stem}_{profile}.csv")
    append_csv(trace_output, rows)
    records = result["trajectory_records"][first_record:]
    result["trajectory_summary"] = summarize_trajectory_records(records)


def run_trajectory_feedforward_ab(port, buffer: bytearray, output: Path,
                                  result: dict) -> None:
    summaries = {}
    for enabled, rates in (
        (False, TRAJECTORY_RATES_DEG_PER_SEC),
        (True, tuple(reversed(TRAJECTORY_RATES_DEG_PER_SEC))),
    ):
        first_record = len(result["trajectory_records"])
        run_trajectory_validation(port, buffer, output, result, enabled, rates)
        profile = f"ff_{'on' if enabled else 'off'}"
        summaries[profile] = summarize_trajectory_records(
            result["trajectory_records"][first_record:]
        )
    result["trajectory_summary"] = summaries


def run_trajectory_trapezoid_ab(port, buffer: bytearray, output: Path,
                                result: dict,
                                acceleration_time_sec: float) -> None:
    summaries = {}
    for enabled, rates in (
        (False, TRAJECTORY_RATES_DEG_PER_SEC),
        (True, tuple(reversed(TRAJECTORY_RATES_DEG_PER_SEC))),
    ):
        first_record = len(result["trajectory_records"])
        run_trajectory_validation(
            port, buffer, output, result, enabled, rates, "trapezoid",
            acceleration_time_sec,
        )
        profile = f"trapezoid_ff_{'on' if enabled else 'off'}"
        summaries[profile] = summarize_trajectory_records(
            result["trajectory_records"][first_record:]
        )
    result["trajectory_summary"] = summaries


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM30")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--default-smoke", action="store_true")
    parser.add_argument("--position-holdout", action="store_true")
    parser.add_argument("--p4-diagnostic", action="store_true")
    parser.add_argument("--trace-diagnostic", action="store_true")
    parser.add_argument("--trajectory-validation", action="store_true")
    parser.add_argument("--trajectory-feedforward-ab", action="store_true")
    parser.add_argument("--trajectory-trapezoid-ab", action="store_true")
    parser.add_argument(
        "--trapezoid-acceleration-time-sec", type=float,
        choices=TRAPEZOID_ACCELERATION_TIMES_SEC,
        default=DEFAULT_TRAPEZOID_ACCELERATION_TIME_SEC,
        help="D-046/D-047 acceleration time; only the frozen 0.10/0.20 s profiles are accepted",
    )
    args = parser.parse_args()
    test_plan = plan(args.trapezoid_acceleration_time_sec)
    if args.plan_only:
        print(json.dumps(test_plan, indent=2))
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path("tests/results") / f"stage2_position_tuning_{stamp}.json"
    result = {
        "schema": "stage2-position-tuning-result-v1", "plan": test_plan,
        "step_records": [], "controller_summaries": None,
        "selected_controller": None, "trajectory_records": [],
        "trajectory_summary": None,
        "status": "running", "error": None, "final_mode": None,
        "defaults_restored": False,
    }
    buffer = bytearray()
    port = open_serial_with_retry(args.port, args.baudrate)
    try:
        port.reset_input_buffer()
        wait_mode(port, buffer, MODE_READY, 20.0, ACTUAL_DQ_ABORT_A, 1600.0)
        try:
            if args.trajectory_trapezoid_ab:
                run_trajectory_trapezoid_ab(
                    port, buffer, output, result,
                    args.trapezoid_acceleration_time_sec,
                )
            elif args.trajectory_feedforward_ab:
                run_trajectory_feedforward_ab(port, buffer, output, result)
            elif args.trajectory_validation:
                run_trajectory_validation(port, buffer, output, result)
            elif args.trace_diagnostic:
                run_trace_diagnostic(port, buffer, output, result)
                candidate = SELECTED_POSITION
                result["controller_summaries"] = {
                    candidate["name"]: summarize_candidate(
                        candidate, result["step_records"]
                    )
                }
            elif args.p4_diagnostic:
                diagnostic = CANDIDATES[1]
                for round_index in range(2):
                    run_steps(
                        port, buffer, diagnostic, round_index, output, result,
                        DEFAULT_SMOKE_OFFSETS_DEG, True,
                    )
                records = [record for record in result["step_records"]
                           if record["controller"] == diagnostic["name"]]
                result["controller_summaries"] = {
                    diagnostic["name"]: summarize_candidate(diagnostic, records)
                }
            elif args.position_holdout:
                holdout = (CANDIDATES[2], CANDIDATES[3])
                visits = ([(0, candidate) for candidate in holdout]
                          + [(1, candidate) for candidate in reversed(holdout)])
                for round_index, candidate in visits:
                    run_steps(
                        port, buffer, candidate, round_index, output, result,
                        DEFAULT_SMOKE_OFFSETS_DEG, True,
                    )
                summaries = {}
                for candidate in holdout:
                    records = [record for record in result["step_records"]
                               if record["controller"] == candidate["name"]]
                    summaries[candidate["name"]] = summarize_candidate(
                        candidate, records
                    )
                result["controller_summaries"] = summaries
                result["selected_controller"] = select_candidate(summaries)
            elif args.default_smoke:
                run_steps(
                    port, buffer, SELECTED_POSITION, -1, output, result,
                    DEFAULT_SMOKE_OFFSETS_DEG, False,
                )
                result["controller_summaries"] = {
                    SELECTED_POSITION["name"]: summarize_candidate(
                        SELECTED_POSITION, result["step_records"]
                    )
                }
                result["selected_controller"] = SELECTED_POSITION["name"]
                run_trajectory_smoke(
                    port, buffer, SELECTED_POSITION, output, result, False
                )
            else:
                for round_index, candidate in visit_order():
                    run_steps(port, buffer, candidate, round_index, output, result)
                summaries = {}
                for candidate in CANDIDATES:
                    records = [record for record in result["step_records"]
                               if record["controller"] == candidate["name"]]
                    summaries[candidate["name"]] = summarize_candidate(candidate, records)
                result["controller_summaries"] = summaries
                selected = select_candidate(summaries)
                result["selected_controller"] = selected
                if selected is not None:
                    candidate = next(item for item in CANDIDATES if item["name"] == selected)
                    run_trajectory_smoke(port, buffer, candidate, output, result)
            result["status"] = "completed"
        except (RuntimeError, serial.SerialException) as error:
            result["status"] = "error"
            result["error"] = str(error)
        finally:
            try:
                proto.send_command(port, "STOP")
                wait_mode(port, buffer, MODE_READY, 4.0, ACTUAL_DQ_ABORT_A, 1600.0)
                for command in (
                    f"PI POSITION {SELECTED_POSITION['kp']:.9g} 0 0",
                    "POS FF OFF",
                    "POS SPEED LIMIT 5000",
                    "SPEED LIMIT 7", "POS TRACK RAMP 180", "PI CLEAR",
                ):
                    proto.send_command(port, command)
                result["defaults_restored"] = True
                result["final_mode"] = MODE_READY
            except (RuntimeError, serial.SerialException):
                result["final_mode"] = None
    finally:
        port.close()
        save_checkpoint(output, result)

    print(json.dumps({
        "result": str(output.resolve()), "status": result["status"],
        "step_records": len(result["step_records"]),
        "selected_controller": result["selected_controller"],
        "controller_summaries": result["controller_summaries"],
        "trajectory_records": len(result["trajectory_records"]),
        "trajectory_summary": result["trajectory_summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
        "error": result["error"],
    }, indent=2))
    if result["status"] != "completed" or result["final_mode"] != MODE_READY:
        raise RuntimeError(result["error"] or "position tuning did not close safely")


if __name__ == "__main__":
    main()
