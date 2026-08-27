#!/usr/bin/env python3
"""Run the focused Stage-3 OFF/BOTH voltage-limit pulse matrix."""

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
from stage2_current_combination_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    append_record,
    capture_transition,
    read_checked,
    wait_mode,
)
from stage3_feedforward_component_validation import (
    FF_MODES,
    IDENTIFIED_FLUX_WB,
    IDENTIFIED_LD_H,
    IDENTIFIED_LQ_H,
    TEST_SPEED_IQ_LIMIT_A,
    check_mode,
    save_report,
    stop_and_restore,
    write_csv,
)


VOLTAGE_LIMIT_RATIO = 0.22
D057_PEAK_REQUESTED_UTILIZATION = 0.25349838801363195
BASE_SPEED_RPM = 3000.0
PULSE_SPEED_RPM = 3300.0
SPEED_RAMP_RPM_PER_SEC = 3000.0
SCOPE_DIVIDER = 10
SCOPE_DURATION_SEC = 4096.0 * SCOPE_DIVIDER / 40000.0
PULSE_START_SEC = 0.100
PULSE_END_SEC = 0.140

ROUND_MODE_ORDER = (("OFF", "BOTH"), ("BOTH", "OFF"))
ROUND_DIRECTION_ORDER = ((1, -1), (-1, 1))


def experiment_plan() -> dict:
    visits = []
    for round_index in (1, 2):
        for mode in ROUND_MODE_ORDER[round_index - 1]:
            for direction in ROUND_DIRECTION_ORDER[round_index - 1]:
                visits.append({
                    "round": round_index,
                    "mode": mode,
                    "mode_code": FF_MODES[mode],
                    "direction": direction,
                    "base_rpm": direction * BASE_SPEED_RPM,
                    "pulse_rpm": direction * PULSE_SPEED_RPM,
                })
    return {
        "schema": "stage3-voltage-saturation-plan-v1",
        "purpose": "validate vector limiting and PI tracking entry/exit with OFF and BOTH",
        "model": "IDENTIFIED",
        "modes": ["OFF", "BOTH"],
        "voltage_limit_ratio": VOLTAGE_LIMIT_RATIO,
        "d057_peak_requested_utilization": D057_PEAK_REQUESTED_UTILIZATION,
        "predicted_minimum_voltage_scale": (
            VOLTAGE_LIMIT_RATIO / D057_PEAK_REQUESTED_UTILIZATION
        ),
        "visits": visits,
        "visit_count": len(visits),
        "scope_capture_count": len(visits),
        "scope": {
            "channels": list(scope.CHANNEL_NAMES),
            "divider": SCOPE_DIVIDER,
            "sample_rate_hz": 40000,
            "samples": 4096,
            "duration_sec": SCOPE_DURATION_SEC,
            "pulse_start_sec": PULSE_START_SEC,
            "pulse_end_sec": PULSE_END_SEC,
            "pulse_duration_sec": PULSE_END_SEC - PULSE_START_SEC,
        },
        "safety": {
            "speed_iq_reference_limit_a": TEST_SPEED_IQ_LIMIT_A,
            "abort_actual_dq_current_a": 5.0,
            "abort_mechanical_speed_rpm": 3500.0,
            "minimum_bus_voltage_v": 18.0,
            "maximum_bus_voltage_v": 28.0,
            "abort_voltage_scale": 0.70,
        },
        "requires_24v": True,
        "motor_will_rotate": True,
        "maximum_commanded_speed_rpm": PULSE_SPEED_RPM,
        "finally_stop_and_confirm_ready": True,
        "restore_voltage_limit_ratio": 1.0,
        "restore_identified_model_and_ff_both": True,
    }


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def feedforward(mode: str, sample: tuple[float, ...]) -> tuple[float, float]:
    if mode == "OFF":
        return 0.0, 0.0
    omega_e = sample[9]
    return (
        omega_e * (IDENTIFIED_FLUX_WB + IDENTIFIED_LD_H * sample[3]),
        -omega_e * IDENTIFIED_LQ_H * sample[2],
    )


def longest_true_run(values: list[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def summarize_scope(mode: str, metadata: dict,
                    samples: list[tuple[float, ...]]) -> dict:
    sample_period = metadata["divider"] / float(metadata["base_sample_hz"])
    timed = [(index * sample_period, sample) for index, sample in enumerate(samples)]
    pre = [sample for timestamp, sample in timed if timestamp < PULSE_START_SEC]
    pulse = [
        sample for timestamp, sample in timed
        if PULSE_START_SEC <= timestamp < PULSE_END_SEC
    ]
    recovery = [sample for timestamp, sample in timed if timestamp >= PULSE_END_SEC]
    tail = [sample for timestamp, sample in timed if timestamp >= SCOPE_DURATION_SEC - 0.020]
    if not pre or not pulse or not recovery or not tail:
        raise ValueError("scope windows are incomplete")

    scales = [sample[11] for sample in samples]
    limited = [value < 0.999 for value in scales]
    post_return_limited = [
        timestamp for timestamp, sample in timed
        if timestamp >= PULSE_END_SEC and sample[11] < 0.999
    ]
    recovery_scale_time = (
        post_return_limited[-1] - PULSE_END_SEC + sample_period
        if post_return_limited else 0.0
    )

    def pi_voltage(sample: tuple[float, ...]) -> tuple[float, float]:
        uq_ff, ud_ff = feedforward(mode, sample)
        return sample[6] - uq_ff, sample[7] - ud_ff

    pre_pi = [pi_voltage(sample) for sample in pre]
    tail_pi = [pi_voltage(sample) for sample in tail]
    return {
        "mode": mode,
        "mode_code": FF_MODES[mode],
        "sample_rate_hz": metadata["base_sample_hz"] / metadata["divider"],
        "samples": len(samples),
        "minimum_voltage_scale": min(scales),
        "limited_sample_fraction": sum(limited) / len(limited),
        "longest_limited_duration_ms": longest_true_run(limited) * sample_period * 1000.0,
        "post_return_scale_recovery_ms": recovery_scale_time * 1000.0,
        "tail_minimum_voltage_scale": min(sample[11] for sample in tail),
        "pulse_iq_error_rms_a": rms([sample[2] - sample[4] for sample in pulse]),
        "pulse_id_error_rms_a": rms([sample[3] - sample[5] for sample in pulse]),
        "pre_iq_error_rms_a": rms([sample[2] - sample[4] for sample in pre]),
        "pre_id_error_rms_a": rms([sample[3] - sample[5] for sample in pre]),
        "recovery_iq_error_rms_a": rms([
            sample[2] - sample[4] for sample in recovery
        ]),
        "recovery_id_error_rms_a": rms([
            sample[3] - sample[5] for sample in recovery
        ]),
        "tail_iq_error_rms_a": rms([sample[2] - sample[4] for sample in tail]),
        "tail_id_error_rms_a": rms([sample[3] - sample[5] for sample in tail]),
        "pre_speed_error_rms_rpm": rms([
            (sample[9] - sample[10]) / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi)
            for sample in pre
        ]),
        "tail_speed_error_rms_rpm": rms([
            (sample[9] - sample[10]) / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi)
            for sample in tail
        ]),
        "pre_pi_voltage_mean_v": [
            mean([value[0] for value in pre_pi]),
            mean([value[1] for value in pre_pi]),
        ],
        "tail_pi_voltage_mean_v": [
            mean([value[0] for value in tail_pi]),
            mean([value[1] for value in tail_pi]),
        ],
        "pre_to_tail_pi_offset_v": math.hypot(
            mean([value[0] for value in tail_pi]) - mean([value[0] for value in pre_pi]),
            mean([value[1] for value in tail_pi]) - mean([value[1] for value in pre_pi]),
        ),
        "peak_dq_current_a": max(math.hypot(sample[2], sample[3]) for sample in samples),
        "peak_abs_speed_rpm": max(
            abs(sample[9]) / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi)
            for sample in samples
        ),
    }


def capture_voltage_pulse(port, buffer: bytearray, visit: dict,
                          started: float, records: list[dict], limits: dict) -> None:
    base_rpm = float(visit["base_rpm"])
    pulse_rpm = float(visit["pulse_rpm"])
    mode = str(visit["mode"])
    round_index = int(visit["round"])

    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, "TRACK STEP")
    scope_started = time.monotonic()
    scope.arm_scope(port, SCOPE_DIVIDER)
    pulse_sent = False
    return_sent = False
    deadline = scope_started + SCOPE_DURATION_SEC + 0.05
    while time.monotonic() < deadline:
        elapsed = time.monotonic() - scope_started
        if not pulse_sent and elapsed >= PULSE_START_SEC:
            proto.send_command(port, f"SPEED {pulse_rpm:g}")
            pulse_sent = True
        if pulse_sent and not return_sent and elapsed >= PULSE_END_SEC:
            proto.send_command(port, f"SPEED {base_rpm:g}")
            return_sent = True
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.01), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode during voltage pulse")
        check_mode(frame, mode)
        append_record(
            records, started, timestamp, round_index, mode,
            f"voltage_pulse_{base_rpm:+g}_to_{pulse_rpm:+g}",
            pulse_rpm if not return_sent else base_rpm, frame,
        )
    if not pulse_sent or not return_sent:
        raise RuntimeError("voltage pulse command sequence was incomplete")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    plan = experiment_plan()
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / (
        f"stage3_voltage_saturation_{stamp}.json"
    )
    telemetry_path = output.with_suffix(".csv")
    scope_dir = output.with_name(f"{output.stem}_scope")
    limits = {
        "max_current": plan["safety"]["abort_actual_dq_current_a"],
        "max_rpm": plan["safety"]["abort_mechanical_speed_rpm"],
        "min_vbus": plan["safety"]["minimum_bus_voltage_v"],
        "max_vbus": plan["safety"]["maximum_bus_voltage_v"],
        "min_voltage_scale": plan["safety"]["abort_voltage_scale"],
    }
    records: list[dict] = []
    scopes: list[dict] = []
    result = {
        "schema": "stage3-voltage-saturation-result-v1",
        "plan": plan,
        "telemetry_csv": str(telemetry_path.resolve()),
        "scope_directory": str(scope_dir.resolve()),
        "scopes": scopes,
        "status": "running",
        "error": None,
        "final_mode": None,
        "defaults_restored": False,
    }
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(
            args.port, args.baudrate, timeout=0.02, write_timeout=0.2,
        ) as port:
            port.reset_input_buffer()
            try:
                wait_mode(
                    port, buffer, MODE_READY, 20.0, started, records,
                    0, "setup", "initial_ready", limits,
                )
                for command in (
                    "SPEED EST WIN",
                    f"SPEED LIMIT {TEST_SPEED_IQ_LIMIT_A:g}",
                    "FF MODEL IDENTIFIED",
                ):
                    proto.send_command(port, command)

                for visit in plan["visits"]:
                    mode = str(visit["mode"])
                    round_index = int(visit["round"])
                    base_rpm = float(visit["base_rpm"])
                    buffer.clear()
                    port.reset_input_buffer()
                    for command in (
                        f"FF MODE {mode}",
                        f"VOLT LIMIT {VOLTAGE_LIMIT_RATIO:g}",
                        f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}",
                        "PI CLEAR",
                        "START SPEED",
                    ):
                        proto.send_command(port, command)
                    wait_mode(
                        port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                        started, records, round_index, mode,
                        "start_speed", limits,
                    )
                    proto.send_command(port, f"SPEED {base_rpm:g}")
                    transition = capture_transition(
                        port, buffer, base_rpm,
                        abs(base_rpm) / SPEED_RAMP_RPM_PER_SEC + 4.0,
                        0.5, started, records, round_index, mode, limits,
                        stable_tolerance_rpm=90.0,
                    )
                    for _, frame in transition:
                        check_mode(frame, mode)

                    capture_voltage_pulse(
                        port, buffer, visit, started, records, limits,
                    )
                    proto.send_command(port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}")
                    proto.send_command(port, f"SPEED {base_rpm:g}")
                    capture_transition(
                        port, buffer, base_rpm, 4.0, 0.4,
                        started, records, round_index, mode, limits,
                        stable_tolerance_rpm=90.0,
                    )
                    proto.send_command(port, "SPEED 0")
                    capture_transition(
                        port, buffer, 0.0,
                        abs(base_rpm) / SPEED_RAMP_RPM_PER_SEC + 4.0,
                        0.4, started, records, round_index, mode, limits,
                        stable_tolerance_rpm=90.0,
                    )
                    proto.send_command(port, "STOP")
                    wait_mode(
                        port, buffer, MODE_READY, 3.0,
                        started, records, round_index, mode,
                        "ready_after_visit", limits,
                    )
                    proto.send_command(port, "VOLT LIMIT 1")

                    metadata, samples = scope.dump_scope(port, 25.0)
                    scope_dir.mkdir(parents=True, exist_ok=True)
                    scope_path = scope_dir / (
                        f"r{round_index}_{mode.lower()}_"
                        f"{base_rpm:+g}_pulse_{float(visit['pulse_rpm']):+g}rpm.csv"
                    )
                    scope.write_csv(
                        scope_path, samples,
                        metadata["divider"] / float(metadata["base_sample_hz"]),
                    )
                    summary = summarize_scope(mode, metadata, samples)
                    summary.update({
                        "round": round_index,
                        "direction": int(visit["direction"]),
                        "base_rpm": base_rpm,
                        "pulse_rpm": float(visit["pulse_rpm"]),
                        "scope_csv": str(scope_path.resolve()),
                    })
                    scopes.append(summary)
                    write_csv(telemetry_path, records)
                    save_report(output, result)

                result["status"] = "completed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "error"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                final_mode, restored, cleanup_error = stop_and_restore(
                    port, args.port, args.baudrate, buffer,
                    started, records, limits,
                )
                result["final_mode"] = final_mode
                result["defaults_restored"] = restored
                if cleanup_error is not None:
                    result["error"] = result["error"] or cleanup_error
    except serial.SerialException as error:
        result["status"] = "error"
        result["error"] = str(error)
    finally:
        write_csv(telemetry_path, records)
        save_report(output, result)

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["defaults_restored"]:
        raise RuntimeError("voltage limit/model/feedforward defaults were not restored")
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "Stage-3 voltage matrix failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "scopes": len(scopes),
        "minimum_voltage_scale": min(item["minimum_voltage_scale"] for item in scopes),
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2))


if __name__ == "__main__":
    main()
