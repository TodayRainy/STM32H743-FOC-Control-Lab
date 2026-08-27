#!/usr/bin/env python3
"""Run the focused Stage-4 OFF/FEEDBACK/HYBRID field-weakening matrix."""

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
from stage3_feedforward_component_validation import save_report, write_csv


MODES = ("OFF", "FEEDBACK", "HYBRID")
ROUND_MODE_ORDER = (MODES, tuple(reversed(MODES)))
ROUND_DIRECTION_ORDER = ((1, -1), (-1, 1))
SPEED_RAMP_RPM_PER_SEC = 3000.0
SPEED_IQ_LIMIT_A = 5.0
TOTAL_DQ_CURRENT_A = 5.0
DEFAULT_SPEED_IQ_LIMIT_A = 7.0
TARGET_UTILIZATION = 0.95
SCOPE_DIVIDER = 10
SCOPE_DURATION_SEC = 4096.0*SCOPE_DIVIDER/40000.0
PULSE_START_SEC = 0.100
PULSE_END_SEC = 0.550

PHASES = {
    "main": {
        "base_rpm": 4500.0,
        "pulse_rpm": 5000.0,
        "voltage_limit_ratio": 0.35,
        "abort_rpm": 5350.0,
    },
    "extension": {
        "base_rpm": 6000.0,
        "pulse_rpm": 6500.0,
        "voltage_limit_ratio": 0.45,
        "abort_rpm": 6850.0,
    },
}


def experiment_plan(phase_name: str) -> dict:
    phase = PHASES[phase_name]
    visits = []
    for round_index in (1, 2):
        for mode in ROUND_MODE_ORDER[round_index-1]:
            for direction in ROUND_DIRECTION_ORDER[round_index-1]:
                visits.append({
                    "round": round_index,
                    "mode": mode,
                    "direction": direction,
                    "base_rpm": direction*phase["base_rpm"],
                    "pulse_rpm": direction*phase["pulse_rpm"],
                })
    return {
        "schema": "stage4-field-weakening-plan-v1",
        "phase": phase_name,
        "purpose": (
            "compare standard voltage feedback with IDENTIFIED hybrid feedforward"
        ),
        "modes": list(MODES),
        "current_feedforward": "IDENTIFIED+BOTH",
        "total_dq_current_limit_a": TOTAL_DQ_CURRENT_A,
        "target_utilization": TARGET_UTILIZATION,
        "voltage_limit_ratio": phase["voltage_limit_ratio"],
        "base_speed_rpm": phase["base_rpm"],
        "pulse_speed_rpm": phase["pulse_rpm"],
        "visits": visits,
        "visit_count": len(visits),
        "scope_capture_count": len(visits),
        "scope": {
            "channels": list(scope.CHANNEL_NAMES),
            "divider": SCOPE_DIVIDER,
            "sample_rate_hz": 4000.0,
            "duration_sec": SCOPE_DURATION_SEC,
            "pulse_start_sec": PULSE_START_SEC,
            "pulse_end_sec": PULSE_END_SEC,
        },
        "safety": {
            "speed_iq_reference_limit_a": SPEED_IQ_LIMIT_A,
            "abort_actual_dq_current_a": 5.5,
            "abort_mechanical_speed_rpm": phase["abort_rpm"],
            "minimum_bus_voltage_v": 18.0,
            "maximum_bus_voltage_v": 28.0,
            "abort_voltage_scale": 0.75,
        },
        "requires_24v": True,
        "motor_will_rotate_bidirectionally": True,
        "maximum_commanded_speed_rpm": phase["pulse_rpm"],
        "finally_stop_and_confirm_ready": True,
    }


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value*value for value in values)/len(values))


def mean(values: list[float]) -> float:
    return sum(values)/len(values)


def release_time(timed, threshold=0.1):
    release_start = PULSE_END_SEC
    hold_samples = max(1, int(0.020/(SCOPE_DURATION_SEC/4096.0)))
    for index, (timestamp, sample) in enumerate(timed):
        if timestamp < release_start:
            continue
        window = timed[index:index+hold_samples]
        if len(window) == hold_samples and all(abs(item[1][5]) < threshold for item in window):
            return timestamp-release_start
    return None


def summarize_scope(mode: str, metadata: dict,
                    samples: list[tuple[float, ...]], mean_vbus: float,
                    voltage_limit_ratio: float) -> dict:
    sample_period = metadata["divider"]/float(metadata["base_sample_hz"])
    timed = [(index*sample_period, sample) for index, sample in enumerate(samples)]
    pre = [sample for timestamp, sample in timed if timestamp < 0.080]
    high = [
        sample for timestamp, sample in timed
        if 0.300 <= timestamp < PULSE_END_SEC
    ]
    tail = [sample for timestamp, sample in timed if timestamp >= 0.800]
    if not pre or not high or not tail:
        raise ValueError("field-weakening Scope windows are incomplete")

    voltage_target = mean_vbus/math.sqrt(3.0)*voltage_limit_ratio
    raw_voltage = [
        math.hypot(sample[6], sample[7])/max(sample[11], 1.0e-9)
        for sample in samples
    ]
    # The caller overwrites this phase-specific value below.
    high_indices = [
        index for index, (timestamp, _) in enumerate(timed)
        if 0.300 <= timestamp < PULSE_END_SEC
    ]
    high_voltage = [raw_voltage[index] for index in high_indices]
    target = voltage_target*TARGET_UTILIZATION
    activation = next((
        timestamp-PULSE_START_SEC for timestamp, sample in timed
        if timestamp >= PULSE_START_SEC and sample[5] < -0.1
    ), None)

    q_margins = []
    reference_magnitudes = []
    for sample in samples:
        iq_ref, id_ref = sample[4], sample[5]
        q_limit = math.sqrt(max(0.0, TOTAL_DQ_CURRENT_A**2-id_ref**2))
        q_margins.append(q_limit-abs(iq_ref))
        reference_magnitudes.append(math.hypot(iq_ref, id_ref))

    speed_errors = [
        (sample[9]-sample[10])/proto.POLE_PAIRS*60.0/(2.0*math.pi)
        for sample in high
    ]
    released = release_time(timed)
    return {
        "mode": mode,
        "sample_rate_hz": metadata["base_sample_hz"]/metadata["divider"],
        "samples": len(samples),
        "mean_vbus_v": mean_vbus,
        "voltage_target_v": target,
        "high_voltage_request_mean_v": mean(high_voltage),
        "high_voltage_error_mean_v": mean(high_voltage)-target,
        "high_voltage_error_rms_v": rms([value-target for value in high_voltage]),
        "high_id_reference_mean_a": mean([sample[5] for sample in high]),
        "high_id_current_mean_a": mean([sample[3] for sample in high]),
        "high_id_tracking_rms_a": rms([sample[3]-sample[5] for sample in high]),
        "high_iq_tracking_rms_a": rms([sample[2]-sample[4] for sample in high]),
        "high_speed_error_rms_rpm": rms(speed_errors),
        "pre_id_reference_mean_a": mean([sample[5] for sample in pre]),
        "tail_id_reference_mean_a": mean([sample[5] for sample in tail]),
        "tail_speed_error_rms_rpm": rms([
            (sample[9]-sample[10])/proto.POLE_PAIRS*60.0/(2.0*math.pi)
            for sample in tail
        ]),
        "activation_time_ms": activation*1000.0 if activation is not None else None,
        "release_time_ms": released*1000.0 if released is not None else None,
        "minimum_q_reference_margin_a": min(q_margins),
        "maximum_reference_current_a": max(reference_magnitudes),
        "peak_actual_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "minimum_voltage_scale": min(sample[11] for sample in samples),
    }


def qualify(scopes: list[dict]) -> dict:
    checks = []
    for item in scopes:
        mode = item["mode"]
        checks.extend([
            {
                "scope": item["scope_csv"],
                "metric": "reference_current_circle",
                "passed": item["maximum_reference_current_a"] <= 5.02,
            },
            {
                "scope": item["scope_csv"],
                "metric": "actual_current_peak",
                "passed": item["peak_actual_dq_current_a"] <= 5.5,
            },
            {
                "scope": item["scope_csv"],
                "metric": "tail_id_release",
                "passed": abs(item["tail_id_reference_mean_a"]) <= 0.12,
            },
        ])
        if mode == "OFF":
            checks.append({
                "scope": item["scope_csv"],
                "metric": "off_id_zero",
                "passed": abs(item["high_id_reference_mean_a"]) <= 0.05,
            })
        else:
            checks.extend([
                {
                    "scope": item["scope_csv"],
                    "metric": "steady_speed_tracking",
                    "passed": item["high_speed_error_rms_rpm"] <= 150.0,
                },
                {
                    "scope": item["scope_csv"],
                    "metric": "weakening_activated",
                    "passed": item["high_id_reference_mean_a"] <= -0.3,
                },
                {
                    "scope": item["scope_csv"],
                    "metric": "voltage_tracking",
                    "passed": abs(item["high_voltage_error_mean_v"]) <= 0.25,
                },
            ])
    return {
        "passed": all(item["passed"] for item in checks),
        "checks": checks,
    }


def capture_pulse(port, buffer, visit, started, records, limits):
    mode = visit["mode"]
    base_rpm = visit["base_rpm"]
    pulse_rpm = visit["pulse_rpm"]
    round_index = visit["round"]
    frames = []
    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, "TRACK STEP")
    scope_started = time.monotonic()
    scope.arm_scope(port, SCOPE_DIVIDER)
    pulse_sent = False
    return_sent = False
    deadline = scope_started+SCOPE_DURATION_SEC+0.05
    while time.monotonic() < deadline:
        elapsed = time.monotonic()-scope_started
        if not pulse_sent and elapsed >= PULSE_START_SEC:
            proto.send_command(port, f"SPEED {pulse_rpm:g}")
            pulse_sent = True
        if pulse_sent and not return_sent and elapsed >= PULSE_END_SEC:
            proto.send_command(port, f"SPEED {base_rpm:g}")
            return_sent = True
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic()+0.01), **limits
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode during field-weakening pulse")
        if int(round(frame[35])) != 1:
            raise RuntimeError("IDENTIFIED+BOTH current feedforward is not active")
        append_record(
            records, started, timestamp, round_index, mode,
            f"fw_pulse_{base_rpm:+g}_to_{pulse_rpm:+g}",
            pulse_rpm if not return_sent else base_rpm, frame,
        )
        frames.append(frame)
    if not pulse_sent or not return_sent or not frames:
        raise RuntimeError("field-weakening pulse sequence was incomplete")
    return mean([frame[29] for frame in frames])


def stop_and_restore(port, buffer, started, records, limits):
    buffer.clear()
    for command in ("SPEED 0", "STOP"):
        proto.send_command(port, command)
    final = wait_mode(
        port, buffer, MODE_READY, 5.0, started, records,
        0, "cleanup", "final_ready", limits,
    )
    for command in (
        "FW MODE HYBRID", "VOLT LIMIT 1", "FF MODEL IDENTIFIED",
        "FF MODE BOTH", f"SPEED LIMIT {DEFAULT_SPEED_IQ_LIMIT_A:g}",
        "SPEED EST WIN", "PI CLEAR",
    ):
        proto.send_command(port, command)
    wait_mode(
        port, buffer, MODE_READY, 2.0, started, records,
        0, "cleanup", "defaults_restored", limits,
    )
    return int(round(final[27]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=tuple(PHASES), default="main")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    plan = experiment_plan(args.phase)
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage4_field_weakening_{args.phase}_{stamp}.json"
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
    records = []
    scopes = []
    result = {
        "schema": "stage4-field-weakening-result-v1",
        "plan": plan,
        "telemetry_csv": str(telemetry_path.resolve()),
        "scope_directory": str(scope_dir.resolve()),
        "scopes": scopes,
        "qualification": None,
        "status": "running",
        "error": None,
        "final_mode": None,
        "defaults_restored": False,
    }
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(
            args.port, args.baudrate, timeout=0.02, write_timeout=0.2
        ) as port:
            port.reset_input_buffer()
            try:
                wait_mode(
                    port, buffer, MODE_READY, 20.0, started, records,
                    0, "setup", "initial_ready", limits,
                )
                for command in (
                    "SPEED EST WIN", f"SPEED LIMIT {SPEED_IQ_LIMIT_A:g}",
                    "FF MODEL IDENTIFIED", "FF MODE BOTH",
                ):
                    proto.send_command(port, command)

                for visit in plan["visits"]:
                    mode = visit["mode"]
                    round_index = visit["round"]
                    base_rpm = visit["base_rpm"]
                    for command in (
                        f"FW MODE {mode}",
                        f"VOLT LIMIT {plan['voltage_limit_ratio']:g}",
                        f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}",
                        "PI CLEAR", "START SPEED",
                    ):
                        proto.send_command(port, command)
                    wait_mode(
                        port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                        started, records, round_index, mode,
                        "start_speed", limits,
                    )
                    proto.send_command(port, f"SPEED {base_rpm:g}")
                    capture_transition(
                        port, buffer, base_rpm,
                        abs(base_rpm)/SPEED_RAMP_RPM_PER_SEC+4.0,
                        0.5, started, records, round_index, mode, limits,
                        stable_tolerance_rpm=120.0,
                    )
                    mean_vbus = capture_pulse(
                        port, buffer, visit, started, records, limits
                    )
                    proto.send_command(port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}")
                    proto.send_command(port, "SPEED 0")
                    capture_transition(
                        port, buffer, 0.0,
                        abs(base_rpm)/SPEED_RAMP_RPM_PER_SEC+4.0,
                        0.4, started, records, round_index, mode, limits,
                        stable_tolerance_rpm=120.0,
                    )
                    proto.send_command(port, "STOP")
                    wait_mode(
                        port, buffer, MODE_READY, 3.0,
                        started, records, round_index, mode,
                        "ready_after_visit", limits,
                    )

                    metadata, samples = scope.dump_scope(port, 25.0)
                    scope_dir.mkdir(parents=True, exist_ok=True)
                    scope_path = scope_dir/(
                        f"r{round_index}_{mode.lower()}_"
                        f"{base_rpm:+g}_pulse_{visit['pulse_rpm']:+g}rpm.csv"
                    )
                    scope.write_csv(
                        scope_path, samples,
                        metadata["divider"]/float(metadata["base_sample_hz"]),
                    )
                    summary = summarize_scope(
                        mode, metadata, samples, mean_vbus,
                        plan["voltage_limit_ratio"],
                    )
                    summary.update({
                        "round": round_index,
                        "direction": visit["direction"],
                        "base_rpm": base_rpm,
                        "pulse_rpm": visit["pulse_rpm"],
                        "scope_csv": str(scope_path.resolve()),
                    })
                    scopes.append(summary)
                    write_csv(telemetry_path, records)
                    save_report(output, result)

                result["qualification"] = qualify(scopes)
                result["status"] = "completed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "error"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    result["final_mode"] = stop_and_restore(
                        port, buffer, started, records, limits
                    )
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    except serial.SerialException as error:
        result["status"] = "error"
        result["error"] = str(error)
    finally:
        if records:
            write_csv(telemetry_path, records)
        save_report(output, result)

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["defaults_restored"]:
        raise RuntimeError("field-weakening defaults were not restored")
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "field-weakening matrix failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "qualification_passed": result["qualification"]["passed"],
        "scopes": len(scopes),
        "minimum_voltage_scale": min(
            item["minimum_voltage_scale"] for item in scopes
        ),
        "peak_actual_dq_current_a": max(
            item["peak_actual_dq_current_a"] for item in scopes
        ),
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
