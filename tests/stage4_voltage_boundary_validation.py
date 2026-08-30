#!/usr/bin/env python3
"""Validate field weakening during a fixed-speed voltage-boundary step."""

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
from host.foc_client import CommandResult, StreamParser
from stage2_current_combination_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    append_record,
    capture_transition,
    read_checked,
    wait_mode,
)
from stage3_feedforward_component_validation import save_report, write_csv
from stage4_field_weakening_validation import stop_and_restore


SPEED_RPM = 6500.0
SPEED_RAMP_RPM_PER_SEC = 3000.0
SPEED_IQ_LIMIT_A = 5.0
TOTAL_DQ_CURRENT_A = 5.0
HIGH_VOLTAGE_RATIO = 1.0
LOW_VOLTAGE_RATIO = 0.45
TARGET_UTILIZATION = 0.95
SCOPE_DIVIDER = 10
SCOPE_SAMPLE_PERIOD_SEC = SCOPE_DIVIDER/40000.0
SCOPE_DURATION_SEC = 4096.0*SCOPE_SAMPLE_PERIOD_SEC
BOUNDARY_DOWN_SEC = 0.200
BOUNDARY_UP_SEC = 0.650
HOLD_SEC = 0.020
COMMAND_GAP_SEC = 0.020

VISITS = (
    {"visit": 1, "mode": "FEEDBACK", "direction": 1},
    {"visit": 2, "mode": "HYBRID", "direction": -1},
    {"visit": 3, "mode": "HYBRID", "direction": 1},
    {"visit": 4, "mode": "FEEDBACK", "direction": -1},
)


def send_sequence(port, commands):
    """Pace command bursts for the firmware's single USB RX command slot."""
    commands = tuple(commands)
    for index, command in enumerate(commands):
        proto.send_command(port, command)
        if index+1 < len(commands):
            time.sleep(COMMAND_GAP_SEC)


def send_and_expect(port, command, expected_name, timeout=1.0):
    """Send a command and require its structured firmware acknowledgement."""
    port.reset_input_buffer()
    parser = StreamParser()
    proto.send_command(port, command)
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        events = parser.feed(port.read(port.in_waiting or 1))
        for event in events:
            if isinstance(event, CommandResult) and event.name == expected_name:
                if not event.accepted:
                    raise RuntimeError(
                        f"{command} rejected by firmware: {event.reason}"
                    )
                port.reset_input_buffer()
                return
    raise RuntimeError(f"timeout waiting for {expected_name} acknowledgement")


def experiment_plan() -> dict:
    visits = [{
        **item,
        "speed_rpm": item["direction"]*SPEED_RPM,
    } for item in VISITS]
    return {
        "schema": "stage4-dynamic-voltage-boundary-plan-v1",
        "purpose": "fixed-speed voltage-boundary entry and release",
        "current_feedforward": "IDENTIFIED+BOTH",
        "modes": ["FEEDBACK", "HYBRID"],
        "speed_rpm": SPEED_RPM,
        "high_voltage_ratio": HIGH_VOLTAGE_RATIO,
        "low_voltage_ratio": LOW_VOLTAGE_RATIO,
        "target_utilization": TARGET_UTILIZATION,
        "total_dq_current_limit_a": TOTAL_DQ_CURRENT_A,
        "visits": visits,
        "visit_count": len(visits),
        "scope_capture_count": len(visits),
        "scope": {
            "channels": list(scope.CHANNEL_NAMES),
            "divider": SCOPE_DIVIDER,
            "sample_rate_hz": 4000.0,
            "duration_sec": SCOPE_DURATION_SEC,
            "boundary_down_sec": BOUNDARY_DOWN_SEC,
            "boundary_up_sec": BOUNDARY_UP_SEC,
        },
        "safety": {
            "speed_iq_reference_limit_a": SPEED_IQ_LIMIT_A,
            "abort_actual_dq_current_a": 5.5,
            "abort_mechanical_speed_rpm": 6850.0,
            "minimum_bus_voltage_v": 18.0,
            "maximum_bus_voltage_v": 28.0,
            "abort_voltage_scale": 0.75,
        },
        "requires_24v": True,
        "motor_will_rotate_bidirectionally": True,
        "maximum_commanded_speed_rpm": SPEED_RPM,
        "finally_stop_and_confirm_ready": True,
    }


def mean(values):
    return sum(values)/len(values)


def rms(values):
    return math.sqrt(mean([value*value for value in values]))


def first_held(timed_values, start_time, predicate, hold_sec=HOLD_SEC):
    hold_samples = max(1, int(math.ceil(hold_sec/SCOPE_SAMPLE_PERIOD_SEC)))
    for index, (timestamp, _) in enumerate(timed_values):
        if timestamp < start_time:
            continue
        window = timed_values[index:index+hold_samples]
        if (len(window) == hold_samples
                and all(predicate(value) for _, value in window)):
            return timestamp-start_time
    return None


def first_window_mean(timed_values, start_time, predicate, window_sec=HOLD_SEC):
    window_samples = max(1, int(math.ceil(window_sec/SCOPE_SAMPLE_PERIOD_SEC)))
    for index, (timestamp, _) in enumerate(timed_values):
        if timestamp < start_time:
            continue
        window = timed_values[index:index+window_samples]
        if (len(window) == window_samples
                and predicate(mean([value for _, value in window]))):
            return timestamp-start_time
    return None


def longest_duration(timed_values, predicate):
    longest = 0
    current = 0
    for _, value in timed_values:
        if predicate(value):
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest*SCOPE_SAMPLE_PERIOD_SEC


def summarize_scope(mode: str, direction: int, metadata: dict,
                    samples: list[tuple[float, ...]], mean_vbus: float) -> dict:
    sample_period = metadata["divider"]/float(metadata["base_sample_hz"])
    timed = [(index*sample_period, sample) for index, sample in enumerate(samples)]
    pre = [sample for timestamp, sample in timed if 0.080 <= timestamp < 0.180]
    low = [sample for timestamp, sample in timed if 0.400 <= timestamp < 0.600]
    tail = [sample for timestamp, sample in timed if 0.820 <= timestamp < 1.000]
    if not pre or not low or not tail:
        raise ValueError("dynamic voltage-boundary Scope windows are incomplete")

    low_target = (mean_vbus/math.sqrt(3.0)
                  * LOW_VOLTAGE_RATIO*TARGET_UTILIZATION)
    raw_voltage = [
        math.hypot(sample[6], sample[7])/max(sample[11], 1.0e-9)
        for sample in samples
    ]
    timed_id_ref = [(timestamp, sample[5]) for timestamp, sample in timed]
    timed_voltage_error = [
        (timestamp, raw_voltage[index]-low_target)
        for index, (timestamp, _) in enumerate(timed)
    ]
    timed_scale = [(timestamp, sample[11]) for timestamp, sample in timed]

    def speed_error_rpm(sample):
        return ((sample[9]-sample[10])/proto.POLE_PAIRS
                * 60.0/(2.0*math.pi))

    reference_current = [math.hypot(sample[4], sample[5]) for sample in samples]
    actual_current = [math.hypot(sample[2], sample[3]) for sample in samples]
    low_indices = [
        index for index, (timestamp, _) in enumerate(timed)
        if 0.400 <= timestamp < 0.600
    ]
    activation = first_held(
        timed_id_ref, BOUNDARY_DOWN_SEC, lambda value: value < -0.1
    )
    release = first_held(
        timed_id_ref, BOUNDARY_UP_SEC, lambda value: abs(value) < 0.1
    )
    # The reconstructed voltage vector contains normal current/PWM ripple.
    # Settling therefore applies to its 10 ms mean, not every raw 4 kHz sample.
    voltage_settle = first_window_mean(
        timed_voltage_error, BOUNDARY_DOWN_SEC,
        lambda value: abs(value) <= 0.25,
    )

    return {
        "mode": mode,
        "direction": direction,
        "speed_rpm": direction*SPEED_RPM,
        "sample_rate_hz": 1.0/sample_period,
        "samples": len(samples),
        "mean_vbus_v": mean_vbus,
        "low_voltage_target_v": low_target,
        "pre_id_reference_mean_a": mean([sample[5] for sample in pre]),
        "low_id_reference_mean_a": mean([sample[5] for sample in low]),
        "tail_id_reference_mean_a": mean([sample[5] for sample in tail]),
        "low_voltage_request_mean_v": mean([raw_voltage[index] for index in low_indices]),
        "low_voltage_error_mean_v": (
            mean([raw_voltage[index] for index in low_indices])-low_target
        ),
        "low_voltage_error_rms_v": rms([
            raw_voltage[index]-low_target for index in low_indices
        ]),
        "pre_speed_error_rms_rpm": rms([speed_error_rpm(sample) for sample in pre]),
        "low_speed_error_rms_rpm": rms([speed_error_rpm(sample) for sample in low]),
        "tail_speed_error_rms_rpm": rms([speed_error_rpm(sample) for sample in tail]),
        "low_peak_speed_error_rpm": max(abs(speed_error_rpm(sample)) for sample in low),
        "activation_time_ms": activation*1000.0 if activation is not None else None,
        "voltage_settling_time_ms": (
            voltage_settle*1000.0 if voltage_settle is not None else None
        ),
        "release_time_ms": release*1000.0 if release is not None else None,
        "low_hard_saturation_fraction": mean([
            1.0 if sample[11] < 0.999 else 0.0 for sample in low
        ]),
        "longest_hard_saturation_ms": 1000.0*longest_duration(
            timed_scale, lambda value: value < 0.999
        ),
        "minimum_voltage_scale": min(sample[11] for sample in samples),
        "maximum_reference_current_a": max(reference_current),
        "peak_actual_dq_current_a": max(actual_current),
    }


def qualify(scopes: list[dict]) -> dict:
    checks = []
    for item in scopes:
        timed_metrics = (
            ("activation_time", item["activation_time_ms"], 50.0),
            ("voltage_settling_time", item["voltage_settling_time_ms"], 100.0),
            ("release_time", item["release_time_ms"], 50.0),
        )
        checks.extend([
            {"scope": item["scope_csv"], "metric": "reference_current_circle",
             "passed": item["maximum_reference_current_a"] <= 5.02},
            {"scope": item["scope_csv"], "metric": "actual_current_peak",
             "passed": item["peak_actual_dq_current_a"] <= 5.5},
            {"scope": item["scope_csv"], "metric": "pre_id_zero",
             "passed": abs(item["pre_id_reference_mean_a"]) <= 0.12},
            {"scope": item["scope_csv"], "metric": "weakening_activated",
             "passed": item["low_id_reference_mean_a"] <= -0.3},
            {"scope": item["scope_csv"], "metric": "voltage_tracking",
             "passed": abs(item["low_voltage_error_mean_v"]) <= 0.25},
            {"scope": item["scope_csv"], "metric": "steady_speed_tracking",
             "passed": item["low_speed_error_rms_rpm"] <= 150.0},
            {"scope": item["scope_csv"], "metric": "tail_id_release",
             "passed": abs(item["tail_id_reference_mean_a"]) <= 0.12},
            {"scope": item["scope_csv"], "metric": "tail_speed_tracking",
             "passed": item["tail_speed_error_rms_rpm"] <= 150.0},
        ])
        for metric, value, limit in timed_metrics:
            checks.append({
                "scope": item["scope_csv"],
                "metric": metric,
                "passed": value is not None and value <= limit,
            })
    return {"passed": all(item["passed"] for item in checks), "checks": checks}


def capture_boundary(port, buffer, visit, started, records, limits):
    speed_rpm = visit["speed_rpm"]
    frames = []
    buffer.clear()
    port.reset_input_buffer()
    scope_origin = time.monotonic()
    scope.arm_scope(port, SCOPE_DIVIDER)
    boundary_down_sent = False
    boundary_up_sent = False
    deadline = scope_origin+SCOPE_DURATION_SEC+0.05
    while time.monotonic() < deadline:
        elapsed = time.monotonic()-scope_origin
        if not boundary_down_sent and elapsed >= BOUNDARY_DOWN_SEC:
            proto.send_command(port, f"VOLT LIMIT {LOW_VOLTAGE_RATIO:g}")
            boundary_down_sent = True
        if (boundary_down_sent and not boundary_up_sent
                and elapsed >= BOUNDARY_UP_SEC):
            proto.send_command(port, f"VOLT LIMIT {HIGH_VOLTAGE_RATIO:g}")
            boundary_up_sent = True
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic()+0.01), **limits
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode during voltage-boundary step")
        if int(round(frame[35])) != 1:
            raise RuntimeError("IDENTIFIED+BOTH current feedforward is not active")
        phase = (
            "boundary_high_pre" if not boundary_down_sent
            else "boundary_low" if not boundary_up_sent
            else "boundary_high_post"
        )
        append_record(
            records, started, timestamp, visit["visit"], visit["mode"],
            phase, speed_rpm, frame,
        )
        frames.append(frame)
    if not boundary_down_sent or not boundary_up_sent or not frames:
        raise RuntimeError("dynamic voltage-boundary sequence was incomplete")
    return mean([frame[29] for frame in frames])


def main():
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
    output = args.output or Path("tests/results")/(
        f"stage4_voltage_boundary_{stamp}.json"
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
        "schema": "stage4-dynamic-voltage-boundary-result-v1",
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
                proto.send_command(port, "SPEED EST WIN")
                send_and_expect(
                    port, f"SPEED LIMIT {SPEED_IQ_LIMIT_A:g}", "SPEED_LIMIT"
                )
                send_and_expect(port, "FF MODEL IDENTIFIED", "FF_MODEL")
                send_and_expect(port, "FF MODE BOTH", "FF_MODE")

                for visit in plan["visits"]:
                    speed_rpm = visit["speed_rpm"]
                    send_and_expect(
                        port, f"FW MODE {visit['mode']}", "FW_MODE"
                    )
                    send_and_expect(
                        port, f"VOLT LIMIT {HIGH_VOLTAGE_RATIO:g}",
                        "VOLT_LIMIT",
                    )
                    send_sequence(port, (
                        f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}", "PI CLEAR",
                    ))
                    send_and_expect(port, "START SPEED", "START_SPEED")
                    wait_mode(
                        port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                        started, records, visit["visit"], visit["mode"],
                        "start_speed", limits,
                    )
                    send_and_expect(port, f"SPEED {speed_rpm:g}", "SPEED")
                    capture_transition(
                        port, buffer, speed_rpm,
                        abs(speed_rpm)/SPEED_RAMP_RPM_PER_SEC+4.0,
                        0.5, started, records, visit["visit"], visit["mode"],
                        limits, stable_tolerance_rpm=180.0,
                    )
                    mean_vbus = capture_boundary(
                        port, buffer, visit, started, records, limits
                    )
                    proto.send_command(
                        port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_SEC:g}"
                    )
                    send_and_expect(port, "SPEED 0", "SPEED")
                    capture_transition(
                        port, buffer, 0.0,
                        abs(speed_rpm)/SPEED_RAMP_RPM_PER_SEC+4.0,
                        0.4, started, records, visit["visit"], visit["mode"],
                        limits, stable_tolerance_rpm=120.0,
                    )
                    send_and_expect(port, "STOP", "STOP")
                    wait_mode(
                        port, buffer, MODE_READY, 3.0,
                        started, records, visit["visit"], visit["mode"],
                        "ready_after_visit", limits,
                    )

                    metadata, samples = scope.dump_scope(port, 25.0)
                    scope_dir.mkdir(parents=True, exist_ok=True)
                    scope_path = scope_dir/(
                        f"v{visit['visit']}_{visit['mode'].lower()}_"
                        f"{speed_rpm:+g}rpm_ratio_1_to_045_to_1.csv"
                    )
                    scope.write_csv(
                        scope_path, samples,
                        metadata["divider"]/float(metadata["base_sample_hz"]),
                    )
                    summary = summarize_scope(
                        visit["mode"], visit["direction"], metadata,
                        samples, mean_vbus,
                    )
                    summary.update({
                        "visit": visit["visit"],
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
        raise RuntimeError(result["error"] or "dynamic voltage-boundary test failed")
    if not result["qualification"]["passed"]:
        raise RuntimeError("dynamic voltage-boundary qualification failed")
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
