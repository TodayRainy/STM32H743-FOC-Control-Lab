#!/usr/bin/env python3
"""Compare the active current loop with the Stage-2 PI/feedforward combination."""

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
import scope_capture as scope
from stage2_current_loop_design import build_controllers


MODE_ABZENC_CALIB = 6
MODE_SPEED_TORQUE_RUN = 12
MODE_READY = 17
TRANSITION_CURRENT_LIMIT_A = 5.0
PROFILE_ORDER = (
    "active_ff_off",
    "active_ff_identified",
    "identified_2000hz_ff_identified",
)
PROFILE_SETTINGS = {
    "active_ff_off": ("active_legacy", "LEGACY", False),
    "active_ff_identified": ("active_legacy", "IDENTIFIED", True),
    "identified_2000hz_ff_identified": (
        "identified_2000hz", "IDENTIFIED", True,
    ),
}


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def mean(values):
    return sum(values) / len(values)


def append_record(records, started, timestamp, round_index, profile,
                  phase, target_rpm, frame):
    records.append({
        "elapsed_sec": timestamp - started,
        "round": round_index,
        "profile": profile,
        "phase": phase,
        "target_rpm": target_rpm,
        "actual_rpm": proto.mechanical_rpm(frame),
        "iq": frame[6],
        "id": frame[7],
        "uq": frame[20],
        "ud": frame[21],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "voltage_scale": frame[26],
        "motor_mode": int(round(frame[27])),
        "speed_ref_rpm": frame[28] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "vbus": frame[29],
        "uq_feedforward": frame[33],
        "ud_feedforward": frame[34],
        "feedforward_enabled": int(round(frame[35])),
    })


def read_checked(port, buffer, deadline, max_current, max_rpm,
                 min_vbus, max_vbus, min_voltage_scale):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    mode = int(round(frame[27]))
    current_limit = (
        TRANSITION_CURRENT_LIMIT_A if mode == MODE_ABZENC_CALIB else max_current
    )
    proto.check_frame(frame, current_limit, max_rpm)
    if mode == MODE_SPEED_TORQUE_RUN:
        if math.hypot(frame[6], frame[7]) > max_current:
            raise RuntimeError("dq current vector exceeded limit")
        if frame[26] < min_voltage_scale:
            raise RuntimeError(
                f"voltage scale fell below safety limit: {frame[26]:.3f}"
            )
    if not min_vbus <= frame[29] <= max_vbus:
        raise RuntimeError(f"bus voltage is out of range: {frame[29]:.3f} V")
    return sample


def wait_mode(port, buffer, expected_mode, timeout, started, records,
              round_index, profile, phase, limits):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(
            records, started, timestamp, round_index, profile, phase, 0.0, frame,
        )
        if int(round(frame[27])) == expected_mode:
            buffer.clear()
            port.reset_input_buffer()
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected_mode} during {phase}")


def capture_transition(port, buffer, target_rpm, timeout, stable_duration,
                       started, records, round_index, profile, limits,
                       stable_tolerance_rpm=60.0):
    phase = f"speed_{target_rpm:g}"
    phase_started = time.monotonic()
    deadline = phase_started + timeout
    stable_since = None
    samples = []
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {profile}/{phase}")
        append_record(
            records, started, timestamp, round_index, profile,
            phase, target_rpm, frame,
        )
        samples.append((timestamp - phase_started, frame))
        speed_ref = frame[28] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi)
        if (abs(speed_ref - target_rpm) <= 10.0
                and abs(proto.mechanical_rpm(frame) - target_rpm)
                <= stable_tolerance_rpm):
            stable_since = stable_since or timestamp
            if timestamp - stable_since >= stable_duration:
                return samples
        else:
            stable_since = None
    raise RuntimeError(f"speed did not settle at {target_rpm:g} rpm")


def summarize_transition(round_index, profile, source_rpm, target_rpm, samples):
    frames = [frame for _, frame in samples]
    times = [elapsed for elapsed, _ in samples]
    speed_ref = [
        frame[28] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi)
        for frame in frames
    ]
    actual = [proto.mechanical_rpm(frame) for frame in frames]
    ramp_indices = [
        index for index, value in enumerate(speed_ref)
        if abs(value - target_rpm) > 10.0
    ]
    if not ramp_indices:
        ramp_indices = list(range(len(frames)))
    steady_start = max(0.0, times[-1] - 0.4)
    steady_indices = [index for index, value in enumerate(times) if value >= steady_start]

    def metric(indices, getter):
        return rms([getter(frames[index], actual[index], speed_ref[index])
                    for index in indices])

    pi_voltage = [
        math.hypot(frame[20] - frame[33], frame[21] - frame[34])
        for frame in frames
    ]
    return {
        "round": round_index,
        "profile": profile,
        "source_rpm": source_rpm,
        "target_rpm": target_rpm,
        "samples": len(frames),
        "duration_sec": times[-1],
        "ramp_speed_error_rms_rpm": metric(
            ramp_indices, lambda _f, speed, reference: speed - reference,
        ),
        "ramp_iq_error_rms_a": metric(
            ramp_indices, lambda frame, _s, _r: frame[6] - frame[22],
        ),
        "ramp_id_error_rms_a": metric(
            ramp_indices, lambda frame, _s, _r: frame[7] - frame[23],
        ),
        "steady_speed_error_rms_rpm": metric(
            steady_indices, lambda _f, speed, reference: speed - reference,
        ),
        "steady_iq_error_rms_a": metric(
            steady_indices, lambda frame, _s, _r: frame[6] - frame[22],
        ),
        "steady_id_error_rms_a": metric(
            steady_indices, lambda frame, _s, _r: frame[7] - frame[23],
        ),
        "pi_voltage_rms_v": rms(pi_voltage),
        "peak_dq_current_a": max(math.hypot(frame[6], frame[7]) for frame in frames),
        "peak_iq_reference_a": max(abs(frame[22]) for frame in frames),
        "peak_voltage_vector_v": max(
            math.hypot(frame[20], frame[21]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
        "vbus_mean_v": mean([frame[29] for frame in frames]),
    }


def summarize_scope(round_index, profile, metadata, samples):
    peak_iq_ref = max(abs(sample[4]) for sample in samples)
    threshold = max(0.1, 0.1 * peak_iq_ref)
    excited = [sample for sample in samples if abs(sample[4]) >= threshold]
    selected = excited or samples
    return {
        "round": round_index,
        "profile": profile,
        "sample_rate_hz": metadata["base_sample_hz"] / metadata["divider"],
        "samples": len(samples),
        "excited_samples": len(excited),
        "excitation_threshold_a": threshold,
        "peak_iq_reference_a": peak_iq_ref,
        "excited_iq_error_rms_a": rms([sample[2] - sample[4] for sample in selected]),
        "excited_id_error_rms_a": rms([sample[3] - sample[5] for sample in selected]),
        "peak_dq_current_a": max(math.hypot(sample[2], sample[3]) for sample in samples),
        "peak_voltage_vector_v": max(
            math.hypot(sample[6], sample[7]) for sample in samples
        ),
        "speed_start_rpm": samples[0][9] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "speed_end_rpm": samples[-1][9] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "max_abs_speed_rpm": max(
            abs(sample[9] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi))
            for sample in samples
        ),
    }


def aggregate_profiles(transitions, scopes):
    summaries = []
    for profile in PROFILE_ORDER:
        profile_transitions = [item for item in transitions if item["profile"] == profile]
        profile_scopes = [item for item in scopes if item["profile"] == profile]
        if not profile_transitions:
            continue
        summaries.append({
            "profile": profile,
            "profile_visits": len({item["round"] for item in profile_transitions}),
            "transitions": len(profile_transitions),
            "mean_ramp_speed_error_rms_rpm": mean([
                item["ramp_speed_error_rms_rpm"] for item in profile_transitions
            ]),
            "mean_ramp_iq_error_rms_a": mean([
                item["ramp_iq_error_rms_a"] for item in profile_transitions
            ]),
            "mean_ramp_id_error_rms_a": mean([
                item["ramp_id_error_rms_a"] for item in profile_transitions
            ]),
            "mean_steady_speed_error_rms_rpm": mean([
                item["steady_speed_error_rms_rpm"] for item in profile_transitions
            ]),
            "mean_steady_iq_error_rms_a": mean([
                item["steady_iq_error_rms_a"] for item in profile_transitions
            ]),
            "mean_steady_id_error_rms_a": mean([
                item["steady_id_error_rms_a"] for item in profile_transitions
            ]),
            "mean_pi_voltage_rms_v": mean([
                item["pi_voltage_rms_v"] for item in profile_transitions
            ]),
            "peak_dq_current_a": max(
                item["peak_dq_current_a"] for item in profile_transitions
            ),
            "minimum_voltage_scale": min(
                item["minimum_voltage_scale"] for item in profile_transitions
            ),
            "mean_scope_excited_iq_error_rms_a": (
                mean([item["excited_iq_error_rms_a"] for item in profile_scopes])
                if profile_scopes else None
            ),
            "mean_scope_excited_id_error_rms_a": (
                mean([item["excited_id_error_rms_a"] for item in profile_scopes])
                if profile_scopes else None
            ),
            "minimum_scope_peak_iq_reference_a": (
                min(item["peak_iq_reference_a"] for item in profile_scopes)
                if profile_scopes else None
            ),
        })
    return summaries


def evaluate_candidate(profile_summaries):
    by_name = {item["profile"]: item for item in profile_summaries}
    reference = by_name.get("active_ff_identified")
    candidate = by_name.get("identified_2000hz_ff_identified")
    if reference is None or candidate is None:
        return {
            "passed": False,
            "status": "incomplete profiles",
            "checks": [],
        }

    checks = []

    def add_relative(name, field, relative_limit, absolute_allowance):
        reference_value = reference[field]
        candidate_value = candidate[field]
        if reference_value is None or candidate_value is None:
            checks.append({
                "name": name,
                "candidate": candidate_value,
                "reference": reference_value,
                "limit": None,
                "passed": False,
            })
            return
        limit = max(
            reference_value * relative_limit,
            reference_value + absolute_allowance,
        )
        checks.append({
            "name": name,
            "candidate": candidate_value,
            "reference": reference_value,
            "limit": limit,
            "passed": candidate_value <= limit,
        })

    add_relative(
        "ramp_speed_error", "mean_ramp_speed_error_rms_rpm", 1.10, 20.0,
    )
    add_relative(
        "ramp_iq_error", "mean_ramp_iq_error_rms_a", 1.10, 0.01,
    )
    add_relative(
        "ramp_id_error", "mean_ramp_id_error_rms_a", 1.15, 0.02,
    )
    add_relative(
        "steady_speed_error", "mean_steady_speed_error_rms_rpm", 1.10, 10.0,
    )
    add_relative(
        "steady_iq_error", "mean_steady_iq_error_rms_a", 1.10, 0.01,
    )
    add_relative(
        "steady_id_error", "mean_steady_id_error_rms_a", 1.15, 0.02,
    )
    add_relative(
        "scope_iq_error", "mean_scope_excited_iq_error_rms_a", 1.10, 0.01,
    )
    add_relative(
        "scope_id_error", "mean_scope_excited_id_error_rms_a", 1.15, 0.02,
    )
    checks.extend([
        {
            "name": "candidate_voltage_margin",
            "candidate": candidate["minimum_voltage_scale"],
            "limit": 0.98,
            "passed": candidate["minimum_voltage_scale"] >= 0.98,
        },
        {
            "name": "reference_scope_excitation",
            "candidate": reference["minimum_scope_peak_iq_reference_a"],
            "limit": 0.2,
            "passed": (
                reference["minimum_scope_peak_iq_reference_a"] is not None
                and reference["minimum_scope_peak_iq_reference_a"] >= 0.2
            ),
        },
        {
            "name": "candidate_scope_excitation",
            "candidate": candidate["minimum_scope_peak_iq_reference_a"],
            "limit": 0.2,
            "passed": (
                candidate["minimum_scope_peak_iq_reference_a"] is not None
                and candidate["minimum_scope_peak_iq_reference_a"] >= 0.2
            ),
        },
        {
            "name": "two_ordered_visits",
            "candidate": candidate["profile_visits"],
            "limit": 2,
            "passed": (candidate["profile_visits"] >= 2
                       and reference["profile_visits"] >= 2),
        },
    ])
    excitation_names = {
        "reference_scope_excitation",
        "candidate_scope_excitation",
        "two_ordered_visits",
    }
    evidence_sufficient = all(
        check["passed"] for check in checks if check["name"] in excitation_names
    )
    performance_passed = all(
        check["passed"] for check in checks if check["name"] not in excitation_names
    )
    passed = evidence_sufficient and performance_passed
    if not evidence_sufficient:
        status = "inconclusive due to insufficient rotating excitation; retain active PI"
    elif not performance_passed:
        status = "rotating performance gate not passed; retain active PI"
    else:
        status = "rotating gate passed; combine with prior step/FRF evidence before writeback"
    return {
        "passed": passed,
        "evidence_sufficient": evidence_sufficient,
        "performance_passed": performance_passed,
        "status": status,
        "reference_profile": "active_ff_identified",
        "candidate_profile": "identified_2000hz_ff_identified",
        "checks": checks,
    }


def experiment_plan(controllers, speed_ramp):
    rounds = [
        {
            "round": 1,
            "profiles": list(PROFILE_ORDER),
            "waypoints_rpm": [3000.0, 1500.0, -1500.0, -3000.0, 0.0],
            "scope_transition_rpm": [1500.0, -1500.0],
        },
        {
            "round": 2,
            "profiles": list(reversed(PROFILE_ORDER)),
            "waypoints_rpm": [-3000.0, -1500.0, 1500.0, 3000.0, 0.0],
            "scope_transition_rpm": [-1500.0, 1500.0],
        },
    ]
    profiles = {}
    for name, (controller_name, model, ff_enabled) in PROFILE_SETTINGS.items():
        profiles[name] = {
            "controller": controller_name,
            "gains": controllers[controller_name],
            "feedforward_model": model,
            "feedforward_enabled": ff_enabled,
        }
    return {
        "profiles": profiles,
        "rounds": rounds,
        "speed_ramp_rpm_per_sec": speed_ramp,
        "scope": {
            "captures": 6,
            "divider": 1,
            "sample_rate_hz": 40000,
            "duration_sec": 4096 / 40000.0,
            "purpose": "40 kHz current tracking at the start of a +/-1500 rpm reversal",
        },
        "selection_rule": {
            "comparison": "identified_2000hz_ff_identified versus active_ff_identified",
            "minimum_scope_peak_iq_reference_a": 0.2,
            "candidate_limits": {
                "ramp_speed_error": "no more than 10% worse, with 20 rpm absolute allowance",
                "scope_iq_error": "no more than 10% worse, with 0.01 A absolute allowance",
                "ramp_or_scope_id_error": "no more than 15% worse, with 0.02 A absolute allowance",
                "minimum_voltage_scale": 0.98,
            },
            "writeback": "requires consistent benefit or equivalence plus the prior step/FRF advantage; otherwise retain active PI",
        },
    }


def write_records(path, records):
    if not records:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(
        description="Compare active and 2 kHz current PI in a guarded rotating test",
    )
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--speed-ramp", type=float, default=3000.0)
    parser.add_argument("--stable-duration", type=float, default=0.5)
    parser.add_argument("--startup-timeout", type=float, default=25.0)
    parser.add_argument("--max-current", type=float, default=3.0)
    parser.add_argument("--max-rpm", type=float, default=3500.0)
    parser.add_argument("--min-vbus", type=float, default=18.0)
    parser.add_argument("--max-vbus", type=float, default=28.0)
    parser.add_argument("--min-voltage-scale", type=float, default=0.90)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    controllers = build_controllers((2000.0,))
    plan = experiment_plan(controllers, args.speed_ramp)
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")
    if args.speed_ramp <= 0.0:
        parser.error("--speed-ramp must be positive")

    csv_path = args.csv or Path("tests/results") / (
        f"stage2_current_combination_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    json_path = csv_path.with_suffix(".json")
    scope_dir = csv_path.with_suffix("").with_name(csv_path.stem + "_scope")
    limits = {
        "max_current": args.max_current,
        "max_rpm": args.max_rpm,
        "min_vbus": args.min_vbus,
        "max_vbus": args.max_vbus,
        "min_voltage_scale": args.min_voltage_scale,
    }
    records = []
    transitions = []
    scope_results = []
    started = time.monotonic()
    final_mode = None
    error = None

    try:
        with serial.Serial(
            args.port, args.baudrate, timeout=0.02, write_timeout=0.2,
        ) as port:
            buffer = bytearray()
            port.reset_input_buffer()
            try:
                wait_mode(
                    port, buffer, MODE_READY, args.startup_timeout,
                    started, records, 0, "setup", "initial_ready", limits,
                )
                proto.send_command(port, f"TRACK RAMP {args.speed_ramp:g}")

                for round_plan in plan["rounds"]:
                    round_index = round_plan["round"]
                    waypoints = round_plan["waypoints_rpm"]
                    scope_source, scope_target = round_plan["scope_transition_rpm"]
                    for profile in round_plan["profiles"]:
                        controller_name, model, ff_enabled = PROFILE_SETTINGS[profile]
                        gains = controllers[controller_name]
                        proto.send_command(port, "FF OFF")
                        proto.send_command(port, f"FF MODEL {model}")
                        proto.send_command(
                            port, f"PI IQ {gains['q']['kp']:.9g} {gains['q']['ki']:.9g}",
                        )
                        proto.send_command(
                            port, f"PI ID {gains['d']['kp']:.9g} {gains['d']['ki']:.9g}",
                        )
                        proto.send_command(port, "PI CLEAR")
                        proto.send_command(port, "FF ON" if ff_enabled else "FF OFF")
                        proto.send_command(port, "START SPEED")
                        wait_mode(
                            port, buffer, MODE_SPEED_TORQUE_RUN,
                            args.startup_timeout, started, records,
                            round_index, profile, "start_speed", limits,
                        )
                        proto.send_command(port, "SPEED ID 0")

                        source_rpm = 0.0
                        armed = False
                        for target_rpm in waypoints:
                            if source_rpm == scope_source and target_rpm == scope_target:
                                scope.arm_scope(port, 1)
                                armed = True
                            proto.send_command(port, f"SPEED {target_rpm:g}")
                            timeout = abs(target_rpm - source_rpm) / args.speed_ramp + 4.0
                            samples = capture_transition(
                                port, buffer, target_rpm, timeout,
                                args.stable_duration, started, records,
                                round_index, profile, limits,
                            )
                            transitions.append(summarize_transition(
                                round_index, profile, source_rpm, target_rpm,
                                samples,
                            ))
                            source_rpm = target_rpm

                        proto.send_command(port, "SPEED ID 0")
                        proto.send_command(port, "SPEED 0")
                        proto.send_command(port, "STOP")
                        wait_mode(
                            port, buffer, MODE_READY, 2.0, started, records,
                            round_index, profile, "ready_after_profile", limits,
                        )
                        if not armed:
                            raise RuntimeError(f"scope transition was not armed for {profile}")
                        metadata, samples = scope.dump_scope(port, 15.0)
                        scope_dir.mkdir(parents=True, exist_ok=True)
                        scope_path = scope_dir / f"r{round_index}_{profile}.csv"
                        scope.write_csv(
                            scope_path, samples,
                            metadata["divider"] / float(metadata["base_sample_hz"]),
                        )
                        result = summarize_scope(round_index, profile, metadata, samples)
                        result["csv"] = str(scope_path.resolve())
                        scope_results.append(result)
            finally:
                proto.send_command(port, "SPEED ID 0")
                proto.send_command(port, "SPEED 0")
                proto.send_command(port, "STOP")
                try:
                    final = wait_mode(
                        port, buffer, MODE_READY, 2.0, started, records,
                        0, "cleanup", "final_ready", limits,
                    )
                    final_mode = int(round(final[27]))
                    active = controllers["active_legacy"]
                    proto.send_command(port, "FF OFF")
                    proto.send_command(port, "FF MODEL LEGACY")
                    proto.send_command(
                        port, f"PI IQ {active['q']['kp']:.9g} {active['q']['ki']:.9g}",
                    )
                    proto.send_command(
                        port, f"PI ID {active['d']['kp']:.9g} {active['d']['ki']:.9g}",
                    )
                    proto.send_command(port, "PI CLEAR")
                except RuntimeError:
                    final_mode = None
    except Exception as exc:
        error = str(exc)
    finally:
        write_records(csv_path, records)
        profile_summary = aggregate_profiles(transitions, scope_results)
        report = {
            "port": args.port,
            "plan": plan,
            "safety_limits": limits,
            "final_mode": final_mode,
            "restored_active_pi_and_ff_off": final_mode == MODE_READY,
            "error": error,
            "csv": str(csv_path.resolve()),
            "scope_directory": str(scope_dir.resolve()),
            "profile_summary": profile_summary,
            "candidate_evaluation": evaluate_candidate(profile_summary),
            "transitions": transitions,
            "scope_results": scope_results,
        }
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report, indent=2))

    if error is not None:
        raise RuntimeError(error)
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP/READY confirmation failed")


if __name__ == "__main__":
    main()
