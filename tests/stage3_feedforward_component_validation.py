#!/usr/bin/env python3
"""Run the guarded OFF/CROSS/BEMF/BOTH Stage-3 feedforward matrix."""

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
from stage2_current_combination_validation import (
    MODE_READY,
    MODE_SPEED_TORQUE_RUN,
    append_record,
    capture_transition,
    read_checked,
    wait_mode,
)
from stage2_speed_autotune_validation import open_serial_with_retry


FF_MODES = {
    "OFF": 0,
    "BOTH": 1,
    "CROSS": 2,
    "BEMF": 3,
}
ROUND_MODES = (
    ("OFF", "CROSS", "BEMF", "BOTH"),
    ("BOTH", "BEMF", "CROSS", "OFF"),
)
ROUND_SPEEDS_RPM = (
    (1500.0, 3000.0, -1500.0, -3000.0),
    (-1500.0, -3000.0, 1500.0, 3000.0),
)
ROUND_ID_A = (
    (0.0, 1.0, -1.0),
    (0.0, -1.0, 1.0),
)
ROUND_SCOPE_STEP_RPM = (
    (3000.0, 3300.0),
    (-3000.0, -3300.0),
)

POLE_PAIRS = 10
IDENTIFIED_LD_H = 26.043582e-6
IDENTIFIED_LQ_H = 40.343765e-6
IDENTIFIED_FLUX_WB = 0.90795034e-3
RAMP_RPM_PER_SEC = 3000.0
DEFAULT_HOLD_SEC = 0.7
SCOPE_DIVIDER = 1
SCOPE_PRESTEP_SEC = 0.02
SCOPE_COMPLETE_SEC = 4096.0 / 40000.0 + 0.05
DEFAULT_SPEED_IQ_LIMIT_A = 7.0
TEST_SPEED_IQ_LIMIT_A = 4.0


def component_voltage(mode: str, omega_e: float, iq: float, id_: float) -> tuple[float, float]:
    uq = 0.0
    ud = 0.0
    if mode in ("CROSS", "BOTH"):
        uq += omega_e * IDENTIFIED_LD_H * id_
        ud -= omega_e * IDENTIFIED_LQ_H * iq
    if mode in ("BEMF", "BOTH"):
        uq += omega_e * IDENTIFIED_FLUX_WB
    return uq, ud


def experiment_plan(speed_ramp: float = RAMP_RPM_PER_SEC,
                    hold_sec: float = DEFAULT_HOLD_SEC) -> dict:
    visits = []
    for round_index, modes in enumerate(ROUND_MODES, start=1):
        for mode in modes:
            visits.append({
                "round": round_index,
                "mode": mode,
                "mode_code": FF_MODES[mode],
                "speed_order_rpm": list(ROUND_SPEEDS_RPM[round_index - 1]),
                "id_order_a": list(ROUND_ID_A[round_index - 1]),
                "scope_step_rpm": list(ROUND_SCOPE_STEP_RPM[round_index - 1]),
            })
    return {
        "schema": "stage3-feedforward-component-plan-v1",
        "purpose": (
            "separate dq cross-coupling and back-EMF feedforward before any "
            "voltage-boundary experiment"
        ),
        "model": "IDENTIFIED",
        "feedforward_modes": FF_MODES,
        "visits": visits,
        "visit_count": len(visits),
        "steady_hold_count": len(visits) * len(ROUND_SPEEDS_RPM[0]) * len(ROUND_ID_A[0]),
        "scope_capture_count": len(visits),
        "speed_ramp_rpm_per_sec": speed_ramp,
        "steady_hold_sec": hold_sec,
        "scope": {
            "divider": SCOPE_DIVIDER,
            "sample_rate_hz": 40000,
            "samples": 4096,
            "prestep_sec": SCOPE_PRESTEP_SEC,
            "local_step_amplitude_rpm": 300.0,
        },
        "safety": {
            "speed_iq_reference_limit_a": TEST_SPEED_IQ_LIMIT_A,
            "abort_actual_dq_current_a": 5.0,
            "abort_mechanical_speed_rpm": 3500.0,
            "minimum_bus_voltage_v": 18.0,
            "maximum_bus_voltage_v": 28.0,
            "abort_voltage_scale": 0.90,
            "analysis_voltage_scale_floor": 0.98,
        },
        "interpretation": {
            "cross_observability": "mainly +/-1 A d-axis holds and local-step iq",
            "bemf_observability": "mainly speed and direction reversal",
            "physical_claim_boundary": "no-load only; no loaded decoupling or efficiency claim",
            "no_automatic_default_writeback": True,
        },
        "requires_24v": True,
        "motor_will_rotate": True,
        "maximum_commanded_speed_rpm": 3300.0,
        "automatic_flash": False,
        "finally_stop_and_confirm_ready": True,
        "restore_identified_model_and_ff_both": True,
    }


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def check_mode(frame: tuple[float, ...], expected_mode: str) -> None:
    actual = int(round(frame[35]))
    expected = FF_MODES[expected_mode]
    if actual != expected:
        raise RuntimeError(
            f"feedforward mode telemetry mismatch: expected {expected_mode}/{expected}, got {actual}"
        )


def summarize_frames(mode: str, frames: list[tuple[float, ...]]) -> dict:
    if not frames:
        raise RuntimeError("no telemetry frames captured")
    expected = [
        component_voltage(mode, frame[25], frame[6], frame[7])
        for frame in frames
    ]
    applied_voltage = [math.hypot(frame[20], frame[21]) for frame in frames]
    pi_voltage = [
        math.hypot(frame[20] - frame[33], frame[21] - frame[34])
        for frame in frames
    ]
    voltage_limit = [frame[29] / math.sqrt(3.0) for frame in frames]
    requested_utilization = [
        applied_voltage[index]
        / max(frame[26], 1.0e-6)
        / max(voltage_limit[index], 1.0e-6)
        for index, frame in enumerate(frames)
    ]
    speed_error = [
        proto.mechanical_rpm(frame)
        - frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi)
        for frame in frames
    ]
    return {
        "frames": len(frames),
        "mean_speed_rpm": mean([proto.mechanical_rpm(frame) for frame in frames]),
        "speed_error_rms_rpm": rms(speed_error),
        "iq_error_rms_a": rms([frame[6] - frame[22] for frame in frames]),
        "id_error_rms_a": rms([frame[7] - frame[23] for frame in frames]),
        "pi_voltage_rms_v": rms(pi_voltage),
        "applied_voltage_rms_v": rms(applied_voltage),
        "peak_requested_voltage_utilization": max(requested_utilization),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
        "peak_dq_current_a": max(math.hypot(frame[6], frame[7]) for frame in frames),
        "expected_cross_peak_v": max(
            math.hypot(
                frame[25] * IDENTIFIED_LD_H * frame[7],
                -frame[25] * IDENTIFIED_LQ_H * frame[6],
            )
            for frame in frames
        ),
        "expected_bemf_peak_v": max(
            abs(frame[25] * IDENTIFIED_FLUX_WB) for frame in frames
        ),
        "feedforward_residual_rms_v": rms([
            math.hypot(
                frame[33] - expected[index][0],
                frame[34] - expected[index][1],
            )
            for index, frame in enumerate(frames)
        ]),
    }


def capture_hold(port, buffer: bytearray, duration_sec: float, mode: str,
                 round_index: int, speed_rpm: float, id_ref: float,
                 started: float, records: list[dict], limits: dict) -> dict:
    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, f"SPEED ID {id_ref:g}")
    phase = f"hold_{speed_rpm:+g}rpm_{id_ref:+g}a"
    accept_deadline = time.monotonic() + 0.5
    accepted_frame = None
    last_id_ref = None
    while time.monotonic() < accept_deadline:
        sample = read_checked(
            port, buffer, min(accept_deadline, time.monotonic() + 0.1), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {mode}/{phase}")
        check_mode(frame, mode)
        last_id_ref = frame[23]
        if abs(last_id_ref - id_ref) <= 0.02:
            append_record(
                records, started, timestamp, round_index, mode,
                phase, speed_rpm, frame,
            )
            accepted_frame = frame
            break
        append_record(
            records, started, timestamp, round_index, mode,
            f"{phase}_settle", speed_rpm, frame,
        )
    if accepted_frame is None:
        observed = "no telemetry" if last_id_ref is None else f"{last_id_ref:.4f} A"
        raise RuntimeError(
            f"d-axis reference was not accepted: requested {id_ref:g} A, "
            f"telemetry {observed}"
        )

    deadline = time.monotonic() + duration_sec
    frames = [accepted_frame]
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {mode}/{phase}")
        check_mode(frame, mode)
        if abs(frame[23] - id_ref) > 0.02:
            raise RuntimeError(
                f"d-axis reference changed during hold: requested {id_ref:g} A, "
                f"telemetry {frame[23]:.4f} A"
            )
        append_record(
            records, started, timestamp, round_index, mode,
            phase, speed_rpm, frame,
        )
        frames.append(frame)
    summary = summarize_frames(mode, frames)
    summary.update({
        "round": round_index,
        "mode": mode,
        "mode_code": FF_MODES[mode],
        "speed_rpm": speed_rpm,
        "id_reference_a": id_ref,
    })
    return summary


def capture_scope_step(port, buffer: bytearray, mode: str,
                       base_rpm: float, target_rpm: float,
                       started: float, records: list[dict], limits: dict,
                       round_index: int) -> None:
    buffer.clear()
    port.reset_input_buffer()
    proto.send_command(port, "SPEED ID 0")
    proto.send_command(port, "TRACK STEP")
    scope.arm_scope(port, SCOPE_DIVIDER)
    phase_started = time.monotonic()
    commanded = False
    deadline = phase_started + SCOPE_COMPLETE_SEC
    while time.monotonic() < deadline:
        if not commanded and time.monotonic() - phase_started >= SCOPE_PRESTEP_SEC:
            proto.send_command(port, f"SPEED {target_rpm:g}")
            commanded = True
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.01), **limits,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError("motor left speed mode during local scope step")
        check_mode(frame, mode)
        append_record(
            records, started, timestamp, round_index, mode,
            f"scope_{base_rpm:+g}_to_{target_rpm:+g}", target_rpm, frame,
        )
    if not commanded:
        raise RuntimeError("scope step command was not sent")


def summarize_scope(mode: str, metadata: dict,
                    samples: list[tuple[float, ...]]) -> dict:
    expected = [
        component_voltage(mode, sample[9], sample[2], sample[3])
        for sample in samples
    ]
    return {
        "mode": mode,
        "mode_code": FF_MODES[mode],
        "sample_rate_hz": metadata["base_sample_hz"] / metadata["divider"],
        "samples": len(samples),
        "iq_error_rms_a": rms([sample[2] - sample[4] for sample in samples]),
        "id_error_rms_a": rms([sample[3] - sample[5] for sample in samples]),
        "pi_voltage_rms_v": rms([
            math.hypot(
                sample[6] - expected[index][0],
                sample[7] - expected[index][1],
            )
            for index, sample in enumerate(samples)
        ]),
        "peak_dq_current_a": max(
            math.hypot(sample[2], sample[3]) for sample in samples
        ),
        "peak_iq_reference_a": max(abs(sample[4]) for sample in samples),
        "expected_cross_peak_v": max(
            math.hypot(
                sample[9] * IDENTIFIED_LD_H * sample[3],
                -sample[9] * IDENTIFIED_LQ_H * sample[2],
            )
            for sample in samples
        ),
        "expected_bemf_peak_v": max(
            abs(sample[9] * IDENTIFIED_FLUX_WB) for sample in samples
        ),
    }


def aggregate_modes(holds: list[dict], scopes: list[dict]) -> list[dict]:
    result = []
    for mode in FF_MODES:
        mode_holds = [item for item in holds if item["mode"] == mode]
        mode_scopes = [item for item in scopes if item["mode"] == mode]
        if not mode_holds:
            continue
        result.append({
            "mode": mode,
            "visits": len({item["round"] for item in mode_holds}),
            "steady_holds": len(mode_holds),
            "scope_captures": len(mode_scopes),
            "mean_steady_pi_voltage_rms_v": mean([
                item["pi_voltage_rms_v"] for item in mode_holds
            ]),
            "mean_steady_iq_error_rms_a": mean([
                item["iq_error_rms_a"] for item in mode_holds
            ]),
            "mean_steady_id_error_rms_a": mean([
                item["id_error_rms_a"] for item in mode_holds
            ]),
            "mean_scope_pi_voltage_rms_v": (
                mean([item["pi_voltage_rms_v"] for item in mode_scopes])
                if mode_scopes else None
            ),
            "peak_requested_voltage_utilization": max(
                item["peak_requested_voltage_utilization"] for item in mode_holds
            ),
            "minimum_voltage_scale": min(
                item["minimum_voltage_scale"] for item in mode_holds
            ),
            "peak_dq_current_a": max(
                [item["peak_dq_current_a"] for item in mode_holds]
                + [item["peak_dq_current_a"] for item in mode_scopes]
            ),
            "maximum_feedforward_residual_rms_v": max(
                item["feedforward_residual_rms_v"] for item in mode_holds
            ),
            "maximum_expected_cross_v": max(
                [item["expected_cross_peak_v"] for item in mode_holds]
                + [item["expected_cross_peak_v"] for item in mode_scopes]
            ),
            "maximum_expected_bemf_v": max(
                [item["expected_bemf_peak_v"] for item in mode_holds]
                + [item["expected_bemf_peak_v"] for item in mode_scopes]
            ),
        })
    return result


def factorial_summary(mode_summary: list[dict]) -> dict | None:
    by_mode = {item["mode"]: item for item in mode_summary}
    if set(by_mode) != set(FF_MODES):
        return None
    voltage = {
        mode: by_mode[mode]["mean_steady_pi_voltage_rms_v"] for mode in FF_MODES
    }
    return {
        "both_vs_off_pi_voltage_reduction": 1.0 - voltage["BOTH"] / voltage["OFF"],
        "cross_main_effect_v": 0.5 * (
            voltage["OFF"] - voltage["CROSS"]
            + voltage["BEMF"] - voltage["BOTH"]
        ),
        "bemf_main_effect_v": 0.5 * (
            voltage["OFF"] - voltage["BEMF"]
            + voltage["CROSS"] - voltage["BOTH"]
        ),
        "interaction_v": (
            voltage["BOTH"] - voltage["BEMF"]
            - voltage["CROSS"] + voltage["OFF"]
        ),
        "interpretation": (
            "effects are descriptive no-load factorial contrasts; use round-wise raw "
            "records before claiming a separable physical benefit"
        ),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def save_report(path: Path, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def stop_and_restore(active_port, port_name: str, baudrate: int,
                     buffer: bytearray, started: float,
                     records: list[dict], limits: dict) -> tuple[int | None, bool, str | None]:
    last_error = None
    for attempt in range(2):
        cleanup_port = active_port
        owns_port = False
        if attempt != 0:
            try:
                cleanup_port = open_serial_with_retry(port_name, baudrate)
                cleanup_port.reset_input_buffer()
                owns_port = True
            except (RuntimeError, serial.SerialException) as error:
                last_error = str(error)
                continue
        try:
            buffer.clear()
            for command in ("SPEED ID 0", "SPEED 0", "STOP"):
                proto.send_command(cleanup_port, command)
            final = wait_mode(
                cleanup_port, buffer, MODE_READY, 5.0, started, records,
                0, "cleanup", "final_ready", limits,
            )
            for command in (
                "VOLT LIMIT 1",
                "FF MODEL IDENTIFIED",
                "FF MODE BOTH",
                f"SPEED LIMIT {DEFAULT_SPEED_IQ_LIMIT_A:g}",
                "SPEED EST WIN",
                "PI CLEAR",
            ):
                proto.send_command(cleanup_port, command)
            restored = wait_mode(
                cleanup_port, buffer, MODE_READY, 2.0, started, records,
                0, "cleanup", "defaults_restored", limits,
            )
            check_mode(restored, "BOTH")
            return int(round(final[27])), True, None
        except (RuntimeError, serial.SerialException) as error:
            last_error = str(error)
            try:
                active_port.close()
            except serial.SerialException:
                pass
        finally:
            if owns_port:
                try:
                    cleanup_port.close()
                except serial.SerialException:
                    pass
    return None, False, last_error


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--speed-ramp", type=float, default=RAMP_RPM_PER_SEC)
    parser.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    plan = experiment_plan(args.speed_ramp, args.hold_sec)
    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")
    if args.speed_ramp <= 0.0 or args.hold_sec <= 0.0:
        parser.error("--speed-ramp and --hold-sec must be positive")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / (
        f"stage3_feedforward_component_{stamp}.json"
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
    holds: list[dict] = []
    scopes: list[dict] = []
    result = {
        "schema": "stage3-feedforward-component-result-v1",
        "plan": plan,
        "telemetry_csv": str(telemetry_path.resolve()),
        "scope_directory": str(scope_dir.resolve()),
        "holds": holds,
        "scopes": scopes,
        "mode_summary": None,
        "factorial_summary": None,
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
                    round_index = int(visit["round"])
                    mode = str(visit["mode"])
                    proto.send_command(port, f"FF MODE {mode}")
                    proto.send_command(port, f"TRACK RAMP {args.speed_ramp:g}")
                    proto.send_command(port, "PI CLEAR")
                    proto.send_command(port, "START SPEED")
                    wait_mode(
                        port, buffer, MODE_SPEED_TORQUE_RUN, 20.0,
                        started, records, round_index, mode,
                        "start_speed", limits,
                    )

                    current_speed = 0.0
                    scope_pending = None
                    for speed_rpm in visit["speed_order_rpm"]:
                        buffer.clear()
                        port.reset_input_buffer()
                        proto.send_command(port, "SPEED ID 0")
                        proto.send_command(port, f"SPEED {speed_rpm:g}")
                        samples = capture_transition(
                            port, buffer, speed_rpm,
                            abs(speed_rpm - current_speed) / args.speed_ramp + 4.0,
                            0.5, started, records, round_index, mode, limits,
                            stable_tolerance_rpm=90.0,
                        )
                        for _, frame in samples:
                            check_mode(frame, mode)
                        current_speed = speed_rpm

                        for id_ref in visit["id_order_a"]:
                            holds.append(capture_hold(
                                port, buffer, args.hold_sec, mode, round_index,
                                speed_rpm, id_ref, started, records, limits,
                            ))
                            write_csv(telemetry_path, records)
                            save_report(output, result)

                        scope_base, scope_target = visit["scope_step_rpm"]
                        if speed_rpm == scope_base:
                            capture_scope_step(
                                port, buffer, mode, scope_base, scope_target,
                                started, records, limits, round_index,
                            )
                            scope_pending = (scope_base, scope_target)
                            proto.send_command(port, f"TRACK RAMP {args.speed_ramp:g}")
                            proto.send_command(port, f"SPEED {scope_base:g}")
                            capture_transition(
                                port, buffer, scope_base, 4.0, 0.4,
                                started, records, round_index, mode, limits,
                                stable_tolerance_rpm=90.0,
                            )
                            current_speed = scope_base

                    if scope_pending is None:
                        raise RuntimeError(f"scope step was not captured for {mode}")
                    buffer.clear()
                    port.reset_input_buffer()
                    proto.send_command(port, "SPEED ID 0")
                    proto.send_command(port, f"TRACK RAMP {args.speed_ramp:g}")
                    proto.send_command(port, "SPEED 0")
                    capture_transition(
                        port, buffer, 0.0,
                        abs(current_speed) / args.speed_ramp + 4.0,
                        0.4, started, records, round_index, mode, limits,
                        stable_tolerance_rpm=90.0,
                    )
                    proto.send_command(port, "STOP")
                    wait_mode(
                        port, buffer, MODE_READY, 3.0, started, records,
                        round_index, mode, "ready_after_visit", limits,
                    )
                    metadata, samples = scope.dump_scope(port, 25.0)
                    scope_base, scope_target = scope_pending
                    scope_dir.mkdir(parents=True, exist_ok=True)
                    scope_path = scope_dir / (
                        f"r{round_index}_{mode.lower()}_"
                        f"{scope_base:+g}_to_{scope_target:+g}rpm.csv"
                    )
                    scope.write_csv(
                        scope_path, samples,
                        metadata["divider"] / float(metadata["base_sample_hz"]),
                    )
                    scope_capture = summarize_scope(mode, metadata, samples)
                    scope_capture.update({
                        "round": round_index,
                        "base_rpm": scope_base,
                        "target_rpm": scope_target,
                        "scope_csv": str(scope_path.resolve()),
                    })
                    scopes.append(scope_capture)
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
        result["mode_summary"] = aggregate_modes(holds, scopes)
        result["factorial_summary"] = factorial_summary(result["mode_summary"])
        save_report(output, result)

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    if not result["defaults_restored"]:
        raise RuntimeError("IDENTIFIED/FF BOTH/source limits were not restored")
    if result["status"] != "completed":
        raise RuntimeError(result["error"] or "Stage-3 feedforward matrix failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "holds": len(holds),
        "scopes": len(scopes),
        "factorial_summary": result["factorial_summary"],
        "final_mode": result["final_mode"],
        "defaults_restored": result["defaults_restored"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
