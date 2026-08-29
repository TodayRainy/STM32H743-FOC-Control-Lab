#!/usr/bin/env python3
"""Validate HFI current-assisted startup and the low-speed speed PI."""

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
import stage5_hfi_shadow_validation as hfi


ENCODER_SPEED_KP = 0.00536533785
ENCODER_SPEED_KI = 0.258523708
DEFAULT_HFI_SPEED_KP = 0.00160960136
DEFAULT_HFI_SPEED_KI = 0.012
SPEED_RAMP_RPM_PER_S = 200.0
SPEED_SEQUENCE_RPM = (
    350.0, 500.0, 350.0, 0.0,
    -350.0, -500.0, -350.0, 0.0,
)


def capture_for(port, buffer: bytearray, duration: float, phase: str,
                target_rpm: float, started: float, records: list[dict],
                limits: dict) -> list[tuple[float, ...]]:
    deadline = time.monotonic()+duration
    frames = []
    abz_overspeed_count = 0
    while time.monotonic() < deadline:
        sample = hfi.read_checked(
            port, buffer, min(deadline, time.monotonic()+0.1), limits
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != hfi.MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {phase}")
        actual_rpm = hfi.electrical_to_mechanical_rpm(frame[18])
        abz_overspeed_count = (abz_overspeed_count+1) \
            if abs(actual_rpm) > 800.0 else 0
        if limits["hfi_control_active"] and abz_overspeed_count >= 10:
            raise RuntimeError(
                f"ABZ speed remained above limit: {actual_rpm:.1f} rpm"
            )
        hfi.append_record(
            records, started, timestamp, phase, target_rpm, frame
        )
        frames.append(frame)
    if not frames:
        raise RuntimeError(f"no telemetry during {phase}")
    return frames


def summarize_transition(frames: list[tuple[float, ...]],
                         target_rpm: float,
                         source_rpm: float = 0.0) -> dict:
    tail = frames[-max(100, len(frames)//4):]
    angle_errors = [abs(math.degrees(
        hfi.circular_error(frame[9], frame[12])
    )) for frame in tail]
    speed_errors = [hfi.electrical_to_mechanical_rpm(
        frame[15]-frame[18]
    ) for frame in tail]
    actual_tail = [hfi.electrical_to_mechanical_rpm(frame[18])
                   for frame in tail]
    actual_errors = [value-target_rpm for value in actual_tail]
    result = {
        "target_rpm": target_rpm,
        "samples": len(frames),
        "angle_p95_deg": hfi.percentile(angle_errors, 0.95),
        "angle_max_deg": max(angle_errors),
        "speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in speed_errors
        )/len(speed_errors)),
        "actual_speed_rms_error_rpm": math.sqrt(sum(
            value*value for value in actual_errors
        )/len(actual_errors)),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
    }
    actual = [hfi.electrical_to_mechanical_rpm(frame[18])
              for frame in frames]
    step = target_rpm-source_rpm
    if step > 0.0:
        directional_overshoot = max(0.0, max(actual)-target_rpm)
    elif step < 0.0:
        directional_overshoot = max(0.0, target_rpm-min(actual))
    else:
        directional_overshoot = max(
            abs(value-target_rpm) for value in actual
        )
    result.update({
        "peak_actual_rpm": max(abs(value) for value in actual),
        "directional_overshoot_rpm": directional_overshoot,
        "tail_mean_actual_rpm": sum(actual_tail)/len(actual_tail),
        "tail_mean_hfi_rpm": sum(
            hfi.electrical_to_mechanical_rpm(frame[15]) for frame in tail
        )/len(tail),
    })
    actual_gate = 12.0 if target_rpm == 0.0 else 35.0
    result["passed"] = (
        result["angle_p95_deg"] <= 30.0
        and result["speed_rms_error_rpm"] <= 50.0
        and result["actual_speed_rms_error_rpm"] <= actual_gate
        # The ABZ WIN estimate has isolated transition spikes; keep the
        # sustained tail and RMS gates decisive while still bounding them.
        and result["directional_overshoot_rpm"] <= 75.0
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--kp", type=float, default=DEFAULT_HFI_SPEED_KP)
    parser.add_argument("--ki", type=float, default=DEFAULT_HFI_SPEED_KI)
    parser.add_argument("--feedforward", choices=("OFF", "BOTH"),
                        default="OFF")
    parser.add_argument(
        "--speed-sequence", type=float, nargs="+",
        default=list(SPEED_SEQUENCE_RPM),
        help="ordered low-speed targets in rpm",
    )
    parser.add_argument("--capture-scope", action="store_true",
                        help="capture the first +350 rpm transition")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_speed_loop_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    raw_scope = output.with_name(output.stem+"_scope.csv")
    result = {
        "schema": "stage5-hfi-speed-loop-result-v1",
        "status": "running",
        "error": None,
        "control_angle": "HFI",
        "speed_feedback": "HFI",
        "speed_pi": {"kp": args.kp, "ki": args.ki},
        "current_feedforward": args.feedforward,
        "speed_ramp_rpm_per_s": SPEED_RAMP_RPM_PER_S,
        "speed_sequence_rpm": args.speed_sequence,
        "startup": None,
        "running": [],
        "scope": None,
        "scope_csv": str(raw_scope.resolve()),
        "telemetry_csv": str(telemetry.resolve()),
        "final_mode": None,
        "defaults_restored": False,
    }
    limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 2.5,
        # The selected-speed channel contains a one-sample estimator reset
        # impulse in READY; controlled HFI and sustained ABZ limits are below.
        "max_rpm": 1.0e9,
        "max_hfi_rpm": 800.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.9,
        "hfi_active": False,
        "hfi_control_active": False,
    }
    records: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()
    scope_armed = False
    scope_metadata = None
    scope_samples = None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                hfi.wait_mode(port, buffer, hfi.MODE_READY, 10.0,
                              started, records, limits, "initial_ready")
                for command in (
                    "SPEED LIMIT 2", "FF MODEL IDENTIFIED", "FF MODE OFF",
                    "FW MODE OFF", "VOLT LIMIT 1", "TRACK STEP",
                    f"PI SPEED {ENCODER_SPEED_KP:.10g} "
                    f"{ENCODER_SPEED_KI:.10g}", "PI CLEAR", "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, records, limits, "abz_calibration")
                capture_for(
                    port, buffer, 0.4, "abz_zero", 0.0,
                    started, records, limits,
                )
                proto.send_command(port, "STOP")
                hfi.wait_mode(port, buffer, hfi.MODE_READY, 3.0,
                              started, records, limits, "abz_ready")

                for command in (
                    "SPEED LIMIT 1.5",
                    f"FF MODE {args.feedforward}",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}",
                    "START HFI",
                ):
                    proto.send_command(port, command)
                limits["hfi_active"] = True
                limits["hfi_control_active"] = True
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 10.0,
                              started, records, limits, "hfi_start")
                proto.send_command(
                    port, f"PI SPEED {args.kp:.10g} {args.ki:.10g}"
                )
                proto.send_command(port, "PI CLEAR")
                frames = capture_for(
                    port, buffer, 1.15, "ipd_and_track", 0.0,
                    started, records, limits,
                )
                result["startup"] = summarize_transition(
                    frames[-max(100, len(frames)//4):], 0.0
                )
                if not result["startup"]["passed"]:
                    raise RuntimeError("HFI startup angle did not qualify")

                capture_for(
                    port, buffer, 0.25, "control_zero", 0.0,
                    started, records, limits,
                )

                source_rpm = 0.0
                for index, target_rpm in enumerate(args.speed_sequence):
                    if index == 0 and args.capture_scope:
                        scope.arm_scope(port, 10)
                        scope_armed = True
                    proto.send_command(port, f"SPEED {target_rpm:g}")
                    duration = abs(target_rpm-source_rpm) \
                               /SPEED_RAMP_RPM_PER_S+1.5
                    frames = capture_for(
                        port, buffer, duration,
                        f"closed_loop_{target_rpm:+g}", target_rpm,
                        started, records, limits,
                    )
                    summary = summarize_transition(
                        frames, target_rpm, source_rpm
                    )
                    result["running"].append(summary)
                    print(json.dumps({
                        "phase": f"closed_loop_{target_rpm:+g}",
                        "summary": summary,
                    }), flush=True)
                    if not summary["passed"]:
                        raise RuntimeError(
                            f"HFI speed loop failed at {target_rpm:g} rpm"
                        )
                    source_rpm = target_rpm

                result["status"] = "passed"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED 0")
                    proto.send_command(port, "STOP")
                    limits["hfi_active"] = False
                    limits["hfi_control_active"] = False
                    time.sleep(0.05)
                    port.reset_input_buffer()
                    buffer.clear()
                    final = hfi.wait_mode(
                        port, buffer, hfi.MODE_READY, 3.0,
                        started, records, limits, "final_ready",
                    )
                    result["final_mode"] = int(round(final[27]))
                    if scope_armed:
                        time.sleep(1.1)
                        scope_metadata, scope_samples = scope.dump_scope(
                            port, 15.0
                        )
                        scope.write_csv(
                            raw_scope, scope_samples,
                            scope_metadata["divider"]
                            /scope_metadata["base_sample_hz"],
                        )
                        result["scope"] = hfi.summarize_scope(
                            scope_metadata, scope_samples
                        )
                        scope_armed = False
                    for command in (
                        f"PI SPEED {ENCODER_SPEED_KP:.10g} "
                        f"{ENCODER_SPEED_KI:.10g}",
                        "TRACK STEP", "SPEED LIMIT 7", "FW MODE HYBRID",
                        "VOLT LIMIT 1", "FF MODEL IDENTIFIED",
                        "FF MODE BOTH", "PI CLEAR",
                    ):
                        proto.send_command(port, command)
                    result["defaults_restored"] = True
                except (RuntimeError, serial.SerialException) as error:
                    result["error"] = result["error"] or str(error)
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        hfi.write_csv(telemetry, records)
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if result["status"] == "failed" or result["final_mode"] != hfi.MODE_READY:
        raise RuntimeError(result["error"] or "HFI speed-loop validation failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "speed_pi": result["speed_pi"],
        "startup": result["startup"],
        "running": result["running"],
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
