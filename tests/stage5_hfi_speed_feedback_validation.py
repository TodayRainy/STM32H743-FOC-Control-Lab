#!/usr/bin/env python3
"""Measure HFI speed-feedback gain and phase while ABZ remains in control."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
from stage2_current_frequency_response import (
    fit_phasor,
    magnitude_db,
    phase_deg,
)
import stage5_hfi_shadow_validation as hfi


POLE_PAIRS = 10.0
TELEMETRY_PERIOD_S = 1.0/500.0
BIAS_SPEEDS_RPM = (250.0, -250.0)
FREQUENCIES_HZ = (1.0, 2.0, 4.0, 8.0, 12.0, 20.0)
SINE_AMPLITUDE_RPM = 50.0
SPEED_RAMP_RPM_PER_S = 500.0
ENCODER_SPEED_KP = 0.00536533785
ENCODER_SPEED_KI = 0.258523708


def complex_record(value: complex) -> dict:
    return {
        "real": value.real,
        "imag": value.imag,
        "magnitude_db": magnitude_db(value),
        "phase_deg": phase_deg(value),
    }


def analyze_frames(frames: list[tuple[float, ...]],
                   frequency_hz: float, bias_rpm: float) -> dict:
    reference, reference_offset = fit_phasor(
        [frame[28] for frame in frames], frequency_hz, TELEMETRY_PERIOD_S
    )
    abz, abz_offset = fit_phasor(
        [frame[18] for frame in frames], frequency_hz, TELEMETRY_PERIOD_S
    )
    hfi_speed, hfi_offset = fit_phasor(
        [frame[15] for frame in frames], frequency_hz, TELEMETRY_PERIOD_S
    )
    iq_reference, _ = fit_phasor(
        [frame[22] for frame in frames], frequency_hz, TELEMETRY_PERIOD_S
    )
    rpm_scale = 60.0/(2.0*math.pi*POLE_PAIRS)
    # HFI/ABZ is the observer transfer we need for loop design.  The host
    # speed-reference channel can be distorted at the upper probe points, but
    # remains usable for the auxiliary closed-loop estimate at modest gain.
    if abs(reference)*rpm_scale < 5.0:
        raise RuntimeError("recorded speed reference amplitude is too small")
    if abs(abz)*rpm_scale < 5.0:
        raise RuntimeError("ABZ speed response is too small for HFI fitting")

    reference_to_abz = abz/reference
    abz_to_hfi = hfi_speed/abz
    angle_errors = [abs(math.degrees(
        hfi.circular_error(frame[9], frame[12])
    )) for frame in frames]
    return {
        "bias_rpm": bias_rpm,
        "frequency_hz": frequency_hz,
        "samples": len(frames),
        "reference_amplitude_rpm": abs(reference)*rpm_scale,
        "abz_amplitude_rpm": abs(abz)*rpm_scale,
        "hfi_amplitude_rpm": abs(hfi_speed)*rpm_scale,
        "reference_to_abz": complex_record(reference_to_abz),
        "abz_to_hfi": complex_record(abz_to_hfi),
        "reference_offset_rpm": reference_offset*rpm_scale,
        "abz_offset_rpm": abz_offset*rpm_scale,
        "hfi_offset_rpm": hfi_offset*rpm_scale,
        "hfi_offset_error_rpm": (hfi_offset-abz_offset)*rpm_scale,
        "angle_error_p95_deg": hfi.percentile(angle_errors, 0.95),
        "angle_error_max_deg": max(angle_errors),
        "angle_errors_over_90_deg": sum(value > 90.0 for value in angle_errors),
        "iq_reference_amplitude_a": abs(iq_reference),
        "peak_actual_dq_current_a": max(
            math.hypot(frame[6], frame[7]) for frame in frames
        ),
        "minimum_voltage_scale": min(frame[26] for frame in frames),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--bias-speeds", nargs="+", type=float,
                        default=list(BIAS_SPEEDS_RPM))
    parser.add_argument("--frequencies", nargs="+", type=float,
                        default=list(FREQUENCIES_HZ))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results")/(
        f"stage5_hfi_speed_feedback_{stamp}.json"
    )
    telemetry = output.with_suffix(".csv")
    result = {
        "schema": "stage5-hfi-speed-feedback-result-v1",
        "status": "running",
        "error": None,
        "control_angle": "ABZ",
        "hfi_control_authority": False,
        "bias_speeds_rpm": args.bias_speeds,
        "sine_amplitude_rpm": SINE_AMPLITUDE_RPM,
        "frequencies_hz": args.frequencies,
        "encoder_speed_pi": {
            "kp": ENCODER_SPEED_KP,
            "ki": ENCODER_SPEED_KI,
        },
        "points": [],
        "telemetry_csv": str(telemetry.resolve()),
        "final_mode": None,
        "defaults_restored": False,
    }
    limits = {
        "max_current": 5.5,
        "max_hfi_dq_current": 3.5,
        # Encoder electrical-zero calibration briefly spins faster than the
        # later low-speed experiment.  The HFI-specific 400 rpm limit below
        # remains active for every fitted point.
        "max_rpm": 1800.0,
        "max_hfi_rpm": 400.0,
        "min_vbus": 18.0,
        "max_vbus": 28.0,
        "min_voltage_scale": 0.9,
        "hfi_active": False,
    }
    records: list[dict] = []
    buffer = bytearray()
    started = time.monotonic()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.02,
                           write_timeout=0.2) as port:
            port.reset_input_buffer()
            try:
                hfi.wait_mode(port, buffer, hfi.MODE_READY, 10.0,
                              started, records, limits, "initial_ready")
                for command in (
                    "SPEED LIMIT 2", "FF MODEL IDENTIFIED", "FF MODE BOTH",
                    "FW MODE OFF", "VOLT LIMIT 1",
                    f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}", "PI CLEAR",
                    "START SPEED",
                ):
                    proto.send_command(port, command)
                hfi.wait_mode(port, buffer, hfi.MODE_SPEED_TORQUE_RUN, 20.0,
                              started, records, limits, "abz_calibration")
                hfi.wait_speed(port, buffer, 0.0, 0.0,
                               started, records, limits)

                proto.send_command(port, "HFI SHADOW 0.5 0")
                limits["hfi_active"] = True
                hfi.capture_for(port, buffer, 0.9, "tensor", 0.0,
                                started, records, limits)
                proto.send_command(port, "HFI DTRACK")
                hfi.capture_for(port, buffer, 0.9, "d_axis", 0.0,
                                started, records, limits)

                source_rpm = 0.0
                for bias_rpm in args.bias_speeds:
                    hfi.wait_speed(port, buffer, bias_rpm, source_rpm,
                                   started, records, limits)
                    source_rpm = bias_rpm
                    for frequency_hz in args.frequencies:
                        # SPEED SINE selects step tracking internally. Restore
                        # the ramp before clearing the sine so the later
                        # +250 -> -250 rpm transition is not an abrupt step.
                        proto.send_command(
                            port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}"
                        )
                        proto.send_command(port, f"SPEED {bias_rpm:g}")
                        hfi.capture_for(port, buffer, 0.55, "bias_settle",
                                        bias_rpm, started, records, limits)
                        proto.send_command(
                            port,
                            f"SPEED SINE {SINE_AMPLITUDE_RPM:g} "
                            f"{frequency_hz:g}",
                        )
                        hfi.capture_for(
                            port, buffer, max(0.25, 1.0/frequency_hz),
                            f"sine_settle_{bias_rpm:+g}_{frequency_hz:g}",
                            bias_rpm, started, records, limits,
                        )
                        frames = hfi.capture_for(
                            port, buffer, max(1.0, 4.0/frequency_hz),
                            f"sine_{bias_rpm:+g}_{frequency_hz:g}",
                            bias_rpm, started, records, limits,
                        )
                        point = analyze_frames(frames, frequency_hz, bias_rpm)
                        result["points"].append(point)
                        print(json.dumps(point), flush=True)
                        if point["angle_errors_over_90_deg"] != 0:
                            raise RuntimeError("HFI angle branch slipped")
                        proto.send_command(
                            port, f"TRACK RAMP {SPEED_RAMP_RPM_PER_S:g}"
                        )
                        proto.send_command(port, f"SPEED {bias_rpm:g}")

                result["status"] = "characterized"
            except (RuntimeError, serial.SerialException, ValueError,
                    KeyboardInterrupt) as error:
                result["status"] = "failed"
                result["error"] = str(error) or error.__class__.__name__
            finally:
                try:
                    proto.send_command(port, "SPEED SINE OFF")
                    proto.send_command(port, "HFI OFF")
                    limits["hfi_active"] = False
                    proto.send_command(port, "SPEED 0")
                    time.sleep(0.3)
                    proto.send_command(port, "STOP")
                    final = hfi.wait_mode(
                        port, buffer, hfi.MODE_READY, 3.0,
                        started, records, limits, "final_ready",
                    )
                    result["final_mode"] = int(round(final[27]))
                    for command in (
                        "FW MODE HYBRID", "VOLT LIMIT 1", "SPEED LIMIT 7",
                        "SPEED EST WIN", "FF MODEL IDENTIFIED",
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

    if (result["status"] == "failed"
            or result["final_mode"] != hfi.MODE_READY
            or not result["defaults_restored"]):
        raise RuntimeError(result["error"] or "HFI feedback test failed")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "points": len(result["points"]),
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
