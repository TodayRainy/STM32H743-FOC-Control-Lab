import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto


MODE_SPEED_TORQUE_RUN = 12
MODE_READY = 17
MODE_ABZENC_CALIB = 6
TRANSITION_CURRENT_LIMIT_A = 5.0
LEGACY_MODEL = {"ld": 34.98624e-6, "lq": 53.17701e-6, "flux": 0.90451e-3}
IDENTIFIED_MODEL = {"ld": 26.043582e-6, "lq": 40.343765e-6, "flux": 0.90795034e-3}
PROFILE_DEFINITIONS = {
    "ff_off": ("ff_off", "LEGACY", False),
    "ff_legacy": ("ff_legacy", "LEGACY", True),
    "ff_identified": ("ff_identified", "IDENTIFIED", True),
}


def parse_values(text):
    values = [float(item) for item in text.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("at least one value is required")
    return values


def parse_profiles(text):
    names = [item.strip() for item in text.split(",") if item.strip()]
    if not names or any(name not in PROFILE_DEFINITIONS for name in names):
        raise argparse.ArgumentTypeError(
            "profiles must use ff_off, ff_legacy or ff_identified"
        )
    return names


def append_record(records, started, timestamp, profile, phase,
                  speed_target_rpm, id_target, frame):
    records.append({
        "elapsed_sec": timestamp - started,
        "profile": profile,
        "phase": phase,
        "speed_target_rpm": speed_target_rpm,
        "id_target_a": id_target,
        "motor_mode": int(round(frame[27])),
        "actual_rpm": proto.mechanical_rpm(frame),
        "phase_a": frame[3] - 12.0,
        "phase_b": frame[4] - 12.0,
        "phase_c": frame[5] - 12.0,
        "iq": frame[6],
        "id": frame[7],
        "uq": frame[20],
        "ud": frame[21],
        "iq_ref": frame[22],
        "id_ref": frame[23],
        "voltage_scale": frame[26],
        "speed_ref_rpm": frame[28] / proto.POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "vbus": frame[29],
        "uq_feedforward": frame[33],
        "ud_feedforward": frame[34],
        "feedforward_enabled": int(round(frame[35])),
    })


def read_checked(port, buffer, deadline, max_current, max_rpm):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is None:
        return None
    frame = sample[1]
    current_limit = (
        TRANSITION_CURRENT_LIMIT_A
        if int(round(frame[27])) == MODE_ABZENC_CALIB
        else max_current
    )
    proto.check_frame(frame, current_limit, max_rpm)
    if (int(round(frame[27])) == MODE_SPEED_TORQUE_RUN
            and math.hypot(frame[6], frame[7]) > max_current):
        raise RuntimeError("dq current vector exceeded limit")
    return sample


def wait_mode(port, buffer, expected_mode, timeout, started, records,
              profile, phase, max_current, max_rpm):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25),
            max_current, max_rpm,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, profile, phase, 0.0, 0.0, frame)
        if int(round(frame[27])) == expected_mode:
            buffer.clear()
            port.reset_input_buffer()
            return frame
    raise RuntimeError(f"timeout waiting for mode {expected_mode} during {phase}")


def wait_speed(port, buffer, target_rpm, timeout, started, records,
               profile, max_current, max_rpm):
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25),
            max_current, max_rpm,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        phase = f"settle_speed_{target_rpm:g}"
        append_record(records, started, timestamp, profile, phase, target_rpm, 0.0, frame)
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {phase}")
        if abs(proto.mechanical_rpm(frame) - target_rpm) <= 60.0:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 0.5:
                return
        else:
            stable_since = None
    raise RuntimeError(f"speed did not settle at {target_rpm:g} rpm")


def capture_point(port, buffer, duration, started, records, profile,
                  speed_target_rpm, id_target, max_current, max_rpm):
    samples = []
    deadline = time.monotonic() + duration
    phase = f"speed_{speed_target_rpm:g}_id_{id_target:g}"
    while time.monotonic() < deadline:
        sample = read_checked(
            port, buffer, min(deadline, time.monotonic() + 0.25),
            max_current, max_rpm,
        )
        if sample is None:
            continue
        timestamp, frame = sample
        if int(round(frame[27])) != MODE_SPEED_TORQUE_RUN:
            raise RuntimeError(f"motor left speed mode during {profile}/{phase}")
        append_record(
            records, started, timestamp, profile, phase,
            speed_target_rpm, id_target, frame,
        )
        samples.append(frame)
    if not samples:
        raise RuntimeError(f"no telemetry during {profile}/{phase}")
    return samples


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def summarize_point(profile, speed_target_rpm, id_target, frames):
    steady = frames[-max(1, len(frames) // 2):]
    model = IDENTIFIED_MODEL if profile == "ff_identified" else LEGACY_MODEL
    pi_q = [frame[20] - frame[33] for frame in steady]
    pi_d = [frame[21] - frame[34] for frame in steady]
    iq_error = [frame[6] - frame[22] for frame in steady]
    id_error = [frame[7] - frame[23] for frame in steady]
    formula_residual = []
    for frame in steady:
        omega_e = proto.mechanical_rpm(frame) * proto.POLE_PAIRS * 2.0 * math.pi / 60.0
        if profile == "ff_off":
            expected_q = 0.0
            expected_d = 0.0
        else:
            expected_q = omega_e * (model["flux"] + model["ld"] * frame[7])
            expected_d = -omega_e * model["lq"] * frame[6]
        formula_residual.append(math.hypot(
            frame[33] - expected_q, frame[34] - expected_d,
        ))
    return {
        "profile": profile,
        "speed_target_rpm": speed_target_rpm,
        "id_target_a": id_target,
        "samples": len(steady),
        "actual_rpm_mean": sum(proto.mechanical_rpm(frame) for frame in steady) / len(steady),
        "iq_mean_a": sum(frame[6] for frame in steady) / len(steady),
        "id_mean_a": sum(frame[7] for frame in steady) / len(steady),
        "iq_error_rms_a": rms(iq_error),
        "id_error_rms_a": rms(id_error),
        "pi_q_mean_v": sum(pi_q) / len(pi_q),
        "pi_d_mean_v": sum(pi_d) / len(pi_d),
        "pi_voltage_rms_v": rms([
            math.hypot(q_value, d_value)
            for q_value, d_value in zip(pi_q, pi_d)
        ]),
        "uq_feedforward_mean_v": sum(frame[33] for frame in steady) / len(steady),
        "ud_feedforward_mean_v": sum(frame[34] for frame in steady) / len(steady),
        "feedforward_formula_residual_rms_v": rms(formula_residual),
        "minimum_voltage_scale": min(frame[26] for frame in steady),
        "vbus_mean_v": sum(frame[29] for frame in steady) / len(steady),
    }


def write_csv(path, records):
    if not records:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def summarize_profiles(results):
    summaries = []
    for profile in dict.fromkeys(result["profile"] for result in results):
        points = [result for result in results if result["profile"] == profile]
        summaries.append({
            "profile": profile,
            "points": len(points),
            "mean_pi_voltage_rms_v": sum(
                point["pi_voltage_rms_v"] for point in points
            ) / len(points),
            "mean_iq_error_rms_a": sum(
                point["iq_error_rms_a"] for point in points
            ) / len(points),
            "mean_id_error_rms_a": sum(
                point["id_error_rms_a"] for point in points
            ) / len(points),
            "worst_pi_voltage_rms_v": max(
                point["pi_voltage_rms_v"] for point in points
            ),
            "minimum_voltage_scale": min(
                point["minimum_voltage_scale"] for point in points
            ),
        })
    return summaries


def main():
    parser = argparse.ArgumentParser(
        description="Compare disabled, legacy and identified dq feedforward models",
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--speeds", type=parse_values,
                        default=parse_values("1500,-1500,3000,-3000"))
    parser.add_argument("--id-points", type=parse_values,
                        default=parse_values("0,1,-1"))
    parser.add_argument("--profiles", type=parse_profiles,
                        default=parse_profiles("ff_off,ff_legacy,ff_identified"))
    parser.add_argument("--point-duration", type=float, default=1.5)
    parser.add_argument("--startup-timeout", type=float, default=25.0)
    parser.add_argument("--speed-timeout", type=float, default=8.0)
    parser.add_argument("--speed-ramp", type=float, default=1500.0)
    parser.add_argument("--max-current", type=float, default=3.0)
    parser.add_argument("--max-rpm", type=float, default=3500.0)
    parser.add_argument("--min-vbus", type=float, default=18.0)
    parser.add_argument("--max-vbus", type=float, default=28.0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    if any(abs(value) > 1.5 for value in args.id_points):
        parser.error("all d-axis current points must be within +/-1.5 A")

    profiles = [PROFILE_DEFINITIONS[name] for name in args.profiles]
    csv_path = args.csv or Path("tests/results") / (
        f"stage1_model_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    json_path = csv_path.with_suffix(".json")
    records = []
    results = []
    buffer = bytearray()
    started = time.monotonic()
    final_mode = None
    error = None

    try:
        with serial.Serial(
            args.port, args.baudrate, timeout=0.02, write_timeout=0.2,
        ) as port:
            port.reset_input_buffer()
            try:
                ready = wait_mode(
                    port, buffer, MODE_READY, args.startup_timeout,
                    started, records, "setup", "initial_ready",
                    args.max_current, args.max_rpm,
                )
                if not args.min_vbus <= ready[29] <= args.max_vbus:
                    raise RuntimeError(
                        f"measured bus voltage is out of range: {ready[29]:.3f} V"
                    )

                proto.send_command(port, f"TRACK RAMP {args.speed_ramp:g}")
                for profile, model, ff_enabled in profiles:
                    proto.send_command(port, "FF OFF")
                    proto.send_command(port, f"FF MODEL {model}")
                    proto.send_command(port, "FF ON" if ff_enabled else "FF OFF")
                    proto.send_command(port, "START SPEED")
                    wait_mode(
                        port, buffer, MODE_SPEED_TORQUE_RUN,
                        args.startup_timeout, started, records,
                        profile, "start_speed", args.max_current, args.max_rpm,
                    )

                    for speed_rpm in args.speeds:
                        proto.send_command(port, "SPEED ID 0")
                        proto.send_command(port, f"SPEED {speed_rpm:g}")
                        wait_speed(
                            port, buffer, speed_rpm, args.speed_timeout,
                            started, records, profile,
                            args.max_current, args.max_rpm,
                        )
                        for id_target in args.id_points:
                            proto.send_command(port, f"SPEED ID {id_target:g}")
                            frames = capture_point(
                                port, buffer, args.point_duration, started,
                                records, profile, speed_rpm, id_target,
                                args.max_current, args.max_rpm,
                            )
                            if abs(frames[-1][23] - id_target) > 0.1:
                                raise RuntimeError(
                                    f"d-axis reference was not applied: {frames[-1][23]:.3f} A"
                                )
                            expected_ff = 1 if ff_enabled else 0
                            if int(round(frames[-1][35])) != expected_ff:
                                raise RuntimeError("feedforward state was not applied")
                            results.append(summarize_point(
                                profile, speed_rpm, id_target, frames,
                            ))

                    proto.send_command(port, "SPEED ID 0")
                    proto.send_command(port, "SPEED 0")
                    wait_speed(
                        port, buffer, 0.0, args.speed_timeout,
                        started, records, profile,
                        args.max_current, args.max_rpm,
                    )
                    proto.send_command(port, "STOP")
                    wait_mode(
                        port, buffer, MODE_READY, 2.0, started, records,
                        profile, "ready_after_profile",
                        args.max_current, args.max_rpm,
                    )
            finally:
                proto.send_command(port, "SPEED ID 0")
                proto.send_command(port, "SPEED 0")
                proto.send_command(port, "STOP")
                try:
                    final = wait_mode(
                        port, buffer, MODE_READY, 2.0, started, records,
                        "cleanup", "final_ready",
                        args.max_current, args.max_rpm,
                    )
                    final_mode = int(round(final[27]))
                except RuntimeError:
                    final_mode = None
    except Exception as exc:
        error = str(exc)
    finally:
        write_csv(csv_path, records)
        report = {
            "port": args.port,
            "profiles": [profile for profile, _, _ in profiles],
            "speeds_rpm": args.speeds,
            "id_points_a": args.id_points,
            "pi_unchanged": True,
            "final_mode": final_mode,
            "error": error,
            "csv": str(csv_path.resolve()),
            "profile_summary": summarize_profiles(results) if results else [],
            "results": results,
        }
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False)
        print(json.dumps(report, indent=2, ensure_ascii=False))

    if error is not None:
        raise RuntimeError(error)
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP/READY confirmation failed")


if __name__ == "__main__":
    main()
