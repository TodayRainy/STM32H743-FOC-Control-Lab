import argparse
import csv
import json
import math
import struct
import time
from datetime import datetime
from pathlib import Path

import serial


CHANNEL_COUNT = 36
PAYLOAD_SIZE = CHANNEL_COUNT * 4
FRAME_SIZE = PAYLOAD_SIZE + 4
TAIL = b"\x00\x00\x80\x7f"
MODE_IDLE = 0
MODE_TORQUE_RUN = 11
MODE_SPEED_TORQUE_RUN = 12
MODE_POSITION_SPEED_TORQUE_RUN = 15
MODE_CURRENT_CALIB_FAILED = 16
MODE_READY = 17
POLE_PAIRS = 10.0


def parse_speeds(value):
    speeds = [float(item) for item in value.split(",") if item.strip()]
    if not speeds:
        raise argparse.ArgumentTypeError("at least one speed is required")
    return speeds


def parse_current_pi_profile(value):
    fields = [item.strip() for item in value.split(",")]
    if len(fields) != 5 or not fields[0]:
        raise argparse.ArgumentTypeError("expected NAME,KP_Q,KI_Q,KP_D,KI_D")
    try:
        gains = [float(item) for item in fields[1:]]
    except ValueError as error:
        raise argparse.ArgumentTypeError("PI gains must be numbers") from error
    if any(not math.isfinite(gain) or gain < 0.0 for gain in gains):
        raise argparse.ArgumentTypeError("PI gains must be finite and non-negative")
    return fields[0], *gains


def send_command(port, command):
    port.write((command + "\r\n").encode("ascii"))
    port.flush()
    time.sleep(0.02)


def read_frame(port, buffer, deadline):
    while time.monotonic() < deadline:
        buffer.extend(port.read(port.in_waiting or FRAME_SIZE))
        position = buffer.find(TAIL)
        if position < 0:
            continue

        end = position + len(TAIL)
        if position < PAYLOAD_SIZE:
            del buffer[:end]
            continue

        values = struct.unpack_from(f"<{CHANNEL_COUNT}f", buffer, position - PAYLOAD_SIZE)
        del buffer[:end]
        if all(math.isfinite(value) for value in values):
            return time.monotonic(), values
    return None


def mechanical_rpm(frame):
    return frame[25] / POLE_PAIRS * 60.0 / (2.0 * math.pi)


def check_frame(frame, max_current, max_rpm):
    mode = int(round(frame[27]))
    if mode == MODE_CURRENT_CALIB_FAILED:
        raise RuntimeError("current calibration failed")
    if mode in (MODE_TORQUE_RUN, MODE_SPEED_TORQUE_RUN, MODE_POSITION_SPEED_TORQUE_RUN):
        current_peak = max(abs(frame[6]), abs(frame[7]))
        current_name = "q/d"
    else:
        current_peak = max(abs(frame[3] - 12.0), abs(frame[4] - 12.0), abs(frame[5] - 12.0))
        current_name = "phase"
    if current_peak > max_current:
        raise RuntimeError(
            f"{current_name} current exceeded limit: {current_peak:.3f} A, mode={mode}"
        )
    if abs(mechanical_rpm(frame)) > max_rpm:
        raise RuntimeError("mechanical speed exceeded limit")


def statistics(values):
    mean = sum(values) / len(values)
    return {
        "min": min(values),
        "mean": mean,
        "max": max(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
        "peak_abs": max(abs(value) for value in values),
    }


def first_crossing(samples, threshold):
    return next((timestamp for timestamp, value in samples if value >= threshold), None)


def summarize_stage(samples, target_rpm, profile):
    timestamps = [timestamp for timestamp, _ in samples]
    frames = [frame for _, frame in samples]
    rpm = [mechanical_rpm(frame) for frame in frames]
    direction = 1.0 if target_rpm >= 0.0 else -1.0
    directed_rpm = [(timestamp, direction * value) for timestamp, value in zip(timestamps, rpm)]
    target_abs = abs(target_rpm)

    t10 = first_crossing(directed_rpm, target_abs * 0.1)
    t90 = first_crossing(directed_rpm, target_abs * 0.9)
    rise_time = t90 - t10 if t10 is not None and t90 is not None and t90 >= t10 else None

    band = target_abs * 0.05
    settle_time = None
    for index, (timestamp, value) in enumerate(directed_rpm):
        if all(abs(later - target_abs) <= band for _, later in directed_rpm[index:]):
            settle_time = timestamp
            break

    peak = max(value for _, value in directed_rpm)
    steady_count = max(1, len(rpm) // 5)
    channel_values = {
        "mechanical_rpm": rpm,
        "phase_a": [frame[3] - 12.0 for frame in frames],
        "phase_b": [frame[4] - 12.0 for frame in frames],
        "phase_c": [frame[5] - 12.0 for frame in frames],
        "iq": [frame[6] for frame in frames],
        "id": [frame[7] for frame in frames],
        "uq": [frame[20] for frame in frames],
        "ud": [frame[21] for frame in frames],
        "iq_ref": [frame[22] for frame in frames],
        "id_ref": [frame[23] for frame in frames],
        "voltage_scale": [frame[26] for frame in frames],
        "speed_ref_rpm": [frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi) for frame in frames],
        "vbus": [frame[29] for frame in frames],
        "uq_feedforward": [frame[33] for frame in frames],
        "ud_feedforward": [frame[34] for frame in frames],
    }
    return {
        "profile": profile,
        "command_rpm": target_rpm,
        "frames": len(frames),
        "rise_time_10_90_sec": rise_time,
        "settling_time_5pct_sec": settle_time,
        "overshoot_pct": max(0.0, (peak - target_abs) / target_abs * 100.0),
        "steady_rpm_mean": sum(rpm[-steady_count:]) / steady_count,
        "channels": {name: statistics(values) for name, values in channel_values.items()},
    }


def append_record(records, started, timestamp, profile, phase, target_rpm, frame):
    records.append({
        "elapsed_sec": timestamp - started,
        "profile": profile,
        "phase": phase,
        "target_rpm": target_rpm,
        "actual_rpm": mechanical_rpm(frame),
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
        "motor_mode": int(round(frame[27])),
        "speed_ref_rpm": frame[28] / POLE_PAIRS * 60.0 / (2.0 * math.pi),
        "vbus": frame[29],
        "target_position_rad": frame[30],
        "position_ref_rad": frame[31],
        "position_feedback_rad": frame[32],
        "uq_feedforward": frame[33],
        "ud_feedforward": frame[34],
        "feedforward_enabled": int(round(frame[35])),
    })


def capture_until(port, buffer, deadline, started, profile, phase, target_rpm, records,
                  max_current, max_rpm, required_mode=None):
    samples = []
    phase_started = time.monotonic()
    while time.monotonic() < deadline:
        sample = read_frame(port, buffer, min(deadline, time.monotonic() + 0.25))
        if sample is None:
            continue
        timestamp, frame = sample
        check_frame(frame, max_current, max_rpm)
        if required_mode is not None and int(round(frame[27])) != required_mode:
            raise RuntimeError(f"motor left required mode during {phase}")
        append_record(records, started, timestamp, profile, phase, target_rpm, frame)
        samples.append((timestamp - phase_started, frame))
    return samples


def write_csv(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(description="Run guarded continuous FOC speed steps")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--speeds", type=parse_speeds, default=parse_speeds("300,800,1500"))
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--initial-settle-duration", type=float, default=2.0)
    parser.add_argument("--settle-duration", type=float, default=2.0)
    parser.add_argument("--calibration-timeout", type=float, default=20.0)
    parser.add_argument("--max-current", type=float, default=4.0)
    parser.add_argument("--max-rpm", type=float, default=2000.0)
    parser.add_argument("--speed-pi", type=float, nargs=2, metavar=("KP", "KI"))
    parser.add_argument("--iq-pi", type=float, nargs=2, metavar=("KP", "KI"))
    parser.add_argument("--id-pi", type=float, nargs=2, metavar=("KP", "KI"))
    parser.add_argument(
        "--current-pi-profile", action="append", type=parse_current_pi_profile,
        metavar="NAME,KP_Q,KI_Q,KP_D,KI_D",
    )
    parser.add_argument("--profile-settle-duration", type=float, default=1.0)
    parser.add_argument("--ramp-rpm-per-sec", type=float)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    if args.current_pi_profile and (args.iq_pi or args.id_pi):
        parser.error("--current-pi-profile cannot be combined with --iq-pi or --id-pi")

    csv_path = args.csv or Path("tests/results") / (
        f"motor_baseline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    buffer = bytearray()
    records = []
    stages = []
    final_mode = None
    started = time.monotonic()

    with serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        try:
            ready_deadline = time.monotonic() + args.calibration_timeout
            while time.monotonic() < ready_deadline:
                sample = read_frame(port, buffer, min(ready_deadline, time.monotonic() + 0.25))
                if sample is None:
                    continue
                _, frame = sample
                check_frame(frame, args.max_current, args.max_rpm)
                if int(round(frame[27])) == MODE_READY:
                    break
            else:
                raise RuntimeError("motor did not reach READY before START")

            if args.speed_pi:
                send_command(port, f"PI SPEED {args.speed_pi[0]:g} {args.speed_pi[1]:g}")
            if args.iq_pi:
                send_command(port, f"PI IQ {args.iq_pi[0]:g} {args.iq_pi[1]:g}")
            if args.id_pi:
                send_command(port, f"PI ID {args.id_pi[0]:g} {args.id_pi[1]:g}")
            if args.ramp_rpm_per_sec:
                send_command(port, f"TRACK RAMP {args.ramp_rpm_per_sec:g}")
            else:
                send_command(port, "TRACK STEP")

            send_command(port, "START SPEED")
            deadline = time.monotonic() + args.calibration_timeout
            while time.monotonic() < deadline:
                sample = read_frame(port, buffer, min(deadline, time.monotonic() + 0.25))
                if sample is None:
                    continue
                timestamp, frame = sample
                check_frame(frame, args.max_current, args.max_rpm)
                mode = int(round(frame[27]))
                append_record(records, started, timestamp, "calibration", "calibration", 0.0, frame)
                if mode == MODE_IDLE:
                    raise RuntimeError("motor returned to IDLE during calibration")
                if mode == MODE_SPEED_TORQUE_RUN:
                    break
            else:
                raise RuntimeError("encoder calibration timeout")

            capture_until(
                port, buffer, time.monotonic() + args.initial_settle_duration, started,
                "calibration", "initial_zero", 0.0, records,
                args.max_current, args.max_rpm, MODE_SPEED_TORQUE_RUN,
            )

            profiles = args.current_pi_profile or [("configured", None, None, None, None)]
            for profile, kp_q, ki_q, kp_d, ki_d in profiles:
                if kp_q is not None:
                    send_command(port, f"PI IQ {kp_q:g} {ki_q:g}")
                    send_command(port, f"PI ID {kp_d:g} {ki_d:g}")
                    send_command(port, "PI CLEAR")
                capture_until(
                    port, buffer, time.monotonic() + args.profile_settle_duration, started,
                    profile, "profile_zero", 0.0, records,
                    args.max_current, args.max_rpm, MODE_SPEED_TORQUE_RUN,
                )

                for speed_rpm in args.speeds:
                    send_command(port, f"SPEED {speed_rpm:g}")
                    samples = capture_until(
                        port, buffer, time.monotonic() + args.duration, started,
                        profile, f"speed_{speed_rpm:g}", speed_rpm, records,
                        args.max_current, args.max_rpm, MODE_SPEED_TORQUE_RUN,
                    )
                    if not samples:
                        raise RuntimeError(f"no telemetry captured at {speed_rpm:g} rpm")
                    stages.append(summarize_stage(samples, speed_rpm, profile))

                    send_command(port, "SPEED 0")
                    capture_until(
                        port, buffer, time.monotonic() + args.settle_duration, started,
                        profile, f"zero_after_{speed_rpm:g}", 0.0, records,
                        args.max_current, args.max_rpm, MODE_SPEED_TORQUE_RUN,
                    )
        finally:
            send_command(port, "STOP")
            stop_deadline = time.monotonic() + 1.0
            while time.monotonic() < stop_deadline:
                sample = read_frame(port, buffer, min(stop_deadline, time.monotonic() + 0.2))
                if sample is None:
                    continue
                timestamp, frame = sample
                final_mode = int(round(frame[27]))
                append_record(records, started, timestamp, "stop", "stop", 0.0, frame)
                if final_mode == MODE_READY:
                    break

    if final_mode != MODE_READY:
        raise RuntimeError("STOP was not confirmed by telemetry")

    write_csv(csv_path, records)
    print(json.dumps({
        "port": args.port,
        "speeds_rpm": args.speeds,
        "speed_pi": args.speed_pi,
        "iq_pi": args.iq_pi,
        "id_pi": args.id_pi,
        "current_pi_profiles": args.current_pi_profile,
        "ramp_rpm_per_sec": args.ramp_rpm_per_sec,
        "final_mode": final_mode,
        "csv": str(csv_path.resolve()),
        "stages": stages,
    }, indent=2))


if __name__ == "__main__":
    main()
