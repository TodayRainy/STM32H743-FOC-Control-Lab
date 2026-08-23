import argparse
import csv
import json
import math
import re
import time
from datetime import datetime
from pathlib import Path

import serial

import motor_baseline as proto
import scope_capture as scope
import scope_current_step as current_scope


MODE_TORQUE_RUN = 11
MODE_SPEED_TORQUE_RUN = 12
MODE_POSITION_SPEED_TORQUE_RUN = 15
MODE_READY = 17
MODE_ABZENC_CALIB = 6
TRANSITION_CURRENT_LIMIT = 5.0


def parse_current_points(text):
    points = []
    for item in text.split(","):
        iq_text, id_text = item.split(":", 1)
        points.append((float(iq_text), float(id_text)))
    if not points:
        raise argparse.ArgumentTypeError("at least one iq:id point is required")
    return points


def parse_values(text):
    values = [float(item) for item in text.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("at least one value is required")
    return values


def append_record(records, started, timestamp, phase, target_a, target_b, frame):
    records.append({
        "elapsed_sec": timestamp - started,
        "phase": phase,
        "target_a": target_a,
        "target_b": target_b,
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
        "position_target_deg": math.degrees(frame[30]),
        "position_ref_deg": math.degrees(frame[31]),
        "position_feedback_deg": math.degrees(frame[32]),
        "uq_feedforward": frame[33],
        "ud_feedforward": frame[34],
        "feedforward_enabled": int(round(frame[35])),
    })


def read_checked(port, buffer, deadline, max_current, max_rpm):
    sample = proto.read_frame(port, buffer, deadline)
    if sample is not None:
        frame = sample[1]
        current_limit = (
            TRANSITION_CURRENT_LIMIT
            if int(round(frame[27])) == MODE_ABZENC_CALIB
            else max_current
        )
        proto.check_frame(frame, current_limit, max_rpm)
    return sample


def wait_mode(port, buffer, expected_mode, timeout, started, records,
              phase, max_current, max_rpm):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer, min(deadline, time.monotonic() + 0.25),
                              max(max_current, TRANSITION_CURRENT_LIMIT), max_rpm)
        if sample is None:
            continue
        timestamp, frame = sample
        append_record(records, started, timestamp, phase, 0.0, 0.0, frame)
        if int(round(frame[27])) == expected_mode:
            buffer.clear()
            port.reset_input_buffer()
            return frame
    raise RuntimeError(f"timeout waiting for motor mode {expected_mode} during {phase}")


def capture(port, buffer, duration, expected_mode, started, records,
            phase, target_a, target_b, max_current, max_rpm):
    samples = []
    phase_started = time.monotonic()
    deadline = phase_started + duration
    while time.monotonic() < deadline:
        sample = read_checked(port, buffer, min(deadline, time.monotonic() + 0.25),
                              max_current, max_rpm)
        if sample is None:
            continue
        timestamp, frame = sample
        mode = int(round(frame[27]))
        if mode != expected_mode:
            raise RuntimeError(f"motor left mode {expected_mode} during {phase}: {mode}")
        append_record(records, started, timestamp, phase, target_a, target_b, frame)
        samples.append((timestamp - phase_started, frame))
    if not samples:
        raise RuntimeError(f"no telemetry captured during {phase}")
    return samples


def steady_frames(samples):
    frames = [frame for _, frame in samples]
    return frames[-max(1, len(frames) // 4):]


def current_summary(samples, iq_ref, id_ref):
    frames = steady_frames(samples)
    iq = [frame[6] for frame in frames]
    id_values = [frame[7] for frame in frames]
    return {
        "iq_ref": iq_ref,
        "id_ref": id_ref,
        "iq_mean": sum(iq) / len(iq),
        "id_mean": sum(id_values) / len(id_values),
        "iq_error_rms": math.sqrt(sum((value - iq_ref) ** 2 for value in iq) / len(iq)),
        "id_error_rms": math.sqrt(sum((value - id_ref) ** 2 for value in id_values) / len(id_values)),
        "max_abs_rpm": max(abs(proto.mechanical_rpm(frame)) for frame in frames),
    }


def position_summary(samples, target_deg):
    frames = steady_frames(samples)
    feedback_deg = [math.degrees(frame[32]) for frame in frames]
    errors = [target_deg - value for value in feedback_deg]
    response = [(timestamp, math.degrees(frame[32])) for timestamp, frame in samples]
    initial_deg = response[0][1]
    amplitude = target_deg - initial_deg
    direction = 1.0 if amplitude >= 0.0 else -1.0
    directed = [
        (timestamp, direction * (value - initial_deg))
        for timestamp, value in response
    ]
    amplitude_abs = abs(amplitude)
    t10 = next((timestamp for timestamp, value in directed
                if value >= amplitude_abs * 0.1), None)
    t90 = next((timestamp for timestamp, value in directed
                if value >= amplitude_abs * 0.9), None)
    rise_time = t90 - t10 if t10 is not None and t90 is not None and t90 >= t10 else None
    overshoot = 0.0
    if amplitude_abs > 0.0:
        overshoot = max(
            0.0,
            (max(value for _, value in directed) - amplitude_abs) / amplitude_abs * 100.0,
        )
    settling_time = None
    settling_band = max(amplitude_abs * 0.05, 0.05)
    for index, (timestamp, value) in enumerate(response):
        if all(abs(later - target_deg) <= settling_band for _, later in response[index:]):
            settling_time = timestamp
            break
    reported_target_deg = math.degrees(frames[-1][30])
    if abs(reported_target_deg - target_deg) > 0.5:
        raise RuntimeError(
            f"position command was not applied: requested={target_deg:.3f} deg, "
            f"reported={reported_target_deg:.3f} deg"
        )
    return {
        "target_deg": target_deg,
        "reported_target_deg": reported_target_deg,
        "rise_time_10_90_sec": rise_time,
        "settling_time_5pct_sec": settling_time,
        "overshoot_pct": overshoot,
        "feedback_mean_deg": sum(feedback_deg) / len(feedback_deg),
        "error_mean_deg": sum(errors) / len(errors),
        "error_rms_deg": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "error_peak_abs_deg": max(abs(value) for value in errors),
        "max_abs_rpm": max(abs(proto.mechanical_rpm(frame)) for frame in frames),
    }


def capture_scoped(port, buffer, duration, expected_mode, started, records,
                   phase, target_a, target_b, max_current, max_rpm,
                   command, divider, scope_timeout):
    scope.arm_scope(port, divider)
    proto.send_command(port, command)
    telemetry = capture(
        port, buffer, duration, expected_mode, started, records,
        phase, target_a, target_b, max_current, max_rpm,
    )
    scope_info, fast_samples = scope.dump_scope(port, scope_timeout)
    buffer.clear()
    port.reset_input_buffer()
    return telemetry, scope_info, fast_samples


def save_scope_stage(directory, phase, target_a, target_b, scope_info, samples):
    sample_period = scope_info["divider"] / float(scope_info["base_sample_hz"])
    filename = re.sub(r"[^A-Za-z0-9_.-]+", "_", phase) + ".csv"
    path = directory / filename
    scope.write_csv(path, samples, sample_period)
    iq = [sample[2] for sample in samples]
    id_values = [sample[3] for sample in samples]
    return {
        "phase": phase,
        "target_a": target_a,
        "target_b": target_b,
        "csv": str(path.resolve()),
        "sample_rate_hz": scope_info["base_sample_hz"] / scope_info["divider"],
        "duration_sec": scope_info["sample_count"] * sample_period,
        "raw_peak_current_a": max(math.hypot(q, d) for q, d in zip(iq, id_values)),
        "raw_peak_abs_iq_a": max(abs(value) for value in iq),
        "raw_peak_abs_id_a": max(abs(value) for value in id_values),
        "peak_abs_elec_speed_rad_s": max(abs(sample[9]) for sample in samples),
    }


def write_csv(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(
        description="Continuously validate current, speed, position and dq feedforward",
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--current-points", type=parse_current_points,
                        default=parse_current_points("0:0,0:0.2,0:-0.2,0:0"))
    parser.add_argument("--speeds", type=parse_values, default=parse_values("500,1500,3000"))
    parser.add_argument("--position-offsets-deg", type=parse_values,
                        default=parse_values("2,-2,5,0"))
    parser.add_argument("--current-duration", type=float, default=1.0)
    parser.add_argument("--speed-duration", type=float, default=3.0)
    parser.add_argument("--zero-duration", type=float, default=1.0)
    parser.add_argument("--position-duration", type=float, default=2.0)
    parser.add_argument("--position-rate-deg-s", type=float, default=30.0)
    parser.add_argument("--startup-timeout", type=float, default=25.0)
    parser.add_argument("--max-current", type=float, default=3.0)
    parser.add_argument("--max-rpm", type=float, default=4000.0)
    parser.add_argument("--min-vbus", type=float, default=18.0)
    parser.add_argument("--max-vbus", type=float, default=28.0)
    parser.add_argument("--scope-timeout", type=float, default=15.0)
    parser.add_argument("--skip-feedforward-comparison", action="store_true")
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    csv_path = args.csv or Path("tests/results") / (
        f"stage0_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    scope_dir = csv_path.parent / f"{csv_path.stem}_scope"
    buffer = bytearray()
    records = []
    current_results = []
    speed_results = []
    position_results = []
    scope_results = []
    started = time.monotonic()
    final_mode = None

    with serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        try:
            ready = wait_mode(
                port, buffer, MODE_READY, args.startup_timeout, started, records,
                "initial_ready", args.max_current, args.max_rpm,
            )
            if not args.min_vbus <= ready[29] <= args.max_vbus:
                raise RuntimeError(f"measured bus voltage is out of range: {ready[29]:.3f} V")

            proto.send_command(port, "FF OFF")
            proto.send_command(port, "START CURRENT")
            wait_mode(
                port, buffer, MODE_TORQUE_RUN, args.startup_timeout, started, records,
                "start_current", args.max_current, args.max_rpm,
            )
            for point_index, (iq_ref, id_ref) in enumerate(args.current_points):
                phase = f"current_{point_index}_{iq_ref:g}_{id_ref:g}"
                samples, scope_info, fast_samples = capture_scoped(
                    port, buffer, args.current_duration, MODE_TORQUE_RUN,
                    started, records, phase, iq_ref, id_ref,
                    args.max_current, args.max_rpm,
                    f"CURRENT {iq_ref:g} {id_ref:g}", 1, args.scope_timeout,
                )
                current_results.append(current_summary(samples, iq_ref, id_ref))
                fast_result = save_scope_stage(
                    scope_dir, phase, iq_ref, id_ref, scope_info, fast_samples,
                )
                fast_result["current_response"] = current_scope.summarize(
                    fast_samples, id_ref, scope_info["divider"] / scope_info["base_sample_hz"],
                )
                scope_results.append(fast_result)
            proto.send_command(port, "CURRENT 0 0")
            proto.send_command(port, "STOP")
            wait_mode(
                port, buffer, MODE_READY, 2.0, started, records,
                "ready_after_current", args.max_current, args.max_rpm,
            )

            proto.send_command(port, "TRACK RAMP 3000")
            proto.send_command(port, "START SPEED")
            wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 5.0, started, records,
                "start_speed", args.max_current, args.max_rpm,
            )
            ff_profiles = [False] if args.skip_feedforward_comparison else [False, True]
            for ff_enabled in ff_profiles:
                proto.send_command(port, "FF ON" if ff_enabled else "FF OFF")
                profile = "ff_on" if ff_enabled else "ff_off"
                for speed_rpm in args.speeds:
                    phase = f"{profile}_speed_{speed_rpm:g}"
                    samples, scope_info, fast_samples = capture_scoped(
                        port, buffer, args.speed_duration, MODE_SPEED_TORQUE_RUN,
                        started, records, phase, speed_rpm, float(ff_enabled),
                        args.max_current, args.max_rpm,
                        f"SPEED {speed_rpm:g}", 10, args.scope_timeout,
                    )
                    result = proto.summarize_stage(samples, speed_rpm, profile)
                    result["feedforward_enabled"] = ff_enabled
                    speed_results.append(result)
                    scope_results.append(save_scope_stage(
                        scope_dir, phase, speed_rpm, float(ff_enabled),
                        scope_info, fast_samples,
                    ))
                    proto.send_command(port, "SPEED 0")
                    capture(
                        port, buffer, args.zero_duration, MODE_SPEED_TORQUE_RUN,
                        started, records, f"{profile}_zero_after_{speed_rpm:g}",
                        0.0, float(ff_enabled), args.max_current, args.max_rpm,
                    )
            proto.send_command(port, "STOP")
            wait_mode(
                port, buffer, MODE_READY, 2.0, started, records,
                "ready_after_speed", args.max_current, args.max_rpm,
            )

            proto.send_command(port, f"POS TRACK RAMP {args.position_rate_deg_s:g}")
            proto.send_command(port, "FF ON")
            proto.send_command(port, "START POSITION")
            wait_mode(
                port, buffer, MODE_POSITION_SPEED_TORQUE_RUN, 5.0, started, records,
                "start_position", args.max_current, args.max_rpm,
            )
            hold_samples = capture(
                port, buffer, 1.0, MODE_POSITION_SPEED_TORQUE_RUN, started, records,
                "position_initial_hold", 0.0, 0.0, args.max_current, args.max_rpm,
            )
            position_origin_deg = math.degrees(hold_samples[-1][1][30])
            for offset_deg in args.position_offsets_deg:
                target_deg = position_origin_deg + offset_deg
                phase = f"position_offset_{offset_deg:g}"
                samples, scope_info, fast_samples = capture_scoped(
                    port, buffer, args.position_duration, MODE_POSITION_SPEED_TORQUE_RUN,
                    started, records, phase, target_deg, offset_deg,
                    args.max_current, args.max_rpm,
                    f"POSITION {offset_deg:g}", 10, args.scope_timeout,
                )
                result = position_summary(samples, target_deg)
                result["offset_deg"] = offset_deg
                position_results.append(result)
                scope_results.append(save_scope_stage(
                    scope_dir, phase, target_deg, offset_deg,
                    scope_info, fast_samples,
                ))
        finally:
            proto.send_command(port, "STOP")
            try:
                final = wait_mode(
                    port, buffer, MODE_READY, 2.0, started, records,
                    "final_ready", args.max_current, args.max_rpm,
                )
                final_mode = int(round(final[27]))
            except RuntimeError:
                final_mode = None

    if records:
        write_csv(csv_path, records)
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")

    report_path = csv_path.with_suffix(".json")
    report = {
        "port": args.port,
        "final_mode": final_mode,
        "csv": str(csv_path.resolve()),
        "report": str(report_path.resolve()),
        "current": current_results,
        "speed": speed_results,
        "position": position_results,
        "scope": scope_results,
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
