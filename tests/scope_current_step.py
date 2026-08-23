import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from statistics import median

import serial

import motor_baseline as proto
import scope_capture as scope


MODE_TORQUE_RUN = 11
MODE_READY = 17


def wait_mode(port, buffer, expected_mode, timeout, max_current, max_rpm):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = proto.read_frame(port, buffer, min(deadline, time.monotonic() + 0.25))
        if sample is None:
            continue
        frame = sample[1]
        proto.check_frame(frame, max_current, max_rpm)
        if int(round(frame[27])) == expected_mode:
            buffer.clear()
            port.reset_input_buffer()
            return frame
    raise RuntimeError(f"timeout waiting for motor mode {expected_mode}")


def first_crossing(values, start, threshold, direction):
    for index in range(start, len(values)):
        if direction * values[index] >= direction * threshold:
            return index
    return None


def median_filter(values, width=5):
    half = width // 2
    return [
        median(values[max(0, index - half):min(len(values), index + half + 1)])
        for index in range(len(values))
    ]


def summarize(samples, target_id, sample_period):
    iq = [sample[2] for sample in samples]
    id_values = [sample[3] for sample in samples]
    id_filtered = median_filter(id_values)
    id_ref = [sample[5] for sample in samples]
    tolerance = max(0.005, abs(target_id) * 0.05)
    step_index = next(
        (index for index, value in enumerate(id_ref) if abs(value - target_id) <= tolerance),
        None,
    )
    if step_index is None:
        raise RuntimeError("d-axis current command is not present in the scope data")

    baseline_values = id_filtered[:step_index] or [id_filtered[0]]
    baseline = sum(baseline_values) / len(baseline_values)
    steady_start = max(step_index + 1, len(samples) * 3 // 4)
    steady = id_values[steady_start:]
    steady_mean = sum(steady) / len(steady)
    amplitude = target_id - baseline
    direction = 1.0 if amplitude >= 0.0 else -1.0
    index_10 = first_crossing(
        id_filtered, step_index, baseline + 0.1 * amplitude, direction,
    )
    index_90 = first_crossing(
        id_filtered, step_index, baseline + 0.9 * amplitude, direction,
    )
    rise_time = None
    if index_10 is not None and index_90 is not None and index_90 >= index_10:
        rise_time = (index_90 - index_10) * sample_period

    directed_peak = max(direction * (value - baseline) for value in id_filtered[step_index:])
    amplitude_abs = abs(amplitude)
    overshoot = None
    if amplitude_abs >= 0.02:
        overshoot = max(0.0, (directed_peak - amplitude_abs) / amplitude_abs * 100.0)
    else:
        rise_time = None

    return {
        "target_id_a": target_id,
        "command_time_sec": step_index * sample_period,
        "rise_time_10_90_sec": rise_time,
        "filtered_overshoot_pct": overshoot,
        "steady_id_mean_a": steady_mean,
        "steady_error_a": target_id - steady_mean,
        "steady_error_rms_a": math.sqrt(
            sum((target_id - value) ** 2 for value in steady) / len(steady)
        ),
        "raw_peak_id_a": max(abs(value) for value in id_values),
        "raw_peak_current_a": max(math.hypot(q, d) for q, d in zip(iq, id_values)),
        "raw_peak_abs_iq_a": max(abs(value) for value in iq),
    }


def main():
    parser = argparse.ArgumentParser(description="Capture one small d-axis current step")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--target-id", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    csv_path = args.csv or Path("tests/results") / (
        f"scope_current_step_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    buffer = bytearray()
    final_mode = None

    with serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        ready = wait_mode(port, buffer, MODE_READY, args.timeout, 5.0, 2000.0)
        if not 18.0 <= ready[29] <= 28.0:
            raise RuntimeError(f"measured bus voltage is out of range: {ready[29]:.3f} V")

        try:
            proto.send_command(port, "FF OFF")
            proto.send_command(port, "START CURRENT")
            wait_mode(port, buffer, MODE_TORQUE_RUN, args.timeout, 5.0, 2000.0)
            proto.send_command(port, "CURRENT 0 0")
            time.sleep(0.25)

            scope.arm_scope(port, 1)
            proto.send_command(port, f"CURRENT 0 {args.target_id:g}")
            time.sleep(4096 / 40000.0 + 0.05)
            proto.send_command(port, "CURRENT 0 0")
            capture, samples = scope.dump_scope(port, args.timeout)
        finally:
            proto.send_command(port, "CURRENT 0 0")
            proto.send_command(port, "STOP")
            try:
                final = wait_mode(port, buffer, MODE_READY, 2.0, 5.0, 2000.0)
                final_mode = int(round(final[27]))
            except RuntimeError:
                final_mode = None

    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")

    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    scope.write_csv(csv_path, samples, sample_period)
    result = summarize(samples, args.target_id, sample_period)
    result.update({
        "csv": str(csv_path.resolve()),
        "samples": capture["sample_count"],
        "sample_rate_hz": capture["base_sample_hz"] / capture["divider"],
        "final_mode": final_mode,
    })
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
