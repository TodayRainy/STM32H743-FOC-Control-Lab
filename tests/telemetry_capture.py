import argparse
import json
import math
import struct
import time

import serial


CHANNEL_NAMES = [
    "ua", "ub", "uc",
    "ia_shifted", "ib_shifted", "ic_shifted",
    "iq", "id",
    "vf_angle", "hfi_angle", "smo_angle", "flux_angle", "abz_angle", "abs_angle",
    "vf_speed", "hfi_speed", "smo_speed", "flux_speed", "abz_speed", "abs_speed",
    "uq", "ud", "iq_ref", "id_ref",
    "target_elec_speed", "elec_speed", "voltage_scale", "motor_mode",
    "speed_ref",
    "vbus", "target_position", "position_ref", "position_feedback",
    "uq_feedforward", "ud_feedforward", "feedforward_enabled",
]

SUMMARY_CHANNELS = [
    "ia_shifted", "ib_shifted", "ic_shifted", "iq", "id",
    "abz_speed", "uq", "ud", "iq_ref", "id_ref",
    "target_elec_speed", "elec_speed", "voltage_scale", "motor_mode",
    "speed_ref", "vbus", "target_position", "position_ref", "position_feedback",
    "uq_feedforward", "ud_feedforward", "feedforward_enabled",
]


def main():
    parser = argparse.ArgumentParser(description="Capture and validate JustFloat telemetry frames")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    channel_count = len(CHANNEL_NAMES)
    payload_size = channel_count * 4
    frame_size = payload_size + 4
    tail = b"\x00\x00\x80\x7f"
    data = bytearray()

    started = time.monotonic()
    with serial.Serial(args.port, args.baudrate, timeout=0.05) as port:
        while time.monotonic() - started < args.duration:
            data.extend(port.read(port.in_waiting or frame_size))
    elapsed = time.monotonic() - started

    tail_positions = []
    cursor = 0
    while True:
        position = data.find(tail, cursor)
        if position < 0:
            break
        tail_positions.append(position)
        cursor = position + len(tail)

    frames = []
    frame_ends = []
    for position in tail_positions:
        start = position - payload_size
        end = position + len(tail)
        if start < 0:
            continue
        values = struct.unpack_from(f"<{channel_count}f", data, start)
        if all(math.isfinite(value) for value in values):
            frames.append(values)
            frame_ends.append(end)

    discontinuities = sum(
        current - previous != frame_size
        for previous, current in zip(frame_ends, frame_ends[1:])
    )
    selected = {}
    for index, name in enumerate(CHANNEL_NAMES):
        if name not in SUMMARY_CHANNELS:
            continue
        values = [frame[index] for frame in frames]
        if values:
            selected[name] = {
                "min": min(values),
                "mean": sum(values) / len(values),
                "max": max(values),
            }

    result = {
        "port": args.port,
        "elapsed_sec": elapsed,
        "bytes_received": len(data),
        "frame_size": frame_size,
        "complete_frames": len(frames),
        "frame_rate_hz": len(frames) / elapsed if elapsed else 0.0,
        "discontinuities": discontinuities,
        "selected_channels": selected,
    }
    print(json.dumps(result, indent=2))

    if not frames or discontinuities != 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
