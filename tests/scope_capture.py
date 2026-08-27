import argparse
import csv
import json
import struct
import time
from datetime import datetime
from pathlib import Path

import serial


MAGIC = b"SCP1"
HEADER_FORMAT = "<IHH8I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CHANNEL_NAMES = [
    "ia", "ib", "iq", "id", "iq_ref", "id_ref",
    "uq", "ud", "elec_angle", "elec_speed", "speed_ref", "voltage_scale",
]


def read_exact(port, size, timeout):
    data = bytearray()
    deadline = time.monotonic() + timeout
    while len(data) < size and time.monotonic() < deadline:
        data.extend(port.read(size - len(data)))
    if len(data) != size:
        raise RuntimeError(f"scope dump timed out: received {len(data)} of {size} bytes")
    return bytes(data)


def find_header(port, timeout):
    window = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        window.extend(port.read(port.in_waiting or 200))
        position = window.find(MAGIC)
        if position >= 0:
            header = bytes(window[position:])
            if len(header) < HEADER_SIZE:
                header += read_exact(port, HEADER_SIZE - len(header), timeout)
            return header[:HEADER_SIZE], header[HEADER_SIZE:]
        if len(window) > 4096:
            del window[:-len(MAGIC)]
    raise RuntimeError("scope header was not received; capture may not be complete")


def write_csv(path, samples, sample_period):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *CHANNEL_NAMES])
        for index, sample in enumerate(samples):
            writer.writerow([index * sample_period, *sample])


def arm_scope(port, divider):
    port.write(f"SCOPE ARM {divider}\r\n".encode("ascii"))
    port.flush()
    time.sleep(0.02)


def dump_scope(port, timeout):
    port.reset_input_buffer()
    port.write(b"SCOPE DUMP\r\n")
    port.flush()

    header_bytes, payload_prefix = find_header(port, timeout)
    fields = struct.unpack(HEADER_FORMAT, header_bytes)
    (magic, version, channel_count, sample_count, divider,
     base_sample_hz, sample_size, data_bytes, max_capture_cycles,
     cpu_hz, _) = fields

    if magic != int.from_bytes(MAGIC, "little") or version != 1:
        raise RuntimeError("unsupported scope protocol")
    if channel_count != len(CHANNEL_NAMES) or sample_size != channel_count * 4:
        raise RuntimeError("scope channel layout does not match the host script")
    if data_bytes != sample_count * sample_size:
        raise RuntimeError("scope header contains an invalid data size")

    payload = payload_prefix
    if len(payload) < data_bytes:
        payload += read_exact(port, data_bytes - len(payload), timeout)
    samples = list(struct.iter_unpack(f"<{channel_count}f", payload[:data_bytes]))
    return {
        "sample_count": sample_count,
        "divider": divider,
        "base_sample_hz": base_sample_hz,
        "sample_size": sample_size,
        "data_bytes": data_bytes,
        "max_capture_cycles": max_capture_cycles,
        "cpu_hz": cpu_hz,
    }, samples


def capture_scope(port, divider, timeout):
    arm_scope(port, divider)
    time.sleep(4096 * divider / 40000.0 + 0.1)
    return dump_scope(port, timeout)


def main():
    parser = argparse.ArgumentParser(description="Capture the 40 kHz FOC memory scope")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--divider", type=int, choices=(1, 10, 100), default=1)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    csv_path = args.csv or Path("tests/results") / (
        f"scope_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    with serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=0.2) as port:
        port.reset_input_buffer()
        capture, samples = capture_scope(port, args.divider, args.timeout)

    sample_period = capture["divider"] / float(capture["base_sample_hz"])
    write_csv(csv_path, samples, sample_period)

    print(json.dumps({
        "csv": str(csv_path.resolve()),
        "samples": capture["sample_count"],
        "channels": CHANNEL_NAMES,
        "sample_rate_hz": capture["base_sample_hz"] / capture["divider"],
        "duration_sec": capture["sample_count"] * sample_period,
        "buffer_bytes": capture["data_bytes"],
        "max_capture_cycles": capture["max_capture_cycles"],
        "max_capture_time_us": capture["max_capture_cycles"] / capture["cpu_hz"] * 1e6,
    }, indent=2))


if __name__ == "__main__":
    main()
