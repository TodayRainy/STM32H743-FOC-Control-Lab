import argparse
import csv
import json
import re
import struct
import subprocess
import tempfile
import time
from datetime import datetime
from pathlib import Path


MAGIC = 0x31504353
STATE_DONE = 2
CONTROL_FORMAT = "<9I"
CONTROL_SIZE = struct.calcsize(CONTROL_FORMAT)
CHANNEL_NAMES = [
    "ia", "ib", "iq", "id", "iq_ref", "id_ref",
    "uq", "ud", "elec_angle", "elec_speed", "speed_ref", "voltage_scale",
    "hfi_phase_error", "hfi_d_response", "hfi_q_response",
]


def symbol_address(map_path, symbol):
    text = map_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        rf"^\s*{re.escape(symbol)}\s+(0x[0-9A-Fa-f]+)\s+Data\b",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise RuntimeError(f"symbol not found in map: {symbol}")
    return int(match.group(1), 16)


def run_jlink(args, commands):
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".jlink", delete=False, encoding="ascii"
    ) as script:
        script.write(commands)
        script_path = Path(script.name)

    command = [
        str(args.jlink), "-NoGui", "1", "-ExitOnError", "1",
        "-AutoConnect", "1", "-Device", args.device,
    ]
    if args.probe_serial:
        command.extend(["-SelectEmuBySN", args.probe_serial])
    command.extend(["-CommandFile", str(script_path)])

    try:
        result = subprocess.run(
            command,
            cwd=args.workspace,
            capture_output=True,
            text=True,
            timeout=30,
        )
    finally:
        script_path.unlink(missing_ok=True)

    output = result.stdout + result.stderr
    if result.returncode != 0 or "Script processing completed." not in output:
        raise RuntimeError(f"J-Link command failed:\n{output[-2000:]}")
    return output


def write_csv(path, samples, sample_period):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *CHANNEL_NAMES])
        for index, sample in enumerate(samples):
            writer.writerow([index * sample_period, *sample])


def main():
    workspace_default = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Capture the FOC memory scope through J-Link")
    parser.add_argument("--workspace", type=Path, default=workspace_default)
    parser.add_argument(
        "--jlink", type=Path,
        default=Path(r"C:\Program Files\SEGGER\JLink\JLink.exe"),
    )
    parser.add_argument("--device", default="STM32H743VG")
    parser.add_argument("--probe-serial")
    parser.add_argument("--speed", type=int, default=4000)
    parser.add_argument("--divider", type=int, choices=(1, 10, 100), default=1)
    parser.add_argument("--cpu-hz", type=int, default=480_000_000)
    parser.add_argument("--map", type=Path, default=Path("mdk/build/STM32H743VGT6_FOC.map"))
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    args.workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else args.workspace / args.map
    control_address = symbol_address(map_path, "foc_scope_control")
    buffer_address = symbol_address(map_path, "foc_scope_buffer")

    common = f"si SWD\nspeed {args.speed}\nconnect\nhalt\n"
    arm_commands = common + (
        f"w4 0x{control_address + 4:08X}, 0x0\n"
        f"w4 0x{control_address + 12:08X}, 0x{args.divider:X}\n"
        f"w4 0x{control_address + 16:08X}, 0x0\n"
        f"w4 0x{control_address + 32:08X}, 0x0\n"
        f"w4 0x{control_address + 4:08X}, 0x1\n"
        "g\nexit\n"
    )
    run_jlink(args, arm_commands)

    capture_time = 4096 * args.divider / 40000.0
    time.sleep(capture_time + 0.1)

    with tempfile.TemporaryDirectory() as directory:
        directory_path = Path(directory)
        control_bin = directory_path / "scope_control.bin"
        samples_bin = directory_path / "scope_samples.bin"
        dump_commands = common + (
            f"savebin {control_bin.as_posix()}, 0x{control_address:08X}, 0x{CONTROL_SIZE:X}\n"
            f"savebin {samples_bin.as_posix()}, 0x{buffer_address:08X}, 0x28000\n"
            "g\nexit\n"
        )
        run_jlink(args, dump_commands)

        (magic, state, base_sample_hz, divider, sample_count, sample_capacity,
         channel_count, sample_size, max_capture_cycles) = struct.unpack(
            CONTROL_FORMAT, control_bin.read_bytes()[:CONTROL_SIZE]
        )
        payload = samples_bin.read_bytes()[:sample_count * sample_size]

    if magic != MAGIC or state != STATE_DONE:
        raise RuntimeError(f"scope did not complete: magic=0x{magic:08X}, state={state}")
    if sample_count != sample_capacity:
        raise RuntimeError(f"incomplete capture: {sample_count}/{sample_capacity} samples")
    if channel_count != len(CHANNEL_NAMES) or sample_size != channel_count * 4:
        raise RuntimeError("scope channel layout does not match the host script")

    samples = list(struct.iter_unpack(f"<{channel_count}f", payload))
    sample_period = divider / float(base_sample_hz)
    csv_path = args.csv or Path("tests/results") / (
        f"scope_jlink_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    if not csv_path.is_absolute():
        csv_path = args.workspace / csv_path
    write_csv(csv_path, samples, sample_period)

    print(json.dumps({
        "backend": "jlink",
        "csv": str(csv_path.resolve()),
        "samples": sample_count,
        "channels": CHANNEL_NAMES,
        "sample_rate_hz": base_sample_hz / divider,
        "duration_sec": sample_count * sample_period,
        "buffer_bytes": len(payload),
        "max_capture_cycles": max_capture_cycles,
        "max_capture_time_us": max_capture_cycles / args.cpu_hz * 1e6,
        "foc_period_usage_pct": max_capture_cycles / args.cpu_hz / 25e-6 * 100.0,
    }, indent=2))


if __name__ == "__main__":
    main()
