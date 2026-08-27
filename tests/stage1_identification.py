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


MODE_IDEN_RL = 8
MODE_IDEN_FLUX = 9
MODE_ABZENC_CALIB = 6
MODE_READY = 17
TRANSITION_CURRENT_LIMIT = 5.0
IDENT_PATTERN = re.compile(rb"IDENT_(RL|RS_DIAG|LCHECK|FLUX),([^\r\n]+)\r\n")
RS_POINT_PATTERN = re.compile(rb"IDENT_RS_POINTS,([^\r\n]+)\r\n")


def append_record(records, started, phase, timestamp, frame):
    records.append({
        "elapsed_sec": timestamp - started,
        "phase": phase,
        "motor_mode": int(round(frame[27])),
        "mechanical_rpm": proto.mechanical_rpm(frame),
        "phase_a": frame[3] - 12.0,
        "phase_b": frame[4] - 12.0,
        "phase_c": frame[5] - 12.0,
        "iq": frame[6],
        "id": frame[7],
        "uq": frame[20],
        "ud": frame[21],
        "vbus": frame[29],
    })


def wait_mode(port, buffer, expected_mode, timeout, started, records,
              phase, max_current, max_rpm):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = proto.read_frame(port, buffer, min(deadline, time.monotonic() + 0.25))
        if sample is None:
            continue
        timestamp, frame = sample
        current_limit = (
            TRANSITION_CURRENT_LIMIT
            if int(round(frame[27])) == MODE_ABZENC_CALIB
            else max_current
        )
        proto.check_frame(frame, current_limit, max_rpm)
        append_record(records, started, phase, timestamp, frame)
        if int(round(frame[27])) == expected_mode:
            return frame
    raise RuntimeError(f"timeout waiting for motor mode {expected_mode} during {phase}")


def read_ident_result(port, timeout):
    port.reset_input_buffer()
    proto.send_command(port, "IDENT RESULT")
    data = bytearray()
    result = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data.extend(port.read(port.in_waiting or 64))
        for match in IDENT_PATTERN.finditer(data):
            section = match.group(1).decode("ascii").lower()
            fields = result.setdefault(section, {})
            for item in match.group(2).decode("ascii").split(","):
                name, value = item.split("=", 1)
                fields[name] = float(value)
        if {"rl", "rs_diag", "lcheck", "flux"}.issubset(result):
            merged = {}
            for section in ("rl", "rs_diag", "lcheck", "flux"):
                merged.update(result[section])
            return merged
        if len(data) > 4096:
            del data[:-2048]
    raise RuntimeError("timeout waiting for IDENT RESULT")


def read_rs_points(port, timeout):
    port.reset_input_buffer()
    proto.send_command(port, "IDENT POINTS")
    data = bytearray()
    points = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data.extend(port.read(port.in_waiting or 64))
        for match in RS_POINT_PATTERN.finditer(data):
            for item in match.group(1).decode("ascii").split(","):
                index_text, values = item.split("=", 1)
                current_text, voltage_text = values.split(":", 1)
                points[int(index_text)] = {
                    "current_a": float(current_text),
                    "voltage_v": float(voltage_text),
                }
        if len(points) == 16:
            return [dict(index=index, **points[index]) for index in sorted(points)]
        if len(data) > 4096:
            del data[:-2048]
    raise RuntimeError(f"timeout waiting for IDENT POINTS ({len(points)}/16 received)")


def run_identification(port, buffer, command, mode, timeout, started, records,
                       phase, max_current, max_rpm):
    proto.send_command(port, command)
    wait_mode(port, buffer, mode, 30.0, started, records,
              phase + "_start", max_current, max_rpm)
    wait_mode(port, buffer, MODE_READY, timeout, started, records,
              phase + "_run", max_current, max_rpm)
    result = read_ident_result(port, 3.0)
    result["RsPoints"] = read_rs_points(port, 3.0)
    buffer.clear()
    port.reset_input_buffer()
    return result


def add_flux_derived_metrics(result, pole_pairs, nameplate_kv):
    flux = result.get("Flux", 0.0)
    if flux <= 0.0:
        return
    model_kv = 60.0 / (2.0 * math.pi * math.sqrt(3.0) * pole_pairs * flux)
    result["ModelKvRpmPerVdc"] = model_kv
    result["ModelKvErrorFraction"] = model_kv / nameplate_kv - 1.0
    result["TorqueConstantNmPerPeakA"] = 1.5 * pole_pairs * flux


def write_csv(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(
        description="Run repeatable stage-1 electrical parameter identification."
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rl-repeats", type=int, default=2)
    parser.add_argument("--flux-repeats", type=int, default=2)
    parser.add_argument("--startup-timeout", type=float, default=25.0)
    parser.add_argument("--rl-timeout", type=float, default=15.0)
    parser.add_argument("--flux-timeout", type=float, default=60.0)
    parser.add_argument("--max-current", type=float, default=5.0)
    parser.add_argument("--max-rpm", type=float, default=3500.0)
    parser.add_argument("--min-vbus", type=float, default=18.0)
    parser.add_argument("--max-vbus", type=float, default=28.0)
    parser.add_argument("--pole-pairs", type=int, default=10)
    parser.add_argument("--nameplate-kv", type=float, default=650.0)
    parser.add_argument("--skip-rl", action="store_true")
    parser.add_argument("--skip-flux", action="store_true")
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    if ((not args.skip_rl and args.rl_repeats < 1)
            or (not args.skip_flux and args.flux_repeats < 1)):
        parser.error("repeat counts must be positive")
    if args.pole_pairs < 1 or args.nameplate_kv <= 0.0:
        parser.error("pole pairs and nameplate Kv must be positive")

    csv_path = args.csv or Path("tests/results") / (
        f"stage1_identification_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    report_path = csv_path.with_suffix(".json")
    started = time.monotonic()
    buffer = bytearray()
    records = []
    results = {"rl": [], "flux": []}
    final_mode = None
    failure = None

    with serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        try:
            ready = wait_mode(
                port, buffer, MODE_READY, args.startup_timeout, started, records,
                "initial_ready", args.max_current, args.max_rpm,
            )
            if not args.min_vbus <= ready[29] <= args.max_vbus:
                raise RuntimeError(f"measured bus voltage is out of range: {ready[29]:.3f} V")

            if not args.skip_rl:
                for repeat in range(args.rl_repeats):
                    results["rl"].append(run_identification(
                        port, buffer, "START IDENT RL", MODE_IDEN_RL,
                        args.rl_timeout, started, records, f"rl_{repeat}",
                        args.max_current, args.max_rpm,
                    ))

            if not args.skip_flux:
                for repeat in range(args.flux_repeats):
                    result = run_identification(
                        port, buffer, "START IDENT FLUX", MODE_IDEN_FLUX,
                        args.flux_timeout, started, records, f"flux_{repeat}",
                        args.max_current, args.max_rpm,
                    )
                    add_flux_derived_metrics(result, args.pole_pairs, args.nameplate_kv)
                    results["flux"].append(result)
        except Exception as exc:
            failure = exc
        finally:
            proto.send_command(port, "STOP")
            try:
                final = wait_mode(
                    port, buffer, MODE_READY, 3.0, started, records,
                    "final_ready", args.max_current, args.max_rpm,
                )
                final_mode = int(round(final[27]))
            except RuntimeError:
                final_mode = None

    if records:
        write_csv(csv_path, records)

    report = {
        "port": args.port,
        "final_mode": final_mode,
        "csv": str(csv_path.resolve()),
        "error": None if failure is None else str(failure),
        "results": results,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    if final_mode != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry") from failure
    if failure is not None:
        raise failure


if __name__ == "__main__":
    main()
