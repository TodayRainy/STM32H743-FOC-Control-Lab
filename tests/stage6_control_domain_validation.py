from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from host.foc_client import CommandResult, FocDevice, StatusReport, TelemetryFrame
from host.foc_client.protocol import TELEMETRY_CHANNELS


RPM_PER_ELEC_RAD_S = 60.0 / (2.0 * math.pi * 10.0)
DEG_PER_RAD = 180.0 / math.pi


class DomainRunner:
    def __init__(self, port: str, baudrate: int) -> None:
        self.device = FocDevice(baudrate)
        self.device.open(port)
        self.started = time.monotonic()
        self.segment = "startup"
        self.records: list[dict] = []
        self.command_results: list[CommandResult] = []
        self.status: StatusReport | None = None

    def close(self) -> None:
        self.device.close(send_stop=False)

    def poll(self, duration: float = 0.0) -> None:
        deadline = time.monotonic() + duration
        first = True
        while first or time.monotonic() < deadline:
            first = False
            for event in self.device.poll():
                if isinstance(event, TelemetryFrame):
                    row = {
                        "time_s": time.monotonic() - self.started,
                        "segment": self.segment,
                    }
                    row.update(event.channels)
                    self.records.append(row)
                    self._safety_check(row)
                elif isinstance(event, CommandResult):
                    self.command_results.append(event)
                elif isinstance(event, StatusReport):
                    self.status = event
            if duration > 0.0:
                time.sleep(0.002)

    @staticmethod
    def _safety_check(row: dict) -> None:
        speed_rpm = row["elec_speed"] * RPM_PER_ELEC_RAD_S
        dq_current = math.hypot(row["iq"], row["id"])
        if dq_current > 5.0:
            raise RuntimeError(f"dq current safety limit: {dq_current:.3f} A")
        if abs(speed_rpm) > 6700.0:
            raise RuntimeError(f"speed safety limit: {speed_rpm:.1f} rpm")
        if not 18.0 <= row["vbus"] <= 28.0:
            raise RuntimeError(f"bus voltage safety limit: {row['vbus']:.3f} V")

    def command(self, text: str, expected: str, timeout: float = 2.0) -> CommandResult:
        start = len(self.command_results)
        self.device.command(text)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll(0.01)
            for result in self.command_results[start:]:
                if result.name == expected:
                    if not result.accepted:
                        raise RuntimeError(
                            f"{text!r} rejected: {result.reason}"
                        )
                    return result
        raise RuntimeError(f"no response for {text!r}")

    def raw_command(self, text: str) -> None:
        self.device.command(text)
        self.poll(0.05)

    def wait_status(self, lifecycle: str, operation: str | None = None,
                    timeout: float = 20.0) -> StatusReport:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.device.request_status()
            self.poll(0.12)
            if self.status is not None and self.status.lifecycle == lifecycle:
                if operation is None or self.status.operation == operation:
                    return self.status
        raise RuntimeError(f"timeout waiting for {lifecycle}/{operation}")

    def wait_reference(self, channel: str, target: float, scale: float,
                       tolerance: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        qualified_since = None
        while time.monotonic() < deadline:
            before = len(self.records)
            self.poll(0.02)
            if len(self.records) == before:
                continue
            value = self.records[-1][channel] * scale
            if abs(value - target) <= tolerance:
                qualified_since = qualified_since or time.monotonic()
                if time.monotonic() - qualified_since >= 0.10:
                    return
            else:
                qualified_since = None
        raise RuntimeError(f"{channel} did not reach {target:g}")


def metric(values: list[float]) -> dict:
    return {
        "mean": statistics.fmean(values),
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
        "minimum": min(values),
        "maximum": max(values),
    }


def steady_rows(runner: DomainRunner, start: int) -> list[dict]:
    rows = runner.records[start:]
    if len(rows) < 50:
        raise RuntimeError("insufficient steady telemetry")
    return rows


def run_speed_matrix(runner: DomainRunner, targets: list[float]) -> list[dict]:
    runner.command("FF MODEL IDENTIFIED", "FF_MODEL")
    runner.command("FF MODE BOTH", "FF_MODE")
    runner.command("FW MODE HYBRID", "FW_MODE")
    runner.command("SPEED LIMIT 4", "SPEED_LIMIT")
    runner.command("VOLT LIMIT 1", "VOLT_LIMIT")
    runner.raw_command("TRACK RAMP 3000")
    runner.command("START SPEED", "START_SPEED")
    runner.wait_status("RUNNING", "SPEED")

    results = []
    for target in targets:
        runner.segment = f"speed_{target:+g}_transition"
        transition_start = len(runner.records)
        runner.command(f"SPEED {target:g}", "SPEED")
        runner.wait_reference(
            "speed_ref", target, RPM_PER_ELEC_RAD_S,
            tolerance=max(2.0, abs(target) * 0.005), timeout=6.0,
        )
        runner.segment = f"speed_{target:+g}_steady"
        steady_start = len(runner.records)
        runner.poll(0.80)
        rows = steady_rows(runner, steady_start)
        speed = [row["elec_speed"] * RPM_PER_ELEC_RAD_S for row in rows]
        reference = [row["speed_ref"] * RPM_PER_ELEC_RAD_S for row in rows]
        error = [ref - actual for ref, actual in zip(reference, speed)]
        transition = runner.records[transition_start:]
        current_peak = max(math.hypot(row["iq"], row["id"]) for row in transition)
        tolerance = max(5.0, abs(target) * 0.02)
        error_stats = metric(error)
        results.append({
            "target_rpm": target,
            "speed_rpm": metric(speed),
            "error_rpm": error_stats,
            "peak_dq_current_a": current_peak,
            "minimum_voltage_scale": min(row["voltage_scale"] for row in transition),
            "passed": abs(error_stats["mean"]) <= tolerance
                      and error_stats["rms"] <= max(15.0, tolerance * 1.5),
        })

    runner.command("STOP", "STOP")
    runner.wait_status("READY", "NONE", 5.0)
    return results


def run_position_matrix(runner: DomainRunner, name: str, configure: list[str],
                        targets: list[float]) -> list[dict]:
    for command in configure:
        runner.raw_command(command)
    runner.command("START POSITION", "START_POSITION")
    runner.wait_status("RUNNING", "POSITION_SPEED")
    runner.poll(0.1)

    results = []
    for target in targets:
        runner.segment = f"{name}_{target:+g}_transition"
        transition_start = len(runner.records)
        runner.command(f"POSITION {target:g}", "POSITION")
        deadline = time.monotonic() + 7.0
        qualified_since = None
        while time.monotonic() < deadline:
            runner.poll(0.02)
            row = runner.records[-1]
            reference_error = abs(
                (row["target_position"] - row["position_ref"]) * DEG_PER_RAD
            )
            if reference_error <= 0.2:
                qualified_since = qualified_since or time.monotonic()
                if time.monotonic() - qualified_since >= 0.10:
                    break
            else:
                qualified_since = None
        else:
            raise RuntimeError(f"position reference did not reach {target:g} deg")

        runner.segment = f"{name}_{target:+g}_steady"
        steady_start = len(runner.records)
        runner.poll(0.80)
        rows = steady_rows(runner, steady_start)
        error_deg = [
            (row["position_ref"] - row["position_feedback"]) * DEG_PER_RAD
            for row in rows
        ]
        transition = runner.records[transition_start:]
        error_stats = metric(error_deg)
        results.append({
            "target_offset_deg": target,
            "error_deg": error_stats,
            "peak_abs_speed_rpm": max(
                abs(row["elec_speed"] * RPM_PER_ELEC_RAD_S)
                for row in transition
            ),
            "peak_dq_current_a": max(
                math.hypot(row["iq"], row["id"]) for row in transition
            ),
            "passed": abs(error_stats["mean"]) <= 0.75
                      and error_stats["rms"] <= 1.0,
        })

    runner.command("STOP", "STOP")
    runner.wait_status("READY", "NONE", 5.0)
    return results


def save_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ("time_s", "segment") + TELEMETRY_CHANNELS
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Unified ABZ speed-domain and position-domain validation"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--output", type=Path,
        default=Path("tests/results/stage6_control_domain_d080.json"),
    )
    parser.add_argument(
        "--csv", type=Path,
        default=Path("tests/results/stage6_control_domain_d080.csv"),
    )
    args = parser.parse_args()

    runner = DomainRunner(args.port, args.baudrate)
    result = {"status": "running"}
    try:
        runner.poll(0.5)
        runner.wait_status("READY", "NONE")
        speed_targets = [
            50.0, 100.0, 250.0, 500.0, 1500.0, 3000.0, 5000.0, 6500.0,
            0.0,
            -50.0, -100.0, -250.0, -500.0, -1500.0, -3000.0, -5000.0,
            -6500.0, 0.0,
        ]
        result["abz_speed"] = run_speed_matrix(runner, speed_targets)
        result["position_ramp"] = run_position_matrix(
            runner, "position_ramp",
            ["POS TRACK RAMP 180", "POS FF OFF"],
            [1.0, -1.0, 2.2, -2.2, 5.0, -5.0, 30.0, -30.0,
             90.0, -90.0, 360.0, -360.0, 0.0],
        )
        result["position_trap"] = run_position_matrix(
            runner, "position_trap",
            ["POS TRACK TRAP 180 900", "POS FF ON"],
            [90.0, -90.0, 360.0, -360.0, 0.0],
        )
        runner.raw_command("POS TRACK RAMP 180")
        runner.raw_command("POS FF OFF")
        runner.raw_command("TRACK STEP")
        runner.command("FF MODEL IDENTIFIED", "FF_MODEL")
        runner.command("FF MODE BOTH", "FF_MODE")
        runner.command("FW MODE HYBRID", "FW_MODE")
        runner.command("SPEED LIMIT 4", "SPEED_LIMIT")
        runner.command("VOLT LIMIT 1", "VOLT_LIMIT")
        runner.device.request_status()
        runner.poll(0.2)
        result["final"] = {
            "lifecycle": runner.status.lifecycle if runner.status else "UNKNOWN",
            "active_faults": runner.status.fault_active if runner.status else -1,
            "latched_faults": runner.status.fault_latched if runner.status else -1,
        }
        checks = [
            item["passed"]
            for group in ("abz_speed", "position_ramp", "position_trap")
            for item in result[group]
        ]
        result["checks"] = {"passed": sum(checks), "total": len(checks)}
        result["status"] = "passed" if all(checks) else "completed_with_failures"
    except Exception as error:
        result["status"] = "failed"
        result["error"] = str(error)
        raise
    finally:
        if runner.device.connected:
            try:
                runner.device.command("STOP")
                runner.poll(0.2)
            finally:
                runner.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        save_csv(args.csv, runner.records)

    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
