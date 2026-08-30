from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from stage6_control_domain_validation import (  # noqa: E402
    DomainRunner, RPM_PER_ELEC_RAD_S, save_csv, steady_rows,
)


TARGETS = {
    "WIN": [25.0, 50.0, 100.0, 250.0, 500.0, 0.0,
            -25.0, -50.0, -100.0, -250.0, -500.0, 0.0],
    "FUSED": [-25.0, -50.0, -100.0, -250.0, -500.0, 0.0,
              25.0, 50.0, 100.0, 250.0, 500.0, 0.0],
}


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values))


def harmonic_amplitude(rows: list[dict], values: list[float],
                       frequency_hz: float) -> float:
    if frequency_hz <= 0.0:
        return 0.0
    mean = statistics.fmean(values)
    origin = rows[0]["time_s"]
    cosine = 0.0
    sine = 0.0
    for row, value in zip(rows, values):
        phase = 2.0 * math.pi * frequency_hz * (row["time_s"] - origin)
        centered = value - mean
        cosine += centered * math.cos(phase)
        sine += centered * math.sin(phase)
    scale = 2.0 / len(values)
    return scale * math.hypot(cosine, sine)


def summarize_hold(rows: list[dict], target_rpm: float) -> dict:
    speed = [row["elec_speed"] * RPM_PER_ELEC_RAD_S for row in rows]
    reference = [row["speed_ref"] * RPM_PER_ELEC_RAD_S for row in rows]
    error = [actual - ref for actual, ref in zip(speed, reference)]
    iq_ref = [row["iq_ref"] for row in rows]
    iq = [row["iq"] for row in rows]
    fundamental = abs(target_rpm) / 60.0
    return {
        "target_rpm": target_rpm,
        "sample_count": len(rows),
        "speed_mean_rpm": statistics.fmean(speed),
        "speed_error_mean_rpm": statistics.fmean(error),
        "speed_error_rms_rpm": rms(error),
        "speed_error_peak_rpm": max(abs(value) for value in error),
        "speed_h18_amplitude_rpm": harmonic_amplitude(
            rows, speed, 18.0 * fundamental
        ),
        "speed_h20_amplitude_rpm": harmonic_amplitude(
            rows, speed, 20.0 * fundamental
        ),
        "iq_ref_rms_a": rms(iq_ref),
        "iq_rms_a": rms(iq),
        "voltage_scale_min": min(row["voltage_scale"] for row in rows),
    }


def run_mode(runner: DomainRunner, mode: str) -> list[dict]:
    runner.command(f"SPEED EST {mode}", "SPEED_EST")
    runner.command("START SPEED", "START_SPEED")
    runner.wait_status("RUNNING", "SPEED")
    results = []
    for target in TARGETS[mode]:
        runner.segment = f"{mode.lower()}_{target:+g}_transition"
        runner.command(f"SPEED {target:g}", "SPEED")
        runner.wait_reference(
            "speed_ref", target, RPM_PER_ELEC_RAD_S,
            tolerance=max(1.0, abs(target) * 0.01), timeout=6.0,
        )
        runner.poll(0.30)
        runner.segment = f"{mode.lower()}_{target:+g}_steady"
        start = len(runner.records)
        runner.poll(1.20)
        results.append(summarize_hold(steady_rows(runner, start), target))
    runner.command("STOP", "STOP")
    runner.wait_status("READY", "NONE", 5.0)
    return results


def comparisons(groups: dict[str, list[dict]]) -> list[dict]:
    output = []
    for target in sorted({row["target_rpm"] for row in groups["WIN"]}):
        win_rows = [row for row in groups["WIN"] if row["target_rpm"] == target]
        fused_rows = [
            row for row in groups["FUSED"] if row["target_rpm"] == target
        ]
        if len(win_rows) != 1 or len(fused_rows) != 1:
            continue
        win = win_rows[0]
        fused = fused_rows[0]
        output.append({
            "target_rpm": target,
            "speed_error_rms_ratio": (
                fused["speed_error_rms_rpm"] / win["speed_error_rms_rpm"]
                if win["speed_error_rms_rpm"] > 0.0 else 0.0
            ),
            "iq_ref_rms_ratio": (
                fused["iq_ref_rms_a"] / win["iq_ref_rms_a"]
                if win["iq_ref_rms_a"] > 0.0 else 0.0
            ),
            "h18_amplitude_ratio": (
                fused["speed_h18_amplitude_rpm"]
                / win["speed_h18_amplitude_rpm"]
                if win["speed_h18_amplitude_rpm"] > 0.0 else 0.0
            ),
            "h20_amplitude_ratio": (
                fused["speed_h20_amplitude_rpm"]
                / win["speed_h20_amplitude_rpm"]
                if win["speed_h20_amplitude_rpm"] > 0.0 else 0.0
            ),
        })
    return output


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ABZ WIN/FUSED low-speed closed-loop A/B"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--output", type=Path,
        default=Path("tests/results/stage7_abz_speed_fusion_d082.json"),
    )
    parser.add_argument(
        "--csv", type=Path,
        default=Path("tests/results/stage7_abz_speed_fusion_d082.csv"),
    )
    args = parser.parse_args()

    runner = DomainRunner(args.port, args.baudrate)
    result: dict = {"status": "running", "profiles": {}}
    try:
        runner.poll(0.5)
        runner.wait_status("READY", "NONE")
        runner.command("FF MODEL IDENTIFIED", "FF_MODEL")
        runner.command("FF MODE BOTH", "FF_MODE")
        runner.command("FW MODE HYBRID", "FW_MODE")
        runner.command("SPEED LIMIT 4", "SPEED_LIMIT")
        runner.command("VOLT LIMIT 1", "VOLT_LIMIT")
        runner.raw_command("TRACK RAMP 3000")

        for mode in ("WIN", "FUSED"):
            result["profiles"][mode] = run_mode(runner, mode)
        result["comparisons"] = comparisons(result["profiles"])

        runner.command("SPEED EST WIN", "SPEED_EST")
        runner.raw_command("TRACK STEP")
        runner.command("FF MODEL IDENTIFIED", "FF_MODEL")
        runner.command("FF MODE BOTH", "FF_MODE")
        runner.command("FW MODE HYBRID", "FW_MODE")
        runner.command("SPEED LIMIT 4", "SPEED_LIMIT")
        runner.command("VOLT LIMIT 1", "VOLT_LIMIT")
        final = runner.wait_status("READY", "NONE", 5.0)
        result["final"] = {
            "lifecycle": final.lifecycle,
            "operation": final.operation,
            "fault_active": final.fault_active,
            "fault_latched": final.fault_latched,
        }
        result["status"] = (
            "passed" if final.fault_active == 0 and final.fault_latched == 0
            else "completed_with_faults"
        )
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
