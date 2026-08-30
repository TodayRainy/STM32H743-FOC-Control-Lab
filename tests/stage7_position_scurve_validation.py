from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from stage6_control_domain_validation import (  # noqa: E402
    DEG_PER_RAD, DomainRunner, RPM_PER_ELEC_RAD_S, save_csv, steady_rows,
)


AMPLITUDES_DEG = (1.0, 2.2, 5.0, 30.0, 90.0, 360.0)
PROFILE_COMMANDS = {
    "TRAP": "POS TRACK TRAP 180 900",
    "SCURVE": "POS TRACK SCURVE 180 900 22500",
}


def target_sequence(first_direction: float) -> list[float]:
    targets = []
    for amplitude in AMPLITUDES_DEG:
        targets.extend((first_direction * amplitude,
                        -first_direction * amplitude))
    targets.append(0.0)
    return targets


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values))


def settling_time(rows: list[dict], target: float,
                  tolerance_deg: float = 0.3,
                  speed_tolerance_rpm: float = 5.0) -> float | None:
    qualified_since = None
    start = rows[0]["time_s"]
    for row in rows:
        error = abs((target-row["position_feedback"]) * DEG_PER_RAD)
        speed = abs(row["elec_speed"] * RPM_PER_ELEC_RAD_S)
        if error <= tolerance_deg and speed <= speed_tolerance_rpm:
            qualified_since = qualified_since or row["time_s"]
            if row["time_s"]-qualified_since >= 0.10:
                return qualified_since-start
        else:
            qualified_since = None
    return None


def summarize_move(rows: list[dict], steady: list[dict], target_deg: float,
                   reference_duration: float, initial_position: float) -> dict:
    target = rows[-1]["target_position"]
    direction = 1.0 if target >= initial_position else -1.0
    tracking_error = [
        (row["position_ref"]-row["position_feedback"]) * DEG_PER_RAD
        for row in rows
    ]
    endpoint_error = [
        (target-row["position_feedback"]) * DEG_PER_RAD
        for row in steady
    ]
    overshoot = max(
        0.0,
        max(direction*(row["position_feedback"]-target)*DEG_PER_RAD
            for row in rows),
    )
    iq_ref = [row["iq_ref"] for row in rows]
    dq_current = [math.hypot(row["iq"], row["id"]) for row in rows]
    speeds = [row["elec_speed"] * RPM_PER_ELEC_RAD_S for row in rows]
    commanded_speeds = [
        row["speed_ref"] * RPM_PER_ELEC_RAD_S for row in rows
    ]
    return {
        "target_deg": target_deg,
        "commanded_move_deg": (target-initial_position) * DEG_PER_RAD,
        "reference_duration_s": reference_duration,
        "settling_time_s": settling_time(rows, target),
        "position_tracking_error_rms_deg": rms(tracking_error),
        "position_tracking_error_peak_deg": max(
            abs(value) for value in tracking_error
        ),
        "steady_error_mean_deg": statistics.fmean(endpoint_error),
        "steady_error_rms_deg": rms(endpoint_error),
        "overshoot_deg": overshoot,
        "peak_abs_speed_rpm": max(abs(value) for value in speeds),
        "peak_abs_speed_reference_rpm": max(
            abs(value) for value in commanded_speeds
        ),
        "iq_ref_rms_a": rms(iq_ref),
        "peak_dq_current_a": max(dq_current),
        "minimum_voltage_scale": min(row["voltage_scale"] for row in rows),
    }


def run_visit(runner: DomainRunner, profile: str, direction: float,
              visit: int) -> list[dict]:
    runner.raw_command(PROFILE_COMMANDS[profile])
    runner.raw_command("POS FF ON")
    runner.command("START POSITION", "START_POSITION")
    runner.wait_status("RUNNING", "POSITION_SPEED")
    runner.poll(0.20)

    results = []
    for target_deg in target_sequence(direction):
        runner.segment = f"visit{visit}_{profile.lower()}_{target_deg:+g}_move"
        initial_position = runner.records[-1]["position_feedback"]
        start = len(runner.records)
        start_time = time.monotonic()
        runner.command(f"POSITION {target_deg:g}", "POSITION")
        qualified_since = None
        reference_duration = None
        deadline = time.monotonic()+8.0
        while time.monotonic() < deadline:
            runner.poll(0.02)
            row = runner.records[-1]
            reference_error = abs(
                (row["target_position"]-row["position_ref"])*DEG_PER_RAD
            )
            if reference_error <= 0.2:
                qualified_since = qualified_since or time.monotonic()
                if time.monotonic()-qualified_since >= 0.05:
                    reference_duration = qualified_since-start_time
                    break
            else:
                qualified_since = None
        if reference_duration is None:
            raise RuntimeError(
                f"{profile} reference did not reach {target_deg:g} deg"
            )

        runner.segment = (
            f"visit{visit}_{profile.lower()}_{target_deg:+g}_settle"
        )
        runner.poll(0.80)
        rows = runner.records[start:]
        steady = steady_rows(runner, max(start, len(runner.records)-250))
        summary = summarize_move(
            rows, steady, target_deg, reference_duration, initial_position
        )
        summary["visit"] = visit
        summary["profile"] = profile
        results.append(summary)

    runner.command("STOP", "STOP")
    runner.wait_status("READY", "NONE", 5.0)
    return results


def aggregate(moves: list[dict], profile: str) -> dict:
    selected = [move for move in moves if move["profile"] == profile]
    settling = [
        move["settling_time_s"] for move in selected
        if move["settling_time_s"] is not None
    ]
    return {
        "move_count": len(selected),
        "reference_duration_mean_s": statistics.fmean(
            move["reference_duration_s"] for move in selected
        ),
        "settled_count": len(settling),
        "settling_time_mean_s": statistics.fmean(settling) if settling else None,
        "tracking_error_rms_mean_deg": statistics.fmean(
            move["position_tracking_error_rms_deg"] for move in selected
        ),
        "maximum_overshoot_deg": max(move["overshoot_deg"] for move in selected),
        "iq_ref_rms_mean_a": statistics.fmean(
            move["iq_ref_rms_a"] for move in selected
        ),
        "maximum_dq_current_a": max(
            move["peak_dq_current_a"] for move in selected
        ),
        "maximum_steady_error_rms_deg": max(
            move["steady_error_rms_deg"] for move in selected
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="TRAP/SCURVE position trajectory closed-loop A/B"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--output", type=Path,
        default=Path("tests/results/stage7_position_scurve_d083.json"),
    )
    parser.add_argument(
        "--csv", type=Path,
        default=Path("tests/results/stage7_position_scurve_d083.csv"),
    )
    args = parser.parse_args()

    runner = DomainRunner(args.port, args.baudrate)
    result: dict = {"status": "running", "moves": []}
    visits = (
        ("TRAP", 1.0), ("SCURVE", -1.0),
        ("SCURVE", 1.0), ("TRAP", -1.0),
    )
    try:
        runner.poll(0.5)
        runner.wait_status("READY", "NONE")
        runner.command("SPEED EST WIN", "SPEED_EST")
        runner.command("FF MODEL IDENTIFIED", "FF_MODEL")
        runner.command("FF MODE BOTH", "FF_MODE")
        runner.command("FW MODE HYBRID", "FW_MODE")
        runner.command("SPEED LIMIT 4", "SPEED_LIMIT")
        runner.command("VOLT LIMIT 1", "VOLT_LIMIT")
        runner.raw_command("TRACK STEP")

        for visit, (profile, direction) in enumerate(visits, start=1):
            result["moves"].extend(
                run_visit(runner, profile, direction, visit)
            )
        result["profiles"] = {
            profile: aggregate(result["moves"], profile)
            for profile in ("TRAP", "SCURVE")
        }

        runner.raw_command("POS TRACK RAMP 180")
        runner.raw_command("POS FF OFF")
        runner.raw_command("TRACK STEP")
        runner.command("SPEED EST WIN", "SPEED_EST")
        final = runner.wait_status("READY", "NONE", 5.0)
        result["final"] = {
            "lifecycle": final.lifecycle,
            "operation": final.operation,
            "fault_active": final.fault_active,
            "fault_latched": final.fault_latched,
        }
        steady_ok = all(
            move["steady_error_rms_deg"] <= 1.0 for move in result["moves"]
        )
        result["checks"] = {
            "steady_error_passed": sum(
                move["steady_error_rms_deg"] <= 1.0
                for move in result["moves"]
            ),
            "total": len(result["moves"]),
        }
        result["status"] = (
            "passed" if steady_ok and final.fault_active == 0
            and final.fault_latched == 0 else "completed_with_failures"
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
