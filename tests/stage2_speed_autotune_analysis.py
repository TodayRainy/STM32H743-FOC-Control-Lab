#!/usr/bin/env python3
"""Recompute speed-loop screen metrics from saved compatible scope CSVs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import scope_capture as scope
from stage2_speed_autotune import DEFAULT_SPEC, load_spec
from stage2_speed_autotune_validation import (
    summarize_candidate,
    summarize_speed_step,
)


def read_scope(path: Path) -> tuple[list[tuple[float, ...]], float]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f"scope CSV is empty: {path}")
    names = [name for name in scope.CHANNEL_NAMES if name in rows[0]]
    samples = [tuple(float(row[name]) for name in names) for row in rows]
    sample_period = float(rows[1]["time_s"]) - float(rows[0]["time_s"])
    return samples, sample_period


def reanalyze(result_path: Path, spec_path: Path) -> dict:
    source = json.loads(result_path.read_text(encoding="utf-8"))
    spec = load_spec(spec_path)
    zero_noise = float(source["speed_noise"]["noise_rms_rpm"])
    pole_pairs = int(spec["control"]["pole_pairs"])
    iq_limit = float(spec["safety"]["speed_iq_reference_limit_a"])
    records = []
    for old in source["pi_records"]:
        samples, sample_period = read_scope(Path(old["scope_csv"]))
        record = summarize_speed_step(
            samples, sample_period, pole_pairs, zero_noise, iq_limit
        )
        for name in ("profile", "round", "kp", "ki", "scope_csv"):
            if name in old:
                record[name] = old[name]
        records.append(record)

    summaries = {}
    for profile in sorted({item["profile"] for item in records}):
        selected = [item for item in records if item["profile"] == profile]
        summaries[profile] = summarize_candidate(selected, spec)
    passing = [name for name, summary in summaries.items() if summary["passed"]]
    return {
        "schema": "stage2-speed-autotune-screen-reanalysis-v1",
        "source_result": str(result_path.resolve()),
        "spec": str(spec_path),
        "pi_records": records,
        "pi_summaries": summaries,
        "passing_profiles": passing,
        "status": "screen_qualified" if passing else "screen_failed",
        "hardware_rerun": False,
        "source_final_mode": source.get("final_mode"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = reanalyze(args.result, args.spec)
    output = args.output or args.result.with_name(
        f"{args.result.stem}_reanalyzed.json"
    )
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output.resolve()),
        "status": report["status"],
        "passing_profiles": report["passing_profiles"],
        "hardware_rerun": False,
    }, indent=2))


if __name__ == "__main__":
    main()
