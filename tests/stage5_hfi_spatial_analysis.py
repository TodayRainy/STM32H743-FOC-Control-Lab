#!/usr/bin/env python3
"""Fit electrical-angle harmonics in HFI shadow error."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def analyze(path: Path) -> dict:
    source = json.loads(path.read_text(encoding="utf-8"))
    telemetry = Path(source["telemetry_csv"])
    with telemetry.open(newline="", encoding="utf-8") as stream:
        dynamic = [row for row in csv.DictReader(stream)
                   if row["phase"].startswith("speed_")
                   and abs(float(row["target_rpm"])) > 0.0]
    segments: list[list[dict[str, str]]] = []
    for row in dynamic:
        if not segments or segments[-1][-1]["phase"] != row["phase"]:
            segments.append([])
        segments[-1].append(row)
    rows = [row for segment in segments
            for row in segment[len(segment)//2:]]
    if not rows:
        raise RuntimeError("result contains no dynamic HFI samples")

    angle = np.asarray([float(row["abz_angle"]) for row in rows])
    error = np.radians([float(row["hfi_angle_error_deg"]) for row in rows])
    target = np.asarray([float(row["target_rpm"]) for row in rows])

    group_indices: dict[float, list[int]] = defaultdict(list)
    for index, value in enumerate(target):
        group_indices[value].append(index)
    group_offsets = {}
    detrended = error.copy()
    for value, indices in group_indices.items():
        offset = math.atan2(float(np.sin(error[indices]).mean()),
                            float(np.cos(error[indices]).mean()))
        detrended[indices] = np.arctan2(
            np.sin(error[indices]-offset),
            np.cos(error[indices]-offset),
        )
        group_offsets[str(value)] = math.degrees(offset)

    columns = []
    labels = []
    for harmonic in range(1, 13):
        columns.extend((np.sin(harmonic*angle),
                        np.cos(harmonic*angle)))
        labels.append(harmonic)
    design = np.column_stack(columns)
    coefficients, _, _, _ = np.linalg.lstsq(design, detrended, rcond=None)
    predicted = design@coefficients
    residual = detrended-predicted
    amplitudes = []
    for index, harmonic in enumerate(labels):
        sine = coefficients[2*index]
        cosine = coefficients[2*index+1]
        amplitudes.append({
            "harmonic": harmonic,
            "amplitude_deg": math.degrees(math.hypot(sine, cosine)),
            "phase_deg": math.degrees(math.atan2(cosine, sine)),
        })
    amplitudes.sort(key=lambda item: item["amplitude_deg"], reverse=True)

    centered_energy = float(np.dot(detrended, detrended))
    residual_energy = float(np.dot(residual, residual))
    return {
        "schema": "stage5-hfi-spatial-analysis-v1",
        "source": str(path),
        "telemetry_csv": str(telemetry),
        "samples": len(rows),
        "speed_group_mean_error_deg": group_offsets,
        "harmonics_by_amplitude": amplitudes,
        "detrended_rms_deg": math.degrees(float(np.sqrt(
            np.mean(detrended*detrended)
        ))),
        "residual_rms_deg": math.degrees(float(np.sqrt(
            np.mean(residual*residual)
        ))),
        "fit_r2": 1.0-residual_energy/centered_energy,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_json", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.result_json)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({
        "samples": result["samples"],
        "speed_group_mean_error_deg": result["speed_group_mean_error_deg"],
        "dominant_harmonics": result["harmonics_by_amplitude"][:6],
        "detrended_rms_deg": result["detrended_rms_deg"],
        "residual_rms_deg": result["residual_rms_deg"],
        "fit_r2": result["fit_r2"],
    }, indent=2))


if __name__ == "__main__":
    main()
