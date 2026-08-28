#!/usr/bin/env python3
"""Fit the 40 kHz current-increment response to delayed HFI voltage samples."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_scope(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in ("time_s", "iq", "id", "uq", "ud")
    }


def fit_lag(data: dict[str, np.ndarray], lag: int,
            parity: int | None) -> dict:
    current = np.column_stack((data["iq"], data["id"]))
    voltage = np.column_stack((data["uq"], data["ud"]))
    delta = current[1:]-current[:-1]
    response_indices = np.arange(max(1, lag), len(current))
    if parity is not None:
        response_indices = response_indices[response_indices % 2 == parity]
    y = delta[response_indices-1]
    x_voltage = voltage[response_indices-lag]
    x = np.column_stack((x_voltage, np.ones(len(x_voltage))))
    coefficients, _, _, _ = np.linalg.lstsq(x, y, rcond=None)
    predicted = x@coefficients
    residual = y-predicted
    centered = y-y.mean(axis=0)
    r2_axis = 1.0-(residual*residual).sum(axis=0) \
        /(centered*centered).sum(axis=0)
    return {
        "lag_samples": lag,
        "parity": parity,
        "samples": len(y),
        "matrix_a_per_v_per_sample": coefficients[:2].T.tolist(),
        "r2_axis": r2_axis.tolist(),
        "r2_mean": float(r2_axis.mean()),
        "residual_rms_a": np.sqrt((residual*residual).mean(axis=0)).tolist(),
    }


def analyze(path: Path) -> dict:
    data = load_scope(path)
    fits = [fit_lag(data, lag, parity)
            for parity in (None, 0, 1)
            for lag in range(0, 9)]
    best = max(fits, key=lambda item: item["r2_mean"])
    return {
        "schema": "stage5-hfi-carrier-analysis-v1",
        "source": str(path),
        "sample_period_s": float(np.median(np.diff(data["time_s"]))),
        "fits": fits,
        "best_fit": best,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scope_csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.scope_csv)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["best_fit"], indent=2))


if __name__ == "__main__":
    main()
