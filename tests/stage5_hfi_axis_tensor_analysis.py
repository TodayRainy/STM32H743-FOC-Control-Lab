#!/usr/bin/env python3
"""Fit the measured 2x2 HFI current-response tensor from a fixed-axis sweep."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def point_vectors(point: dict) -> tuple[np.ndarray, np.ndarray]:
    path = Path(point["scope_csv"])
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    values = np.asarray([
        [float(row[name]) for name in ("iq", "id", "uq", "ud")]
        for row in rows
    ])
    current = values[:, :2]
    voltage = values[:, 2:]
    offset = math.radians(float(point["offset_deg"]))
    carrier_scalar = voltage[:, 0]*math.sin(offset) \
                     +voltage[:, 1]*math.cos(offset)
    sign = np.where(carrier_scalar >= 0.0, 1.0, -1.0)

    positive = voltage[sign > 0.0].mean(axis=0)
    negative = voltage[sign < 0.0].mean(axis=0)
    carrier_voltage = 0.5*(positive-negative)

    complete = sign[1:] == sign[:-1]
    response = (-sign[1:, None]*(current[1:]-current[:-1]))[complete]
    return carrier_voltage, response.mean(axis=0)


def analyze(path: Path) -> dict:
    source = json.loads(path.read_text(encoding="utf-8"))
    points = []
    voltage_vectors = []
    response_vectors = []
    for point in source["points"]:
        voltage, response = point_vectors(point)
        voltage_vectors.append(voltage)
        response_vectors.append(response)
        points.append({
            "offset_deg": point["offset_deg"],
            "carrier_voltage_qd_v": voltage.tolist(),
            "synchronous_response_qd_a": response.tolist(),
        })

    voltage = np.asarray(voltage_vectors)
    response = np.asarray(response_vectors)
    design = np.column_stack((voltage, np.ones(len(voltage))))
    coefficients, _, _, _ = np.linalg.lstsq(design, response, rcond=None)
    tensor = coefficients[:2].T
    bias = coefficients[2]
    predicted = design@coefficients
    residual = response-predicted

    symmetric = 0.5*(tensor+tensor.T)
    eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
    d_index = int(np.argmax(eigenvalues))
    d_vector = eigenvectors[:, d_index]
    if d_vector[1] < 0.0:
        d_vector = -d_vector
    axis_offset = math.atan2(d_vector[0], d_vector[1])
    sample_period = 1.0/40000.0
    effective_inductance = [sample_period/value for value in eigenvalues]
    return {
        "schema": "stage5-hfi-axis-tensor-analysis-v1",
        "source": str(path),
        "points": points,
        "response_tensor_a_per_v_per_sample": tensor.tolist(),
        "symmetric_tensor_a_per_v_per_sample": symmetric.tolist(),
        "response_bias_qd_a": bias.tolist(),
        "eigenvalues_a_per_v_per_sample": eigenvalues.tolist(),
        "effective_inductance_h": effective_inductance,
        "saliency_axis_offset_deg": math.degrees(axis_offset),
        "saliency_ratio": float(eigenvalues.max()/eigenvalues.min()),
        "residual_rms_qd_a": np.sqrt((residual*residual).mean(axis=0)).tolist(),
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
        "source": result["source"],
        "tensor": result["response_tensor_a_per_v_per_sample"],
        "axis_offset_deg": result["saliency_axis_offset_deg"],
        "effective_inductance_uH": [
            value*1.0e6 for value in result["effective_inductance_h"]
        ],
        "saliency_ratio": result["saliency_ratio"],
        "residual_rms_qd_a": result["residual_rms_qd_a"],
    }, indent=2))


if __name__ == "__main__":
    main()
