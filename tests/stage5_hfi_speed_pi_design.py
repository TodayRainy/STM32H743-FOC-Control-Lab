#!/usr/bin/env python3
"""Recover the HFI-feedback speed plant and place a conservative PI crossover."""

from __future__ import annotations

import argparse
import cmath
import json
import math
from pathlib import Path


ENCODER_SPEED_KP = 0.00536533785
ENCODER_SPEED_KI = 0.258523708


def read_complex(record: dict) -> complex:
    return complex(record["real"], record["imag"])


def db(value: complex | float) -> float:
    return 20.0*math.log10(max(abs(value), 1.0e-12))


def phase_deg(value: complex) -> float:
    return math.degrees(cmath.phase(value))


def recover_point(point: dict) -> dict:
    frequency_hz = point["frequency_hz"]
    closed_loop = read_complex(point["reference_to_abz"])
    observer = read_complex(point["abz_to_hfi"])
    encoder_pi = complex(
        ENCODER_SPEED_KP,
        -ENCODER_SPEED_KI/(2.0*math.pi*frequency_hz),
    )
    mechanical_plant = closed_loop/(1.0-closed_loop)/encoder_pi
    hfi_feedback_plant = mechanical_plant*observer
    return {
        "bias_rpm": point["bias_rpm"],
        "frequency_hz": frequency_hz,
        "reference_amplitude_rpm": point["reference_amplitude_rpm"],
        "abz_amplitude_rpm": point["abz_amplitude_rpm"],
        "hfi_feedback_plant": hfi_feedback_plant,
        "hfi_feedback_plant_magnitude_db": db(hfi_feedback_plant),
        "hfi_feedback_plant_phase_deg": phase_deg(hfi_feedback_plant),
    }


def interpolate_at(points: list[dict], frequency_hz: float) -> tuple[float, float]:
    ordered = sorted(points, key=lambda point: point["frequency_hz"])
    lower = max(
        (point for point in ordered if point["frequency_hz"] <= frequency_hz),
        key=lambda point: point["frequency_hz"],
    )
    upper = min(
        (point for point in ordered if point["frequency_hz"] >= frequency_hz),
        key=lambda point: point["frequency_hz"],
    )
    if lower is upper:
        return (abs(lower["hfi_feedback_plant"]),
                phase_deg(lower["hfi_feedback_plant"]))
    fraction = math.log(frequency_hz/lower["frequency_hz"]) \
        / math.log(upper["frequency_hz"]/lower["frequency_hz"])
    magnitude_db = lower["hfi_feedback_plant_magnitude_db"] + fraction*(
        upper["hfi_feedback_plant_magnitude_db"]
        - lower["hfi_feedback_plant_magnitude_db"]
    )
    phase = lower["hfi_feedback_plant_phase_deg"] + fraction*(
        upper["hfi_feedback_plant_phase_deg"]
        - lower["hfi_feedback_plant_phase_deg"]
    )
    return 10.0**(magnitude_db/20.0), phase


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--crossover-hz", type=float, default=5.0)
    parser.add_argument("--zero-hz", type=float, default=1.2)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    recovered = []
    for path in args.inputs:
        result = json.loads(path.read_text(encoding="utf-8"))
        recovered.extend(recover_point(point) for point in result["points"])

    # Repeated short captures are intentional.  Keep the point with the
    # strongest measured ABZ excitation for each direction and frequency.
    best_points = {}
    for point in recovered:
        key = (point["bias_rpm"], point["frequency_hz"])
        if (key not in best_points
                or point["abz_amplitude_rpm"]
                > best_points[key]["abz_amplitude_rpm"]):
            best_points[key] = point
    recovered = list(best_points.values())

    directions = sorted(set(point["bias_rpm"] for point in recovered))
    interpolated = []
    for direction in directions:
        points = [point for point in recovered if point["bias_rpm"] == direction]
        magnitude, phase = interpolate_at(points, args.crossover_hz)
        interpolated.append({
            "bias_rpm": direction,
            "plant_magnitude": magnitude,
            "plant_magnitude_db": db(magnitude),
            "plant_phase_deg": phase,
        })

    geometric_mean_plant = math.exp(sum(
        math.log(point["plant_magnitude"]) for point in interpolated
    )/len(interpolated))
    kp = 1.0/(geometric_mean_plant*math.hypot(
        1.0, args.zero_hz/args.crossover_hz
    ))
    ki = 2.0*math.pi*args.zero_hz*kp
    controller_phase = -math.degrees(math.atan2(
        args.zero_hz, args.crossover_hz
    ))
    predicted_phase_margins = [
        180.0+point["plant_phase_deg"]+controller_phase
        for point in interpolated
    ]

    serializable_points = []
    for point in recovered:
        copy = dict(point)
        plant = copy.pop("hfi_feedback_plant")
        controller = complex(
            kp, -ki/(2.0*math.pi*point["frequency_hz"])
        )
        copy["candidate_open_loop_magnitude_db"] = db(plant*controller)
        copy["candidate_open_loop_phase_deg"] = phase_deg(plant*controller)
        serializable_points.append(copy)

    output = {
        "schema": "stage5-hfi-speed-pi-design-v1",
        "inputs": [str(path.resolve()) for path in args.inputs],
        "encoder_speed_pi": {
            "kp": ENCODER_SPEED_KP,
            "ki": ENCODER_SPEED_KI,
        },
        "target_crossover_hz": args.crossover_hz,
        "pi_zero_hz": args.zero_hz,
        "interpolated_plants": interpolated,
        "candidate": {"kp": kp, "ki": ki},
        "predicted_phase_margin_deg": {
            "minimum": min(predicted_phase_margins),
            "maximum": max(predicted_phase_margins),
        },
        "points": serializable_points,
    }
    text = json.dumps(output, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
