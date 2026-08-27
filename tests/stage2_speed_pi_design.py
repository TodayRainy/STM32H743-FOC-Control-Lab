#!/usr/bin/env python3
"""Recover the measured speed plant and design six PI crossover candidates."""

from __future__ import annotations

import argparse
import cmath
import json
import math
from pathlib import Path


DEFAULT_SOURCE = Path(
    "tests/results/stage2_speed_baseline_20260825_162540.json"
)
DEFAULT_OUTPUT = Path(
    "tests/baselines/stage2_speed_pi_design_d039_20260825.json"
)
CURRENT_KP = 0.015764175 / 10.0
CURRENT_KI = 0.246037498 / 10.0
TARGET_BANDWIDTHS_HZ = (15.0, 20.0, 25.0, 30.0, 35.0, 40.0)
TARGET_PHASE_MARGIN_DEG = 65.0
REFERENCE_AMPLITUDE_RPM = 100.0
POLE_PAIRS = 10


def from_db_phase(magnitude_db: float, phase_deg: float) -> complex:
    return 10.0 ** (magnitude_db / 20.0) * cmath.exp(
        1j * math.radians(phase_deg)
    )


def recover_plant(closed_loop: complex, frequency_hz: float,
                  kp: float = CURRENT_KP, ki: float = CURRENT_KI) -> complex:
    omega = 2.0 * math.pi * frequency_hz
    controller = kp + ki / (1j * omega)
    open_loop = closed_loop / (1.0 - closed_loop)
    return open_loop / controller


def interpolate_plant(points: dict[float, complex], frequency_hz: float) -> complex:
    if frequency_hz in points:
        return points[frequency_hz]
    frequencies = sorted(points)
    lower = max(value for value in frequencies if value < frequency_hz)
    upper = min(value for value in frequencies if value > frequency_hz)
    fraction = math.log(frequency_hz / lower) / math.log(upper / lower)
    log_magnitude = (
        (1.0 - fraction) * math.log(abs(points[lower]))
        + fraction * math.log(abs(points[upper]))
    )
    phase = (
        (1.0 - fraction) * cmath.phase(points[lower])
        + fraction * cmath.phase(points[upper])
    )
    return math.exp(log_magnitude) * cmath.exp(1j * phase)


def mean_plant(plants: list[complex]) -> complex:
    magnitude = math.exp(
        sum(math.log(abs(value)) for value in plants) / len(plants)
    )
    phase = sum(cmath.phase(value) for value in plants) / len(plants)
    return magnitude * cmath.exp(1j * phase)


def design_pi(plant: complex, crossover_hz: float,
              phase_margin_deg: float) -> dict:
    omega = 2.0 * math.pi * crossover_hz
    desired_loop_phase = math.radians(-180.0 + phase_margin_deg)
    controller_phase = desired_loop_phase - cmath.phase(plant)
    if controller_phase > 1.0e-9 or controller_phase < -math.pi / 2.0:
        raise ValueError("requested crossover/phase margin is not reachable by PI")
    ratio = math.tan(-controller_phase)
    kp = 1.0 / (abs(plant) * math.sqrt(1.0 + ratio * ratio))
    ki = ratio * kp * omega
    return {
        "kp": kp,
        "ki": ki,
        "integral_time_sec": kp / ki if ki > 0.0 else None,
        "controller_phase_deg_at_crossover": math.degrees(controller_phase),
    }


def controller(kp: float, ki: float, frequency_hz: float) -> complex:
    return kp + ki / (1j * 2.0 * math.pi * frequency_hz)


def closed_loop(plant: complex, kp: float, ki: float,
                frequency_hz: float) -> complex:
    loop = controller(kp, ki, frequency_hz) * plant
    return loop / (1.0 + loop)


def nominal_plant_at(direction_plants: list[dict], frequency_hz: float) -> complex:
    return mean_plant([
        interpolate_plant(item["points"], frequency_hz)
        for item in direction_plants
    ])


def predicted_bandwidth(direction_plants: list[dict], kp: float, ki: float) -> float:
    target_db = -3.0103
    previous = None
    for index in range(601):
        frequency = math.exp(
            math.log(1.0) + index / 600.0 * math.log(60.0)
        )
        response = closed_loop(
            nominal_plant_at(direction_plants, frequency), kp, ki, frequency,
        )
        magnitude_db = 20.0 * math.log10(abs(response))
        if (previous is not None and previous[1] > target_db
                and magnitude_db <= target_db):
            fraction = (target_db - previous[1]) / (magnitude_db - previous[1])
            return math.exp(
                math.log(previous[0])
                + fraction * math.log(frequency / previous[0])
            )
        previous = (frequency, magnitude_db)
    raise ValueError("predicted -3 dB crossing is outside the measured plant range")


def design_for_bandwidth(direction_plants: list[dict], bandwidth_hz: float,
                         phase_margin_deg: float) -> dict:
    lower = 2.0
    upper = min(40.0, bandwidth_hz)
    for _ in range(48):
        crossover = 0.5 * (lower + upper)
        plant = nominal_plant_at(direction_plants, crossover)
        gains = design_pi(plant, crossover, phase_margin_deg)
        bandwidth = predicted_bandwidth(
            direction_plants, gains["kp"], gains["ki"],
        )
        if bandwidth < bandwidth_hz:
            lower = crossover
        else:
            upper = crossover
    crossover = 0.5 * (lower + upper)
    plant = nominal_plant_at(direction_plants, crossover)
    gains = design_pi(plant, crossover, phase_margin_deg)
    return {
        **gains,
        "designed_gain_crossover_hz": crossover,
        "predicted_closed_loop_bandwidth_hz": predicted_bandwidth(
            direction_plants, gains["kp"], gains["ki"],
        ),
        "nominal_plant": plant,
    }


def build_report(source_path: Path) -> dict:
    source = json.loads(source_path.read_text(encoding="utf-8"))
    direction_plants = []
    for visit in source["visits"]:
        points = {}
        for record in visit["records"]:
            frequency = float(record["frequency_hz"])
            response = from_db_phase(
                float(record["closed_loop_magnitude_db"]),
                float(record["closed_loop_phase_deg"]),
            )
            points[frequency] = recover_plant(response, frequency)
        direction_plants.append({
            "bias_rpm": float(visit["bias_rpm"]),
            "points": points,
        })

    reference_amplitude = (
        REFERENCE_AMPLITUDE_RPM * 2.0 * math.pi / 60.0 * POLE_PAIRS
    )
    candidates = []
    for target_bandwidth_hz in TARGET_BANDWIDTHS_HZ:
        solution = design_for_bandwidth(
            direction_plants, target_bandwidth_hz, TARGET_PHASE_MARGIN_DEG,
        )
        crossover_hz = solution["designed_gain_crossover_hz"]
        plants = [
            interpolate_plant(item["points"], crossover_hz)
            for item in direction_plants
        ]
        nominal = solution.pop("nominal_plant")
        gains = solution
        direction_checks = []
        for item, plant in zip(direction_plants, plants):
            loop = controller(
                gains["kp"], gains["ki"], crossover_hz,
            ) * plant
            predicted = []
            for frequency, measured_plant in sorted(item["points"].items()):
                response = closed_loop(
                    measured_plant, gains["kp"], gains["ki"], frequency,
                )
                iq_per_reference = controller(
                    gains["kp"], gains["ki"], frequency,
                ) / (
                    1.0 + controller(
                        gains["kp"], gains["ki"], frequency,
                    ) * measured_plant
                )
                predicted.append({
                    "frequency_hz": frequency,
                    "closed_loop_magnitude_db": 20.0 * math.log10(abs(response)),
                    "closed_loop_phase_deg": math.degrees(cmath.phase(response)),
                    "iq_reference_amplitude_a": (
                        abs(iq_per_reference) * reference_amplitude
                    ),
                })
            direction_checks.append({
                "bias_rpm": item["bias_rpm"],
                "loop_gain_db_at_target": 20.0 * math.log10(abs(loop)),
                "phase_margin_deg_at_target": (
                    180.0 + math.degrees(cmath.phase(loop))
                ),
                "predicted_maximum_peak_db_on_measured_grid": max(
                    point["closed_loop_magnitude_db"] for point in predicted
                ),
                "predicted_peak_iq_reference_a_on_measured_grid": max(
                    point["iq_reference_amplitude_a"] for point in predicted
                ),
            })
        candidates.append({
            "name": f"bw{int(target_bandwidth_hz):02d}_pm65",
            "target_closed_loop_bandwidth_hz": target_bandwidth_hz,
            "target_phase_margin_deg": TARGET_PHASE_MARGIN_DEG,
            "nominal_plant_magnitude_db": 20.0 * math.log10(abs(nominal)),
            "nominal_plant_phase_deg": math.degrees(cmath.phase(nominal)),
            **gains,
            "direction_checks": direction_checks,
        })

    phase_feasibility = []
    for frequency in (15.0, 20.0, 30.0, 40.0, 50.0, 60.0):
        phases = [
            math.degrees(cmath.phase(interpolate_plant(item["points"], frequency)))
            for item in direction_plants
        ]
        phase_feasibility.append({
            "frequency_hz": frequency,
            "plant_phase_deg": phases,
            "maximum_pi_phase_margin_deg": min(180.0 + value for value in phases),
        })

    return {
        "schema": "stage2-speed-pi-measured-plant-design-v1",
        "source": str(source_path.resolve()),
        "method": "P=T/(1-T)/C0; numerically choose gain crossover so PI gives the target closed-loop -3 dB bandwidth with 65 deg phase margin",
        "current_pi": {"kp": CURRENT_KP, "ki": CURRENT_KI},
        "target_closed_loop_bandwidths_hz": TARGET_BANDWIDTHS_HZ,
        "target_phase_margin_deg": TARGET_PHASE_MARGIN_DEG,
        "candidates": candidates,
        "phase_feasibility": phase_feasibility,
        "boundary_evidence": {
            "80hz_source": "tests/baselines/encoder_speed_estimation_biased_holdout_20260825.json",
            "80hz_plant_phase_deg": [-143.38276845629682, -144.82876772097308],
            "80hz_maximum_pi_phase_margin_deg": 35.17123227902692,
            "100hz_status": "not designed: no direct plant point and 80 Hz already fails the 65 deg PI feasibility target",
        },
        "status": "six_offline_candidates_generated",
        "hardware_executed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    report = build_report(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "output": str(args.output.resolve()),
        "status": report["status"],
        "candidates": [
            {
                "name": item["name"],
                "kp": item["kp"],
                "ki": item["ki"],
                "phase_margin_deg": [
                    check["phase_margin_deg_at_target"]
                    for check in item["direction_checks"]
                ],
                "predicted_peak_db": [
                    check["predicted_maximum_peak_db_on_measured_grid"]
                    for check in item["direction_checks"]
                ],
            }
            for item in report["candidates"]
        ],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
