#!/usr/bin/env python3
"""Design the six D-042 speed PI candidates from the D-040/D-041 plant."""

from __future__ import annotations

import argparse
import cmath
import json
import math
from pathlib import Path

from stage2_speed_high_bandwidth_probe import (
    MINIMUM_MEASURED_SINE_SNR_DB,
    negative_feedback_phase_deg,
)
from stage2_speed_high_bandwidth_probe_analysis import (
    CORE_AMPLITUDE_RPM,
    CORE_CONTROLLER,
)
from stage2_speed_pi_design import design_pi


DEFAULT_LOW_SOURCE = Path(
    "tests/results/stage2_speed_pi_frequency_validation_20260825_165849.json"
)
DEFAULT_HIGH_SOURCE = Path(
    "tests/results/stage2_speed_high_bandwidth_probe_20260825_180920.json"
)
DEFAULT_OUTPUT = Path(
    "tests/baselines/stage2_speed_high_bandwidth_pi_design_d042_20260825.json"
)
TARGET_GAIN_CROSSOVERS_HZ = (22.0, 24.0, 26.0, 28.0, 30.0, 32.0)
TARGET_PHASE_MARGIN_DEG = 70.0
SCENARIO_BIASES_RPM = (1500.0, 3000.0, -3000.0, -1500.0)
LOW_FREQUENCY_LIMIT_HZ = 35.0
HIGH_FREQUENCY_LIMIT_HZ = 100.0
PREDICTION_SINE_AMPLITUDE_RPM = 200.0
POLE_PAIRS = 10
MAXIMUM_PREDICTED_CROSSOVER_ERROR_FRACTION = 0.10
MINIMUM_PREDICTED_PHASE_MARGIN_DEG = 60.0
MAXIMUM_PREDICTED_CLOSED_LOOP_PEAK_DB = 2.0
MAXIMUM_PREDICTED_IQ_AMPLITUDE_A = 3.0


def phasor_from_db_phase(magnitude_db: float, phase_deg: float) -> complex:
    return 10.0 ** (magnitude_db / 20.0) * cmath.exp(
        1j * math.radians(phase_deg)
    )


def recover_plant(record: dict, kp: float, ki: float) -> complex:
    frequency_hz = float(record["frequency_hz"])
    response = phasor_from_db_phase(
        float(record["closed_loop_magnitude_db"]),
        float(record["closed_loop_phase_deg"]),
    )
    loop = response / (1.0 - response)
    controller = kp + ki / (1j * 2.0 * math.pi * frequency_hz)
    return loop / controller


def mean_plant(plants: list[complex]) -> complex:
    magnitude = math.exp(
        sum(math.log(abs(value)) for value in plants) / len(plants)
    )
    phase_deg = sum(negative_feedback_phase_deg(value) for value in plants)
    phase_deg /= len(plants)
    return cmath.rect(magnitude, math.radians(phase_deg))


def interpolate_plant(points: dict[float, complex], frequency_hz: float) -> complex:
    if frequency_hz in points:
        return points[frequency_hz]
    frequencies = sorted(points)
    if frequency_hz < frequencies[0] or frequency_hz > frequencies[-1]:
        raise ValueError("plant interpolation requested outside measured range")
    lower = max(value for value in frequencies if value < frequency_hz)
    upper = min(value for value in frequencies if value > frequency_hz)
    fraction = math.log(frequency_hz / lower) / math.log(upper / lower)
    log_magnitude = (
        (1.0 - fraction) * math.log(abs(points[lower]))
        + fraction * math.log(abs(points[upper]))
    )
    lower_phase = negative_feedback_phase_deg(points[lower])
    upper_phase = negative_feedback_phase_deg(points[upper])
    phase_deg = (1.0 - fraction) * lower_phase + fraction * upper_phase
    return cmath.rect(math.exp(log_magnitude), math.radians(phase_deg))


def load_plant_scenarios(low_path: Path, high_path: Path) -> list[dict]:
    low_data = json.loads(low_path.read_text(encoding="utf-8"))
    high_data = json.loads(high_path.read_text(encoding="utf-8"))
    low_by_bias: dict[float, dict[float, complex]] = {}
    for visit in low_data["visits"]:
        if visit["candidate"] != CORE_CONTROLLER:
            continue
        bias = float(visit["bias_rpm"])
        low_by_bias[bias] = {
            float(record["frequency_hz"]): recover_plant(
                record, float(visit["kp"]), float(visit["ki"]),
            )
            for record in visit["records"]
            if float(record["frequency_hz"]) <= LOW_FREQUENCY_LIMIT_HZ
        }
    if set(low_by_bias) != {1500.0, -1500.0}:
        raise ValueError("D-040 bw40 low-frequency directions are incomplete")

    low_nominal = {
        frequency_hz: mean_plant([
            low_by_bias[1500.0][frequency_hz],
            low_by_bias[-1500.0][frequency_hz],
        ])
        for frequency_hz in low_by_bias[1500.0]
    }
    high_by_bias: dict[float, dict[float, complex]] = {
        bias: {} for bias in SCENARIO_BIASES_RPM
    }
    for record in high_data["records"]:
        if (
            record["controller"] != CORE_CONTROLLER
            or float(record["amplitude_rpm"]) != CORE_AMPLITUDE_RPM
            or float(record["frequency_hz"]) > HIGH_FREQUENCY_LIMIT_HZ
            or record["measured_speed_fit"]["sine_snr_db"]
            < MINIMUM_MEASURED_SINE_SNR_DB
        ):
            continue
        high_by_bias[float(record["bias_rpm"])][
            float(record["frequency_hz"])
        ] = phasor_from_db_phase(
            float(record["plant_magnitude_db"]),
            float(record["plant_phase_deg"]),
        )
    expected_high = {40.0, 50.0, 60.0, 70.0, 80.0, 100.0}
    for bias, points in high_by_bias.items():
        if set(points) != expected_high:
            raise ValueError(f"D-041 core is incomplete at {bias:g} rpm")

    scenarios = []
    for bias in SCENARIO_BIASES_RPM:
        low_points = low_by_bias.get(bias, low_nominal)
        scenarios.append({
            "bias_rpm": bias,
            "low_frequency_source": (
                "measured_d040" if bias in low_by_bias
                else "d040_direction_geometric_mean"
            ),
            "points": {**low_points, **high_by_bias[bias]},
        })
    return scenarios


def controller(kp: float, ki: float, frequency_hz: float) -> complex:
    return kp + ki / (1j * 2.0 * math.pi * frequency_hz)


def nominal_plant(scenarios: list[dict], frequency_hz: float) -> complex:
    return mean_plant([
        interpolate_plant(item["points"], frequency_hz)
        for item in scenarios
    ])


def logarithmic_grid(start_hz: float = 4.0, stop_hz: float = 100.0,
                     count: int = 2001) -> list[float]:
    return [
        math.exp(
            math.log(start_hz)
            + index / (count - 1) * math.log(stop_hz / start_hz)
        )
        for index in range(count)
    ]


def downward_crossing(samples: list[tuple[float, float]],
                      threshold: float) -> float | None:
    previous = samples[0]
    for current in samples[1:]:
        if previous[1] > threshold and current[1] <= threshold:
            fraction = (threshold - previous[1]) / (current[1] - previous[1])
            return math.exp(
                math.log(previous[0])
                + fraction * math.log(current[0] / previous[0])
            )
        previous = current
    return None


def response_metrics(points: dict[float, complex], kp: float,
                     ki: float) -> dict:
    frequencies = logarithmic_grid()
    loop_values = []
    closed_values = []
    iq_values = []
    reference_electrical_rad_s = (
        PREDICTION_SINE_AMPLITUDE_RPM
        * 2.0 * math.pi / 60.0 * POLE_PAIRS
    )
    for frequency_hz in frequencies:
        plant = interpolate_plant(points, frequency_hz)
        control = controller(kp, ki, frequency_hz)
        loop = control * plant
        closed = loop / (1.0 + loop)
        loop_values.append((frequency_hz, 20.0 * math.log10(abs(loop))))
        closed_values.append(
            (frequency_hz, 20.0 * math.log10(abs(closed)))
        )
        iq_values.append(abs(control / (1.0 + loop)) * reference_electrical_rad_s)
    crossover_hz = downward_crossing(loop_values, 0.0)
    bandwidth_hz = downward_crossing(closed_values, -3.0103)
    phase_margin_deg = None
    if crossover_hz is not None:
        loop = (
            controller(kp, ki, crossover_hz)
            * interpolate_plant(points, crossover_hz)
        )
        phase_margin_deg = 180.0 + negative_feedback_phase_deg(loop)
    return {
        "gain_crossover_hz": crossover_hz,
        "closed_loop_bandwidth_hz": bandwidth_hz,
        "phase_margin_deg": phase_margin_deg,
        "maximum_closed_loop_peak_db": max(value for _, value in closed_values),
        "maximum_predicted_iq_amplitude_a_for_200rpm": max(iq_values),
    }


def design_candidate(scenarios: list[dict], target_crossover_hz: float) -> dict:
    nominal_points = {
        frequency: nominal_plant(scenarios, frequency)
        for frequency in sorted(scenarios[0]["points"])
    }
    gains = design_pi(
        nominal_plant(scenarios, target_crossover_hz),
        target_crossover_hz,
        TARGET_PHASE_MARGIN_DEG,
    )
    nominal_metrics = response_metrics(
        nominal_points, gains["kp"], gains["ki"],
    )
    scenario_metrics = []
    for scenario in scenarios:
        metrics = response_metrics(scenario["points"], gains["kp"], gains["ki"])
        scenario_metrics.append({"bias_rpm": scenario["bias_rpm"], **metrics})
    crossover_errors = [
        abs(item["gain_crossover_hz"] - target_crossover_hz)
        / target_crossover_hz
        for item in scenario_metrics
    ]
    preflight = {
        "gain_crossover_envelope": max(crossover_errors)
        <= MAXIMUM_PREDICTED_CROSSOVER_ERROR_FRACTION,
        "phase_margin_envelope": min(
            item["phase_margin_deg"] for item in scenario_metrics
        ) >= MINIMUM_PREDICTED_PHASE_MARGIN_DEG,
        "closed_loop_peak_envelope": max(
            item["maximum_closed_loop_peak_db"] for item in scenario_metrics
        ) <= MAXIMUM_PREDICTED_CLOSED_LOOP_PEAK_DB,
        "iq_amplitude_envelope": max(
            item["maximum_predicted_iq_amplitude_a_for_200rpm"]
            for item in scenario_metrics
        ) <= MAXIMUM_PREDICTED_IQ_AMPLITUDE_A,
    }
    return {
        "name": (
            f"d042_fc{int(target_crossover_hz):02d}_"
            f"pm{int(TARGET_PHASE_MARGIN_DEG):02d}"
        ),
        "target_gain_crossover_hz": target_crossover_hz,
        "target_nominal_phase_margin_deg": TARGET_PHASE_MARGIN_DEG,
        "kp": gains["kp"],
        "ki": gains["ki"],
        "integral_time_sec": gains["integral_time_sec"],
        "nominal_metrics": nominal_metrics,
        "scenario_metrics": scenario_metrics,
        "maximum_predicted_crossover_error_fraction": max(crossover_errors),
        "minimum_predicted_phase_margin_deg": min(
            item["phase_margin_deg"] for item in scenario_metrics
        ),
        "maximum_predicted_closed_loop_peak_db": max(
            item["maximum_closed_loop_peak_db"] for item in scenario_metrics
        ),
        "maximum_predicted_iq_amplitude_a_for_200rpm": max(
            item["maximum_predicted_iq_amplitude_a_for_200rpm"]
            for item in scenario_metrics
        ),
        "preflight": preflight,
        "preflight_pass": all(preflight.values()),
    }


def build_report(low_path: Path, high_path: Path) -> dict:
    scenarios = load_plant_scenarios(low_path, high_path)
    candidates = [
        design_candidate(scenarios, crossover_hz)
        for crossover_hz in TARGET_GAIN_CROSSOVERS_HZ
    ]
    return {
        "schema": "stage2-speed-high-bandwidth-pi-design-v1",
        "low_frequency_source": str(low_path),
        "high_frequency_source": str(high_path),
        "model": {
            "speed_biases_rpm": list(SCENARIO_BIASES_RPM),
            "low_frequency_range_hz": [4.0, LOW_FREQUENCY_LIMIT_HZ],
            "high_frequency_range_hz": [40.0, HIGH_FREQUENCY_LIMIT_HZ],
            "high_frequency_controller": CORE_CONTROLLER,
            "high_frequency_amplitude_rpm": CORE_AMPLITUDE_RPM,
            "high_frequency_minimum_snr_db": MINIMUM_MEASURED_SINE_SNR_DB,
            "note": (
                "D-041 core remains exploratory; D-042 hardware is the "
                "independent qualification"
            ),
        },
        "targets": {
            "gain_crossovers_hz": list(TARGET_GAIN_CROSSOVERS_HZ),
            "nominal_phase_margin_deg": TARGET_PHASE_MARGIN_DEG,
        },
        "preflight_limits": {
            "maximum_gain_crossover_error_fraction": (
                MAXIMUM_PREDICTED_CROSSOVER_ERROR_FRACTION
            ),
            "minimum_phase_margin_deg": MINIMUM_PREDICTED_PHASE_MARGIN_DEG,
            "maximum_closed_loop_peak_db": MAXIMUM_PREDICTED_CLOSED_LOOP_PEAK_DB,
            "maximum_iq_amplitude_a_for_200rpm": (
                MAXIMUM_PREDICTED_IQ_AMPLITUDE_A
            ),
        },
        "candidates": candidates,
        "status": (
            "six_candidates_preflight_passed"
            if all(item["preflight_pass"] for item in candidates)
            else "candidate_preflight_failed"
        ),
        "hardware_executed": False,
        "default_pi_changed": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--low-source", type=Path, default=DEFAULT_LOW_SOURCE)
    parser.add_argument("--high-source", type=Path, default=DEFAULT_HIGH_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    report = build_report(args.low_source, args.high_source)
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
                "gain_crossover_hz": item["target_gain_crossover_hz"],
                "minimum_phase_margin_deg": item[
                    "minimum_predicted_phase_margin_deg"
                ],
                "maximum_peak_db": item[
                    "maximum_predicted_closed_loop_peak_db"
                ],
                "maximum_iq_amplitude_a": item[
                    "maximum_predicted_iq_amplitude_a_for_200rpm"
                ],
                "preflight_pass": item["preflight_pass"],
            }
            for item in report["candidates"]
        ],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
