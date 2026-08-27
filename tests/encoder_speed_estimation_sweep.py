#!/usr/bin/env python3
"""Compare ABZ speed estimators on one quantized 4096-CPR model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import encoder_speed_estimation as estimation


SAMPLE_HZ = 40000.0
SAMPLE_PERIOD = 1.0 / SAMPLE_HZ
ENCODER_CPR = 4096
POLE_PAIRS = 10
COUNT_STEP_ELECTRICAL = 2.0 * math.pi * POLE_PAIRS / ENCODER_CPR
CONSTANT_SPEEDS_RPM = (1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1500.0, 3000.0, 4200.0)
FREQUENCIES_HZ = (2.0, 5.0, 10.0, 20.0, 40.0, 80.0, 120.0, 160.0)


CANDIDATES = (
    {"name": "win_0p5ms_fc100", "kind": "win", "window_sec": 0.0005, "cutoff_hz": 100.0},
    {"name": "win_1ms_fc100", "kind": "win", "window_sec": 0.001, "cutoff_hz": 100.0},
    {"name": "win_1ms_fc150", "kind": "win", "window_sec": 0.001, "cutoff_hz": 150.0},
    {"name": "win_2ms_fc100", "kind": "win", "window_sec": 0.002, "cutoff_hz": 100.0},
    {"name": "pll_50hz", "kind": "pll", "bandwidth_hz": 50.0},
    {"name": "pll_100hz", "kind": "pll", "bandwidth_hz": 100.0},
    {"name": "pll_150hz", "kind": "pll", "bandwidth_hz": 150.0},
    {"name": "pll_200hz", "kind": "pll", "bandwidth_hz": 200.0},
    {"name": "adaptive_4c_fc100", "kind": "adaptive", "minimum_counts": 4,
     "maximum_window_sec": 0.05, "cutoff_hz": 100.0},
    {"name": "adaptive_4c_fc150", "kind": "adaptive", "minimum_counts": 4,
     "maximum_window_sec": 0.05, "cutoff_hz": 150.0},
)


def rpm_to_electrical_speed(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0 * POLE_PAIRS


def electrical_speed_to_rpm(speed: float) -> float:
    return speed * 60.0 / (2.0 * math.pi * POLE_PAIRS)


def quantize_electrical_angle(true_angle: list[float]) -> list[float]:
    return [
        math.floor(angle / COUNT_STEP_ELECTRICAL) * COUNT_STEP_ELECTRICAL
        for angle in true_angle
    ]


def run_candidate(candidate: dict, measured_angle: list[float]) -> list[float]:
    if candidate["kind"] == "win":
        return estimation.window_iir_speed(
            measured_angle, SAMPLE_PERIOD, candidate["window_sec"],
            candidate["cutoff_hz"],
        )
    if candidate["kind"] == "pll":
        return estimation.tracking_pll_speed(
            measured_angle, SAMPLE_PERIOD, candidate["bandwidth_hz"],
        )
    return estimation.adaptive_count_speed(
        measured_angle, SAMPLE_PERIOD, COUNT_STEP_ELECTRICAL,
        candidate["minimum_counts"], candidate["maximum_window_sec"],
        candidate["cutoff_hz"],
    )


def signal_stats(values: list[float], start: int) -> tuple[float, float]:
    finite = [value for value in values[start:] if math.isfinite(value)]
    mean = sum(finite) / len(finite)
    ripple = math.sqrt(sum((value - mean) ** 2 for value in finite) / len(finite))
    return mean, ripple


def phasor(values: list[float], frequency_hz: float, start: int) -> complex:
    selected = [value for value in values[start:] if math.isfinite(value)]
    times = [
        (start + index) * SAMPLE_PERIOD for index in range(len(selected))
    ]
    mean = sum(selected) / len(selected)
    cosine = 0.0
    sine = 0.0
    for value, time_sec in zip(selected, times):
        angle = 2.0 * math.pi * frequency_hz * time_sec
        centered = value - mean
        cosine += centered * math.cos(angle)
        sine += centered * math.sin(angle)
    scale = 2.0 / len(selected)
    return complex(cosine * scale, -sine * scale)


def constant_speed_results(candidate: dict) -> list[dict]:
    results = []
    samples = round(1.0 * SAMPLE_HZ)
    trim = round(0.2 * SAMPLE_HZ)
    for rpm in CONSTANT_SPEEDS_RPM:
        expected = rpm_to_electrical_speed(rpm)
        true_angle = [expected * index * SAMPLE_PERIOD for index in range(samples)]
        measured = quantize_electrical_angle(true_angle)
        estimated = run_candidate(candidate, measured)
        mean, ripple = signal_stats(estimated, trim)
        results.append({
            "rpm": rpm,
            "mean_error_rpm": electrical_speed_to_rpm(mean - expected),
            "ripple_rms_rpm": electrical_speed_to_rpm(ripple),
        })
    return results


def frequency_results(candidate: dict) -> list[dict]:
    results = []
    amplitude = rpm_to_electrical_speed(300.0)
    trim_sec = 0.2
    for frequency_hz in FREQUENCIES_HZ:
        fit_sec = max(1.0, 8.0 / frequency_hz)
        samples = round((trim_sec + fit_sec) * SAMPLE_HZ)
        true_speed = []
        true_angle = []
        position = 0.0
        for index in range(samples):
            time_sec = index * SAMPLE_PERIOD
            speed = amplitude * math.sin(2.0 * math.pi * frequency_hz * time_sec)
            position += speed * SAMPLE_PERIOD
            true_speed.append(speed)
            true_angle.append(position)
        measured = quantize_electrical_angle(true_angle)
        estimated = run_candidate(candidate, measured)
        trim = round(trim_sec * SAMPLE_HZ)
        response = phasor(estimated, frequency_hz, trim) / phasor(
            true_speed, frequency_hz, trim,
        )
        results.append({
            "frequency_hz": frequency_hz,
            "magnitude_db": 20.0 * math.log10(max(abs(response), 1.0e-12)),
            "phase_deg": math.degrees(math.atan2(response.imag, response.real)),
        })
    return results


def screen(candidate: dict, constants: list[dict], frequency: list[dict]) -> dict:
    by_speed = {point["rpm"]: point for point in constants}
    by_frequency = {point["frequency_hz"]: point for point in frequency}
    maximum_bias = max(abs(point["mean_error_rpm"]) for point in constants)
    maximum_peak = max(point["magnitude_db"] for point in frequency)
    checks = {
        "maximum_constant_bias_rpm_le_0p5": maximum_bias <= 0.5,
        "ten_rpm_ripple_le_2rpm": by_speed[10.0]["ripple_rms_rpm"] <= 2.0,
        "forty_hz_gain_abs_le_1db": abs(by_frequency[40.0]["magnitude_db"]) <= 1.0,
        "forty_hz_phase_lag_le_35deg": by_frequency[40.0]["phase_deg"] >= -35.0,
        "eighty_hz_gain_ge_minus3db": by_frequency[80.0]["magnitude_db"] >= -3.0,
        "eighty_hz_phase_lag_le_60deg": by_frequency[80.0]["phase_deg"] >= -60.0,
        "maximum_peak_le_2db": maximum_peak <= 2.0,
    }
    return {
        "qualified": all(checks.values()),
        "checks": checks,
        "maximum_constant_bias_rpm": maximum_bias,
        "ten_rpm_ripple_rms_rpm": by_speed[10.0]["ripple_rms_rpm"],
        "gain_40hz_db": by_frequency[40.0]["magnitude_db"],
        "phase_40hz_deg": by_frequency[40.0]["phase_deg"],
        "gain_80hz_db": by_frequency[80.0]["magnitude_db"],
        "phase_80hz_deg": by_frequency[80.0]["phase_deg"],
        "maximum_peak_db": maximum_peak,
    }


def build_report() -> dict:
    candidates = []
    for definition in CANDIDATES:
        constants = constant_speed_results(definition)
        frequency = frequency_results(definition)
        candidates.append({
            "definition": definition,
            "screen": screen(definition, constants, frequency),
            "constant_speed": constants,
            "frequency_response": frequency,
        })
    qualified = [
        item for item in candidates if item["screen"]["qualified"]
    ]
    qualified.sort(key=lambda item: (
        abs(item["screen"]["phase_80hz_deg"]),
        item["screen"]["ten_rpm_ripple_rms_rpm"],
    ))
    return {
        "schema": "encoder-speed-estimation-synthetic-sweep-v1",
        "model": {
            "sample_hz": SAMPLE_HZ,
            "encoder_cpr": ENCODER_CPR,
            "pole_pairs": POLE_PAIRS,
            "constant_speeds_rpm": CONSTANT_SPEEDS_RPM,
            "sine_amplitude_rpm": 300.0,
            "frequencies_hz": FREQUENCIES_HZ,
            "limitations": [
                "ideal encoder with position quantization only",
                "no electrical noise, missed counts, eccentricity or mechanical resonance",
                "screen selects offline candidates, not firmware defaults",
            ],
        },
        "screen": {
            "purpose": "select at most two candidates for hardware A/B",
            "qualified_order": [item["definition"]["name"] for item in qualified],
        },
        "candidates": candidates,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = build_report()
    encoded = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
