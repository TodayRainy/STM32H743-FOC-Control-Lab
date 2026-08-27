#!/usr/bin/env python3
"""Offline replay tools for ABZ encoder speed-estimator candidates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


TWO_PI = 2.0 * math.pi
BASE_WINDOW_SEC = 500.0 / 40000.0


def unwrap_angles(angles: list[float]) -> list[float]:
    if not angles:
        return []
    unwrapped = [angles[0]]
    previous = angles[0]
    for angle in angles[1:]:
        delta = angle - previous
        if delta > math.pi:
            delta -= TWO_PI
        elif delta < -math.pi:
            delta += TWO_PI
        unwrapped.append(unwrapped[-1] + delta)
        previous = angle
    return unwrapped


def fixed_window_speed(unwrapped_angle: list[float], sample_period: float,
                       window_samples: int) -> list[float]:
    speed = [math.nan] * len(unwrapped_angle)
    window_sec = window_samples * sample_period
    for index in range(window_samples, len(unwrapped_angle)):
        speed[index] = (
            unwrapped_angle[index] - unwrapped_angle[index - window_samples]
        ) / window_sec
    return speed


def first_order_lowpass(values: list[float], sample_period: float,
                        cutoff_hz: float) -> list[float]:
    alpha = math.exp(-TWO_PI * cutoff_hz * sample_period)
    filtered = [math.nan] * len(values)
    state = None
    for index, value in enumerate(values):
        if not math.isfinite(value):
            continue
        if state is None:
            state = value
        else:
            state = alpha * state + (1.0 - alpha) * value
        filtered[index] = state
    return filtered


def window_iir_speed(unwrapped_angle: list[float], sample_period: float,
                     window_sec: float, cutoff_hz: float) -> list[float]:
    window_samples = max(1, round(window_sec / sample_period))
    raw_speed = fixed_window_speed(
        unwrapped_angle, sample_period, window_samples,
    )
    return first_order_lowpass(raw_speed, sample_period, cutoff_hz)


def tracking_pll_speed(unwrapped_angle: list[float], sample_period: float,
                       bandwidth_hz: float) -> list[float]:
    if not unwrapped_angle:
        return []
    natural_frequency = TWO_PI * bandwidth_hz
    kp = 2.0 * natural_frequency
    ki = natural_frequency * natural_frequency
    position = unwrapped_angle[0]
    speed = 0.0
    result = []
    for measured_position in unwrapped_angle:
        position += sample_period * speed
        position_error = measured_position - position
        position += sample_period * kp * position_error
        speed += sample_period * ki * position_error
        result.append(speed)
    return result


def adaptive_count_speed(unwrapped_angle: list[float], sample_period: float,
                         count_step_angle: float, minimum_counts: int,
                         maximum_window_sec: float,
                         cutoff_hz: float) -> list[float]:
    if not unwrapped_angle:
        return []
    anchor_angle = unwrapped_angle[0]
    anchor_index = 0
    raw_speed = 0.0
    raw = []
    count_threshold = minimum_counts * count_step_angle
    for index, angle in enumerate(unwrapped_angle):
        elapsed = (index - anchor_index) * sample_period
        delta = angle - anchor_angle
        enough_counts = abs(delta) >= count_threshold
        timed_out = elapsed >= maximum_window_sec
        if index > anchor_index and (enough_counts or timed_out):
            raw_speed = delta / elapsed
            anchor_angle = angle
            anchor_index = index
        raw.append(raw_speed)
    return first_order_lowpass(raw, sample_period, cutoff_hz)


def boxcar_characteristics(sample_hz: float, window_samples: int) -> dict:
    window_sec = window_samples / sample_hz
    return {
        "window_samples": window_samples,
        "window_sec": window_sec,
        "group_delay_sec": (window_samples - 1.0) / (2.0 * sample_hz),
        "minus_3db_hz_approx": 0.442946 / window_sec,
        "first_notch_hz": sample_hz / window_samples,
    }


def finite_rmse(first: list[float], second: list[float], start: int = 0) -> float:
    errors = [
        a - b for a, b in zip(first[start:], second[start:])
        if math.isfinite(a) and math.isfinite(b)
    ]
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def finite_rms(values: list[float], start: int = 0) -> float:
    finite = [value for value in values[start:] if math.isfinite(value)]
    mean = sum(finite) / len(finite)
    return math.sqrt(sum((value - mean) ** 2 for value in finite) / len(finite))


def load_scope(path: Path) -> tuple[list[float], list[float], list[float] | None]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "time_s" not in rows[0] or "elec_angle" not in rows[0]:
        raise RuntimeError("scope CSV must contain time_s and elec_angle")
    time_s = [float(row["time_s"]) for row in rows]
    angle = [float(row["elec_angle"]) for row in rows]
    recorded_speed = None
    if "elec_speed" in rows[0]:
        recorded_speed = [float(row["elec_speed"]) for row in rows]
    return time_s, angle, recorded_speed


def analyze_scope(path: Path, win_window_ms: float, win_cutoff_hz: float,
                  pll_bandwidth_hz: float,
                  adaptive_minimum_counts: int = 4,
                  adaptive_maximum_window_ms: float = 50.0,
                  adaptive_cutoff_hz: float = 100.0,
                  encoder_cpr: int = 4096,
                  pole_pairs: int = 10) -> dict:
    time_s, angle, recorded_speed = load_scope(path)
    sample_period = statistics.median(
        second - first for first, second in zip(time_s, time_s[1:])
    )
    sample_hz = 1.0 / sample_period
    unwrapped = unwrap_angles(angle)
    base_samples = round(BASE_WINDOW_SEC / sample_period)
    base = fixed_window_speed(unwrapped, sample_period, base_samples)
    win_iir = window_iir_speed(
        unwrapped, sample_period, win_window_ms * 1.0e-3, win_cutoff_hz,
    )
    pll = tracking_pll_speed(unwrapped, sample_period, pll_bandwidth_hz)
    electrical_count_step = TWO_PI * pole_pairs / encoder_cpr
    adaptive = adaptive_count_speed(
        unwrapped, sample_period, electrical_count_step,
        adaptive_minimum_counts, adaptive_maximum_window_ms * 1.0e-3,
        adaptive_cutoff_hz,
    )
    trim_samples = max(base_samples, round(0.1 / sample_period))

    report = {
        "schema": "encoder-speed-estimation-replay-v1",
        "source": str(path),
        "sample_hz": sample_hz,
        "samples": len(time_s),
        "input_limitations": [
            "elec_angle is sampled scope data, not raw TIM3 count or edge timestamp",
            "closed-loop captures were generated while BASE was in feedback",
            "exact edge-capture M/T cannot be reconstructed without edge timing",
        ],
        "base": {
            **boxcar_characteristics(sample_hz, base_samples),
            "ac_rms_rad_s": finite_rms(base, trim_samples),
        },
        "win_iir": {
            "window_ms": win_window_ms,
            "cutoff_hz": win_cutoff_hz,
            "ac_rms_rad_s": finite_rms(win_iir, trim_samples),
        },
        "pll": {
            "bandwidth_hz": pll_bandwidth_hz,
            "ac_rms_rad_s": finite_rms(pll, trim_samples),
        },
        "adaptive_count": {
            "minimum_counts": adaptive_minimum_counts,
            "maximum_window_ms": adaptive_maximum_window_ms,
            "cutoff_hz": adaptive_cutoff_hz,
            "ac_rms_rad_s": finite_rms(adaptive, trim_samples),
        },
    }
    if recorded_speed is not None:
        report["base"]["recorded_reproduction_rmse_rad_s"] = finite_rmse(
            base, recorded_speed, trim_samples,
        )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scope_csv", type=Path)
    parser.add_argument("--win-window-ms", type=float, default=1.0)
    parser.add_argument("--win-cutoff-hz", type=float, default=100.0)
    parser.add_argument("--pll-bandwidth-hz", type=float, default=100.0)
    parser.add_argument("--adaptive-minimum-counts", type=int, default=4)
    parser.add_argument("--adaptive-maximum-window-ms", type=float, default=50.0)
    parser.add_argument("--adaptive-cutoff-hz", type=float, default=100.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = analyze_scope(
        args.scope_csv, args.win_window_ms, args.win_cutoff_hz,
        args.pll_bandwidth_hz, args.adaptive_minimum_counts,
        args.adaptive_maximum_window_ms, args.adaptive_cutoff_hz,
    )
    encoded = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
