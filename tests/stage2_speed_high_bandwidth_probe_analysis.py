#!/usr/bin/env python3
"""Analyze D-041 without turning a diagnostic subset into a strict pass."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import stage2_speed_high_bandwidth_probe as probe


DEFAULT_INPUT = Path(
    "tests/results/stage2_speed_high_bandwidth_probe_20260825_180920.json"
)
DEFAULT_OUTPUT = Path(
    "tests/baselines/stage2_speed_high_bandwidth_probe_d041_20260825.json"
)
CORE_CONTROLLER = "bw40_pm65"
CORE_AMPLITUDE_RPM = 300.0


def region(maximum_margin_deg: float | None) -> str:
    if maximum_margin_deg is None:
        return "insufficient_snr"
    if maximum_margin_deg >= probe.GREEN_MAXIMUM_PI_MARGIN_DEG:
        return "green"
    if maximum_margin_deg >= probe.YELLOW_MAXIMUM_PI_MARGIN_DEG:
        return "yellow"
    return "red"


def core_frequency_summary(records: list[dict]) -> list[dict]:
    result = []
    for frequency_hz in probe.FREQUENCIES_HZ:
        points = [
            item for item in records
            if item["controller"] == CORE_CONTROLLER
            and item["amplitude_rpm"] == CORE_AMPLITUDE_RPM
            and item["frequency_hz"] == frequency_hz
        ]
        valid = [
            item for item in points
            if item["measured_speed_fit"]["sine_snr_db"]
            >= probe.MINIMUM_MEASURED_SINE_SNR_DB
        ]
        bias_pairs = probe.bias_pair_differences(valid)
        magnitude_pair_max = max(
            [item["plant_magnitude_difference_db"] for item in bias_pairs],
            default=None,
        )
        phase_pair_max = max(
            [item["plant_phase_difference_deg"] for item in bias_pairs],
            default=None,
        )
        complete = len(points) == 4 and len(valid) == 4
        repeatable = (
            complete
            and magnitude_pair_max is not None
            and phase_pair_max is not None
            and magnitude_pair_max
            <= probe.MAXIMUM_PLANT_MAGNITUDE_PAIR_DIFFERENCE_DB
            and phase_pair_max
            <= probe.MAXIMUM_PLANT_PHASE_PAIR_DIFFERENCE_DEG
        )
        phases = [item["plant_phase_deg"] for item in valid]
        magnitudes = [item["plant_magnitude_db"] for item in valid]
        worst_margin = min(
            [item["maximum_possible_pi_phase_margin_deg"] for item in valid],
            default=None,
        )
        result.append({
            "frequency_hz": frequency_hz,
            "record_count": len(points),
            "snr_qualified_count": len(valid),
            "minimum_snr_db": min(
                [item["measured_speed_fit"]["sine_snr_db"]
                 for item in points],
                default=None,
            ),
            "median_plant_magnitude_db": (
                statistics.median(magnitudes) if magnitudes else None
            ),
            "plant_magnitude_range_db": (
                max(magnitudes) - min(magnitudes) if magnitudes else None
            ),
            "median_plant_phase_deg": (
                statistics.median(phases) if phases else None
            ),
            "plant_phase_range_deg": (
                max(phases) - min(phases) if phases else None
            ),
            "maximum_bias_pair_magnitude_difference_db": magnitude_pair_max,
            "maximum_bias_pair_phase_difference_deg": phase_pair_max,
            "worst_maximum_possible_pi_margin_deg": worst_margin,
            "all_four_biases_snr_qualified": complete,
            "bias_repeatable": repeatable,
            "phase_region": region(worst_margin) if complete else "insufficient_snr",
        })
    return result


def point_quality(records: list[dict]) -> list[dict]:
    result = []
    for frequency_hz in probe.FREQUENCIES_HZ:
        points = [
            item for item in records if item["frequency_hz"] == frequency_hz
        ]
        valid = [
            item for item in points
            if item["measured_speed_fit"]["sine_snr_db"]
            >= probe.MINIMUM_MEASURED_SINE_SNR_DB
        ]
        result.append({
            "frequency_hz": frequency_hz,
            "record_count": len(points),
            "snr_qualified_count": len(valid),
            "minimum_snr_db": min(
                item["measured_speed_fit"]["sine_snr_db"] for item in points
            ),
            "median_snr_db": statistics.median(
                item["measured_speed_fit"]["sine_snr_db"] for item in points
            ),
            "maximum_snr_db": max(
                item["measured_speed_fit"]["sine_snr_db"] for item in points
            ),
        })
    return result


def build_report(source: Path, data: dict) -> dict:
    records = data["records"]
    if data.get("status") != "completed" or len(records) != 128:
        raise ValueError("D-041 analysis requires a completed 128-record result")
    strict = data["summary"]["frequency_summary"]
    return {
        "schema": "stage2-speed-high-bandwidth-probe-analysis-v1",
        "source": str(source),
        "hardware_completion": {
            "records": len(records),
            "final_mode": data.get("final_mode"),
            "default_pi_restored": data.get("default_pi_restored"),
            "peak_iq_reference_a": data["summary"]["peak_iq_reference_a"],
            "peak_actual_dq_current_a": data["summary"][
                "peak_actual_dq_current_a"
            ],
            "peak_raw_speed_rpm": data["summary"]["peak_raw_speed_rpm"],
        },
        "strict_d041": {
            "qualified_frequencies_hz": [
                item["frequency_hz"] for item in strict
                if item["region"] in ("green", "yellow", "red")
            ],
            "result": "inconclusive_all_frequency_groups_contain_low_snr_points",
            "standards_changed_after_data": False,
        },
        "point_quality": point_quality(records),
        "diagnostic_identification_core": {
            "controller": CORE_CONTROLLER,
            "amplitude_rpm": CORE_AMPLITUDE_RPM,
            "biases_rpm": list(probe.SPEED_BIASES_RPM),
            "status": "exploratory_not_a_strict_d041_pass",
            "frequency_summary": core_frequency_summary(records),
        },
        "interpretation": {
            "weak_measurement_combinations": (
                "low controller gain and 100 rpm excitation produce a speed "
                "fundamental below the periodic speed ripple"
            ),
            "supported": (
                "the fixed bw40/300 rpm subset resolves 40-100 Hz at all four "
                "biases and corroborates the previously observed phase roll-off"
            ),
            "not_supported": (
                "D-041 does not qualify a 100 or 150 Hz speed PI and does not "
                "change the default speed PI"
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    data = json.loads(args.input.read_text(encoding="utf-8"))
    report = build_report(args.input, data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
