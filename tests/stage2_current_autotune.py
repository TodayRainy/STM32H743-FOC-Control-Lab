#!/usr/bin/env python3
"""Offline current-loop autotuning design for a newly identified motor.

The tool deliberately does not read the active/default PI gains.  It consumes
the Stage-1 electrical model and an optional d-axis plant FRF, then produces
the fixed v1.0 set of model anchors and measured-plant loop-shaping candidates.
It never opens a serial port or writes firmware.
"""

from __future__ import annotations

import argparse
import cmath
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path


DEFAULT_STAGE1 = Path("tests/baselines/stage1_20260824.json")
DEFAULT_FRF = Path("tests/results/stage2_current_frf_20260824_145248.json")
DEFAULT_EXCLUDED_PROFILES = ("active_legacy",)
PI_VOLTAGE_LIMIT_V = 15.0
MIN_PHASE_MARGIN_DEG = 60.0
MAX_SENSITIVITY = 1.7
VOLTAGE_RESERVE_FRACTION = 0.8
CURRENT_RESERVE_FRACTION = 0.8


@dataclass(frozen=True)
class AxisBounds:
    nominal_h: float
    minimum_h: float
    maximum_h: float


@dataclass(frozen=True)
class MotorInputs:
    rs_ohm: float
    rs_min_ohm: float
    rs_max_ohm: float
    d: AxisBounds
    q: AxisBounds
    sample_rate_hz: float
    bus_voltage_v: float
    validation_current_a: float
    abort_current_a: float
    quality_passed: bool


def require_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    return value


def load_motor_inputs(path: Path) -> tuple[MotorInputs, dict]:
    document = json.loads(path.read_text(encoding="utf-8"))
    corrected = document["corrected_validation"]
    candidate = corrected["candidate_parameters"]
    runs = corrected["rl_runs"]
    if len(runs) < 2:
        raise ValueError("Stage-1 input needs at least two corrected RL runs")

    rs_values = [require_positive("rl run Rs", run["rs_ohm"]) for run in runs]
    ld_values = [require_positive("rl run Ld", run["ld_h"]) for run in runs]
    lq_values = [require_positive("rl run Lq", run["lq_h"]) for run in runs]
    current_points = corrected["rs_fit_current_points_a"]
    validation_current = max(abs(float(value)) for value in current_points)

    inputs = MotorInputs(
        rs_ohm=require_positive("candidate Rs", candidate["rs_ohm"]),
        rs_min_ohm=min(rs_values),
        rs_max_ohm=max(rs_values),
        d=AxisBounds(
            nominal_h=require_positive("candidate Ld", candidate["ld_h"]),
            minimum_h=min(ld_values),
            maximum_h=max(ld_values),
        ),
        q=AxisBounds(
            nominal_h=require_positive("candidate Lq", candidate["lq_h"]),
            minimum_h=min(lq_values),
            maximum_h=max(lq_values),
        ),
        sample_rate_hz=require_positive(
            "FOC sample rate", document["conditions"]["foc_rate_hz"]
        ),
        bus_voltage_v=require_positive(
            "bus voltage", document["conditions"]["bus_voltage_v"]
        ),
        validation_current_a=require_positive(
            "validation current", validation_current
        ),
        abort_current_a=require_positive(
            "abort current", document["rl"]["abort_current_a"]
        ),
        quality_passed=bool(corrected["all_predeclared_quality_gates_passed"]),
    )
    if not inputs.quality_passed:
        raise ValueError("Stage-1 predeclared quality gates did not pass")
    return inputs, document


def polar_db(magnitude_db: float, phase_deg: float) -> complex:
    magnitude = 10.0 ** (magnitude_db / 20.0)
    return cmath.rect(magnitude, math.radians(phase_deg))


def magnitude_db(value: complex) -> float:
    return 20.0 * math.log10(max(abs(value), 1.0e-15))


def phase_deg(value: complex) -> float:
    return math.degrees(cmath.phase(value))


def unwrap_degrees(values: list[float]) -> list[float]:
    result: list[float] = []
    for value in values:
        if result:
            while value - result[-1] > 180.0:
                value -= 360.0
            while value - result[-1] < -180.0:
                value += 360.0
        result.append(value)
    return result


def load_d_axis_traces(path: Path,
                       excluded_profiles: tuple[str, ...]) -> tuple[list[dict], dict]:
    document = json.loads(path.read_text(encoding="utf-8"))
    grouped: dict[str, list[dict]] = {}
    for record in document["records"]:
        profile = record["profile"]
        if profile in excluded_profiles:
            continue
        if (record.get("plant_magnitude_db_a_per_v") is None
                or record.get("plant_phase_deg") is None):
            continue
        grouped.setdefault(profile, []).append(record)

    traces = []
    for profile, records in sorted(grouped.items()):
        points = sorted(records, key=lambda point: point["frequency_hz"])
        if len(points) < 8:
            continue
        traces.append({
            "name": profile,
            "points": [
                (float(point["frequency_hz"]), polar_db(
                    float(point["plant_magnitude_db_a_per_v"]),
                    float(point["plant_phase_deg"]),
                ))
                for point in points
            ],
        })
    if len(traces) < 2:
        raise ValueError("FRF input needs at least two non-excluded plant traces")
    return traces, document


def interpolate_complex(points: list[tuple[float, complex]],
                        frequency_hz: float) -> complex:
    if not points or frequency_hz < points[0][0] or frequency_hz > points[-1][0]:
        raise ValueError(f"frequency {frequency_hz:g} Hz is outside measured FRF")
    for first, second in zip(points, points[1:]):
        if first[0] <= frequency_hz <= second[0]:
            if frequency_hz == first[0]:
                return first[1]
            if frequency_hz == second[0]:
                return second[1]
            fraction = ((math.log(frequency_hz) - math.log(first[0]))
                        / (math.log(second[0]) - math.log(first[0])))
            db_value = magnitude_db(first[1]) + fraction * (
                magnitude_db(second[1]) - magnitude_db(first[1])
            )
            phases = unwrap_degrees([phase_deg(first[1]), phase_deg(second[1])])
            phase_value = phases[0] + fraction * (phases[1] - phases[0])
            return polar_db(db_value, phase_value)
    return points[-1][1]


def median_trace(traces: list[dict]) -> list[tuple[float, complex]]:
    frequencies = [frequency for frequency, _ in traces[0]["points"]]
    for trace in traces[1:]:
        other = [frequency for frequency, _ in trace["points"]]
        if other != frequencies:
            raise ValueError("all plant traces must use identical frequency points")

    result = []
    for index, frequency in enumerate(frequencies):
        values = [trace["points"][index][1] for trace in traces]
        phases = unwrap_degrees([phase_deg(value) for value in values])
        result.append((frequency, polar_db(
            statistics.median(magnitude_db(value) for value in values),
            statistics.median(phases),
        )))
    return result


def transform_axis_trace(d_trace: list[tuple[float, complex]],
                         nominal_rs: float, nominal_ld: float,
                         target_rs: float, target_l: float) -> list[tuple[float, complex]]:
    result = []
    for frequency, d_plant in d_trace:
        omega = 2.0 * math.pi * frequency
        common_path = d_plant * complex(nominal_rs, omega * nominal_ld)
        result.append((frequency, common_path / complex(target_rs, omega * target_l)))
    return result


def model_corners(inputs: MotorInputs, axis: str) -> list[tuple[float, float]]:
    inductance = inputs.d if axis == "d" else inputs.q
    return [
        (rs, value)
        for rs in (inputs.rs_min_ohm, inputs.rs_max_ohm)
        for value in (inductance.minimum_h, inductance.maximum_h)
    ]


def build_axis_traces(d_traces: list[dict], inputs: MotorInputs,
                      axis: str) -> list[dict]:
    result = []
    for source in d_traces:
        for corner_index, (rs, inductance) in enumerate(model_corners(inputs, axis)):
            result.append({
                "name": f"{source['name']}:corner{corner_index}",
                "points": transform_axis_trace(
                    source["points"], inputs.rs_ohm, inputs.d.nominal_h,
                    rs, inductance,
                ),
            })
    return result


def pi_gains(rs_ohm: float, inductance_h: float,
             bandwidth_hz: float) -> dict:
    omega = 2.0 * math.pi * bandwidth_hz
    return {"kp": omega * inductance_h, "ki": omega * rs_ohm}


def design_from_plant(points: list[tuple[float, complex]],
                      crossover_hz: float, target_pm_deg: float) -> dict:
    plant = interpolate_complex(points, crossover_hz)
    desired_loop = cmath.rect(1.0, math.radians(-180.0 + target_pm_deg))
    controller = desired_loop / plant
    kp = controller.real
    ki = -2.0 * math.pi * crossover_hz * controller.imag
    if not math.isfinite(kp) or not math.isfinite(ki) or kp <= 0.0 or ki <= 0.0:
        raise ValueError(
            f"infeasible PI at {crossover_hz:g} Hz/{target_pm_deg:g} deg"
        )
    return {"kp": kp, "ki": ki}


def simulate_step(rs_ohm: float, inductance_h: float,
                  gains: dict, sample_period_s: float,
                  delay_samples: int, step_current_a: float,
                  voltage_limit_v: float) -> dict:
    decay = math.exp(-rs_ohm * sample_period_s / inductance_h)
    voltage_gain = (1.0 - decay) / rs_ohm
    delay_line = [0.0] * delay_samples
    current = 0.0
    integrator = 0.0
    samples = []
    voltages = []
    sample_count = round(8.0e-3 / sample_period_s)
    step_sample = round(0.5e-3 / sample_period_s)

    for sample in range(sample_count):
        reference = step_current_a if sample >= step_sample else 0.0
        error = reference - current
        integrator += gains["ki"] * error * sample_period_s
        requested = gains["kp"] * error + integrator
        applied_request = max(-voltage_limit_v, min(voltage_limit_v, requested))
        integrator += applied_request - requested
        delay_line.append(applied_request)
        applied = delay_line.pop(0)
        current = decay * current + voltage_gain * applied
        if sample >= step_sample:
            samples.append(current)
            voltages.append(applied_request)

    peak = max(samples)
    overshoot = max(0.0, (peak / step_current_a - 1.0) * 100.0)
    return {
        "peak_current_a": peak,
        "peak_voltage_v": max(abs(value) for value in voltages),
        "overshoot_percent": overshoot,
    }


def rl_safety_at_frequency(inputs: MotorInputs, frequency_hz: float,
                           voltage_limit_v: float,
                           max_delay_samples: int) -> dict:
    worst_current = 0.0
    worst_voltage = 0.0
    worst_overshoot = 0.0
    sample_period = 1.0 / inputs.sample_rate_hz
    for axis in ("d", "q"):
        nominal_l = inputs.d.nominal_h if axis == "d" else inputs.q.nominal_h
        gains = pi_gains(inputs.rs_ohm, nominal_l, frequency_hz)
        for rs, inductance in model_corners(inputs, axis):
            for delay in range(1, max_delay_samples + 1):
                result = simulate_step(
                    rs, inductance, gains, sample_period, delay,
                    inputs.validation_current_a, voltage_limit_v,
                )
                worst_current = max(worst_current, result["peak_current_a"])
                worst_voltage = max(worst_voltage, result["peak_voltage_v"])
                worst_overshoot = max(
                    worst_overshoot, result["overshoot_percent"]
                )
    return {
        "peak_current_a": worst_current,
        "peak_voltage_v": worst_voltage,
        "overshoot_percent": worst_overshoot,
    }


def current_frequency_bound(inputs: MotorInputs, sample_bound_hz: float,
                            voltage_limit_v: float,
                            max_delay_samples: int) -> float:
    current_limit = CURRENT_RESERVE_FRACTION * inputs.abort_current_a

    def passes(frequency: float) -> bool:
        result = rl_safety_at_frequency(
            inputs, frequency, voltage_limit_v, max_delay_samples
        )
        return (result["peak_current_a"] <= current_limit
                and result["peak_voltage_v"]
                <= VOLTAGE_RESERVE_FRACTION * voltage_limit_v)

    if passes(sample_bound_hz):
        return sample_bound_hz
    lower = 1.0
    upper = sample_bound_hz
    for _ in range(40):
        middle = 0.5 * (lower + upper)
        if passes(middle):
            lower = middle
        else:
            upper = middle
    return lower


def compute_frequency_limits(inputs: MotorInputs,
                             max_delay_samples: int) -> dict:
    sample_bound = inputs.sample_rate_hz / 10.0
    delay_s = max_delay_samples / inputs.sample_rate_hz
    pm_rad = math.radians(MIN_PHASE_MARGIN_DEG)
    delay_bound = ((math.pi / 2.0 - pm_rad)
                   / (2.0 * math.pi * delay_s))
    voltage_limit = min(
        PI_VOLTAGE_LIMIT_V, inputs.bus_voltage_v / math.sqrt(3.0)
    )
    voltage_bounds = []
    for axis in (inputs.d, inputs.q):
        numerator = (VOLTAGE_RESERVE_FRACTION * voltage_limit
                     - inputs.rs_max_ohm * inputs.validation_current_a)
        denominator = (2.0 * math.pi * axis.maximum_h
                       * inputs.validation_current_a)
        if numerator <= 0.0:
            raise ValueError("steady resistance voltage consumes voltage reserve")
        voltage_bounds.append(numerator / denominator)
    voltage_bound = min(voltage_bounds)
    current_bound = current_frequency_bound(
        inputs, sample_bound, voltage_limit, max_delay_samples
    )
    upper = min(sample_bound, delay_bound, voltage_bound, current_bound)
    seed = min(0.02 * inputs.sample_rate_hz, 0.5 * upper)
    return {
        "sample_bound_hz": sample_bound,
        "delay_bound_hz": delay_bound,
        "voltage_bound_hz": voltage_bound,
        "current_bound_hz": current_bound,
        "f_upper_hz": upper,
        "f_seed_hz": seed,
        "seed_fallbacks_hz": [seed, 0.5 * seed, 0.25 * seed],
        "voltage_limit_v": voltage_limit,
        "reserved_voltage_limit_v": VOLTAGE_RESERVE_FRACTION * voltage_limit,
        "reserved_current_limit_a": (
            CURRENT_RESERVE_FRACTION * inputs.abort_current_a
        ),
        "max_delay_samples": max_delay_samples,
    }


def crossing(points: list[dict], key: str, target: float) -> tuple[float, float] | None:
    for first, second in zip(points, points[1:]):
        first_value = first[key]
        second_value = second[key]
        if (first_value - target) * (second_value - target) <= 0.0:
            if second_value == first_value:
                fraction = 0.0
            else:
                fraction = (target - first_value) / (second_value - first_value)
            frequency = math.exp(
                math.log(first["frequency_hz"])
                + fraction * (math.log(second["frequency_hz"])
                              - math.log(first["frequency_hz"]))
            )
            return frequency, fraction
    return None


def evaluate_frequency_trace(points: list[tuple[float, complex]],
                             gains: dict) -> dict:
    evaluated = []
    phases = []
    for frequency, plant in points:
        omega = 2.0 * math.pi * frequency
        controller = complex(gains["kp"], -gains["ki"] / omega)
        loop = plant * controller
        sensitivity = 1.0 / (1.0 + loop)
        closed_loop = loop / (1.0 + loop)
        evaluated.append({
            "frequency_hz": frequency,
            "loop_db": magnitude_db(loop),
            "closed_loop_db": magnitude_db(closed_loop),
            "sensitivity": abs(sensitivity),
        })
        phases.append(phase_deg(loop))
    unwrapped = unwrap_degrees(phases)
    for point, value in zip(evaluated, unwrapped):
        point["loop_phase_deg"] = value

    loop_crossing = crossing(evaluated, "loop_db", 0.0)
    if loop_crossing is None:
        phase_margin = None
        crossover = None
        low_frequency_points = evaluated[:2]
    else:
        crossover, fraction = loop_crossing
        low_frequency_points = [
            point for point in evaluated
            if point["frequency_hz"] <= 0.25 * crossover
        ]
        bracket = next(
            (pair for pair in zip(evaluated, evaluated[1:])
             if pair[0]["frequency_hz"] <= crossover <= pair[1]["frequency_hz"]),
            None,
        )
        if bracket is None:
            phase_margin = None
        else:
            phase = (bracket[0]["loop_phase_deg"] + fraction
                     * (bracket[1]["loop_phase_deg"]
                        - bracket[0]["loop_phase_deg"]))
            phase_margin = 180.0 + phase
    low_frequency_valid = len(low_frequency_points) >= 2
    if not low_frequency_valid:
        low_frequency_points = evaluated[:2]
    low_frequency_gain_db = statistics.median(
        point["closed_loop_db"] for point in low_frequency_points
    )
    bandwidth_crossing = crossing(
        evaluated, "closed_loop_db", low_frequency_gain_db - 3.01029995664
    )
    return {
        "loop_crossover_hz": crossover,
        "phase_margin_deg": phase_margin,
        "low_frequency_gain_db": low_frequency_gain_db,
        "low_frequency_point_count": len(low_frequency_points),
        "low_frequency_plateau_valid": low_frequency_valid,
        "closed_loop_bandwidth_hz": (
            None if bandwidth_crossing is None else bandwidth_crossing[0]
        ),
        "max_sensitivity": max(point["sensitivity"] for point in evaluated),
    }


def evaluate_candidate(candidate: dict, axis_traces: dict[str, list[dict]],
                       inputs: MotorInputs, limits: dict) -> dict:
    axes = {}
    candidate_safe = True
    for axis in ("d", "q"):
        trace_results = [
            evaluate_frequency_trace(trace["points"], candidate["axes"][axis])
            for trace in axis_traces[axis]
        ]
        margins = [result["phase_margin_deg"] for result in trace_results
                   if result["phase_margin_deg"] is not None]
        crossovers = [result["loop_crossover_hz"] for result in trace_results
                      if result["loop_crossover_hz"] is not None]
        bandwidths = [result["closed_loop_bandwidth_hz"] for result in trace_results
                      if result["closed_loop_bandwidth_hz"] is not None]
        low_frequency_valid = all(
            result["low_frequency_plateau_valid"] for result in trace_results
        )
        low_frequency_gains = [
            result["low_frequency_gain_db"] for result in trace_results
        ]
        max_sensitivity = max(result["max_sensitivity"] for result in trace_results)
        complete = (len(margins) == len(trace_results)
                    and len(crossovers) == len(trace_results)
                    and len(bandwidths) == len(trace_results)
                    and low_frequency_valid)
        target = candidate["target_crossover_hz"]
        if candidate["method"] == "ideal_rl_pole_cancellation":
            target_consistent = (bool(bandwidths)
                                 and min(bandwidths) >= 0.75 * target
                                 and max(bandwidths) <= 1.25 * target)
            target_metric = "closed_loop_minus_3db_bandwidth"
        else:
            target_consistent = (bool(crossovers)
                                 and min(crossovers) >= 0.85 * target
                                 and max(crossovers) <= 1.15 * target)
            target_metric = "loop_crossover"
        axis_safe = (complete and min(margins) >= MIN_PHASE_MARGIN_DEG
                     and max_sensitivity <= MAX_SENSITIVITY
                     and target_consistent)
        candidate_safe = candidate_safe and axis_safe
        axes[axis] = {
            "trace_count": len(trace_results),
            "complete_within_measured_band": complete,
            "loop_crossover_range_hz": (
                None if not crossovers else [min(crossovers), max(crossovers)]
            ),
            "closed_loop_bandwidth_range_hz": (
                None if not bandwidths else [min(bandwidths), max(bandwidths)]
            ),
            "low_frequency_gain_range_db": [
                min(low_frequency_gains), max(low_frequency_gains)
            ],
            "minimum_phase_margin_deg": None if not margins else min(margins),
            "maximum_sensitivity": max_sensitivity,
            "target_consistency_metric": target_metric,
            "passes_target_consistency": target_consistent,
            "passes_frequency_safety": axis_safe,
        }

    step_worst = {"peak_current_a": 0.0, "peak_voltage_v": 0.0,
                  "overshoot_percent": 0.0}
    sample_period = 1.0 / inputs.sample_rate_hz
    for axis in ("d", "q"):
        for rs, inductance in model_corners(inputs, axis):
            for delay in range(1, limits["max_delay_samples"] + 1):
                result = simulate_step(
                    rs, inductance, candidate["axes"][axis], sample_period,
                    delay, inputs.validation_current_a,
                    limits["voltage_limit_v"],
                )
                for key in step_worst:
                    step_worst[key] = max(step_worst[key], result[key])
    step_safe = (
        step_worst["peak_current_a"] <= limits["reserved_current_limit_a"]
        and step_worst["peak_voltage_v"] <= limits["reserved_voltage_limit_v"]
    )
    candidate_safe = candidate_safe and step_safe
    return {
        "axes": axes,
        "worst_discrete_step_prediction": step_worst,
        "passes_step_safety": step_safe,
        "eligible_for_hardware_coarse_screen": candidate_safe,
    }


def build_candidates(inputs: MotorInputs, limits: dict,
                     nominal_axis_traces: dict[str, list[tuple[float, complex]]],
                     policy: str = "initial") -> list[dict]:
    upper = limits["f_upper_hz"]
    seed = limits["f_seed_hz"]
    high_anchor = min(2.0 * seed, 0.8 * upper)
    candidates = []
    if policy == "d032":
        anchors = ()
    elif policy == "d029":
        anchors = (
            ("rl_seed", seed),
            ("rl_1250", 1250.0),
            ("rl_1500", 1500.0),
            ("rl_1750", 1750.0),
            ("rl_2000", 2000.0),
        )
    elif policy == "refined":
        anchors = (("rl_seed", seed),)
    else:
        anchors = (("rl_seed", seed), ("rl_high", high_anchor))
    for label, frequency in anchors:
        candidates.append({
            "name": label,
            "method": "ideal_rl_pole_cancellation",
            "target_crossover_hz": frequency,
            "target_phase_margin_deg": None,
            "axes": {
                "d": pi_gains(inputs.rs_ohm, inputs.d.nominal_h, frequency),
                "q": pi_gains(inputs.rs_ohm, inputs.q.nominal_h, frequency),
            },
        })

    if policy == "d032":
        measured_grid = (
            (crossover, target_pm)
            for crossover in (850.0, 925.0, 1000.0)
            for target_pm in (80.0, 85.0)
        )
    elif policy == "d029":
        measured_grid = (
            (crossover, 75.0)
            for crossover in (1000.0, 1250.0, 1500.0)
        )
    elif policy == "refined":
        measured_grid = (
            (crossover, target_pm)
            for crossover in (800.0, 900.0, 1000.0)
            for target_pm in (70.0, 75.0)
        )
    elif policy == "initial":
        measured_grid = (
            (fraction * upper, target_pm)
            for fraction in (0.60, 0.75, 0.90)
            for target_pm in (65.0, 75.0)
        )
    else:
        raise ValueError(f"unsupported candidate policy: {policy}")

    for crossover, target_pm in measured_grid:
        candidates.append({
            "name": f"measured_fc{round(crossover):04d}_pm{int(target_pm)}",
            "method": "measured_plant_crossover_phase_margin",
            "target_crossover_hz": crossover,
            "target_phase_margin_deg": target_pm,
            "axes": {
                axis: design_from_plant(
                    nominal_axis_traces[axis], crossover, target_pm
                )
                for axis in ("d", "q")
            },
        })
    return candidates


def candidate_signature(candidate: dict) -> tuple[float, ...]:
    return tuple(
        round(candidate["axes"][axis][gain], 9)
        for axis in ("d", "q") for gain in ("kp", "ki")
    )


def build_replacement(original: dict, target_pm_deg: float,
                      nominal_axis_traces: dict[str, list[tuple[float, complex]]]) -> dict:
    crossover = original["target_crossover_hz"]
    return {
        "name": (f"replacement_fc{round(crossover):04d}_pm{int(target_pm_deg)}"
                 f"_for_{original['name']}"),
        "method": "measured_plant_adjacent_safe_replacement",
        "replacement_for": original["name"],
        "target_crossover_hz": crossover,
        "target_phase_margin_deg": target_pm_deg,
        "axes": {
            axis: design_from_plant(
                nominal_axis_traces[axis], crossover, target_pm_deg
            )
            for axis in ("d", "q")
        },
    }


def build_report(stage1_path: Path, frf_path: Path,
                 excluded_profiles: tuple[str, ...],
                 max_delay_samples: int = 2,
                 policy: str = "initial") -> dict:
    inputs, stage1_document = load_motor_inputs(stage1_path)
    d_traces, frf_document = load_d_axis_traces(frf_path, excluded_profiles)
    limits = compute_frequency_limits(inputs, max_delay_samples)

    nominal_d = median_trace(d_traces)
    nominal_axis_traces = {
        "d": transform_axis_trace(
            nominal_d, inputs.rs_ohm, inputs.d.nominal_h,
            inputs.rs_ohm, inputs.d.nominal_h,
        ),
        "q": transform_axis_trace(
            nominal_d, inputs.rs_ohm, inputs.d.nominal_h,
            inputs.rs_ohm, inputs.q.nominal_h,
        ),
    }
    axis_traces = {
        axis: build_axis_traces(d_traces, inputs, axis)
        for axis in ("d", "q")
    }
    candidates = build_candidates(inputs, limits, nominal_axis_traces, policy)
    for candidate in candidates:
        candidate["offline_evaluation"] = evaluate_candidate(
            candidate, axis_traces, inputs, limits
        )

    selected = [candidate for candidate in candidates
                if candidate["offline_evaluation"]
                ["eligible_for_hardware_coarse_screen"]]
    signatures = {candidate_signature(candidate) for candidate in selected}
    rejected_measured = [
        candidate for candidate in candidates
        if (candidate["method"] == "measured_plant_crossover_phase_margin"
            and not candidate["offline_evaluation"]
            ["eligible_for_hardware_coarse_screen"])
    ]
    for original in rejected_measured if policy == "initial" else ():
        if len(selected) >= 8:
            break
        original_pm = float(original["target_phase_margin_deg"])
        for replacement_pm in (original_pm + 5.0,
                               original_pm + 10.0,
                               original_pm + 15.0):
            replacement = build_replacement(
                original, replacement_pm, nominal_axis_traces
            )
            signature = candidate_signature(replacement)
            if signature in signatures:
                continue
            replacement["offline_evaluation"] = evaluate_candidate(
                replacement, axis_traces, inputs, limits
            )
            candidates.append(replacement)
            if replacement["offline_evaluation"][
                    "eligible_for_hardware_coarse_screen"]:
                selected.append(replacement)
                signatures.add(signature)
                break

    requested_count = (
        6 if policy == "d032" else (7 if policy == "refined" else 8)
    )
    selected = selected[:requested_count]
    selected_names = {candidate["name"] for candidate in selected}
    for candidate in candidates:
        candidate["selected_for_hardware_coarse_screen"] = (
            candidate["name"] in selected_names
        )
    status = "ready_for_hardware_coarse_screen"
    if len(selected) < 6:
        status = "insufficient_safe_candidates"

    return {
        "schema": "stage2_current_autotune_design_v1",
        "method": "host_supervised_no_active_pi_input",
        "sources": {
            "stage1": str(stage1_path),
            "frf": str(frf_path),
            "excluded_profiles": list(excluded_profiles),
            "used_frf_profiles": [trace["name"] for trace in d_traces],
            "source_git": stage1_document.get("source_baseline"),
            "frf_method": frf_document.get("method"),
        },
        "inputs": asdict(inputs),
        "frequency_limits": limits,
        "candidate_policy": {
            "name": policy,
            "requested_count": requested_count,
            "allowed_count": [6, 8],
            "rl_anchor_count": (
                0 if policy == "d032" else
                (5 if policy == "d029" else (1 if policy == "refined" else 2))
            ),
            "measured_design_count": (
                6 if policy == "d032" else (3 if policy == "d029" else 6)
            ),
            "measured_crossover_fractions": (
                [0.60, 0.75, 0.90] if policy == "initial" else None
            ),
            "measured_crossover_hz": (
                [850.0, 925.0, 1000.0] if policy == "d032" else
                ([1000.0, 1250.0, 1500.0] if policy == "d029" else
                ([800.0, 900.0, 1000.0] if policy == "refined" else None)
                )
            ),
            "target_phase_margins_deg": (
                [80.0, 85.0] if policy == "d032" else
                ([75.0] if policy == "d029" else
                ([70.0, 75.0] if policy == "refined" else [65.0, 75.0])
                )
            ),
            "minimum_phase_margin_deg": MIN_PHASE_MARGIN_DEG,
            "maximum_sensitivity": MAX_SENSITIVITY,
        },
        "candidates": candidates,
        "hardware_candidate_names": [candidate["name"] for candidate in selected],
        "hardware_candidate_count": len(selected),
        "status": status,
        "hardware_plan": {
            "requires_24v": True,
            "automatic_flash": False,
            "candidate_pi_is_volatile": True,
            "coarse_step_amplitudes_a": (
                [0.25 * inputs.validation_current_a,
                 0.50 * inputs.validation_current_a,
                 0.75 * inputs.validation_current_a,
                 inputs.validation_current_a]
                if policy in ("refined", "d029", "d032") else
                [0.25 * inputs.validation_current_a,
                 0.50 * inputs.validation_current_a,
                 inputs.validation_current_a]
            ),
            "coarse_step_directions": [-1, 1],
            "coarse_candidate_count": len(selected),
            "dense_finalist_limit": 3,
            "active_pi_usage": "post-selection holdout only",
        },
        "qualification_limit": (
            "q-axis plant is derived from measured d-axis common path and identified Lq; "
            "current output cannot exceed d_axis_qualified_q_axis_model_based"
        ),
    }


def print_report(report: dict) -> None:
    limits = report["frequency_limits"]
    print("current-loop autotuning v1 offline design")
    print(f"  f_upper={limits['f_upper_hz']:.1f} Hz "
          f"f_seed={limits['f_seed_hz']:.1f} Hz")
    print("  FRF profiles:", ", ".join(report["sources"]["used_frf_profiles"]))
    print("  candidates (Kp V/A, Ki V/(A*s))")
    for candidate in report["candidates"]:
        d = candidate["axes"]["d"]
        q = candidate["axes"]["q"]
        evaluation = candidate["offline_evaluation"]
        if candidate["selected_for_hardware_coarse_screen"]:
            state = "SELECT"
        elif evaluation["eligible_for_hardware_coarse_screen"]:
            state = "SAFE"
        else:
            state = "REJECT"
        print(f"    {candidate['name']:28s} {state:8s} "
              f"d=({d['kp']:.6f},{d['ki']:.1f}) "
              f"q=({q['kp']:.6f},{q['ki']:.1f})")
    print(f"  hardware candidates: {report['hardware_candidate_count']}")
    print("  status:", report["status"])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate no-baseline current-loop autotuning candidates"
    )
    parser.add_argument("--stage1", type=Path, default=DEFAULT_STAGE1)
    parser.add_argument("--frf", type=Path, default=DEFAULT_FRF)
    parser.add_argument("--exclude-profile", action="append",
                        default=list(DEFAULT_EXCLUDED_PROFILES))
    parser.add_argument("--max-delay-samples", type=int, default=2)
    parser.add_argument("--policy", choices=("initial", "refined", "d029", "d032"),
                        default="initial")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.max_delay_samples < 1:
        parser.error("--max-delay-samples must be at least 1")

    report = build_report(
        args.stage1, args.frf, tuple(args.exclude_profile),
        args.max_delay_samples, args.policy,
    )
    print_report(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print("  wrote:", args.output)


if __name__ == "__main__":
    main()
