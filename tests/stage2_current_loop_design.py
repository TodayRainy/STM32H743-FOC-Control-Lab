#!/usr/bin/env python3
"""Calculate and offline-check Stage-2 d/q current PI candidates."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


TS_S = 25.0e-6
CURRENT_STEP_A = 2.0
STEP_TIME_S = 0.5e-3
SIMULATION_TIME_S = 8.0e-3
PI_VOLTAGE_LIMIT_V = 15.0
BUS_VOLTAGE_V = 24.0
SVPWM_VOLTAGE_LIMIT_V = BUS_VOLTAGE_V / math.sqrt(3.0)

IDENTIFIED = {
    "rs_ohm": 0.144818365,
    "ld_h": 26.043582e-6,
    "lq_h": 40.343765e-6,
}
LEGACY = {
    "rs_ohm": 0.10102932,
    "ld_h": 34.98624e-6,
    "lq_h": 53.17701e-6,
    "ls_h": 44.08162e-6,
}
DEFAULT_BANDWIDTHS_HZ = (1000.0, 1500.0, 2000.0)
DELAYS_SAMPLES = (1, 2)


@dataclass(frozen=True)
class Metrics:
    rise_time_us: float | None
    settling_time_us: float | None
    overshoot_percent: float
    steady_error_ma: float
    peak_voltage_v: float


def pi_gains(resistance_ohm: float, inductance_h: float,
             bandwidth_hz: float) -> tuple[float, float]:
    omega_c = 2.0 * math.pi * bandwidth_hz
    return omega_c * inductance_h, omega_c * resistance_ohm


def bandwidth_profile_name(bandwidth_hz: float) -> str:
    if not math.isfinite(bandwidth_hz) or bandwidth_hz <= 0.0:
        raise ValueError("bandwidth must be finite and positive")
    if float(bandwidth_hz).is_integer():
        label = str(int(bandwidth_hz))
    else:
        label = f"{bandwidth_hz:g}".replace(".", "p")
    return f"identified_{label}hz"


def build_controllers(bandwidths_hz=DEFAULT_BANDWIDTHS_HZ) -> dict:
    legacy_kp, legacy_ki = pi_gains(LEGACY["rs_ohm"], LEGACY["ls_h"], 2000.0)
    controllers: dict[str, dict[str, dict[str, float]]] = {
        "active_legacy": {
            "d": {"kp": legacy_kp, "ki": legacy_ki},
            "q": {"kp": legacy_kp, "ki": legacy_ki},
        }
    }
    for bandwidth_hz in bandwidths_hz:
        name = bandwidth_profile_name(bandwidth_hz)
        if name in controllers:
            raise ValueError(f"duplicate bandwidth candidate: {bandwidth_hz:g} Hz")
        controllers[name] = {}
        for axis in ("d", "q"):
            kp, ki = pi_gains(IDENTIFIED["rs_ohm"],
                              IDENTIFIED[f"l{axis}_h"], bandwidth_hz)
            controllers[name][axis] = {"kp": kp, "ki": ki}
    return controllers


def first_crossing(times: list[float], values: list[float], threshold: float) -> float | None:
    for time_s, value in zip(times, values):
        if value >= threshold:
            return time_s
    return None


def simulate(resistance_ohm: float, inductance_h: float,
             kp: float, ki: float, delay_samples: int) -> Metrics:
    decay = math.exp(-resistance_ohm * TS_S / inductance_h)
    voltage_gain = (1.0 - decay) / resistance_ohm
    delay_line = [0.0] * delay_samples
    current_a = 0.0
    integrator_v = 0.0
    step_times: list[float] = []
    step_currents: list[float] = []
    peak_voltage_v = 0.0

    sample_count = round(SIMULATION_TIME_S / TS_S)
    for sample in range(sample_count):
        time_s = sample * TS_S
        reference_a = CURRENT_STEP_A if time_s >= STEP_TIME_S else 0.0
        error_a = reference_a - current_a

        integrator_v += ki * error_a * TS_S
        requested_v = kp * error_a + integrator_v
        pi_output_v = max(-PI_VOLTAGE_LIMIT_V,
                          min(PI_VOLTAGE_LIMIT_V, requested_v))
        integrator_v += pi_output_v - requested_v

        applied_request_v = max(-SVPWM_VOLTAGE_LIMIT_V,
                                min(SVPWM_VOLTAGE_LIMIT_V, pi_output_v))
        integrator_v += applied_request_v - pi_output_v
        peak_voltage_v = max(peak_voltage_v, abs(applied_request_v))

        delay_line.append(applied_request_v)
        applied_v = delay_line.pop(0)
        current_a = decay * current_a + voltage_gain * applied_v

        if time_s >= STEP_TIME_S:
            step_times.append(time_s - STEP_TIME_S)
            step_currents.append(current_a)

    t10 = first_crossing(step_times, step_currents, 0.1 * CURRENT_STEP_A)
    t90 = first_crossing(step_times, step_currents, 0.9 * CURRENT_STEP_A)
    rise_s = None if (t10 is None or t90 is None) else t90 - t10

    tolerance_a = 0.02 * CURRENT_STEP_A
    settling_s = None
    for index, time_s in enumerate(step_times):
        if all(abs(value - CURRENT_STEP_A) <= tolerance_a
               for value in step_currents[index:]):
            settling_s = time_s
            break

    overshoot = max(0.0, (max(step_currents) / CURRENT_STEP_A - 1.0) * 100.0)
    return Metrics(
        rise_time_us=None if rise_s is None else rise_s * 1.0e6,
        settling_time_us=None if settling_s is None else settling_s * 1.0e6,
        overshoot_percent=overshoot,
        steady_error_ma=(CURRENT_STEP_A - step_currents[-1]) * 1000.0,
        peak_voltage_v=peak_voltage_v,
    )


def build_report(bandwidths_hz=DEFAULT_BANDWIDTHS_HZ) -> dict:
    bandwidths_hz = tuple(bandwidths_hz)
    controllers = build_controllers(bandwidths_hz)

    cases = []
    plants = {"identified": IDENTIFIED, "legacy": LEGACY}
    for controller_name, axes in controllers.items():
        for plant_name, plant in plants.items():
            for axis, gains in axes.items():
                for delay_samples in DELAYS_SAMPLES:
                    metrics = simulate(plant["rs_ohm"], plant[f"l{axis}_h"],
                                       gains["kp"], gains["ki"], delay_samples)
                    cases.append({
                        "controller": controller_name,
                        "plant": plant_name,
                        "axis": axis,
                        "delay_samples": delay_samples,
                        "metrics": asdict(metrics),
                    })

    phase_margin_bounds = {
        str(int(bandwidth_hz)): 90.0
        - 360.0 * bandwidth_hz * max(DELAYS_SAMPLES) * TS_S
        for bandwidth_hz in bandwidths_hz
    }
    return {
        "method": "parallel PI Kp=2*pi*fc*L, Ki=2*pi*fc*Rs",
        "assumptions": {
            "sample_time_s": TS_S,
            "current_step_a": CURRENT_STEP_A,
            "bus_voltage_v": BUS_VOLTAGE_V,
            "svpwm_axis_voltage_limit_v": SVPWM_VOLTAGE_LIMIT_V,
            "delay_samples_swept": list(DELAYS_SAMPLES),
            "plant": "exact zero-order-held R-L axis model; no cross-coupling or sensor noise",
        },
        "models": {"identified": IDENTIFIED, "legacy": LEGACY},
        "controllers": controllers,
        "two_sample_delay_phase_margin_bound_deg": phase_margin_bounds,
        "cases": cases,
        "recommendation": {
            "selection_rule": "Choose the highest measured bandwidth that passes the predefined transient, noise, symmetry, repeatability, voltage-margin and stability constraints.",
            "status": "offline candidates only; retain active_legacy until hardware validation",
        },
    }


def format_optional(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    parser.add_argument("--bandwidths", type=float, nargs="+",
                        default=DEFAULT_BANDWIDTHS_HZ, metavar="HZ")
    args = parser.parse_args()
    report = build_report(args.bandwidths)

    print("controller gains (Kp V/A, Ki V/(A*s))")
    for name, axes in report["controllers"].items():
        print(f"  {name:20s} d=({axes['d']['kp']:.6f}, {axes['d']['ki']:.3f}) "
              f"q=({axes['q']['kp']:.6f}, {axes['q']['ki']:.3f})")

    print("\nworst case across both parameter models, d/q and 1/2-sample delays")
    for name in report["controllers"]:
        cases = [case for case in report["cases"]
                 if case["controller"] == name]
        worst_rise = max(case["metrics"]["rise_time_us"] or math.inf for case in cases)
        worst_settle = max(case["metrics"]["settling_time_us"] or math.inf for case in cases)
        worst_overshoot = max(case["metrics"]["overshoot_percent"] for case in cases)
        peak_voltage = max(case["metrics"]["peak_voltage_v"] for case in cases)
        print(f"  {name:20s} rise={format_optional(worst_rise)} us "
              f"settle={format_optional(worst_settle)} us "
              f"overshoot={worst_overshoot:.2f}% peakV={peak_voltage:.3f}")

    print("\nselection rule:", report["recommendation"]["selection_rule"])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("wrote", args.output)


if __name__ == "__main__":
    main()
