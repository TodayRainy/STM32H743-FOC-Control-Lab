import argparse
import csv
import glob
import json
import math
import os
import statistics
from pathlib import Path


RS_OHM = 0.144818365
LD_H = 26.043582e-6
LQ_H = 40.343765e-6
FLUX_WB = 0.90795034e-3
POLE_PAIRS = 10
BUS_VOLTAGE_V = 24.0
TOTAL_CURRENT_LIMIT_A = 5.0
TARGET_UTILIZATION = 0.95
FW_BANDWIDTH_HZ = 50.0
CURRENT_BANDWIDTH_HZ = 925.0
IMPLEMENTED_KP_A_PER_V = 0.4
IMPLEMENTED_KI_A_PER_V_S = 2400.0
VOLTAGE_FILTER_HZ = 200.0
SPEED_LOOP_HZ = 4000.0
ID_REFERENCE_RATE_A_PER_S = 500.0

OPERATING_PHASES = (
    {"name": "main", "speed_rpm": 5000.0, "voltage_limit_ratio": 0.35},
    {"name": "extension", "speed_rpm": 6500.0, "voltage_limit_ratio": 0.45},
)


def electrical_speed(speed_rpm):
    return speed_rpm*2.0*math.pi/60.0*POLE_PAIRS


def voltage_target(voltage_limit_ratio):
    return (BUS_VOLTAGE_V/math.sqrt(3.0)
            * voltage_limit_ratio*TARGET_UTILIZATION)


def motor_voltage(speed_rpm, iq_a, id_a):
    omega = electrical_speed(speed_rpm)
    ud = RS_OHM*id_a - omega*LQ_H*iq_a
    uq = RS_OHM*iq_a + omega*(LD_H*id_a + FLUX_WB)
    return ud, uq, math.hypot(ud, uq)


def model_id(speed_rpm, iq_a, target_v):
    omega = electrical_speed(speed_rpm)
    uq_zero = RS_OHM*iq_a + omega*FLUX_WB
    ud_zero = -omega*LQ_H*iq_a
    a = RS_OHM**2 + (omega*LD_H)**2
    b = 2.0*(RS_OHM*ud_zero + omega*LD_H*uq_zero)
    c = ud_zero**2 + uq_zero**2 - target_v**2
    if c <= 0.0:
        return 0.0
    discriminant = b*b-4.0*a*c
    if discriminant <= 0.0:
        return min(0.0, -b/(2.0*a))
    span = math.sqrt(discriminant)/(2.0*a)
    center = -b/(2.0*a)
    roots = [value for value in (center+span, center-span) if value <= 0.0]
    return max(roots) if roots else 0.0


def voltage_sensitivity(speed_rpm, iq_a, id_a):
    omega = electrical_speed(speed_rpm)
    ud, uq, magnitude = motor_voltage(speed_rpm, iq_a, id_a)
    return (ud*RS_OHM + uq*omega*LD_H)/magnitude


def designed_pi():
    phase = OPERATING_PHASES[0]
    target_v = voltage_target(phase["voltage_limit_ratio"])
    nominal_iq = 0.5
    nominal_id = model_id(phase["speed_rpm"], nominal_iq, target_v)
    plant_gain = voltage_sensitivity(
        phase["speed_rpm"], nominal_iq, nominal_id
    )
    fw_omega = 2.0*math.pi*FW_BANDWIDTH_HZ
    current_omega = 2.0*math.pi*CURRENT_BANDWIDTH_HZ
    return {
        "nominal_speed_rpm": phase["speed_rpm"],
        "nominal_iq_a": nominal_iq,
        "nominal_id_a": nominal_id,
        "voltage_sensitivity_v_per_a": plant_gain,
        "derived_kp_a_per_v": fw_omega/(plant_gain*current_omega),
        "derived_ki_a_per_v_s": fw_omega/plant_gain,
        "implemented_kp_a_per_v": IMPLEMENTED_KP_A_PER_V,
        "implemented_ki_a_per_v_s": IMPLEMENTED_KI_A_PER_V_S,
    }


def d058_voltage_noise(repo_root):
    pattern = os.path.join(
        repo_root, "tests", "results",
        "stage3_voltage_saturation_20260827_232614_scope", "*.csv"
    )
    file_stats = []
    for path in sorted(glob.glob(pattern)):
        values = []
        with open(path, newline="") as stream:
            for row in csv.DictReader(stream):
                time_s = float(row["time_s"])
                if 0.02 <= time_s < 0.08:
                    scale = max(float(row["voltage_scale"]), 1.0e-9)
                    values.append(
                        math.hypot(float(row["uq"]), float(row["ud"]))/scale
                    )
        if values:
            file_stats.append({
                "file": os.path.basename(path),
                "mean_v": statistics.mean(values),
                "standard_deviation_v": statistics.pstdev(values),
            })
    return {
        "source": "D-058 4 kHz pre-pulse Scope",
        "file_count": len(file_stats),
        "mean_standard_deviation_v": (
            statistics.mean(item["standard_deviation_v"] for item in file_stats)
            if file_stats else None
        ),
        "files": file_stats,
    }


def operating_points():
    phases = []
    for phase in OPERATING_PHASES:
        target_v = voltage_target(phase["voltage_limit_ratio"])
        points = []
        for iq_a in (0.0, 0.5, 1.0, 2.0, 3.0):
            id_a = model_id(phase["speed_rpm"], iq_a, target_v)
            points.append({
                "iq_a": iq_a,
                "id_a": id_a,
                "current_magnitude_a": math.hypot(iq_a, id_a),
                "feasible_in_5a_circle": (
                    math.hypot(iq_a, id_a) <= TOTAL_CURRENT_LIMIT_A
                ),
            })
        phases.append({
            **phase,
            "target_voltage_v": target_v,
            "points": points,
        })
    return phases


def build_report(repo_root):
    return {
        "decision": "D-061",
        "controller": {
            "baseline": "stator-voltage feedback PI",
            "hybrid": "IDENTIFIED voltage-ellipse feedforward plus feedback trim",
            "modes": ["OFF", "FEEDBACK", "HYBRID"],
            "total_dq_current_limit_a": TOTAL_CURRENT_LIMIT_A,
            "target_utilization": TARGET_UTILIZATION,
            "bandwidth_hz": FW_BANDWIDTH_HZ,
            "voltage_filter_hz": VOLTAGE_FILTER_HZ,
            "id_reference_rate_a_per_s": ID_REFERENCE_RATE_A_PER_S,
            "voltage_filter_alpha_at_4khz": (
                1.0-math.exp(-2.0*math.pi*VOLTAGE_FILTER_HZ/SPEED_LOOP_HZ)
            ),
        },
        "pi_design": designed_pi(),
        "operating_phases": operating_points(),
        "historical_voltage_noise": d058_voltage_noise(repo_root),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Stage-4 field-weakening design calculation"
    )
    parser.add_argument("--write-baseline", action="store_true")
    args = parser.parse_args()
    repo_root = str(Path(__file__).resolve().parents[1])
    report = build_report(repo_root)
    if args.write_baseline:
        output = (Path(repo_root)/"tests"/"baselines"/
                  "stage4_field_weakening_design_d061_20260828.json")
        output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        print(output)
    else:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
