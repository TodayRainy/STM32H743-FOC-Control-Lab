#!/usr/bin/env python3
"""Run the guarded P scan and six-candidate speed-PI screen."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from statistics import median

import serial

import motor_baseline as proto
import scope_capture as scope
from scope_current_step import wait_mode
from stage2_speed_autotune import (
    DEFAULT_SPEC,
    build_p_scan,
    build_pi_candidates,
    load_spec,
)


MODE_SPEED_TORQUE_RUN = 12
MODE_READY = 17
SCOPE_DIVIDER = 10
SCOPE_DURATION_SEC = 4096 * SCOPE_DIVIDER / 40000.0


def electrical_rad_s_to_rpm(value: float, pole_pairs: int) -> float:
    return value / pole_pairs * 60.0 / (2.0 * math.pi)


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def median_filter(values: list[float], width: int = 5) -> list[float]:
    half = width // 2
    return [
        median(values[max(0, index - half):index + half + 1])
        for index in range(len(values))
    ]


def crossing_time(values: list[float], start: int, threshold: float,
                  direction: float, sample_period: float) -> float | None:
    directed_threshold = direction * threshold
    for index in range(start, len(values)):
        current = direction * values[index]
        if current < directed_threshold:
            continue
        if index == start:
            return 0.0
        previous = direction * values[index - 1]
        fraction = 1.0
        if current != previous:
            fraction = (directed_threshold - previous) / (current - previous)
            fraction = max(0.0, min(1.0, fraction))
        return (index - 1 + fraction - start) * sample_period
    return None


def longest_limit_duration(values: list[float], limit: float,
                           sample_period: float) -> float:
    longest = 0
    current = 0
    for value in values:
        if abs(value) >= 0.99 * limit:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest * sample_period


def summarize_speed_step(samples: list[tuple[float, ...]], sample_period: float,
                         pole_pairs: int, speed_noise_rpm: float,
                         iq_limit_a: float) -> dict:
    if len(samples) < 20 or len(samples[0]) < 11:
        raise ValueError("speed step requires the 11-channel scope layout")

    speed = [electrical_rad_s_to_rpm(row[9], pole_pairs) for row in samples]
    reference = [electrical_rad_s_to_rpm(row[10], pole_pairs) for row in samples]
    iq_ref = [row[4] for row in samples]
    actual_current = [math.hypot(row[2], row[3]) for row in samples]
    filtered = median_filter(speed)

    differences = [abs(reference[index] - reference[index - 1])
                   for index in range(1, len(reference))]
    step_index = 1 + max(range(len(differences)), key=differences.__getitem__)
    baseline_count = max(1, step_index)
    baseline_reference = sum(reference[:baseline_count]) / baseline_count
    baseline_window = filtered[max(0, step_index - 400):step_index]
    baseline = sum(baseline_window) / len(baseline_window)
    baseline_noise_rms = rms([
        value - baseline for value in baseline_window
    ])
    tail_count = max(10, len(reference) // 8)
    target = sum(reference[-tail_count:]) / tail_count
    amplitude = target - baseline
    command_amplitude = target - baseline_reference
    if abs(command_amplitude) < 20.0:
        raise ValueError("scope does not contain a usable speed-reference step")

    direction = 1.0 if amplitude > 0.0 else -1.0
    t10 = crossing_time(
        filtered, step_index, baseline + 0.1 * amplitude,
        direction, sample_period,
    )
    t63 = crossing_time(
        filtered, step_index, baseline + 0.6321205588 * amplitude,
        direction, sample_period,
    )
    t90 = crossing_time(
        filtered, step_index, baseline + 0.9 * amplitude,
        direction, sample_period,
    )
    rise_time = None if t10 is None or t90 is None else max(0.0, t90 - t10)

    steady = speed[-tail_count:]
    steady_mean = sum(steady) / len(steady)
    steady_noise_rms = rms([value - steady_mean for value in steady])
    effective_speed_noise = max(
        speed_noise_rpm, baseline_noise_rms, steady_noise_rms,
    )

    directed_response = [direction * (value - baseline)
                         for value in filtered[step_index:]]
    peak = max(directed_response)
    raw_overshoot = max(
        0.0, (peak - abs(amplitude)) / abs(amplitude) * 100.0,
    )
    noise_allowance_pct = 3.0 * effective_speed_noise / abs(amplitude) * 100.0
    noise_adjusted_overshoot = max(0.0, raw_overshoot - noise_allowance_pct)
    band = max(10.0, 0.02 * abs(target), 3.0 * effective_speed_noise)
    hold_samples = max(1, round(0.05 / sample_period))
    settling_time = None
    for index in range(step_index, len(filtered) - hold_samples + 1):
        if all(abs(value - target) <= band
               for value in filtered[index:index + hold_samples]):
            settling_time = (index - step_index) * sample_period
            break

    post = speed[step_index:]
    half = len(post) // 2
    first = post[:half]
    second = post[half:]
    first_mean = sum(first) / len(first)
    second_mean = sum(second) / len(second)
    first_ac_rms = rms([value - first_mean for value in first])
    second_ac_rms = rms([value - second_mean for value in second])
    nondecaying = (
        second_ac_rms > band
        and second_ac_rms >= 0.9 * max(first_ac_rms, 1.0e-6)
    )

    return {
        "baseline_rpm": baseline_reference,
        "baseline_actual_rpm": baseline,
        "baseline_speed_noise_rms_rpm": baseline_noise_rms,
        "steady_speed_ripple_rms_rpm": steady_noise_rms,
        "effective_speed_noise_rms_rpm": effective_speed_noise,
        "target_rpm": target,
        "step_amplitude_rpm": command_amplitude,
        "response_amplitude_rpm": amplitude,
        "rise_time_10_90_sec": rise_time,
        "tau_63_sec": t63,
        "time_to_90_sec": t90,
        "filtered_overshoot_pct": raw_overshoot,
        "noise_adjusted_overshoot_pct": noise_adjusted_overshoot,
        "settling_band_rpm": band,
        "settling_time_sec": settling_time,
        "steady_mean_rpm": steady_mean,
        "steady_error_rpm": target - steady_mean,
        "speed_error_rms_rpm": rms([
            ref - value for ref, value in zip(reference, speed)
        ][step_index:]),
        "peak_iq_ref_a": max(abs(value) for value in iq_ref),
        "rms_iq_ref_a": rms(iq_ref[step_index:]),
        "peak_actual_dq_current_a": max(actual_current),
        "iq_limit_duration_sec": longest_limit_duration(
            iq_ref, iq_limit_a, sample_period
        ),
        "nondecaying_oscillation": nondecaying,
    }


def summarize_noise(samples: list[tuple[float, ...]], pole_pairs: int) -> dict:
    speed = [electrical_rad_s_to_rpm(row[9], pole_pairs) for row in samples]
    mean = sum(speed) / len(speed)
    return {
        "mean_rpm": mean,
        "rms_rpm": rms(speed),
        "noise_rms_rpm": rms([value - mean for value in speed]),
        "peak_abs_rpm": max(abs(value) for value in speed),
    }


def step_passes(record: dict, spec: dict, require_pi_performance: bool) -> bool:
    safety = spec["safety"]
    if record["nondecaying_oscillation"]:
        return False
    if record["peak_actual_dq_current_a"] >= safety["abort_actual_dq_current_a"]:
        return False
    if record["iq_limit_duration_sec"] > safety["maximum_continuous_iq_limit_sec"]:
        return False
    if not require_pi_performance:
        return (
            record["tau_63_sec"] is not None
            and record["filtered_overshoot_pct"]
            <= spec["acceptance"]["maximum_p_scan_overshoot_pct"]
        )

    acceptance = spec["acceptance"]
    return (
        record["noise_adjusted_overshoot_pct"]
        <= acceptance["maximum_noise_adjusted_overshoot_pct"]
        and record["time_to_90_sec"] is not None
        and record["time_to_90_sec"] <= acceptance["maximum_time_to_90_sec"]
        and abs(record["steady_error_rpm"]) <= record["settling_band_rpm"]
        and record["steady_speed_ripple_rms_rpm"]
        <= max(
            acceptance["minimum_steady_ripple_limit_rpm"],
            acceptance["maximum_steady_ripple_fraction"]
            * abs(record["target_rpm"]),
        )
    )


def p_scan_requires_stop(record: dict, spec: dict) -> bool:
    safety = spec["safety"]
    return (
        record["nondecaying_oscillation"]
        or record["peak_actual_dq_current_a"]
        >= safety["abort_actual_dq_current_a"]
        or record["iq_limit_duration_sec"]
        > safety["maximum_continuous_iq_limit_sec"]
    )


def relative_difference(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), abs(b), 1.0e-12)


def summarize_candidate(records: list[dict], spec: dict) -> dict:
    valid = all(step_passes(item, spec, True) for item in records)
    settling = [item["settling_time_sec"] for item in records
                if item["settling_time_sec"] is not None]
    by_round: dict[int, list[dict]] = {}
    for item in records:
        by_round.setdefault(item["round"], []).append(item)

    round_worst = [max(item["settling_time_sec"] for item in items)
                   for items in by_round.values()
                   if all(item["settling_time_sec"] is not None for item in items)]
    repeat_difference = (
        relative_difference(round_worst[0], round_worst[1])
        if len(round_worst) == 2 else math.inf
    )

    positive = [item["settling_time_sec"] for item in records
                if item["step_amplitude_rpm"] > 0.0
                and item["settling_time_sec"] is not None]
    negative = [item["settling_time_sec"] for item in records
                if item["step_amplitude_rpm"] < 0.0
                and item["settling_time_sec"] is not None]
    direction_difference = (
        relative_difference(sum(positive) / len(positive),
                            sum(negative) / len(negative))
        if positive and negative else math.inf
    )
    return {
        "passed": valid,
        "worst_settling_time_sec": max(settling) if settling else None,
        "worst_time_to_90_sec": max(
            item["time_to_90_sec"] for item in records
            if item["time_to_90_sec"] is not None
        ),
        "maximum_overshoot_pct": max(
            item["filtered_overshoot_pct"] for item in records
        ),
        "maximum_noise_adjusted_overshoot_pct": max(
            item["noise_adjusted_overshoot_pct"] for item in records
        ),
        "maximum_steady_speed_ripple_rms_rpm": max(
            item["steady_speed_ripple_rms_rpm"] for item in records
        ),
        "direction_difference": direction_difference,
        "repeat_difference": repeat_difference,
        "peak_iq_ref_a": max(item["peak_iq_ref_a"] for item in records),
        "mean_rms_iq_ref_a": sum(item["rms_iq_ref_a"] for item in records)
        / len(records),
    }


def check_telemetry(frame: tuple[float, ...], spec: dict) -> None:
    safety = spec["safety"]
    actual_current = math.hypot(frame[6], frame[7])
    rpm = abs(proto.mechanical_rpm(frame))
    vbus = frame[29]
    voltage_limit = vbus / math.sqrt(3.0) if vbus > 1.0 else 0.0
    utilization = math.hypot(frame[20], frame[21]) / voltage_limit if voltage_limit else 0.0
    if actual_current >= safety["abort_actual_dq_current_a"]:
        raise RuntimeError(f"actual dq current abort: {actual_current:.3f} A")
    if rpm >= safety["abort_mechanical_speed_rpm"]:
        raise RuntimeError(f"mechanical speed abort: {rpm:.1f} rpm")
    if utilization >= safety["abort_voltage_utilization"]:
        raise RuntimeError(f"voltage utilization abort: {utilization:.3f}")


def supervised_wait(port, buffer: bytearray, duration: float,
                    spec: dict, required_mode: int) -> None:
    deadline = time.monotonic() + duration
    received = 0
    while time.monotonic() < deadline:
        sample = proto.read_frame(
            port, buffer, min(deadline, time.monotonic() + 0.25)
        )
        if sample is None:
            continue
        frame = sample[1]
        received += 1
        check_telemetry(frame, spec)
        if int(round(frame[27])) != required_mode:
            raise RuntimeError("motor left the required speed-control mode")
    if received == 0:
        raise RuntimeError("no telemetry received during supervised wait")


def open_serial_with_retry(port_name: str, baudrate: int,
                           timeout_sec: float = 10.0):
    deadline = time.monotonic() + timeout_sec
    last_error = None
    while time.monotonic() < deadline:
        try:
            return serial.Serial(
                port_name, baudrate, timeout=0.02, write_timeout=0.2
            )
        except serial.SerialException as error:
            last_error = error
            time.sleep(0.25)
    raise RuntimeError(f"serial port did not recover: {last_error}")


def stop_and_confirm_ready(port, port_name: str, baudrate: int,
                           buffer: bytearray) -> int | None:
    try:
        proto.send_command(port, "SPEED SINE OFF")
        proto.send_command(port, "SPEED 0")
        proto.send_command(port, "STOP")
        final = wait_mode(port, buffer, MODE_READY, 3.0, 5.0, 4800.0)
        return int(round(final[27]))
    except (serial.SerialException, RuntimeError):
        try:
            port.close()
        except serial.SerialException:
            pass
        buffer.clear()
        try:
            with open_serial_with_retry(port_name, baudrate) as recovery_port:
                recovery_port.reset_input_buffer()
                proto.send_command(recovery_port, "STOP")
                final = wait_mode(
                    recovery_port, buffer, MODE_READY, 5.0, 5.0, 4800.0
                )
                return int(round(final[27]))
        except (serial.SerialException, RuntimeError):
            return None


def capture_step(port, buffer: bytearray, base_rpm: float, target_rpm: float,
                 spec: dict, speed_noise_rpm: float, path: Path) -> dict:
    proto.send_command(
        port,
        f"TRACK RAMP {spec['screen']['operating_point_ramp_rpm_per_sec']:g}",
    )
    proto.send_command(port, f"SPEED {base_rpm:g}")
    supervised_wait(
        port, buffer, spec["screen"].get("operating_point_settle_sec", 1.5),
        spec, MODE_SPEED_TORQUE_RUN,
    )
    proto.send_command(port, "TRACK STEP")
    scope.arm_scope(port, SCOPE_DIVIDER)
    supervised_wait(port, buffer, 0.10, spec, MODE_SPEED_TORQUE_RUN)
    proto.send_command(port, f"SPEED {target_rpm:g}")
    supervised_wait(
        port, buffer, SCOPE_DURATION_SEC + 0.08, spec, MODE_SPEED_TORQUE_RUN
    )
    capture, samples = scope.dump_scope(port, 25.0)
    sample_period = capture["divider"] / capture["base_sample_hz"]
    scope.write_csv(path, samples, sample_period)
    summary = summarize_speed_step(
        samples, sample_period, int(spec["control"]["pole_pairs"]),
        speed_noise_rpm, float(spec["safety"]["speed_iq_reference_limit_a"]),
    )
    summary["scope_csv"] = str(path.resolve())
    return summary


def plan(spec: dict) -> dict:
    p_candidates = build_p_scan(spec)
    return {
        "schema": "stage2-speed-autotune-screen-plan-v1",
        "p_candidates": p_candidates,
        "p_scope_count": len(p_candidates) * len(spec["p_scan"]["step_rpm"]),
        "pi_candidate_count": spec["pi_candidates"]["candidate_count"],
        "pi_scope_count": (
            spec["pi_candidates"]["candidate_count"]
            * len(spec["screen"]["step_transitions_rpm"])
            * spec["screen"]["rounds"]
        ),
        "requires_24v": True,
        "automatic_flash": False,
        "final_stop_and_ready": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the Stage-2 guarded speed-loop P/PI screen"
    )
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scope-dir", type=Path)
    parser.add_argument("--fixed-kp", type=float)
    parser.add_argument("--fixed-ki", type=float)
    parser.add_argument(
        "--feedforward", choices=("off", "identified"), default="off"
    )
    args = parser.parse_args()

    spec = load_spec(args.spec)
    if args.plan_only:
        print(json.dumps(plan(spec), indent=2))
        return
    if not args.port:
        parser.error("--port is required unless --plan-only is used")
    if (args.fixed_kp is None) != (args.fixed_ki is None):
        parser.error("--fixed-kp and --fixed-ki must be supplied together")
    if args.fixed_kp is not None and (
        args.fixed_kp < 0.0 or args.fixed_ki < 0.0
        or not math.isfinite(args.fixed_kp) or not math.isfinite(args.fixed_ki)
    ):
        parser.error("fixed PI gains must be finite and non-negative")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path("tests/results") / f"stage2_speed_screen_{stamp}.json"
    scope_dir = args.scope_dir or Path("tests/results") / f"stage2_speed_screen_{stamp}_scope"
    scope_dir.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    result = {
        "schema": "stage2-speed-autotune-screen-result-v1",
        "spec": str(args.spec),
        "p_records": [],
        "pi_records": [],
        "status": "running",
        "final_mode": None,
        "feedforward": args.feedforward,
    }

    with open_serial_with_retry(args.port, args.baudrate) as setup_port:
        setup_port.reset_input_buffer()
        wait_mode(setup_port, buffer, MODE_READY, 20.0, 5.0, 4800.0)
        proto.send_command(
            setup_port,
            f"SPEED LIMIT {spec['safety']['speed_iq_reference_limit_a']:g}",
        )
        if args.feedforward == "identified":
            proto.send_command(setup_port, "FF MODEL IDENTIFIED")
            proto.send_command(setup_port, "FF ON")
        else:
            proto.send_command(setup_port, "FF OFF")
        proto.send_command(setup_port, "TRACK STEP")
        proto.send_command(setup_port, "PI SPEED 0 0")
        proto.send_command(setup_port, "PI CLEAR")
        proto.send_command(setup_port, "START SPEED")

    time.sleep(1.0)
    buffer.clear()
    with open_serial_with_retry(args.port, args.baudrate) as port:
        port.reset_input_buffer()
        try:
            running = wait_mode(
                port, buffer, MODE_SPEED_TORQUE_RUN, 20.0, 5.0, 4800.0
            )
            if not 18.0 <= running[29] <= 28.0:
                raise RuntimeError(
                    f"running bus voltage out of range: {running[29]:.3f} V"
                )

            proto.send_command(port, "SPEED 0")
            supervised_wait(port, buffer, 0.4, spec, MODE_SPEED_TORQUE_RUN)
            scope.arm_scope(port, SCOPE_DIVIDER)
            supervised_wait(
                port, buffer, SCOPE_DURATION_SEC + 0.08,
                spec, MODE_SPEED_TORQUE_RUN,
            )
            capture, samples = scope.dump_scope(port, 25.0)
            noise_path = scope_dir / "speed_noise.csv"
            scope.write_csv(
                noise_path, samples,
                capture["divider"] / capture["base_sample_hz"],
            )
            noise = summarize_noise(samples, int(spec["control"]["pole_pairs"]))
            noise["scope_csv"] = str(noise_path.resolve())
            result["speed_noise"] = noise

            valid_p = []
            for candidate in build_p_scan(spec):
                proto.send_command(
                    port,
                    f"TRACK RAMP {spec['screen']['operating_point_ramp_rpm_per_sec']:g}",
                )
                proto.send_command(port, "SPEED 0")
                supervised_wait(port, buffer, 1.0, spec, MODE_SPEED_TORQUE_RUN)
                proto.send_command(port, f"PI SPEED {candidate['kp']:.9g} 0")
                proto.send_command(port, "PI CLEAR")
                records = []
                for target in spec["p_scan"]["step_rpm"]:
                    path = scope_dir / f"{candidate['name']}_{target:+g}rpm.csv"
                    record = capture_step(
                        port, buffer, 0.0, target, spec,
                        noise["noise_rms_rpm"], path,
                    )
                    record.update({"profile": candidate["name"], "round": 0})
                    records.append(record)
                    result["p_records"].append(record)
                if all(step_passes(item, spec, False) for item in records):
                    valid_p.append((candidate, records))
                if any(p_scan_requires_stop(item, spec) for item in records):
                    break

            if not valid_p:
                result["status"] = "seed_failed"
            else:
                base, base_records = min(
                    valid_p,
                    key=lambda item: max(
                        record["tau_63_sec"] for record in item[1]
                    ),
                )
                tau = median(record["tau_63_sec"] for record in base_records)
                result["kp_base"] = base["kp"]
                result["tau_63_sec"] = tau
                if args.fixed_kp is None:
                    candidates = build_pi_candidates(spec, base["kp"], tau)
                else:
                    candidates = [{
                        "name": "diagnostic_fixed_pi",
                        "kp": args.fixed_kp,
                        "ki": args.fixed_ki,
                        "diagnostic_only": True,
                    }]
                for round_index in range(spec["screen"]["rounds"]):
                    ordered = candidates if round_index == 0 else list(reversed(candidates))
                    for candidate in ordered:
                        proto.send_command(
                            port,
                            f"TRACK RAMP {spec['screen']['operating_point_ramp_rpm_per_sec']:g}",
                        )
                        proto.send_command(port, "SPEED 0")
                        supervised_wait(port, buffer, 1.0, spec, MODE_SPEED_TORQUE_RUN)
                        proto.send_command(
                            port, f"PI SPEED {candidate['kp']:.9g} {candidate['ki']:.9g}"
                        )
                        proto.send_command(port, "PI CLEAR")
                        for base, target in spec["screen"]["step_transitions_rpm"]:
                            path = scope_dir / (
                                f"r{round_index}_{candidate['name']}_"
                                f"{base:+g}_to_{target:+g}rpm.csv"
                            )
                            record = capture_step(
                                port, buffer, base, target, spec,
                                noise["noise_rms_rpm"], path,
                            )
                            record.update({
                                "profile": candidate["name"],
                                "round": round_index,
                                "kp": candidate["kp"],
                                "ki": candidate["ki"],
                            })
                            result["pi_records"].append(record)

                summaries = {}
                for candidate in candidates:
                    records = [item for item in result["pi_records"]
                               if item["profile"] == candidate["name"]]
                    summaries[candidate["name"]] = summarize_candidate(records, spec)
                result["pi_summaries"] = summaries
                passing = [candidate for candidate in candidates
                           if summaries[candidate["name"]]["passed"]]
                if passing:
                    best_time_to_90 = min(
                        summaries[candidate["name"]]["worst_time_to_90_sec"]
                        for candidate in passing
                    )
                    shortlist = [
                        candidate for candidate in passing
                        if summaries[candidate["name"]]["worst_time_to_90_sec"]
                        <= 1.1 * best_time_to_90
                    ]
                    shortlist.sort(key=lambda candidate: (
                        summaries[candidate["name"]]["mean_rms_iq_ref_a"],
                        summaries[candidate["name"]]["peak_iq_ref_a"],
                        summaries[candidate["name"]]["direction_difference"],
                        candidate["kp"], candidate["ki"],
                    ))
                    result["screen_leader"] = shortlist[0]
                    result["status"] = "screen_qualified"
                else:
                    result["status"] = "screen_failed"
        finally:
            result["final_mode"] = stop_and_confirm_ready(
                port, args.port, args.baudrate, buffer
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    if result["final_mode"] != MODE_READY:
        raise RuntimeError("final STOP was not confirmed by READY telemetry")
    print(json.dumps({
        "result": str(output.resolve()),
        "status": result["status"],
        "screen_leader": result.get("screen_leader"),
        "final_mode": result["final_mode"],
    }, indent=2))


if __name__ == "__main__":
    main()
