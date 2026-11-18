#!/usr/bin/env python3
"""Analyze recorded AMR motion-control experiments."""

import argparse
import csv
import json
import statistics

from pathlib import Path


def load_samples(path):
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))

    for row in rows:
        row["monotonic_ns"] = int(
            row["monotonic_ns"]
        )

        row["linear_x_mps"] = float(
            row["linear_x_mps"]
        )

        row["angular_z_radps"] = float(
            row["angular_z_radps"]
        )

    return sorted(
        rows,
        key=lambda row: row["monotonic_ns"],
    )


def select_samples(rows, stream, phase):
    return [
        row for row in rows
        if row["stream"] == stream
        and row["phase"] == phase
    ]


def elapsed_ms(start_ns, end_ns):
    return (end_ns - start_ns) / 1_000_000


def velocity_statistics(rows, target):
    values = [
        row["linear_x_mps"]
        for row in rows
    ]

    if not values:
        return {"samples": 0}

    mean = statistics.mean(values)

    return {
        "samples": len(values),
        "mean_mps": mean,
        "minimum_mps": min(values),
        "maximum_mps": max(values),
        "mean_absolute_error_mps": statistics.mean(
            abs(value - target)
            for value in values
        ),
        "mean_bias_pct": (
            100 * (mean - target) / target
            if target != 0
            else None
        ),
    }


def command_timing(commands):
    timestamps = [
        row["monotonic_ns"]
        for row in commands
    ]

    intervals = [
        elapsed_ms(a, b)
        for a, b in zip(
            timestamps,
            timestamps[1:],
        )
    ]

    if not intervals:
        return {
            "commands": len(commands),
        }

    mean_interval = statistics.mean(intervals)

    return {
        "commands": len(commands),
        "mean_interval_ms": mean_interval,
        "minimum_interval_ms": min(intervals),
        "maximum_interval_ms": max(intervals),
        "interval_stddev_ms":
            statistics.pstdev(intervals),
        "observed_rate_hz":
            1000 / mean_interval
            if mean_interval > 0
            else None,
    }


def response_time(
    rows,
    start_ns,
    threshold,
):
    for row in rows:
        if (
            row["monotonic_ns"] >= start_ns
            and row["linear_x_mps"] >= threshold
        ):
            return elapsed_ms(
                start_ns,
                row["monotonic_ns"],
            )

    return None


def first_stop_response(
    rows,
    stop_ns,
    threshold,
):
    for row in rows:
        if (
            row["monotonic_ns"] >= stop_ns
            and abs(row["linear_x_mps"]) <= threshold
        ):
            return elapsed_ms(
                stop_ns,
                row["monotonic_ns"],
            )

    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "experiment_dir",
        type=Path,
    )

    parser.add_argument(
        "--window",
        type=float,
        default=0.5,
        help="Steady-state and final stop window in seconds.",
    )

    args = parser.parse_args()

    if args.window <= 0:
        parser.error("Window must be positive")

    directory = args.experiment_dir

    summary = json.loads(
        (directory / "summary.json").read_text()
    )

    rows = load_samples(
        directory / "samples.csv"
    )

    target = summary["requested_speed_mps"]

    commands = select_samples(
        rows,
        "requested_command",
        "motion",
    )

    controller = select_samples(
        rows,
        "controller_output",
        "motion",
    )

    odometry = select_samples(
        rows,
        "odometry",
        "motion",
    )

    stop_commands = select_samples(
        rows,
        "requested_command",
        "stopping",
    )

    stop_odometry = select_samples(
        rows,
        "odometry",
        "stopping",
    )

    mux_motion = select_samples(
        rows,
        "mux_output",
        "motion",
    )

    stop_mux = select_samples(
        rows,
        "mux_output",
        "stopping",
    )

    stop_controller = select_samples(
        rows,
        "controller_output",
        "stopping",
    )

    if not all([
        commands,
        controller,
        odometry,
        stop_commands,
        stop_odometry,
    ]):
        raise SystemExit(
            "Incomplete experiment: missing required samples"
        )

    motion_start = commands[0]["monotonic_ns"]
    stop_start = stop_commands[0]["monotonic_ns"]

    window_ns = int(
        args.window * 1_000_000_000
    )

    steady_start = max(
        motion_start,
        stop_start - window_ns,
    )

    steady_controller = [
        row for row in controller
        if steady_start
        <= row["monotonic_ns"]
        < stop_start
    ]

    steady_odometry = [
        row for row in odometry
        if steady_start
        <= row["monotonic_ns"]
        < stop_start
    ]

    stop_end = stop_odometry[-1]["monotonic_ns"]

    final_stop = [
        row for row in stop_odometry
        if row["monotonic_ns"] >= max(
            stop_start,
            stop_end - window_ns,
        )
    ]

    ideal_distance = summary[
        "ideal_commanded_distance_m"
    ]

    measured_distance = summary[
        "odom_displacement_m"
    ]

    distance_error = (
        measured_distance - ideal_distance
    )

    motion = summary["motion"]

    results = {
        "experiment": str(directory),

        "requested_speed_mps": target,

        "command_timing": command_timing(
            commands
        ),

        "motion_phase": {
            "controller": velocity_statistics(
                controller,
                target,
            ),
            "odometry": velocity_statistics(
                odometry,
                target,
            ),
        },

        "steady_state": {
            "window_s": args.window,

            "controller": velocity_statistics(
                steady_controller,
                target,
            ),

            "odometry": velocity_statistics(
                steady_odometry,
                target,
            ),
        },

        "observed_response": {
            "controller_10_pct_ms": response_time(
                controller,
                motion_start,
                target * 0.10,
            ),

            "controller_90_pct_ms": response_time(
                controller,
                motion_start,
                target * 0.90,
            ),

            "odometry_10_pct_ms": response_time(
                odometry,
                motion_start,
                target * 0.10,
            ),

            "odometry_90_pct_ms": response_time(
                odometry,
                motion_start,
                target * 0.90,
            ),
        },

        "stop_pipeline": {
            "threshold_mps": target * 0.05,

            "mux_motion_samples": len(mux_motion),

            "mux_stopping_samples": len(stop_mux),

            "controller_stopping_samples":
                len(stop_controller),

            "mux_response_ms": first_stop_response(
                stop_mux,
                stop_start,
                target * 0.05,
            ),

            "controller_response_ms": first_stop_response(
                stop_controller,
                stop_start,
                target * 0.05,
            ),

            "odometry_response_ms": first_stop_response(
                stop_odometry,
                stop_start,
                target * 0.05,
            ),
        },

        "stopping": {
            "threshold_mps": target * 0.05,

            "first_below_threshold_ms":
                first_stop_response(
                    stop_odometry,
                    stop_start,
                    target * 0.05,
                ),

            "final_window_s": args.window,

            "final_velocity": velocity_statistics(
                final_stop,
                0.0,
            ),

            "final_max_abs_velocity_mps": max(
                abs(row["linear_x_mps"])
                for row in final_stop
            ),
        },

        "displacement": {
            "ideal_m": ideal_distance,
            "measured_m": measured_distance,
            "signed_error_m": distance_error,

            "signed_error_pct": (
                100 * distance_error
                / ideal_distance
                if ideal_distance != 0
                else None
            ),
        },

        "effective_motion_rtf": (
            motion["elapsed_sim_s"]
            / motion["elapsed_wall_s"]
        ),

        "experiment_error": summary["error"],
    }

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()