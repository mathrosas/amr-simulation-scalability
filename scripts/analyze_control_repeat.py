#!/usr/bin/env python3
"""Aggregate repeated AMR motion-control experiments."""

import argparse
import json
import math
import statistics

from pathlib import Path


METRICS = {
    "command_rate_hz":
        ("command_timing", "observed_rate_hz"),

    "command_interval_stddev_ms":
        ("command_timing", "interval_stddev_ms"),

    "controller_response_90_ms":
        ("observed_response", "controller_90_pct_ms"),

    "odometry_response_90_ms":
        ("observed_response", "odometry_90_pct_ms"),

    "mux_stopping_response_ms":
        ("stop_pipeline", "mux_response_ms"),

    "controller_stopping_response_ms":
        ("stop_pipeline", "controller_response_ms"),

    "odometry_stopping_response_ms":
        ("stopping", "first_below_threshold_ms"),

    "steady_state_odometry_mps":
        ("steady_state", "odometry", "mean_mps"),

    "steady_state_odometry_mae_mps":
        ("steady_state", "odometry", "mean_absolute_error_mps"),

    "final_max_abs_velocity_mps":
        ("stopping", "final_max_abs_velocity_mps"),

    "displacement_m":
        ("displacement", "measured_m"),

    "displacement_error_pct":
        ("displacement", "signed_error_pct"),

    "effective_motion_rtf":
        ("effective_motion_rtf",),
}


def get_value(data, path):
    value = data

    for key in path:
        if not isinstance(value, dict):
            return None

        value = value.get(key)

    return value


def summarize(values):
    if not values:
        return {
            "samples": 0,
        }

    return {
        "samples": len(values),

        "mean": statistics.mean(values),

        "median": statistics.median(values),

        "minimum": min(values),

        "maximum": max(values),

        "sample_stddev": (
            statistics.stdev(values)
            if len(values) > 1
            else None
        ),
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "experiment_dir",
        type=Path,
    )

    args = parser.parse_args()

    directory = args.experiment_dir

    trial_dirs = sorted(
        path
        for path in directory.glob("trial_*")
        if path.is_dir()
    )

    if not trial_dirs:
        raise SystemExit("No trial directories found")

    trials = []
    failed_trials = []

    for trial_dir in trial_dirs:
        path = trial_dir / "analysis.json"

        if not path.is_file():
            raise SystemExit(
                f"Missing analysis file: {path}"
            )

        data = json.loads(path.read_text())

        if data.get("experiment_error") is not None:
            failed_trials.append(trial_dir.name)
            continue

        trials.append(data)

    if not trials:
        raise SystemExit("No successful trials found")

    speeds = {
        trial["requested_speed_mps"]
        for trial in trials
    }

    if len(speeds) != 1:
        raise SystemExit(
            "Trials have different requested velocities"
        )

    results = {
        "experiment": str(directory),

        "total_trials": len(trial_dirs),

        "successful_trials": len(trials),

        "failed_trials": failed_trials,

        "requested_speed_mps": speeds.pop(),

        "metrics": {},
    }

    for name, path in METRICS.items():
        values = []

        for trial in trials:
            value = get_value(trial, path)

            if (
                isinstance(value, (int, float))
                and math.isfinite(value)
            ):
                values.append(value)

        results["metrics"][name] = summarize(
            values
        )

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()