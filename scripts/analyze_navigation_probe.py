#!/usr/bin/env python3

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path


POSITION_TOLERANCE_M = 0.25
YAW_TOLERANCE_RAD = 0.25


def load_json(path):
    return json.loads(path.read_text())


def load_csv(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def number(value):
    if value is None or value == "":
        return None

    return float(value)


def divide(numerator, denominator):
    if numerator is None or denominator is None:
        return None

    if denominator <= 0:
        return None

    return numerator / denominator


def angle_difference(a, b):
    return math.atan2(
        math.sin(a - b),
        math.cos(a - b),
    )


def distribution(values):
    values = [
        value for value in values
        if value is not None
    ]

    if not values:
        return None

    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "stdev": (
            statistics.stdev(values)
            if len(values) > 1
            else None
        ),
    }


def trajectory_length(points):
    return sum(
        math.dist(a, b)
        for a, b in zip(points, points[1:])
    )


def extract_points(rows, x_field, y_field):
    points = []

    for row in rows:
        x = number(row.get(x_field))
        y = number(row.get(y_field))

        if x is not None and y is not None:
            points.append((x, y))

    return points


def read_ground_truth(path):
    if not path.exists():
        return None

    text = path.read_text()

    pattern = (
        r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:"
        r"\s*\[([^\]]+)\]"
        r"\s*\[([^\]]+)\]"
    )

    match = re.search(pattern, text)

    if match is None:
        raise ValueError(
            f"Could not parse ground truth: {path}"
        )

    x, y, z = map(float, match.group(1).split())
    roll, pitch, yaw = map(
        float,
        match.group(2).split(),
    )

    return {
        "x": x,
        "y": y,
        "z": z,
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
    }


def pose_error(pose, goal):
    if pose is None:
        return None

    position_error = math.hypot(
        pose["x"] - goal["x"],
        pose["y"] - goal["y"],
    )

    yaw_error = abs(
        angle_difference(
            pose["yaw_rad"],
            goal["yaw_rad"],
        )
    )

    return {
        "position_error_m": position_error,
        "yaw_error_rad": yaw_error,
        "yaw_error_deg": math.degrees(yaw_error),
        "within_position_tolerance": (
            position_error <= POSITION_TOLERANCE_M
        ),
        "within_yaw_tolerance": (
            yaw_error <= YAW_TOLERANCE_RAD
        ),
        "within_both_tolerances": (
            position_error <= POSITION_TOLERANCE_M
            and yaw_error <= YAW_TOLERANCE_RAD
        ),
    }


def analyze_sampling(rows):
    timestamps = [
        int(row["monotonic_ns"])
        for row in rows
    ]

    intervals_ms = [
        (b - a) / 1_000_000
        for a, b in zip(
            timestamps,
            timestamps[1:],
        )
        if b > a
    ]

    duration_s = (
        (timestamps[-1] - timestamps[0])
        / 1_000_000_000
        if len(timestamps) >= 2
        else None
    )

    actual_rate_hz = divide(
        len(timestamps) - 1,
        duration_s,
    )

    return {
        "sample_count": len(rows),
        "duration_s": duration_s,
        "actual_rate_hz": actual_rate_hz,
        "interval_ms": distribution(intervals_ms),
        "missing_map_pose_samples": sum(
            row.get("map_x_m", "") == ""
            or row.get("map_y_m", "") == ""
            for row in rows
        ),
        "missing_odometry_samples": sum(
            row.get("odom_x_m", "") == ""
            or row.get("odom_y_m", "") == ""
            for row in rows
        ),
    }


def analyze_velocity(rows):
    linear = []
    angular = []

    for row in rows:
        v = number(row.get("linear_x_mps"))
        w = number(row.get("angular_z_radps"))

        if v is not None:
            linear.append(abs(v))

        if w is not None:
            angular.append(abs(w))

    return {
        "absolute_linear_velocity_mps": (
            distribution(linear)
        ),
        "absolute_angular_velocity_radps": (
            distribution(angular)
        ),
        "stationary_sample_count": sum(
            abs(float(row["linear_x_mps"])) < 0.005
            and abs(float(row["angular_z_radps"])) < 0.01
            for row in rows
            if row.get("linear_x_mps")
            and row.get("angular_z_radps")
        ),
    }


def analyze_log(path):
    if not path.exists():
        return None

    text = path.read_text()
    lines = text.splitlines()

    start_time = None
    success_time = None

    failures = []
    missed_rates = []

    for line in lines:
        match = re.search(
            r"\[(\d+\.\d+)\]",
            line,
        )

        timestamp = (
            float(match.group(1))
            if match
            else None
        )

        if "Begin navigating from" in line:
            if start_time is None:
                start_time = timestamp

        if "Goal succeeded" in line:
            success_time = timestamp

        if "Failed to make progress" in line:
            failures.append(timestamp)

        if "Control loop missed its desired rate" in line:
            rate_match = re.search(
                r"Current loop rate is ([\d.]+) Hz",
                line,
            )

            missed_rates.append({
                "timestamp": timestamp,
                "actual_rate_hz": (
                    float(rate_match.group(1))
                    if rate_match
                    else None
                ),
            })

    failure_offsets = [
        timestamp - start_time
        for timestamp in failures
        if timestamp is not None
        and start_time is not None
    ]

    return {
        "navigation_start_timestamp": start_time,
        "navigation_success_timestamp": success_time,
        "server_navigation_duration_s": (
            success_time - start_time
            if success_time is not None
            and start_time is not None
            else None
        ),
        "progress_failure_count": len(failures),
        "progress_failure_offsets_s": failure_offsets,
        "local_costmap_clear_count": sum(
            "Received request to clear entirely "
            "the local_costmap" in line
            for line in lines
        ),
        "global_costmap_clear_count": sum(
            "Received request to clear entirely "
            "the global_costmap" in line
            for line in lines
        ),
        "controller_rate_warning_count": (
            len(missed_rates)
        ),
        "controller_rate_warnings": missed_rates,
        "local_costmap_tf_drop_count": sum(
            "Message Filter dropping message" in line
            and "local_costmap.local_costmap" in line
            for line in lines
        ),
        "global_costmap_tf_drop_count": sum(
            "Message Filter dropping message" in line
            and "global_costmap.global_costmap" in line
            for line in lines
        ),
    }


def analyze_progress_windows(rows, log_analysis):
    if not rows or log_analysis is None:
        return []

    origin = int(rows[0]["monotonic_ns"])

    windows = []

    for offset in log_analysis[
        "progress_failure_offsets_s"
    ]:
        window_start = max(0.0, offset - 10.0)

        selected = [
            row
            for row in rows
            if window_start
            <= (
                int(row["monotonic_ns"]) - origin
            ) / 1_000_000_000
            <= offset
        ]

        points = extract_points(
            selected,
            "odom_x_m",
            "odom_y_m",
        )

        velocities = analyze_velocity(selected)

        windows.append({
            "failure_offset_s": offset,
            "window_start_s": window_start,
            "window_end_s": offset,
            "sample_count": len(selected),
            "net_odometry_displacement_m": (
                math.dist(points[0], points[-1])
                if len(points) >= 2
                else None
            ),
            "sampled_odometry_distance_m": (
                trajectory_length(points)
                if len(points) >= 2
                else None
            ),
            "velocity": velocities,
        })

    return windows


def analyze_experiment(run_dir, assume_alignment):
    summary = load_json(
        run_dir / "summary.json"
    )

    samples = load_csv(
        run_dir / "samples.csv"
    )

    path_file = run_dir / "planned_path.csv"

    planned_path = (
        load_csv(path_file)
        if path_file.exists()
        else []
    )

    execution_enabled = summary[
        "execution_enabled"
    ]

    planning = summary["planning"]
    navigation = summary["navigation"]
    goal = summary["goal"]

    navigation_rows = [
        row
        for row in samples
        if row["phase"] == "navigation"
    ]

    settling_rows = [
        row
        for row in samples
        if row["phase"] == "settling"
    ]

    motion_rows = navigation_rows + settling_rows

    path_points = extract_points(
        planned_path,
        "x_m",
        "y_m",
    )

    map_points = extract_points(
        motion_rows,
        "map_x_m",
        "map_y_m",
    )

    odom_points = extract_points(
        motion_rows,
        "odom_x_m",
        "odom_y_m",
    )

    wall_duration = navigation.get(
        "client_wall_duration_s"
    )

    sim_duration = navigation.get(
        "client_sim_duration_s"
    )

    actual_length = summary.get(
        "odometry_trajectory_length_m"
    )

    planned_length = planning.get(
        "path_length_m"
    )

    initial_gt = read_ground_truth(
        run_dir / "ground_truth_initial.txt"
    )

    final_gt = read_ground_truth(
        run_dir / "ground_truth_final.txt"
    )

    final_estimate = summary.get(
        "final_map_pose"
    )

    final_accuracy = None

    if execution_enabled:
        final_accuracy = {
            "estimated_goal_error": pose_error(
                final_estimate,
                goal,
            ),
            "ground_truth_goal_error": None,
            "estimated_ground_truth_discrepancy": None,
        }

        if assume_alignment and final_gt is not None:
            final_accuracy[
                "ground_truth_goal_error"
            ] = pose_error(final_gt, goal)

            if final_estimate is not None:
                final_accuracy[
                    "estimated_ground_truth_discrepancy"
                ] = pose_error(
                    final_estimate,
                    final_gt,
                )

    log_analysis = analyze_log(
        run_dir / "navigation.log"
    )

    planning_succeeded = (
        planning.get("status") == "SUCCEEDED"
        and planning.get("error_code") == 0
    )

    navigation_succeeded = (
        navigation.get("status") == "SUCCEEDED"
        and navigation.get("error_code") == 0
        if execution_enabled
        else None
    )

    path_csv_length = (
        trajectory_length(path_points)
        if len(path_points) >= 2
        else None
    )

    analysis = {
        "experiment": run_dir.name,
        "namespace": summary["namespace"],
        "goal": goal,
        "execution_enabled": execution_enabled,
        "completed": summary.get("completed"),
        "requires_manual_stop": summary.get(
            "requires_manual_stop"
        ),
        "planning": {
            **planning,
            "succeeded": planning_succeeded,
            "csv_pose_count": len(path_points),
            "csv_path_length_m": path_csv_length,
            "csv_length_matches_summary": (
                math.isclose(
                    path_csv_length,
                    planned_length,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
                if path_csv_length is not None
                and planned_length is not None
                else None
            ),
        },
        "navigation": {
            **navigation,
            "succeeded": navigation_succeeded,
            "effective_rtf": divide(
                sim_duration,
                wall_duration,
            ),
            "feedback_count": summary.get(
                "feedback_count"
            ),
            "average_feedback_rate_hz": (
                divide(
                    summary.get("feedback_count"),
                    wall_duration,
                )
                if execution_enabled
                else None
            ),
            "max_recoveries": summary.get(
                "max_recoveries"
            ),
        },
        "trajectory": {
            "planned_length_m": planned_length,
            "odometry_length_m": actual_length,
            "odometry_sample_count": summary.get(
                "odometry_trajectory_sample_count"
            ),
            "sampled_map_length_m": (
                trajectory_length(map_points)
                if len(map_points) >= 2
                else None
            ),
            "sampled_odometry_length_m": (
                trajectory_length(odom_points)
                if len(odom_points) >= 2
                else None
            ),
            "odometry_to_planned_length_ratio": divide(
                actual_length,
                planned_length,
            ),
            "average_odometry_speed_mps": divide(
                actual_length,
                (
                    sim_duration + 2.0
                    if sim_duration is not None
                    else None
                ),
            ),
        },
        "sampling": {
            "target_rate_hz": summary.get(
                "sample_rate_hz"
            ),
            "all_samples": analyze_sampling(samples),
            "navigation": analyze_sampling(
                navigation_rows
            ),
            "settling": analyze_sampling(
                settling_rows
            ),
            "sample_count_matches_summary": (
                len(samples)
                == summary.get("sample_count")
            ),
        },
        "velocity": {
            "navigation": analyze_velocity(
                navigation_rows
            ),
            "settling": analyze_velocity(
                settling_rows
            ),
            "final_stationary": summary.get(
                "stationary"
            ),
        },
        "ground_truth": {
            "map_world_alignment_assumed": (
                assume_alignment
            ),
            "initial_pose": initial_gt,
            "final_pose": final_gt,
            "endpoint_displacement_m": (
                math.hypot(
                    final_gt["x"] - initial_gt["x"],
                    final_gt["y"] - initial_gt["y"],
                )
                if initial_gt is not None
                and final_gt is not None
                else None
            ),
        },
        "accuracy": final_accuracy,
        "navigation_log": log_analysis,
        "progress_failure_windows": (
            analyze_progress_windows(
                navigation_rows,
                log_analysis,
            )
        ),
        "limitations": [
            "Ground truth is sampled only at endpoints.",
            "Odometry distance is not ground-truth distance.",
            "Sampled trajectories may bridge missing samples.",
            "The average odometry speed uses an approximate "
            "two-second settling duration.",
            "Progress windows align the first navigation "
            "sample approximately with the server start.",
            "Progress windows do not reproduce internal "
            "progress checker baseline resets.",
            "Log event counts describe recorded messages "
            "rather than all underlying events.",
        ],
    }

    return analysis


def main():
    parser = argparse.ArgumentParser(
        description="Analyze AMR navigation experiments"
    )

    parser.add_argument(
        "run_dir",
        type=Path,
    )

    parser.add_argument(
        "--assume-map-world-aligned",
        action="store_true",
        help=(
            "Enable direct comparisons between map-frame "
            "estimates and Gazebo world-frame ground truth"
        ),
    )

    args = parser.parse_args()

    analysis = analyze_experiment(
        args.run_dir,
        args.assume_map_world_aligned,
    )

    output = json.dumps(
        analysis,
        indent=2,
        allow_nan=False,
    )

    output_file = args.run_dir / "analysis.json"

    output_file.write_text(output + "\n")

    print(output)


if __name__ == "__main__":
    main()