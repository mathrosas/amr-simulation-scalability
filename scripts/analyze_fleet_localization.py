#!/usr/bin/env python3

"""Analyze stationary fleet localization against Gazebo ground truth."""

import argparse
import itertools
import json
import math
import re

from pathlib import Path


MAX_POSITION_ERROR = 0.10
MAX_YAW_ERROR = 0.10

MAX_PHYSICAL_DISPLACEMENT = 0.005
MAX_PHYSICAL_YAW_CHANGE = 0.01

MIN_TF_SAMPLES = 3


def angle_error(a, b):

    difference = a - b

    return abs(
        math.atan2(
            math.sin(difference),
            math.cos(difference),
        )
    )


def position_error(a, b):

    return math.hypot(
        a["x"] - b["x"],
        a["y"] - b["y"],
    )


def read_ground_truth(path):

    text = path.read_text()

    pattern = (
        r"Name:\s+(\S+)/robot\s*"
        r"- Pose \[ XYZ \(m\) \] "
        r"\[ RPY \(rad\) \]:\s*"
        r"\[([^\]]+)\]\s*"
        r"\[([^\]]+)\]"
    )

    poses = {}

    for match in re.finditer(pattern, text):

        robot = match.group(1)

        xyz = [
            float(value)
            for value in match.group(2).split()
        ]

        rpy = [
            float(value)
            for value in match.group(3).split()
        ]

        if robot in poses:
            raise ValueError(
                f"Duplicate model: {robot}"
            )

        if (
            len(xyz) != 3
            or len(rpy) != 3
            or not all(
                math.isfinite(value)
                for value in xyz + rpy
            )
        ):
            raise ValueError(
                f"Invalid pose: {robot}"
            )

        poses[robot] = {
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
            "yaw": rpy[2],
        }

    if not poses:
        raise ValueError(
            f"No robot poses found in {path}"
        )

    return poses


def read_tf_observations(path):

    text = path.read_text()

    sections = re.split(
        r"^===== (\S+) MAP TO (BASE_LINK|ODOM) =====\s*$",
        text,
        flags=re.MULTILINE,
    )

    observations = {}

    for index in range(1, len(sections), 3):

        robot = sections[index]
        transform = sections[index + 1]
        body = sections[index + 2]

        if transform != "BASE_LINK":
            continue

        if robot in observations:
            raise ValueError(
                f"Duplicate TF section: {robot}"
            )

        pattern = (
            r"At time ([\d.]+)\s*"
            r"- Translation: \[([^\]]+)\]"
            r"[^\n]*\n"
            r"- Rotation: in Quaternion[^\n]*\n"
            r"- Rotation: in RPY \(radian\) "
            r"\[([^\]]+)\]"
        )

        samples = []

        for match in re.finditer(pattern, body):

            xyz = [
                float(value.strip())
                for value in match.group(2).split(",")
            ]

            rpy = [
                float(value.strip())
                for value in match.group(3).split(",")
            ]

            timestamp = float(match.group(1))

            if (
                len(xyz) != 3
                or len(rpy) != 3
                or not all(
                    math.isfinite(value)
                    for value in xyz + rpy + [timestamp]
                )
            ):
                raise ValueError(
                    f"Invalid TF sample: {robot}"
                )

            samples.append({
                "time_s": timestamp,
                "x": xyz[0],
                "y": xyz[1],
                "yaw": rpy[2],
            })

        observations[robot] = samples

    return observations


def analyze_robot(
    initialization,
    initial_truth,
    final_truth,
    tf_samples,
):

    estimate = initialization["estimated_pose"]

    displacement = position_error(
        initial_truth,
        final_truth,
    )

    physical_yaw_change = angle_error(
        initial_truth["yaw"],
        final_truth["yaw"],
    )

    initial_position_error = position_error(
        estimate,
        initial_truth,
    )

    initial_yaw_error = angle_error(
        estimate["yaw"],
        initial_truth["yaw"],
    )

    tf_position_errors = [
        position_error(sample, final_truth)
        for sample in tf_samples
    ]

    tf_yaw_errors = [
        angle_error(
            sample["yaw"],
            final_truth["yaw"],
        )
        for sample in tf_samples
    ]

    tf_max_position_error = (
        max(tf_position_errors)
        if tf_position_errors
        else None
    )

    tf_max_yaw_error = (
        max(tf_yaw_errors)
        if tf_yaw_errors
        else None
    )

    tf_max_position_variation = (
        max(
            position_error(
                sample,
                tf_samples[0],
            )
            for sample in tf_samples
        )
        if tf_samples
        else None
    )

    tf_observation_duration = (
        tf_samples[-1]["time_s"]
        - tf_samples[0]["time_s"]
        if len(tf_samples) > 1
        else None
    )

    stationary = (
        displacement <= MAX_PHYSICAL_DISPLACEMENT
        and physical_yaw_change
            <= MAX_PHYSICAL_YAW_CHANGE
    )

    passed = (
        initialization["initialization_passed"]
        and stationary
        and initial_position_error <= MAX_POSITION_ERROR
        and initial_yaw_error <= MAX_YAW_ERROR
        and len(tf_samples) >= MIN_TF_SAMPLES
        and tf_observation_duration is not None
        and tf_observation_duration > 0
        and tf_max_position_error is not None
        and tf_max_position_error <= MAX_POSITION_ERROR
        and tf_max_yaw_error is not None
        and tf_max_yaw_error <= MAX_YAW_ERROR
    )

    return {
        "initial_ground_truth": initial_truth,
        "final_ground_truth": final_truth,
        "initial_amcl_estimate": estimate,
        "physical_endpoint_displacement_m":
            displacement,
        "physical_endpoint_yaw_change_rad":
            physical_yaw_change,
        "stationary_endpoints": stationary,
        "initial_position_error_m":
            initial_position_error,
        "initial_yaw_error_rad":
            initial_yaw_error,
        "initial_yaw_error_deg":
            math.degrees(initial_yaw_error),
        "amcl_standard_deviation": {
            "x_m": math.sqrt(
                initialization["covariance_x"]
            ),
            "y_m": math.sqrt(
                initialization["covariance_y"]
            ),
            "yaw_rad": math.sqrt(
                initialization["covariance_yaw"]
            ),
        },
        "tf_samples": len(tf_samples),
        "tf_observation_duration_s":
            tf_observation_duration,
        "tf_max_position_error_m":
            tf_max_position_error,
        "tf_max_yaw_error_rad":
            tf_max_yaw_error,
        "tf_max_position_variation_m":
            tf_max_position_variation,
        "stationary_localization_passed":
            passed,
    }


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "experiment_dir",
        type=Path,
    )

    parser.add_argument(
        "--assume-map-world-aligned",
        action="store_true",
    )

    args = parser.parse_args()

    if not args.assume_map_world_aligned:
        parser.error(
            "Ground-truth comparison requires "
            "--assume-map-world-aligned"
        )

    directory = args.experiment_dir

    initialization = json.loads(
        (
            directory
            / "initialization_summary.json"
        ).read_text()
    )

    if not initialization[
        "fleet_initialization_passed"
    ]:
        raise ValueError(
            "Fleet initialization did not pass"
        )

    initial_truth = read_ground_truth(
        directory / "ground_truth_initial.txt"
    )

    final_truth = read_ground_truth(
        directory / "ground_truth_final.txt"
    )

    tf_observations = read_tf_observations(
        directory / "localization_tf.txt"
    )

    robots = initialization["robot_results"]

    if not robots:
        raise ValueError("Empty fleet")

    if not (
        set(robots)
        == set(initial_truth)
        == set(final_truth)
        == set(tf_observations)
    ):
        raise ValueError(
            "Inconsistent robot namespaces"
        )

    results = {}

    for robot, data in robots.items():

        results[robot] = analyze_robot(
            data,
            initial_truth[robot],
            final_truth[robot],
            tf_observations[robot],
        )

    relative_positions = []

    for first, second in itertools.combinations(
        robots, 2
    ):

        actual_distance = position_error(
            initial_truth[first],
            initial_truth[second],
        )

        estimated_distance = position_error(
            robots[first]["estimated_pose"],
            robots[second]["estimated_pose"],
        )

        relative_positions.append({
            "robots": [first, second],
            "actual_initial_separation_m":
                actual_distance,
            "estimated_initial_separation_m":
                estimated_distance,
            "signed_separation_error_m":
                estimated_distance - actual_distance,
        })

    passed = all(
        result["stationary_localization_passed"]
        for result in results.values()
    )

    summary = {
        "robot_count": len(robots),
        "assume_map_world_aligned": True,
        "position_error_limit_m":
            MAX_POSITION_ERROR,
        "yaw_error_limit_rad":
            MAX_YAW_ERROR,
        "physical_displacement_limit_m":
            MAX_PHYSICAL_DISPLACEMENT,
        "physical_yaw_change_limit_rad":
            MAX_PHYSICAL_YAW_CHANGE,
        "minimum_tf_samples":
            MIN_TF_SAMPLES,
        "robot_results": results,
        "relative_positions":
            relative_positions,
        "fleet_static_localization_passed":
            passed,
        "limitations": [
            "Map and Gazebo world alignment assumed",
            "Ground truth sampled only at endpoints",
            "AMCL estimates and ground truth are not "
            "strictly time synchronized",
            "TF observations are rounded CLI output",
            "Stationary accuracy does not establish "
            "dynamic localization accuracy",
            "Diagnostic limits are not manufacturer "
            "specifications",
        ],
    }

    output = (
        directory
        / "localization_analysis.json"
    )

    output.write_text(
        json.dumps(
            summary,
            indent=2,
            allow_nan=False,
        ) + "\n"
    )

    print(
        json.dumps(
            summary,
            indent=2,
        )
    )

    if not passed:
        raise SystemExit(
            "Static localization validation failed"
        )


if __name__ == "__main__":
    main()