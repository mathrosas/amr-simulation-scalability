#!/usr/bin/env python3

import argparse
import json
import math
import statistics
from pathlib import Path


METRICS = {
    "planning_time_s":
        "planning.reported_planning_time_s",

    "planning_client_duration_s":
        "planning.client_wall_duration_s",

    "planning_acceptance_latency_s":
        "planning.acceptance_wall_s",

    "planned_path_length_m":
        "planning.path_length_m",

    "planned_path_length_ratio":
        "planning.path_length_ratio",

    "navigation_wall_duration_s":
        "navigation.client_wall_duration_s",

    "navigation_sim_duration_s":
        "navigation.client_sim_duration_s",

    "navigation_acceptance_latency_s":
        "navigation.acceptance_wall_s",

    "navigation_effective_rtf":
        "navigation.effective_rtf",

    "feedback_rate_hz":
        "navigation.average_feedback_rate_hz",

    "odometry_trajectory_length_m":
        "trajectory.odometry_length_m",

    "sampled_map_trajectory_length_m":
        "trajectory.sampled_map_length_m",

    "odometry_to_planned_length_ratio":
        "trajectory.odometry_to_planned_length_ratio",

    "average_odometry_speed_mps":
        "trajectory.average_odometry_speed_mps",

    "telemetry_rate_hz":
        "sampling.navigation.actual_rate_hz",

    "telemetry_interval_stdev_ms":
        "sampling.navigation.interval_ms.stdev",

    "mean_absolute_linear_velocity_mps":
        "velocity.navigation.absolute_linear_velocity_mps.mean",

    "estimated_position_error_m":
        "accuracy.estimated_goal_error.position_error_m",

    "estimated_yaw_error_rad":
        "accuracy.estimated_goal_error.yaw_error_rad",

    "ground_truth_position_error_m":
        "accuracy.ground_truth_goal_error.position_error_m",

    "ground_truth_yaw_error_rad":
        "accuracy.ground_truth_goal_error.yaw_error_rad",

    "localization_position_discrepancy_m":
        "accuracy.estimated_ground_truth_discrepancy.position_error_m",

    "localization_yaw_discrepancy_rad":
        "accuracy.estimated_ground_truth_discrepancy.yaw_error_rad",
}


RELIABILITY_METRICS = {
    "recoveries":
        "navigation.max_recoveries",

    "progress_failures":
        "navigation_log.progress_failure_count",

    "local_costmap_clears":
        "navigation_log.local_costmap_clear_count",

    "global_costmap_clears":
        "navigation_log.global_costmap_clear_count",

    "controller_rate_warnings":
        "navigation_log.controller_rate_warning_count",

    "local_costmap_tf_drops":
        "navigation_log.local_costmap_tf_drop_count",

    "global_costmap_tf_drops":
        "navigation_log.global_costmap_tf_drop_count",

    "missing_navigation_map_samples":
        "sampling.navigation.missing_map_pose_samples",

    "missing_navigation_odometry_samples":
        "sampling.navigation.missing_odometry_samples",
}


def get_value(data, path):
    current = data

    for key in path.split("."):
        if not isinstance(current, dict):
            return None

        current = current.get(key)

    return current


def numeric(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def distribution(values):
    values = [
        value for value in values
        if numeric(value)
    ]

    if not values:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "min": None,
            "max": None,
            "stdev": None,
        }

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


def boolean_rate(values):
    known = [
        value for value in values
        if isinstance(value, bool)
    ]

    passed = sum(value is True for value in known)

    return {
        "passed": passed,
        "failed": len(known) - passed,
        "unknown": len(values) - len(known),
        "evaluated": len(known),
        "rate": (
            passed / len(known)
            if known
            else None
        ),
    }


def goal_signature(goal):
    return (
        float(goal["x"]),
        float(goal["y"]),
        float(goal["yaw_rad"]),
        goal["frame_id"],
    )


def pose_difference(a, b):
    if a is None or b is None:
        return None

    yaw_delta = a["yaw_rad"] - b["yaw_rad"]

    return {
        "position_m": math.hypot(
            a["x"] - b["x"],
            a["y"] - b["y"],
        ),
        "yaw_rad": abs(
            math.atan2(
                math.sin(yaw_delta),
                math.cos(yaw_delta),
            )
        ),
    }


def load_experiment(run_dir):
    analysis_file = run_dir / "analysis.json"
    summary_file = run_dir / "summary.json"

    analysis = json.loads(
        analysis_file.read_text()
    )

    summary = json.loads(
        summary_file.read_text()
    )

    if not analysis.get("execution_enabled"):
        raise ValueError(
            f"Planning-only experiment: {run_dir}"
        )

    if analysis["goal"] != summary["goal"]:
        raise ValueError(
            f"Goal mismatch between files: {run_dir}"
        )

    if analysis["namespace"] != summary["namespace"]:
        raise ValueError(
            f"Namespace mismatch between files: {run_dir}"
        )

    return {
        "directory": str(run_dir),
        "analysis": analysis,
        "summary": summary,
    }


def run_succeeded(analysis):
    return (
        analysis.get("completed") is True
        and analysis.get("requires_manual_stop") is False
        and get_value(
            analysis,
            "planning.succeeded",
        ) is True
        and get_value(
            analysis,
            "navigation.succeeded",
        ) is True
    )


def aggregate(experiments):
    reference = experiments[0]["analysis"]

    reference_goal = goal_signature(
        reference["goal"]
    )

    reference_namespace = reference["namespace"]

    reference_alignment = get_value(
        reference,
        "ground_truth.map_world_alignment_assumed",
    )

    reference_summary = experiments[0]["summary"]

    reference_start = reference_summary.get(
        "initial_map_pose"
    )

    reference_ground_truth = get_value(
        reference,
        "ground_truth.initial_pose",
    )

    successful = []
    failed = []
    run_records = []
    warnings = []

    for experiment in experiments:
        analysis = experiment["analysis"]
        summary = experiment["summary"]

        if goal_signature(analysis["goal"]) != reference_goal:
            raise ValueError(
                "Experiments have different destinations"
            )

        if analysis["namespace"] != reference_namespace:
            raise ValueError(
                "Experiments have different namespaces"
            )

        alignment = get_value(
            analysis,
            "ground_truth.map_world_alignment_assumed",
        )

        if alignment != reference_alignment:
            raise ValueError(
                "Experiments use different alignment assumptions"
            )

        success = run_succeeded(analysis)

        if success:
            successful.append(analysis)
        else:
            failed.append(analysis)

        start_difference = pose_difference(
            summary.get("initial_map_pose"),
            reference_start,
        )

        ground_truth_start_difference = pose_difference(
            get_value(
                analysis,
                "ground_truth.initial_pose",
            ),
            reference_ground_truth,
        )

        run_records.append({
            "experiment": analysis["experiment"],
            "directory": experiment["directory"],
            "succeeded": success,
            "planning_status": get_value(
                analysis,
                "planning.status",
            ),
            "navigation_status": get_value(
                analysis,
                "navigation.status",
            ),
            "requires_manual_stop": analysis.get(
                "requires_manual_stop"
            ),
            "initial_map_pose": summary.get(
                "initial_map_pose"
            ),
            "initial_map_pose_difference_from_reference":
                start_difference,
            "initial_ground_truth_difference_from_reference":
                ground_truth_start_difference,
            "navigation_wall_duration_s": get_value(
                analysis,
                "navigation.client_wall_duration_s",
            ),
            "recoveries": get_value(
                analysis,
                "navigation.max_recoveries",
            ),
            "progress_failures": get_value(
                analysis,
                "navigation_log.progress_failure_count",
            ),
            "estimated_position_error_m": get_value(
                analysis,
                "accuracy.estimated_goal_error.position_error_m",
            ),
            "ground_truth_position_error_m": get_value(
                analysis,
                "accuracy.ground_truth_goal_error.position_error_m",
            ),
        })

        if start_difference is None:
            warnings.append(
                f"{analysis['experiment']}: "
                "Initial map pose comparison unavailable"
            )

        elif (
            start_difference["position_m"] > 0.10
            or start_difference["yaw_rad"] > 0.10
        ):
            warnings.append(
                f"{analysis['experiment']}: "
                "Initial estimated pose differs from reference "
                "by more than 0.10 m or 0.10 rad"
            )

        if ground_truth_start_difference is None:
            warnings.append(
                f"{analysis['experiment']}: "
                "Initial ground-truth comparison unavailable"
            )

        elif (
            ground_truth_start_difference["position_m"] > 0.10
            or ground_truth_start_difference["yaw_rad"] > 0.10
        ):
            warnings.append(
                f"{analysis['experiment']}: "
                "Initial ground-truth pose differs from reference "
                "by more than 0.10 m or 0.10 rad"
            )

    all_analyses = [
        experiment["analysis"]
        for experiment in experiments
    ]

    total_runs = len(all_analyses)

    performance = {
        name: distribution([
            get_value(analysis, path)
            for analysis in successful
        ])
        for name, path in METRICS.items()
    }

    reliability = {}

    for name, path in RELIABILITY_METRICS.items():
        values = [
            get_value(analysis, path)
            for analysis in all_analyses
        ]

        observed = [
            value for value in values
            if numeric(value)
        ]

        reliability[name] = {
            **distribution(observed),
            "total": (
                sum(observed)
                if observed
                else None
            ),
            "missing_runs": total_runs - len(observed),
            "runs_with_event": sum(
                value > 0
                for value in observed
            ),
        }

    if total_runs < 3:
        warnings.append(
            "Fewer than three experiments: "
            "insufficient for meaningful repeatability analysis"
        )

    if len(successful) < 2:
        warnings.append(
            "Fewer than two successful experiments: "
            "sample standard deviations are unavailable"
        )

    warnings.append(
        "Matching destinations do not establish identical "
        "simulation or navigation configurations"
    )

    return {
        "namespace": reference_namespace,
        "goal": reference["goal"],
        "map_world_alignment_assumed": reference_alignment,

        "experiment_count": total_runs,
        "successful_runs": len(successful),
        "failed_runs": len(failed),

        "success_rate": len(successful) / total_runs,

        "planning_success": boolean_rate([
            get_value(item, "planning.succeeded")
            for item in all_analyses
        ]),

        "navigation_action_success": boolean_rate([
            get_value(item, "navigation.succeeded")
            for item in all_analyses
        ]),

        "stationary_at_completion": boolean_rate([
            get_value(item, "velocity.final_stationary")
            for item in all_analyses
        ]),

        "goal_tolerance_compliance_successful_runs": {
            "estimated_pose": boolean_rate([
                get_value(
                    item,
                    "accuracy.estimated_goal_error."
                    "within_both_tolerances",
                )
                for item in successful
            ]),

            "ground_truth_pose": boolean_rate([
                get_value(
                    item,
                    "accuracy.ground_truth_goal_error."
                    "within_both_tolerances",
                )
                for item in successful
            ]),
        },

        "performance_successful_runs": performance,
        "reliability_all_runs": reliability,

        "runs": run_records,
        "warnings": warnings,

        "limitations": [
            "Performance distributions include successful "
            "runs only.",

            "Reliability distributions include failed "
            "and successful runs.",

            "Missing measurements are not treated as zero.",

            "Ground-truth comparisons depend on the "
            "map/world alignment assumption.",

            "Initial pose comparisons use the first run "
            "as reference.",

            "Initial pose warning thresholds are diagnostic "
            "choices rather than Nav2 requirements.",

            "Identical Docker, Nav2, middleware and GUI "
            "configurations must be verified externally.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate AMR navigation experiments"
    )

    parser.add_argument(
        "run_dirs",
        nargs="+",
        type=Path,
    )

    parser.add_argument(
        "--output",
        required=True,
        type=Path,
    )

    args = parser.parse_args()

    resolved = [
        directory.resolve()
        for directory in args.run_dirs
    ]

    if len(set(resolved)) != len(resolved):
        parser.error(
            "Duplicate experiment directories"
        )

    experiments = [
        load_experiment(directory)
        for directory in args.run_dirs
    ]

    result = aggregate(experiments)

    output = json.dumps(
        result,
        indent=2,
        allow_nan=False,
    )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.output.write_text(output + "\n")

    print(output)


if __name__ == "__main__":
    main()