#!/usr/bin/env python3

"""Navigate one fleet robot while monitoring another robot."""

import argparse
import csv
import json
import math
import re
import sys
import time

from pathlib import Path

import rclpy

from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from navigation_probe import (
    NavigationProbe,
    quaternion_to_yaw,
    stamp_ns,
)


POSITION_LIMIT = 0.005
YAW_LIMIT = 0.01

LINEAR_LIMIT = 0.005
ANGULAR_LIMIT = 0.01

MAX_SAMPLE_GAP = 0.2
MIN_PHASE_SAMPLES = 10


ACTIVE_FIELDS = [
    "phase",
    "monotonic_ns",
    "sim_time_ns",
    "map_pose_stamp_ns",
    "map_x_m",
    "map_y_m",
    "map_yaw_rad",
    "odom_stamp_ns",
    "odom_x_m",
    "odom_y_m",
    "odom_yaw_rad",
    "linear_x_mps",
    "angular_z_radps",
    "distance_remaining_m",
    "number_of_recoveries",
]


PASSIVE_FIELDS = [
    "phase",
    "monotonic_ns",
    "odom_stamp_ns",
    "x_m",
    "y_m",
    "yaw_rad",
    "linear_x_mps",
    "angular_z_radps",
]


def angle_difference(a, b):

    return math.atan2(
        math.sin(a - b),
        math.cos(a - b),
    )


def save_json(path, data):

    path.write_text(
        json.dumps(
            data,
            indent=2,
            allow_nan=False,
        ) + "\n"
    )


def save_csv(path, rows):

    with path.open("w", newline="") as file:

        writer = csv.DictWriter(
            file,
            fieldnames=PASSIVE_FIELDS,
            lineterminator="\n",
        )

        writer.writeheader()
        writer.writerows(rows)


class FleetNavigationProbe(NavigationProbe):

    def __init__(self, args, sample_writer):

        super().__init__(
            args,
            sample_writer,
        )

        self.passive_namespace = args.passive

        self.passive_rows = []
        self.invalid_passive_samples = 0

        self.monitor_start_ns = None
        self.monitor_end_ns = None

        self.passive_subscription = (
            self.create_subscription(
                Odometry,
                (
                    f"/{self.passive_namespace}"
                    "/platform/odom"
                ),
                self.on_passive_odometry,
                qos_profile_sensor_data,
            )
        )

    def on_passive_odometry(self, msg):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist

        values = {
            "x_m": position.x,
            "y_m": position.y,
            "yaw_rad": quaternion_to_yaw(
                orientation
            ),
            "linear_x_mps": twist.linear.x,
            "angular_z_radps": twist.angular.z,
        }

        if not all(
            math.isfinite(value)
            for value in values.values()
        ):

            self.invalid_passive_samples += 1
            return

        self.passive_rows.append({
            "phase": self.phase,
            "monotonic_ns": time.monotonic_ns(),
            "odom_stamp_ns": stamp_ns(
                msg.header.stamp
            ),
            **values,
        })

    def wait_ready(self):

        super().wait_ready()

        self.get_logger().info(
            "Waiting for stationary fleet telemetry"
        )

        deadline = time.monotonic() + 15.0

        while time.monotonic() < deadline:

            self.tick()

            if not self.passive_rows:
                continue

            latest = self.passive_rows[-1]

            now_ns = time.monotonic_ns()

            if (
                now_ns - latest["monotonic_ns"]
                <= 200_000_000
            ):
                break

        else:

            raise RuntimeError(
                "Passive robot odometry unavailable"
            )

        self.phase = "preflight"

        self.monitor_start_ns = (
            time.monotonic_ns()
        )

        deadline = time.monotonic() + 1.0

        while time.monotonic() < deadline:

            self.tick()

        preflight = [
            row
            for row in self.passive_rows
            if row["phase"] == "preflight"
        ]

        if len(preflight) < MIN_PHASE_SAMPLES:

            raise RuntimeError(
                "Insufficient passive preflight telemetry"
            )

        active = self.get_odom_state()

        if active is None:

            raise RuntimeError(
                "Active robot odometry unavailable"
            )

        if (
            abs(active["linear_x_mps"])
            > LINEAR_LIMIT
            or abs(active["angular_z_radps"])
            > ANGULAR_LIMIT
        ):

            raise RuntimeError(
                "Active robot is not stationary"
            )

        baseline = preflight[0]

        for row in preflight:

            displacement = math.hypot(
                row["x_m"] - baseline["x_m"],
                row["y_m"] - baseline["y_m"],
            )

            yaw_change = abs(
                angle_difference(
                    row["yaw_rad"],
                    baseline["yaw_rad"],
                )
            )

            if (
                displacement > POSITION_LIMIT
                or yaw_change > YAW_LIMIT
                or abs(row["linear_x_mps"])
                    > LINEAR_LIMIT
                or abs(row["angular_z_radps"])
                    > ANGULAR_LIMIT
            ):

                raise RuntimeError(
                    "Passive robot is not stationary"
                )

        timestamps = [
            self.monitor_start_ns,
            *[
                row["monotonic_ns"]
                for row in preflight
            ],
            time.monotonic_ns(),
        ]

        maximum_gap = max(
            (b - a) / 1e9
            for a, b in zip(
                timestamps,
                timestamps[1:],
            )
        )

        if maximum_gap > MAX_SAMPLE_GAP:

            raise RuntimeError(
                "Passive preflight coverage insufficient"
            )

        if (
            preflight[-1]["odom_stamp_ns"]
            <= preflight[0]["odom_stamp_ns"]
        ):

            raise RuntimeError(
                "Passive odometry timestamps not advancing"
            )

        self.get_logger().info(
            "Both robots stationary; fleet ready"
        )

    def analyze_isolation(self):

        phases = (
            "preflight",
            "planning",
            "navigation",
            "settling",
        )

        rows = [
            row
            for row in self.passive_rows
            if row["phase"] in phases
        ]

        counts = {
            phase: sum(
                row["phase"] == phase
                for row in rows
            )
            for phase in phases
        }

        max_displacement = None
        final_displacement = None

        max_yaw_change = None

        max_linear = None
        max_angular = None

        max_gap = None
        advancing_stamps = 0
        stamp_regressions = 0

        if rows:

            baseline = rows[0]

            displacements = [
                math.hypot(
                    row["x_m"] - baseline["x_m"],
                    row["y_m"] - baseline["y_m"],
                )
                for row in rows
            ]

            yaw_changes = [
                abs(
                    angle_difference(
                        row["yaw_rad"],
                        baseline["yaw_rad"],
                    )
                )
                for row in rows
            ]

            max_displacement = max(displacements)

            final_displacement = displacements[-1]

            max_yaw_change = max(yaw_changes)

            max_linear = max(
                abs(row["linear_x_mps"])
                for row in rows
            )

            max_angular = max(
                abs(row["angular_z_radps"])
                for row in rows
            )

            timestamps = [
                row["monotonic_ns"]
                for row in rows
            ]

            if self.monitor_start_ns is not None:

                timestamps.insert(
                    0,
                    self.monitor_start_ns,
                )

            if self.monitor_end_ns is not None:

                timestamps.append(
                    self.monitor_end_ns
                )

            intervals = [
                (b - a) / 1e9
                for a, b in zip(
                    timestamps,
                    timestamps[1:],
                )
            ]

            max_gap = (
                max(intervals)
                if intervals
                else None
            )

            stamps = [
                row["odom_stamp_ns"]
                for row in rows
            ]

            advancing_stamps = sum(
                b > a
                for a, b in zip(
                    stamps,
                    stamps[1:],
                )
            )

            stamp_regressions = sum(
                b < a
                for a, b in zip(
                    stamps,
                    stamps[1:],
                )
            )

        duration = (
            (
                self.monitor_end_ns
                - self.monitor_start_ns
            ) / 1e9
            if self.monitor_start_ns is not None
            and self.monitor_end_ns is not None
            else None
        )

        sufficient_coverage = (
            all(
                counts[phase] >= MIN_PHASE_SAMPLES
                for phase in (
                    "preflight",
                    "navigation",
                    "settling",
                )
            )
            and max_gap is not None
            and max_gap <= MAX_SAMPLE_GAP
            and advancing_stamps > 0
            and stamp_regressions == 0
            and self.invalid_passive_samples == 0
        )

        passive_stationary = (
            sufficient_coverage
            and max_displacement is not None
            and max_displacement <= POSITION_LIMIT
            and max_yaw_change <= YAW_LIMIT
            and max_linear <= LINEAR_LIMIT
            and max_angular <= ANGULAR_LIMIT
        )

        planning = self.summary["planning"]
        navigation = self.summary["navigation"]

        active_succeeded = (
            self.summary["completed"]
            and not self.summary.get("error")
            and not self.summary["requires_manual_stop"]
            and planning.get("status") == "SUCCEEDED"
            and planning.get("error_code") == 0
            and navigation.get("status") == "SUCCEEDED"
            and navigation.get("error_code") == 0
        )

        active_stationary = (
            self.summary.get("stationary") is True
        )

        passed = (
            active_succeeded
            and active_stationary
            and passive_stationary
        )

        return {
            "active_robot": self.args.namespace,
            "passive_robot": self.passive_namespace,
            "active_navigation_succeeded":
                active_succeeded,
            "active_final_stationary":
                active_stationary,
            "passive_observation_duration_s":
                duration,
            "passive_odometry_samples":
                len(rows),
            "passive_samples_per_phase":
                counts,
            "passive_maximum_sample_gap_s":
                max_gap,
            "passive_advancing_timestamps":
                advancing_stamps,
            "passive_timestamp_regressions":
                stamp_regressions,
            "invalid_passive_samples":
                self.invalid_passive_samples,
            "passive_sufficient_coverage":
                sufficient_coverage,
            "passive_maximum_displacement_m":
                max_displacement,
            "passive_final_displacement_m":
                final_displacement,
            "passive_maximum_yaw_change_rad":
                max_yaw_change,
            "passive_maximum_abs_linear_velocity_mps":
                max_linear,
            "passive_maximum_abs_angular_velocity_radps":
                max_angular,
            "passive_stationary_in_observed_odometry":
                passive_stationary,
            "thresholds": {
                "position_m": POSITION_LIMIT,
                "yaw_rad": YAW_LIMIT,
                "linear_velocity_mps": LINEAR_LIMIT,
                "angular_velocity_radps": ANGULAR_LIMIT,
                "sample_gap_s": MAX_SAMPLE_GAP,
                "minimum_samples_per_required_phase":
                    MIN_PHASE_SAMPLES,
            },
            "odometry_navigation_isolation_passed":
                passed,
            "limitations": [
                "Passive odometry is not independent "
                "physical ground truth",
                "Coverage describes received samples",
                "The active robot stationarity criterion "
                "uses the original probe's final sample",
                "Action success does not establish "
                "physical goal accuracy",
                "Monitoring adds executor workload",
                "Passive robot monitoring is diagnostic, "
                "not an automatic emergency stop",
                "Thresholds are experimental choices",
            ],
        }


def parse_args():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--active",
        required=True,
    )

    parser.add_argument(
        "--passive",
        required=True,
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        required=True,
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        required=True,
    )

    parser.add_argument(
        "--goal-yaw",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--goal-timeout",
        type=float,
        default=120.0,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    args = parser.parse_args()

    args.namespace = args.active.strip("/")
    args.passive = args.passive.strip("/")

    for namespace in (
        args.namespace,
        args.passive,
    ):

        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*",
            namespace,
        ):

            parser.error(
                f"Invalid namespace: {namespace}"
            )

    if args.namespace == args.passive:

        parser.error(
            "Active and passive robots must differ"
        )

    if not all(
        math.isfinite(value)
        for value in (
            args.goal_x,
            args.goal_y,
            args.goal_yaw,
            args.goal_timeout,
        )
    ):

        parser.error(
            "Numeric arguments must be finite"
        )

    if args.goal_timeout <= 0:

        parser.error(
            "Goal timeout must be positive"
        )

    if not args.execute:

        parser.error(
            "Autonomous navigation requires --execute"
        )

    return args


def main():

    args = parse_args()

    args.output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    rclpy.init(
        args=[],
        signal_handler_options=SignalHandlerOptions.NO,
    )

    exit_code = 0

    with (
        args.output_dir / "samples.csv"
    ).open("w", newline="") as file:

        writer = csv.writer(
            file,
            lineterminator="\n",
        )

        writer.writerow(ACTIVE_FIELDS)

        probe = FleetNavigationProbe(
            args,
            writer,
        )

        try:

            probe.run()

        except KeyboardInterrupt:

            probe.summary["error"] = (
                "Interrupted by user"
            )

            exit_code = 130

        except Exception as exc:

            probe.summary["error"] = str(exc)

            probe.get_logger().error(str(exc))

            exit_code = 1

        finally:

            probe.monitor_end_ns = (
                time.monotonic_ns()
            )

            try:

                probe.finalize()

                isolation = (
                    probe.analyze_isolation()
                )

                save_json(
                    args.output_dir / "summary.json",
                    probe.summary,
                )

                save_csv(
                    args.output_dir / "passive_samples.csv",
                    probe.passive_rows,
                )

                save_json(
                    args.output_dir / "isolation_summary.json",
                    isolation,
                )

                print(
                    json.dumps(
                        probe.summary,
                        indent=2,
                    )
                )

                print(
                    "\n===== NAVIGATION ISOLATION ====="
                )

                print(
                    json.dumps(
                        isolation,
                        indent=2,
                    )
                )

                if not isolation[
                    "odometry_navigation_isolation_passed"
                ] and exit_code == 0:

                    exit_code = 1

                if probe.summary[
                    "requires_manual_stop"
                ]:

                    print(
                        "WARNING: Action termination "
                        "not confirmed. Stop fleet navigation.",
                        file=sys.stderr,
                    )

            finally:

                probe.destroy_node()

                if rclpy.ok():

                    rclpy.shutdown()

    return exit_code


if __name__ == "__main__":

    sys.exit(main())