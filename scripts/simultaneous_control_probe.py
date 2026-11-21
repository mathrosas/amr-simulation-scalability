#!/usr/bin/env python3

"""Command and monitor multiple simulated robots simultaneously."""

import argparse
import json
import math
import re
import time

from functools import partial
from pathlib import Path

import rclpy

from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from control_probe import ControlProbe

from fleet_control_probe import (
    SPEED,
    RATE,
    MOTION_DURATION,
    PASSIVE_LINEAR_LIMIT,
    PASSIVE_ANGULAR_LIMIT,
    select,
    save_csv,
    save_json,
    maximum_absolute,
    pose_from_row,
)


MIN_ODOM_SAMPLES = 10

MAX_ODOM_INTERVAL = 0.2

MAX_DISTANCE_ERROR = 0.03


class SimultaneousControlProbe(ControlProbe):

    def __init__(self, robots):
        super().__init__(robots[0])

        self.robots = robots

        self.robot_rows = {
            robots[0]: self.rows,
        }

        self.robot_publishers = {
            robots[0]: self.publisher,
        }

        self.extra_subscriptions = []

        for robot in robots[1:]:

            self.robot_rows[robot] = []

            self.robot_publishers[robot] = (
                self.create_publisher(
                    TwistStamped,
                    f"/{robot}/cmd_vel",
                    10,
                )
            )

            interfaces = [
                (
                    "odometry",
                    Odometry,
                    f"/{robot}/platform/odom",
                    qos_profile_sensor_data,
                ),
                (
                    "controller_output",
                    TwistStamped,
                    (
                        f"/{robot}"
                        "/platform_velocity_controller"
                        "/cmd_vel_out"
                    ),
                    qos_profile_sensor_data,
                ),
                (
                    "mux_output",
                    TwistStamped,
                    f"/{robot}/platform/cmd_vel",
                    10,
                ),
            ]

            for stream, msg_type, topic, qos in interfaces:

                subscription = self.create_subscription(
                    msg_type,
                    topic,
                    partial(
                        self.on_robot_message,
                        robot,
                        stream,
                    ),
                    qos,
                )

                self.extra_subscriptions.append(
                    subscription
                )

    def record_robot(
        self,
        robot,
        stream,
        stamp,
        linear_x,
        angular_z,
        x="",
        y="",
    ):
        self.robot_rows[robot].append({
            "phase": self.phase,
            "stream": stream,
            "monotonic_ns": time.monotonic_ns(),
            "received_sim_time_ns":
                self.get_clock().now().nanoseconds,
            "header_stamp_ns": (
                stamp.sec * 1_000_000_000
                + stamp.nanosec
            ),
            "linear_x_mps": linear_x,
            "angular_z_radps": angular_z,
            "odom_x_m": x,
            "odom_y_m": y,
        })

    def on_robot_message(
        self,
        robot,
        stream,
        msg,
    ):

        if stream == "odometry":
            position = msg.pose.pose.position

            self.record_robot(
                robot,
                stream,
                msg.header.stamp,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
                position.x,
                position.y,
            )

        else:
            self.record_robot(
                robot,
                stream,
                msg.header.stamp,
                msg.twist.linear.x,
                msg.twist.angular.z,
            )

    def publish_velocity(self, speed):

        # One message timestamp per command cycle.
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = speed
        msg.twist.angular.z = 0.0

        # Publish independently to each robot.
        for robot in self.robots:

            self.robot_publishers[robot].publish(
                msg
            )

            self.record_robot(
                robot,
                "requested_command",
                msg.header.stamp,
                speed,
                0.0,
            )

    def wait_all_ready(self, timeout=20.0):

        deadline = time.monotonic() + timeout

        first_clock = None

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            clock = (
                self.get_clock().now().nanoseconds
            )

            if clock > 0 and first_clock is None:
                first_clock = clock

            now_ns = time.monotonic_ns()

            ready = True

            for robot in self.robots:

                odom = select(
                    self.robot_rows[robot],
                    "odometry",
                )

                if (
                    not odom
                    or now_ns - odom[-1]["monotonic_ns"]
                        > 200_000_000
                    or self.robot_publishers[robot]
                        .get_subscription_count() == 0
                ):
                    ready = False

            if (
                ready
                and first_clock is not None
                and clock > first_clock
            ):
                return

        raise RuntimeError(
            "Fleet preflight failed: missing clock, "
            "fresh odometry or command subscribers"
        )


def analyze_robot(
    rows,
    start_pose,
    motion,
    error,
    stop_end_ns,
):

    phases = (
        "initial_stop",
        "motion",
        "stopping",
    )

    odometry = [
        row for row in rows
        if row["stream"] == "odometry"
        and row["phase"] in phases
    ]

    phase_counts = {
        phase: len(
            select(rows, "odometry", phase)
        )
        for phase in phases
    }

    intervals = [
        (
            current["monotonic_ns"]
            - previous["monotonic_ns"]
        ) / 1e9
        for previous, current in zip(
            odometry,
            odometry[1:],
        )
    ]

    maximum_interval = (
        max(intervals)
        if intervals
        else None
    )

    sufficient_coverage = (
        all(
            count >= MIN_ODOM_SAMPLES
            for count in phase_counts.values()
        )
        and maximum_interval is not None
        and maximum_interval <= MAX_ODOM_INTERVAL
    )

    controller = select(
        rows,
        "controller_output",
        "motion",
    )

    motion_odom = select(
        rows,
        "odometry",
        "motion",
    )

    commands = select(
        rows,
        "requested_command",
        "motion",
    )

    mux = select(
        rows,
        "mux_output",
        "motion",
    )

    stop_odom = select(
        rows,
        "odometry",
        "stopping",
    )

    final_window = []

    if stop_odom and stop_end_ns is not None:
        final_window = [
            row for row in stop_odom
            if row["monotonic_ns"]
                >= stop_end_ns - 500_000_000
        ]

    fresh_final_sample = (
        bool(stop_odom)
        and stop_end_ns is not None
        and stop_end_ns
            - stop_odom[-1]["monotonic_ns"]
                <= 200_000_000
    )

    stopped = (
        fresh_final_sample
        and len(final_window) >= 5
        and maximum_absolute(
            final_window,
            "linear_x_mps",
        ) <= PASSIVE_LINEAR_LIMIT
        and maximum_absolute(
            final_window,
            "angular_z_radps",
        ) <= PASSIVE_ANGULAR_LIMIT
    )

    reached_speed = any(
        row["linear_x_mps"] >= SPEED * 0.90
        for row in motion_odom
    )

    controller_reached_speed = any(
        row["linear_x_mps"] >= SPEED * 0.90
        for row in controller
    )

    displacement = (
        math.dist(
            start_pose,
            pose_from_row(stop_odom[-1]),
        )
        if start_pose is not None
        and stop_odom
        else None
    )

    ideal_distance = (
        SPEED * motion["elapsed_sim_s"]
        if motion is not None
        else None
    )

    distance_error = (
        displacement - ideal_distance
        if displacement is not None
        and ideal_distance is not None
        else None
    )

    passed = (
        error is None
        and motion is not None
        and sufficient_coverage
        and len(commands) > 0
        and len(mux) > 0
        and reached_speed
        and controller_reached_speed
        and stopped
        and distance_error is not None
        and abs(distance_error)
            <= MAX_DISTANCE_ERROR
    )

    return {
        "passed": passed,
        "motion_commands": len(commands),
        "motion_mux_samples": len(mux),
        "motion_controller_samples": len(controller),
        "motion_odom_samples": len(motion_odom),
        "odometry_samples_per_phase": phase_counts,
        "maximum_odometry_interval_s":
            maximum_interval,
        "sufficient_coverage": sufficient_coverage,
        "controller_reached_90_percent_speed":
            controller_reached_speed,
        "odometry_reached_90_percent_speed":
            reached_speed,
        "stationary_in_final_window": stopped,
        "ideal_commanded_distance_m":
            ideal_distance,
        "odom_displacement_m": displacement,
        "signed_distance_error_m": distance_error,
        "distance_error_limit_m":
            MAX_DISTANCE_ERROR,
    }


def command_batch_skew(
    robot_rows,
    robots,
    phase,
):

    timestamps = [
        [
            row["monotonic_ns"]
            for row in select(
                robot_rows[robot],
                "requested_command",
                phase,
            )
        ]
        for robot in robots
    ]

    counts = [
        len(values)
        for values in timestamps
    ]

    if (
        not counts
        or min(counts) == 0
        or len(set(counts)) != 1
    ):
        return None

    skew_ms = [
        (max(batch) - min(batch)) / 1e6
        for batch in zip(*timestamps)
    ]

    return {
        "command_batches": len(skew_ms),
        "first_batch_skew_ms": skew_ms[0],
        "mean_batch_skew_ms":
            sum(skew_ms) / len(skew_ms),
        "maximum_batch_skew_ms":
            max(skew_ms),
    }


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--robots",
        nargs="+",
        required=True,
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
    )

    args = parser.parse_args()

    robots = [
        robot.strip("/")
        for robot in args.robots
    ]

    if len(robots) < 2:
        parser.error(
            "Require at least two robots"
        )

    if len(set(robots)) != len(robots):
        parser.error(
            "Duplicate robot namespaces"
        )

    if not all(
        re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*",
            robot,
        )
        for robot in robots
    ):
        parser.error(
            "Invalid robot namespace"
        )

    args.output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = SimultaneousControlProbe(robots)

    error = None
    armed = False
    motion = None
    stop_end_ns = None
    start_poses = {}

    try:

        print(
            "Waiting for all robots...",
            flush=True,
        )

        node.wait_all_ready()

        armed = True

        node.run_phase(
            "initial_stop",
            0.0,
            1.0,
            RATE,
        )

        for robot in robots:

            rows = node.robot_rows[robot]

            odom = select(
                rows,
                "odometry",
                "initial_stop",
            )

            controller = select(
                rows,
                "controller_output",
                "initial_stop",
            )

            mux = select(
                rows,
                "mux_output",
                "initial_stop",
            )

            if (
                len(odom) < MIN_ODOM_SAMPLES
                or not controller
                or not mux
            ):
                raise RuntimeError(
                    f"{robot}: incomplete preflight telemetry"
                )

            if (
                maximum_absolute(
                    odom,
                    "linear_x_mps",
                ) > PASSIVE_LINEAR_LIMIT
                or maximum_absolute(
                    odom,
                    "angular_z_radps",
                ) > PASSIVE_ANGULAR_LIMIT
            ):
                raise RuntimeError(
                    f"{robot}: not stationary"
                )

            start_poses[robot] = pose_from_row(
                odom[-1]
            )

        print(
            f"Moving {len(robots)} robots...",
            flush=True,
        )

        motion = node.run_phase(
            "motion",
            SPEED,
            MOTION_DURATION,
            RATE,
        )

    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__

    finally:

        if armed and rclpy.ok():

            try:

                print(
                    "Stopping all robots...",
                    flush=True,
                )

                node.run_phase(
                    "stopping",
                    0.0,
                    2.0,
                    RATE,
                )

                stop_end_ns = time.monotonic_ns()

            except (Exception, KeyboardInterrupt) as exc:
                error = (
                    f"{error or ''}; "
                    f"Stopping failed: {exc}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    results = {}

    for robot in robots:

        rows = node.robot_rows[robot]

        result = analyze_robot(
            rows,
            start_poses.get(robot),
            motion,
            error,
            stop_end_ns,
        )

        results[robot] = result

        directory = args.output_dir / robot

        directory.mkdir()

        save_csv(
            directory / "samples.csv",
            rows,
        )

        summary = {
            "namespace": f"/{robot}",
            "requested_speed_mps": SPEED,
            "requested_duration_wall_s":
                MOTION_DURATION,
            "target_rate_hz": RATE,
            "motion": motion,
            "motion_controller_samples":
                result["motion_controller_samples"],
            "motion_odom_samples":
                result["motion_odom_samples"],
            "ideal_commanded_distance_m":
                result["ideal_commanded_distance_m"],
            "odom_displacement_m":
                result["odom_displacement_m"],
            "error": error,
        }

        save_json(
            directory / "summary.json",
            summary,
        )

    passed = all(
        result["passed"]
        for result in results.values()
    )

    fleet_summary = {
        "robot_count": len(robots),
        "robots": robots,
        "requested_speed_mps": SPEED,
        "target_rate_per_robot_hz": RATE,
        "total_target_command_rate_hz":
            RATE * len(robots),
        "motion": motion,
        "effective_motion_rtf": (
            motion["elapsed_sim_s"]
            / motion["elapsed_wall_s"]
            if motion is not None
            else None
        ),
        "motion_command_batch_skew":
            command_batch_skew(
                node.robot_rows,
                robots,
                "motion",
            ),
        "stopping_command_batch_skew":
            command_batch_skew(
                node.robot_rows,
                robots,
                "stopping",
            ),
        "robot_results": results,
        "simultaneous_control_passed": passed,
        "error": error,
        "limitations": [
            "Commands are published sequentially "
            "within each shared command cycle",
            "Publication timestamp skew is not "
            "controller execution skew",
            "Odometry is not independent ground truth",
            "Diagnostic thresholds are not "
            "manufacturer specifications",
        ],
    }

    save_json(
        args.output_dir / "fleet_summary.json",
        fleet_summary,
    )

    print(
        json.dumps(
            fleet_summary,
            indent=2,
        )
    )

    if not passed:
        raise SystemExit(
            "Simultaneous control test failed"
        )


if __name__ == "__main__":
    main()