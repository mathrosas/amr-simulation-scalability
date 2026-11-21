#!/usr/bin/env python3

"""Test motion-control isolation between two simulated robots."""

import argparse
import csv
import json
import math
import time

from pathlib import Path

import rclpy

from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from control_probe import ControlProbe


FIELDS = [
    "phase",
    "stream",
    "monotonic_ns",
    "received_sim_time_ns",
    "header_stamp_ns",
    "linear_x_mps",
    "angular_z_radps",
    "odom_x_m",
    "odom_y_m",
]


SPEED = 0.10
RATE = 20.0
MOTION_DURATION = 2.0

PASSIVE_POSITION_LIMIT = 0.005
PASSIVE_LINEAR_LIMIT = 0.005
PASSIVE_ANGULAR_LIMIT = 0.01


class FleetControlProbe(ControlProbe):

    def __init__(self, active, passive):
        super().__init__(active)

        self.passive_namespace = (
            "/" + passive.strip("/")
        )

        self.passive_rows = []
        self.passive_pose = None

        self.passive_odom_subscription = (
            self.create_subscription(
                Odometry,
                (
                    f"{self.passive_namespace}"
                    "/platform/odom"
                ),
                self.on_passive_odometry,
                qos_profile_sensor_data,
            )
        )

        self.passive_controller_subscription = (
            self.create_subscription(
                TwistStamped,
                (
                    f"{self.passive_namespace}"
                    "/platform_velocity_controller"
                    "/cmd_vel_out"
                ),
                self.on_passive_controller,
                qos_profile_sensor_data,
            )
        )

        self.passive_mux_subscription = (
            self.create_subscription(
                TwistStamped,
                (
                    f"{self.passive_namespace}"
                    "/platform/cmd_vel"
                ),
                self.on_passive_mux,
                10,
            )
        )

    def record_passive(
        self,
        stream,
        stamp,
        linear_x,
        angular_z,
        x="",
        y="",
    ):
        self.passive_rows.append({
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

    def on_passive_odometry(self, msg):
        position = msg.pose.pose.position

        self.passive_pose = (
            position.x,
            position.y,
        )

        self.record_passive(
            "odometry",
            msg.header.stamp,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
            position.x,
            position.y,
        )

    def on_passive_controller(self, msg):
        self.record_passive(
            "controller_output",
            msg.header.stamp,
            msg.twist.linear.x,
            msg.twist.angular.z,
        )

    def on_passive_mux(self, msg):
        self.record_passive(
            "mux_output",
            msg.header.stamp,
            msg.twist.linear.x,
            msg.twist.angular.z,
        )

    def wait_fleet_ready(self):
        self.wait_ready(timeout=15.0)

        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if self.passive_pose is not None:
                return

        raise RuntimeError(
            "Passive robot odometry unavailable"
        )


def select(rows, stream, phase=None):
    return [
        row for row in rows
        if row["stream"] == stream
        and (
            phase is None
            or row["phase"] == phase
        )
    ]


def save_csv(path, rows):
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=FIELDS,
            lineterminator="\n",
        )

        writer.writeheader()
        writer.writerows(rows)


def save_json(path, data):
    path.write_text(
        json.dumps(
            data,
            indent=2,
            allow_nan=False,
        ) + "\n"
    )


def maximum_absolute(rows, field):
    if not rows:
        return None

    return max(
        abs(row[field])
        for row in rows
    )


def pose_from_row(row):
    return (
        row["odom_x_m"],
        row["odom_y_m"],
    )


def analyze_passive(rows):
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

    initial = select(
        rows,
        "odometry",
        "initial_stop",
    )

    baseline = (
        pose_from_row(initial[0])
        if initial
        else None
    )

    maximum_displacement = None
    final_displacement = None

    if baseline is not None and odometry:
        maximum_displacement = max(
            math.dist(
                baseline,
                pose_from_row(row),
            )
            for row in odometry
        )

        final_displacement = math.dist(
            baseline,
            pose_from_row(odometry[-1]),
        )

    max_linear = maximum_absolute(
        odometry,
        "linear_x_mps",
    )

    max_angular = maximum_absolute(
        odometry,
        "angular_z_radps",
    )

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

    max_interval = (
        max(intervals)
        if intervals
        else None
    )

    # Diagnostic coverage requirements.
    sufficient_coverage = (
        all(
            count >= 10
            for count in phase_counts.values()
        )
        and max_interval is not None
        and max_interval <= 0.2
    )

    stationary = (
        sufficient_coverage
        and maximum_displacement is not None
        and max_linear is not None
        and max_angular is not None
        and maximum_displacement
            <= PASSIVE_POSITION_LIMIT
        and max_linear <= PASSIVE_LINEAR_LIMIT
        and max_angular <= PASSIVE_ANGULAR_LIMIT
    )

    controller = [
        row for row in rows
        if row["stream"] == "controller_output"
        and row["phase"] in phases
    ]

    mux = [
        row for row in rows
        if row["stream"] == "mux_output"
        and row["phase"] in phases
    ]

    return {
        "odometry_samples": len(odometry),
        "odometry_samples_per_phase": phase_counts,
        "maximum_odometry_interval_s": max_interval,
        "sufficient_coverage": sufficient_coverage,
        "maximum_odom_displacement_m":
            maximum_displacement,
        "final_odom_displacement_m":
            final_displacement,
        "maximum_abs_linear_velocity_mps":
            max_linear,
        "maximum_abs_angular_velocity_radps":
            max_angular,
        "controller_output_samples":
            len(controller),
        "maximum_abs_controller_velocity_mps":
            maximum_absolute(
                controller,
                "linear_x_mps",
            ),
        "mux_output_samples": len(mux),
        "maximum_abs_mux_velocity_mps":
            maximum_absolute(
                mux,
                "linear_x_mps",
            ),
        "position_limit_m":
            PASSIVE_POSITION_LIMIT,
        "linear_velocity_limit_mps":
            PASSIVE_LINEAR_LIMIT,
        "angular_velocity_limit_radps":
            PASSIVE_ANGULAR_LIMIT,
        "stationary_in_observed_odometry":
            stationary,
    }


def main():
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
        "--output-dir",
        required=True,
        type=Path,
    )

    args = parser.parse_args()

    if (
        args.active.strip("/")
        == args.passive.strip("/")
    ):
        parser.error(
            "Active and passive robots must differ"
        )

    args.output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = FleetControlProbe(
        args.active,
        args.passive,
    )

    error = None
    armed = False
    motion = None
    start_pose = None
    finish_pose = None

    try:
        print(
            "Waiting for both robots...",
            flush=True,
        )

        node.wait_fleet_ready()

        armed = True

        node.run_phase(
            "initial_stop",
            0.0,
            1.0,
            RATE,
        )

        for rows in (
            node.rows,
            node.passive_rows,
        ):
            initial_odom = select(
                rows,
                "odometry",
                "initial_stop",
            )

            if len(initial_odom) < 10:
                raise RuntimeError(
                    "Insufficient initial odometry"
                )

            if (
                maximum_absolute(
                    initial_odom,
                    "linear_x_mps",
                ) > PASSIVE_LINEAR_LIMIT
                or maximum_absolute(
                    initial_odom,
                    "angular_z_radps",
                ) > PASSIVE_ANGULAR_LIMIT
            ):
                raise RuntimeError(
                    "A robot is not stationary"
                )

        if not select(
            node.rows,
            "controller_output",
            "initial_stop",
        ):
            raise RuntimeError(
                "Active controller output unavailable"
            )

        start_pose = node.last_pose

        print(
            f"Moving {args.active}...",
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
                    "Stopping active robot...",
                    flush=True,
                )

                node.run_phase(
                    "stopping",
                    0.0,
                    2.0,
                    RATE,
                )

            except (Exception, KeyboardInterrupt) as exc:
                error = (
                    f"{error or ''}; "
                    f"Stopping failed: {exc}"
                )

        finish_pose = node.last_pose

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    active_motion_odom = select(
        node.rows,
        "odometry",
        "motion",
    )

    active_motion_controller = select(
        node.rows,
        "controller_output",
        "motion",
    )

    active_stop_odom = select(
        node.rows,
        "odometry",
        "stopping",
    )

    displacement = (
        math.dist(start_pose, finish_pose)
        if start_pose is not None
        and finish_pose is not None
        else None
    )

    ideal_distance = (
        SPEED * motion["elapsed_sim_s"]
        if motion is not None
        else None
    )

    summary = {
        "namespace": node.namespace_name,
        "requested_speed_mps": SPEED,
        "requested_duration_wall_s":
            MOTION_DURATION,
        "target_rate_hz": RATE,
        "motion": motion,
        "motion_controller_samples":
            len(active_motion_controller),
        "motion_odom_samples":
            len(active_motion_odom),
        "ideal_commanded_distance_m":
            ideal_distance,
        "odom_displacement_m":
            displacement,
        "error": error,
    }

    passive_summary = analyze_passive(
        node.passive_rows
    )

    final_stop = []

    if active_stop_odom:
        end_ns = active_stop_odom[-1][
            "monotonic_ns"
        ]

        final_stop = [
            row for row in active_stop_odom
            if row["monotonic_ns"]
            >= end_ns - 500_000_000
        ]

    active_reached_speed = any(
        row["linear_x_mps"] >= SPEED * 0.90
        for row in active_motion_odom
    )

    active_stopped = (
        len(final_stop) >= 5
        and maximum_absolute(
            final_stop,
            "linear_x_mps",
        ) <= PASSIVE_LINEAR_LIMIT
        and maximum_absolute(
            final_stop,
            "angular_z_radps",
        ) <= PASSIVE_ANGULAR_LIMIT
    )

    isolation_passed = (
        error is None
        and motion is not None
        and bool(active_motion_controller)
        and active_reached_speed
        and active_stopped
        and passive_summary[
            "stationary_in_observed_odometry"
        ]
    )

    isolation_summary = {
        "active_robot": args.active,
        "passive_robot": args.passive,
        "active_reached_90_percent_speed":
            active_reached_speed,
        "active_stationary_in_final_window":
            active_stopped,
        "active_odom_displacement_m":
            displacement,
        "passive_observations":
            passive_summary,
        "odometry_isolation_passed":
            isolation_passed,
        "error": error,
        "limitations": [
            "Odometry is not independent ground truth",
            "Passive control topics may publish no messages",
            "Missing control messages are not zero measurements",
            "Thresholds are diagnostic choices",
            "Observation coverage is sampled",
        ],
    }

    save_csv(
        args.output_dir / "samples.csv",
        node.rows,
    )

    save_csv(
        args.output_dir / "passive_samples.csv",
        node.passive_rows,
    )

    save_json(
        args.output_dir / "summary.json",
        summary,
    )

    save_json(
        args.output_dir / "isolation_summary.json",
        isolation_summary,
    )

    print(
        json.dumps(
            isolation_summary,
            indent=2,
        )
    )

    if not isolation_passed:
        raise SystemExit(
            "Fleet isolation test did not pass"
        )


if __name__ == "__main__":
    main()