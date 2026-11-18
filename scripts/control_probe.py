#!/usr/bin/env python3
"""Measure Jackal velocity commands and odometry response."""

import argparse
import csv
import json
import math
import statistics
import time

from pathlib import Path

import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class ControlProbe(Node):

    def __init__(self, namespace):
        super().__init__("amr_control_probe")

        self.set_parameters([
            Parameter(
                "use_sim_time",
                Parameter.Type.BOOL,
                True,
            ),
        ])

        self.namespace_name = "/" + namespace.strip("/")

        self.phase = "preflight"
        self.rows = []

        self.last_pose = None

        self.publisher = self.create_publisher(
            TwistStamped,
            f"{self.namespace_name}/cmd_vel",
            10,
        )

        self.controller_subscription = (
            self.create_subscription(
                TwistStamped,
                (
                    f"{self.namespace_name}"
                    "/platform_velocity_controller/cmd_vel_out"
                ),
                self.on_controller,
                qos_profile_sensor_data,
            )
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            f"{self.namespace_name}/platform/odom",
            self.on_odometry,
            qos_profile_sensor_data,
        )

        self.mux_subscription = self.create_subscription(
            TwistStamped,
            f"{self.namespace_name}/platform/cmd_vel",
            self.on_mux_output,
            10,
        )

    def record(
        self,
        stream,
        stamp,
        linear_x,
        angular_z,
        x="",
        y="",
    ):
        self.rows.append({
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

    def on_mux_output(self, msg):
        self.record(
            "mux_output",
            msg.header.stamp,
            msg.twist.linear.x,
            msg.twist.angular.z,
        )
    
    def on_controller(self, msg):
        self.record(
            "controller_output",
            msg.header.stamp,
            msg.twist.linear.x,
            msg.twist.angular.z,
        )

    def on_odometry(self, msg):
        position = msg.pose.pose.position

        self.last_pose = (
            position.x,
            position.y,
        )

        self.record(
            "odometry",
            msg.header.stamp,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
            position.x,
            position.y,
        )

    def publish_velocity(self, speed):
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = speed
        msg.twist.angular.z = 0.0

        self.publisher.publish(msg)

        self.record(
            "requested_command",
            msg.header.stamp,
            speed,
            0.0,
        )

    def wait_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout

        first_clock = None

        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            clock = self.get_clock().now().nanoseconds

            if clock > 0 and first_clock is None:
                first_clock = clock

            if (
                first_clock is not None
                and clock > first_clock
                and self.last_pose is not None
                and self.publisher.get_subscription_count() > 0
            ):
                return

        raise RuntimeError(
            "Preflight failed: missing advancing clock, "
            "odometry or command subscriber"
        )

    def run_phase(
        self,
        phase,
        speed,
        duration,
        rate,
    ):
        self.phase = phase

        period = 1.0 / rate

        start_wall = time.monotonic()
        start_sim = self.get_clock().now().nanoseconds

        deadline = start_wall + duration
        next_command = start_wall

        commands = 0

        while time.monotonic() < deadline and rclpy.ok():
            now = time.monotonic()

            if now >= next_command:
                self.publish_velocity(speed)

                commands += 1

                next_command = now + period

            rclpy.spin_once(
                self,
                timeout_sec=min(
                    0.01,
                    max(
                        0.0,
                        next_command - time.monotonic(),
                    ),
                    max(
                        0.0,
                        deadline - time.monotonic(),
                    ),
                ),
            )

        end_sim = self.get_clock().now().nanoseconds

        return {
            "commands_published": commands,
            "elapsed_wall_s": (
                time.monotonic() - start_wall
            ),
            "elapsed_sim_s": (
                (end_sim - start_sim) / 1e9
            ),
        }


def mean_velocity(rows, stream, phase):
    values = [
        row["linear_x_mps"]
        for row in rows
        if row["stream"] == stream
        and row["phase"] == phase
    ]

    return statistics.mean(values) if values else None


def sample_count(rows, stream, phase):
    return sum(
        row["stream"] == stream
        and row["phase"] == phase
        for row in rows
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--namespace",
        default="j100_0000",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--output-dir",
        required=True,
    )

    args = parser.parse_args()

    if not 0 < args.speed <= 0.15:
        parser.error("Require 0 < speed <= 0.15 m/s")

    if not 0.5 <= args.duration <= 3.0:
        parser.error("Require duration between 0.5 and 3 s")

    if not 10 <= args.rate <= 50:
        parser.error("Require rate between 10 and 50 Hz")

    output_dir = Path(args.output_dir)

    output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = ControlProbe(args.namespace)

    error = None
    armed = False

    start_pose = None
    motion = None

    try:
        print("Waiting for ROS interfaces...", flush=True)

        node.wait_ready()

        print("Preflight successful.", flush=True)

        armed = True

        node.run_phase(
            "initial_stop",
            0.0,
            1.0,
            args.rate,
        )

        if sample_count(
            node.rows,
            "controller_output",
            "initial_stop",
        ) == 0:
            raise RuntimeError(
                "No controller output received"
            )

        if sample_count(
            node.rows,
            "odometry",
            "initial_stop",
        ) == 0:
            raise RuntimeError(
                "No fresh odometry received"
            )

        start_pose = node.last_pose

        print("Starting motion phase...", flush=True)

        motion = node.run_phase(
            "motion",
            args.speed,
            args.duration,
            args.rate,
        )

    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__

    finally:
        if armed and rclpy.ok():
            try:
                print("Stopping robot...", flush=True)

                node.run_phase(
                    "stopping",
                    0.0,
                    2.0,
                    args.rate,
                )

            except (Exception, KeyboardInterrupt) as exc:
                error = (
                    f"{error or ''}; "
                    f"Stopping phase failed: {exc}"
                )

        finish_pose = node.last_pose

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    displacement = None

    if start_pose is not None and finish_pose is not None:
        displacement = math.dist(
            start_pose,
            finish_pose,
        )

    motion_controller_mean = mean_velocity(
        node.rows,
        "controller_output",
        "motion",
    )

    motion_odom_mean = mean_velocity(
        node.rows,
        "odometry",
        "motion",
    )

    expected_distance = (
        args.speed * motion["elapsed_sim_s"]
        if motion is not None
        else None
    )

    summary = {
        "namespace": node.namespace_name,
        "requested_speed_mps": args.speed,
        "requested_duration_wall_s": args.duration,
        "target_rate_hz": args.rate,
        "motion": motion,
        "motion_controller_samples": sample_count(
            node.rows,
            "controller_output",
            "motion",
        ),
        "motion_odom_samples": sample_count(
            node.rows,
            "odometry",
            "motion",
        ),
        "mean_controller_velocity_mps":
            motion_controller_mean,
        "mean_odometry_velocity_mps":
            motion_odom_mean,
        "mean_stopping_odometry_velocity_mps":
            mean_velocity(
                node.rows,
                "odometry",
                "stopping",
            ),
        "ideal_commanded_distance_m":
            expected_distance,
        "odom_displacement_m":
            displacement,
        "error": error,
    }

    fields = [
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

    with (output_dir / "samples.csv").open(
        "w",
        newline="",
    ) as output:
        writer = csv.DictWriter(
            output,
            fieldnames=fields,
        )

        writer.writeheader()
        writer.writerows(node.rows)

    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )

    print("\n===== CONTROL PROBE RESULTS =====")
    print(json.dumps(summary, indent=2))

    print(f"\nSaved in: {output_dir}")

    if (
        error is not None
        or summary["motion_controller_samples"] == 0
        or summary["motion_odom_samples"] == 0
    ):
        raise SystemExit(
            "Incomplete control experiment"
        )


if __name__ == "__main__":
    main()