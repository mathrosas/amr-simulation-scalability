#!/usr/bin/env python3

"""Initialize AMCL poses from the fleet configuration."""

import argparse
import json
import math
import re
import time

from functools import partial
from pathlib import Path

import yaml
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


POSITION_VARIANCE = 0.25
YAW_VARIANCE = 0.06854

READY_TIMEOUT = 30.0
POSE_TIMEOUT = 30.0

MAX_POSITION_RESIDUAL = 0.5
MAX_YAW_RESIDUAL = 0.5


def load_fleet(path):

    with path.open() as file:
        config = yaml.safe_load(file)

    robots = {}

    for robot in config["robots"]:

        namespace = robot["namespace"]

        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*",
            namespace,
        ):
            raise ValueError(
                f"Invalid namespace: {namespace}"
            )

        if namespace in robots:
            raise ValueError(
                f"Duplicate namespace: {namespace}"
            )

        pose = {
            key: float(robot["spawn"][key])
            for key in ("x", "y", "yaw")
        }

        if not all(
            math.isfinite(value)
            for value in pose.values()
        ):
            raise ValueError(
                f"Invalid pose: {namespace}"
            )

        robots[namespace] = pose

    if not robots:
        raise ValueError("Fleet is empty")

    return robots


def stamp_ns(stamp):

    return (
        stamp.sec * 1_000_000_000
        + stamp.nanosec
    )


def quaternion_yaw(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference(a, b):

    difference = a - b

    return math.atan2(
        math.sin(difference),
        math.cos(difference),
    )


class FleetInitializer(Node):

    def __init__(self, robots):

        super().__init__("amr_fleet_initializer")

        self.set_parameters([
            Parameter(
                "use_sim_time",
                Parameter.Type.BOOL,
                True,
            ),
        ])

        self.robots = robots

        self.publishers_by_robot = {}
        self.subscriptions_by_robot = []
        self.lifecycle_clients = {}

        self.odometry = {}
        self.scans = {}
        self.estimates = {}

        for robot in robots:

            self.publishers_by_robot[robot] = (
                self.create_publisher(
                    PoseWithCovarianceStamped,
                    f"/{robot}/initialpose",
                    10,
                )
            )

            interfaces = [
                (
                    Odometry,
                    f"/{robot}/platform/odom",
                    self.on_odometry,
                ),
                (
                    LaserScan,
                    f"/{robot}/sensors/lidar2d_0/scan",
                    self.on_scan,
                ),
                (
                    PoseWithCovarianceStamped,
                    f"/{robot}/amcl_pose",
                    self.on_estimate,
                ),
            ]

            for msg_type, topic, callback in interfaces:

                subscription = self.create_subscription(
                    msg_type,
                    topic,
                    partial(callback, robot),
                    qos_profile_sensor_data,
                )

                self.subscriptions_by_robot.append(
                    subscription
                )

            for lifecycle_node in (
                "map_server",
                "amcl",
            ):

                self.lifecycle_clients[
                    (robot, lifecycle_node)
                ] = self.create_client(
                    GetState,
                    (
                        f"/{robot}/{lifecycle_node}"
                        "/get_state"
                    ),
                )

    def on_odometry(self, robot, msg):

        self.odometry[robot] = (
            time.monotonic(),
            msg,
        )

    def on_scan(self, robot, msg):

        self.scans[robot] = (
            time.monotonic(),
            msg,
        )

    def on_estimate(self, robot, msg):

        self.estimates[robot] = (
            time.monotonic(),
            msg,
        )

    def verify_lifecycle(self):

        for key, client in self.lifecycle_clients.items():

            robot, lifecycle_node = key

            if not client.wait_for_service(
                timeout_sec=10.0
            ):
                raise RuntimeError(
                    f"{robot}/{lifecycle_node}: "
                    "service unavailable"
                )

            future = client.call_async(
                GetState.Request()
            )

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=10.0,
            )

            if not future.done():
                raise RuntimeError(
                    f"{robot}/{lifecycle_node}: "
                    "state query timed out"
                )

            state = future.result().current_state

            if state.id != 3:
                raise RuntimeError(
                    f"{robot}/{lifecycle_node}: "
                    f"state is {state.label}"
                )

            print(
                f"{robot}/{lifecycle_node}: active",
                flush=True,
            )

    def wait_ready(self):

        deadline = time.monotonic() + READY_TIMEOUT

        first_clock = None

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            clock = self.get_clock().now().nanoseconds

            if clock > 0 and first_clock is None:
                first_clock = clock

            now = time.monotonic()

            ready = True

            for robot in self.robots:

                odom = self.odometry.get(robot)
                scan = self.scans.get(robot)

                publisher = self.publishers_by_robot[
                    robot
                ]

                if (
                    odom is None
                    or scan is None
                    or now - odom[0] > 1.0
                    or now - scan[0] > 1.0
                    or publisher.get_subscription_count() == 0
                ):
                    ready = False
                    continue

                twist = odom[1].twist.twist

                if (
                    abs(twist.linear.x) > 0.005
                    or abs(twist.angular.z) > 0.01
                ):
                    raise RuntimeError(
                        f"{robot}: robot is moving"
                    )

            if (
                ready
                and first_clock is not None
                and clock > first_clock
            ):
                return

        raise RuntimeError(
            "Fleet readiness timeout"
        )

    def initialize_robot(self, robot):

        target = self.robots[robot]

        previous = self.estimates.get(robot)

        previous_stamp = (
            stamp_ns(previous[1].header.stamp)
            if previous is not None
            else None
        )

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = "map"

        # Zero timestamp follows the initialization
        # procedure validated on the single robot.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = target["x"]
        msg.pose.pose.position.y = target["y"]
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(
            target["yaw"] / 2.0
        )

        msg.pose.pose.orientation.w = math.cos(
            target["yaw"] / 2.0
        )

        msg.pose.covariance[0] = POSITION_VARIANCE
        msg.pose.covariance[7] = POSITION_VARIANCE
        msg.pose.covariance[35] = YAW_VARIANCE

        print(
            f"Initializing {robot}: {target}",
            flush=True,
        )

        sent_wall = time.monotonic()

        sent_sim_ns = (
            self.get_clock().now().nanoseconds
        )

        self.publishers_by_robot[robot].publish(
            msg
        )

        deadline = sent_wall + POSE_TIMEOUT

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            received = self.estimates.get(robot)

            if received is None:
                continue

            received_wall, estimate = received

            if received_wall <= sent_wall:
                continue

            estimate_stamp = stamp_ns(
                estimate.header.stamp
            )

            if estimate_stamp <= 0:
                continue

            if (
                previous_stamp is not None
                and estimate_stamp <= previous_stamp
            ):
                continue

            if (
                estimate_stamp
                < sent_sim_ns - 1_000_000_000
            ):
                continue

            if estimate.header.frame_id != "map":
                continue

            pose = estimate.pose.pose

            estimated = {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(
                    pose.orientation
                ),
            }

            if not all(
                math.isfinite(value)
                for value in estimated.values()
            ):
                raise RuntimeError(
                    f"{robot}: non-finite AMCL estimate"
                )

            position_residual = math.hypot(
                estimated["x"] - target["x"],
                estimated["y"] - target["y"],
            )

            yaw_residual = abs(
                angle_difference(
                    estimated["yaw"],
                    target["yaw"],
                )
            )

            acceptable = (
                position_residual
                <= MAX_POSITION_RESIDUAL
                and yaw_residual
                <= MAX_YAW_RESIDUAL
            )

            return {
                "requested_pose": target,
                "estimated_pose": estimated,
                "pose_stamp_ns": estimate_stamp,
                "initial_pose_published_sim_ns":
                    sent_sim_ns,
                "observed_response_wall_s":
                    received_wall - sent_wall,
                "position_residual_m":
                    position_residual,
                "yaw_residual_rad":
                    yaw_residual,
                "covariance_x":
                    estimate.pose.covariance[0],
                "covariance_y":
                    estimate.pose.covariance[7],
                "covariance_yaw":
                    estimate.pose.covariance[35],
                "initialization_passed":
                    acceptable,
            }

        raise RuntimeError(
            f"{robot}: no fresh AMCL pose received"
        )


def main():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--fleet-config",
        type=Path,
        default=Path("/etc/clearpath/fleet.yaml"),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )

    args = parser.parse_args()

    robots = load_fleet(
        args.fleet_config
    )

    args.output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    rclpy.init()

    node = FleetInitializer(robots)

    results = {}
    error = None

    try:

        node.verify_lifecycle()

        print(
            "Waiting for fleet interfaces...",
            flush=True,
        )

        node.wait_ready()

        # Intended for fresh localization stacks.
        if node.estimates:
            raise RuntimeError(
                "Existing AMCL estimates detected. "
                "Refusing to reset localization."
            )

        for robot in robots:

            results[robot] = (
                node.initialize_robot(robot)
            )

            print(
                json.dumps(
                    results[robot],
                    indent=2,
                    allow_nan=False,
                ),
                flush=True,
            )

            if not results[robot][
                "initialization_passed"
            ]:
                raise RuntimeError(
                    f"{robot}: initial estimate "
                    "outside diagnostic limits"
                )

    except (Exception, KeyboardInterrupt) as exc:

        error = str(exc) or type(exc).__name__

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    passed = (
        error is None
        and len(results) == len(robots)
        and all(
            result["initialization_passed"]
            for result in results.values()
        )
    )

    summary = {
        "robot_count": len(robots),
        "robot_results": results,
        "position_residual_limit_m":
            MAX_POSITION_RESIDUAL,
        "yaw_residual_limit_rad":
            MAX_YAW_RESIDUAL,
        "initial_position_variance_m2":
            POSITION_VARIANCE,
        "initial_yaw_variance_rad2":
            YAW_VARIANCE,
        "fleet_initialization_passed":
            passed,
        "error": error,
        "limitations": [
            "Assumes map and Gazebo world alignment",
            "Uses configured spawn poses",
            "Requires stationary robots at their spawn poses",
            "A fresh AMCL estimate is indirect "
            "confirmation of initialization",
            "Residuals are relative to requested poses, "
            "not independent ground truth",
            "Diagnostic limits do not establish "
            "navigation accuracy",
        ],
    }

    output = (
        args.output_dir / "initialization_summary.json"
    )

    output.write_text(
        json.dumps(
            summary,
            indent=2,
            allow_nan=False,
        ) + "\n"
    )

    print("\n===== FLEET INITIALIZATION =====")

    print(
        json.dumps(
            summary,
            indent=2,
        )
    )

    if not passed:
        raise SystemExit(
            "Fleet initialization failed"
        )


if __name__ == "__main__":
    main()