#!/usr/bin/env python3

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


TERMINAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_ABORTED,
    GoalStatus.STATUS_CANCELED,
}

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def duration_seconds(duration):
    return duration.sec + duration.nanosec / 1_000_000_000


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavigationProbe(Node):

    def __init__(self, args, sample_writer):
        namespace = args.namespace.strip("/")

        super().__init__(
            "amr_navigation_probe",
            namespace=namespace,
            parameter_overrides=[
                Parameter("use_sim_time", value=True)
            ],
            cli_args=[
                "--ros-args",
                "-r", f"/tf:=/{namespace}/tf",
                "-r", f"/tf_static:=/{namespace}/tf_static",
            ],
        )

        self.args = args
        self.sample_writer = sample_writer

        self.phase = "initialization"

        self.last_odom = None
        self.last_feedback = None

        self.feedback_count = 0
        self.max_recoveries = None

        self.odom_distance = 0.0
        self.previous_odom_position = None
        self.odom_sample_count = 0

        self.sample_count = 0
        self.sample_period = 0.1
        self.next_sample_time = time.monotonic()

        self.summary = {
            "namespace": namespace,
            "goal": {
                "x": args.goal_x,
                "y": args.goal_y,
                "yaw_rad": args.goal_yaw,
                "frame_id": "map",
            },
            "execution_enabled": args.execute,
            "sample_rate_hz": 10.0,
            "planning": {},
            "navigation": {},
            "requires_manual_stop": False,
            "completed": False,
        }

        self.tf_buffer = Buffer(node=self)

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "platform/odom",
            self.odom_callback,
            odom_qos,
        )

        self.planning_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
        )

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

    def odom_callback(self, msg):
        self.last_odom = msg

        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        if self.phase in ("navigation", "settling"):
            if self.previous_odom_position is not None:
                self.odom_distance += math.dist(
                    self.previous_odom_position,
                    position,
                )

            self.odom_sample_count += 1

        self.previous_odom_position = position

    def feedback_callback(self, message):
        feedback = message.feedback

        self.last_feedback = feedback
        self.feedback_count += 1

        recoveries = int(
            feedback.number_of_recoveries
        )

        if self.max_recoveries is None:
            self.max_recoveries = recoveries
        else:
            self.max_recoveries = max(
                self.max_recoveries,
                recoveries,
            )

    def get_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
            )

        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_stamp = stamp_ns(
            transform.header.stamp
        )

        current_time = self.get_clock().now().nanoseconds

        if abs(current_time - transform_stamp) > 1_000_000_000:
            return None

        return {
            "stamp_ns": transform_stamp,
            "x": translation.x,
            "y": translation.y,
            "yaw_rad": quaternion_to_yaw(rotation),
        }

    def get_odom_state(self):
        if self.last_odom is None:
            return None

        msg = self.last_odom

        odom_stamp = stamp_ns(msg.header.stamp)
        current_time = self.get_clock().now().nanoseconds

        if abs(current_time - odom_stamp) > 1_000_000_000:
            return None

        return {
            "stamp_ns": odom_stamp,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_rad": quaternion_to_yaw(
                msg.pose.pose.orientation
            ),
            "linear_x_mps": msg.twist.twist.linear.x,
            "angular_z_radps": msg.twist.twist.angular.z,
        }

    def write_sample(self):
        pose = self.get_map_pose() or {}
        odom = self.get_odom_state() or {}

        feedback = self.last_feedback

        self.sample_writer.writerow([
            self.phase,
            time.monotonic_ns(),
            self.get_clock().now().nanoseconds,
            pose.get("stamp_ns"),
            pose.get("x"),
            pose.get("y"),
            pose.get("yaw_rad"),
            odom.get("stamp_ns"),
            odom.get("x"),
            odom.get("y"),
            odom.get("yaw_rad"),
            odom.get("linear_x_mps"),
            odom.get("angular_z_radps"),
            (
                feedback.distance_remaining
                if feedback is not None else None
            ),
            (
                feedback.number_of_recoveries
                if feedback is not None else None
            ),
        ])

        self.sample_count += 1

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

        now = time.monotonic()

        if now >= self.next_sample_time:
            self.write_sample()
            self.next_sample_time = now + self.sample_period

    def wait_future(self, future, timeout, description):
        deadline = time.monotonic() + timeout

        while not future.done() and rclpy.ok():
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timeout waiting for {description}"
                )

            self.tick()

        if not future.done():
            raise RuntimeError(
                f"Interrupted while waiting for {description}"
            )

        return future.result()

    def wait_ready(self):
        deadline = time.monotonic() + 30.0

        initial_clock = self.get_clock().now().nanoseconds

        self.get_logger().info(
            "Waiting for clock, TF, odometry and action servers"
        )

        while time.monotonic() < deadline:
            self.tick()

            clock_ready = (
                self.get_clock().now().nanoseconds
                > initial_clock
            )

            ready = (
                clock_ready
                and self.get_map_pose() is not None
                and self.get_odom_state() is not None
                and self.planning_client.server_is_ready()
                and (
                    not self.args.execute
                    or self.navigation_client.server_is_ready()
                )
            )

            if ready:
                self.get_logger().info(
                    "Navigation interfaces ready"
                )
                return

        raise TimeoutError(
            "Navigation interfaces did not become ready"
        )

    def make_goal_pose(self):
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.args.goal_x
        pose.pose.position.y = self.args.goal_y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(
            self.args.goal_yaw / 2.0
        )

        pose.pose.orientation.w = math.cos(
            self.args.goal_yaw / 2.0
        )

        return pose

    def record_action_result(self, record, response):
        record["status_code"] = int(response.status)

        record["status"] = STATUS_NAMES.get(
            response.status,
            "UNKNOWN",
        )

        record["error_code"] = int(
            response.result.error_code
        )

        record["error_msg"] = response.result.error_msg

    def cancel_action(self, handle, result_future, record):
        self.get_logger().warning(
            "Requesting action cancellation"
        )

        try:
            cancel_response = self.wait_future(
                handle.cancel_goal_async(),
                5.0,
                "cancellation response",
            )

            record["cancel_acknowledged"] = bool(
                cancel_response.goals_canceling
            )

        except Exception as exc:
            record["cancel_error"] = str(exc)

        try:
            if result_future is None:
                result_future = handle.get_result_async()

            response = self.wait_future(
                result_future,
                10.0,
                "terminal action result after cancellation",
            )

            self.record_action_result(record, response)

            return response.status in TERMINAL_STATES

        except Exception as exc:
            record["cancel_result_error"] = str(exc)

            return False

    def run_action(
        self,
        client,
        goal,
        record,
        result_timeout,
        feedback_callback=None,
        motion_action=False,
    ):
        handle = None
        result_future = None

        start_wall = time.monotonic()
        start_sim = self.get_clock().now().nanoseconds

        goal_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )

        try:
            handle = self.wait_future(
                goal_future,
                10.0,
                "goal acceptance",
            )

            record["accepted"] = bool(handle.accepted)

            if not handle.accepted:
                raise RuntimeError("Action goal rejected")

            record["acceptance_wall_s"] = (
                time.monotonic() - start_wall
            )

            result_future = handle.get_result_async()

            response = self.wait_future(
                result_future,
                result_timeout,
                "action result",
            )

            self.record_action_result(record, response)

            return response

        except (Exception, KeyboardInterrupt):

            if handle is None:
                try:
                    handle = self.wait_future(
                        goal_future,
                        5.0,
                        "outstanding goal response",
                    )

                except Exception:
                    record["goal_response_unknown"] = True

                    if motion_action:
                        self.summary[
                            "requires_manual_stop"
                        ] = True

            if handle is not None and handle.accepted:
                if record.get("status_code") not in TERMINAL_STATES:

                    terminal = self.cancel_action(
                        handle,
                        result_future,
                        record,
                    )

                    if motion_action and not terminal:
                        self.summary[
                            "requires_manual_stop"
                        ] = True

            raise

        finally:
            record["client_wall_duration_s"] = (
                time.monotonic() - start_wall
            )

            record["client_sim_duration_s"] = (
                self.get_clock().now().nanoseconds - start_sim
            ) / 1_000_000_000

    def compute_path(self):
        self.phase = "planning"

        self.get_logger().info(
            "Requesting global path"
        )

        goal = ComputePathToPose.Goal()

        goal.goal = self.make_goal_pose()
        goal.planner_id = "GridBased"
        goal.use_start = False

        record = self.summary["planning"]

        response = self.run_action(
            self.planning_client,
            goal,
            record,
            result_timeout=30.0,
        )

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            raise RuntimeError("Global planning failed")

        result = response.result

        poses = [
            item.pose.position
            for item in result.path.poses
        ]

        positions = [
            (point.x, point.y)
            for point in poses
        ]

        if len(positions) < 2:
            raise RuntimeError(
                "Planner returned an insufficient path"
            )

        path_length = sum(
            math.dist(a, b)
            for a, b in zip(
                positions,
                positions[1:],
            )
        )

        direct_distance = math.dist(
            positions[0],
            positions[-1],
        )

        record.update({
            "reported_planning_time_s": duration_seconds(
                result.planning_time
            ),
            "pose_count": len(positions),
            "path_length_m": path_length,
            "direct_distance_m": direct_distance,
            "path_length_ratio": (
                path_length / direct_distance
                if direct_distance > 0.0
                else None
            ),
        })

        path_file = (
            self.args.output_dir / "planned_path.csv"
        )

        with path_file.open("w", newline="") as file:
            writer = csv.writer(
                file,
                lineterminator="\n",
            )

            writer.writerow([
                "index",
                "x_m",
                "y_m",
                "yaw_rad",
            ])

            for index, item in enumerate(
                result.path.poses
            ):
                writer.writerow([
                    index,
                    item.pose.position.x,
                    item.pose.position.y,
                    quaternion_to_yaw(
                        item.pose.orientation
                    ),
                ])

        self.get_logger().info(
            f"Planning succeeded: {path_length:.3f} m"
        )

    def navigate(self):
        self.phase = "navigation"

        self.get_logger().info(
            "Sending navigation goal"
        )

        goal = NavigateToPose.Goal()
        goal.pose = self.make_goal_pose()

        response = self.run_action(
            self.navigation_client,
            goal,
            self.summary["navigation"],
            result_timeout=self.args.goal_timeout,
            feedback_callback=self.feedback_callback,
            motion_action=True,
        )

        self.phase = "settling"

        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            self.tick()

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code != 0
        ):
            raise RuntimeError(
                "Navigation did not succeed"
            )

    def run(self):
        self.wait_ready()

        self.summary["initial_map_pose"] = (
            self.get_map_pose()
        )

        self.summary["initial_odometry"] = (
            self.get_odom_state()
        )

        self.compute_path()

        if self.args.execute:
            self.navigate()

        self.summary["completed"] = True

    def finalize(self):
        self.summary["final_map_pose"] = (
            self.get_map_pose()
        )

        self.summary["final_odometry"] = (
            self.get_odom_state()
        )

        self.summary["sample_count"] = self.sample_count

        self.summary["feedback_count"] = (
            self.feedback_count
        )

        self.summary["max_recoveries"] = (
            self.max_recoveries
        )

        self.summary["odometry_trajectory_length_m"] = (
            self.odom_distance
            if self.args.execute
            else None
        )

        self.summary["odometry_trajectory_sample_count"] = (
            self.odom_sample_count
        )

        feedback = self.last_feedback

        if feedback is not None:
            self.summary["last_feedback"] = {
                "distance_remaining_m": (
                    feedback.distance_remaining
                ),
                "navigation_time_s": duration_seconds(
                    feedback.navigation_time
                ),
                "number_of_recoveries": (
                    feedback.number_of_recoveries
                ),
            }

        pose = self.summary["final_map_pose"]

        if pose is not None:
            self.summary["final_estimated_position_error_m"] = (
                math.hypot(
                    pose["x"] - self.args.goal_x,
                    pose["y"] - self.args.goal_y,
                )
            )

            yaw_error = (
                pose["yaw_rad"] - self.args.goal_yaw
            )

            self.summary["final_estimated_yaw_error_rad"] = (
                math.atan2(
                    math.sin(yaw_error),
                    math.cos(yaw_error),
                )
            )

        odom = self.summary["final_odometry"]

        self.summary["stationary"] = (
            abs(odom["linear_x_mps"]) < 0.005
            and abs(odom["angular_z_radps"]) < 0.01
            if odom is not None
            else None
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="AMR navigation benchmark probe"
    )

    parser.add_argument(
        "--namespace",
        default="j100_0000",
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
        help="Execute navigation after successful planning",
    )

    args = parser.parse_args()

    if not all(
        math.isfinite(value)
        for value in (
            args.goal_x,
            args.goal_y,
            args.goal_yaw,
            args.goal_timeout,
        )
    ):
        parser.error("Numeric arguments must be finite")

    if args.goal_timeout <= 0.0:
        parser.error("Goal timeout must be positive")

    if not args.namespace.strip("/"):
        parser.error("Namespace must not be empty")

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

    with (args.output_dir / "samples.csv").open(
        "w",
        newline="",
    ) as file:

        writer = csv.writer(
            file,
            lineterminator="\n",
        )

        writer.writerow([
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
        ])

        probe = NavigationProbe(args, writer)

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
            probe.finalize()

            summary_file = (
                args.output_dir / "summary.json"
            )

            summary_file.write_text(
                json.dumps(
                    probe.summary,
                    indent=2,
                    allow_nan=False,
                ) + "\n"
            )

            print(
                json.dumps(
                    probe.summary,
                    indent=2,
                    allow_nan=False,
                )
            )

            if probe.summary["requires_manual_stop"]:
                print(
                    "WARNING: Action termination not confirmed. "
                    "Stop the navigation service.",
                    file=sys.stderr,
                )

            probe.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())