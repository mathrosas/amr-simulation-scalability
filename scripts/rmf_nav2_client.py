#!/usr/bin/env python3

"""ROS 2 Nav2 communication interface for an Open-RMF fleet adapter."""

import math
import threading

from dataclasses import dataclass
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile


@dataclass
class NavigationCommand:
    """Track one asynchronous Nav2 navigation request."""

    command_id: int
    on_complete: Callable[[bool, str], None]

    goal_handle: object = None

    cancel_requested: bool = False
    cancel_sent: bool = False


class Nav2RobotClient(Node):
    """Interface between one robot's Nav2 stack and Open-RMF."""

    def __init__(self, namespace: str):

        super().__init__(
            f"rmf_nav2_bridge_{namespace}",
            parameter_overrides=[
                Parameter("use_sim_time", value=True),
            ],
        )

        self.robot_namespace = namespace

        self._lock = threading.RLock()

        self._latest_pose = None

        self._command_counter = 0
        self._active_command: Optional[NavigationCommand] = None

        self._action_name = (
            f"/{namespace}/navigate_to_pose"
        )

        self._pose_topic = (
            f"/{namespace}/amcl_pose"
        )

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self._action_name,
        )

        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._pose_topic,
            self._on_pose,
            amcl_qos,
        )

        self.get_logger().info(
            f"Nav2 interface created for {namespace}"
        )

    @staticmethod
    def _quaternion_to_yaw(q):
        """Convert a quaternion into planar yaw."""

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Return a planar quaternion as (z, w)."""

        return (
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def _on_pose(self, message):
        """Store the most recent AMCL pose."""

        if message.header.frame_id != "map":
            self.get_logger().warning(
                "Ignoring AMCL pose outside map frame"
            )
            return

        pose = message.pose.pose

        x = float(pose.position.x)
        y = float(pose.position.y)

        yaw = self._quaternion_to_yaw(
            pose.orientation
        )

        with self._lock:
            self._latest_pose = (x, y, yaw)

    def get_pose(self):
        """Return the latest (x, y, yaw), or None."""

        with self._lock:
            return self._latest_pose

    def navigation_ready(self):
        """Check whether the Nav2 action server is available."""

        return self._action_client.server_is_ready()

    def navigate(
        self,
        x: float,
        y: float,
        yaw: float,
        on_complete: Callable[[bool, str], None],
    ):
        """Send one asynchronous navigation goal."""

        if not all(
            math.isfinite(value)
            for value in (x, y, yaw)
        ):
            raise ValueError(
                "Navigation goal contains non-finite coordinates"
            )

        if not self.navigation_ready():
            raise RuntimeError(
                f"{self.robot_namespace}: "
                "Nav2 action server is unavailable"
            )

        with self._lock:

            if self._active_command is not None:
                raise RuntimeError(
                    f"{self.robot_namespace}: "
                    "a navigation command is already active"
                )

            self._command_counter += 1

            command = NavigationCommand(
                command_id=self._command_counter,
                on_complete=on_complete,
            )

            self._active_command = command

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        qz, qw = self._yaw_to_quaternion(yaw)

        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"{self.robot_namespace}: "
            f"Sending goal ({x:.3f}, {y:.3f}, {yaw:.3f})"
        )

        try:

            future = self._action_client.send_goal_async(goal)

            future.add_done_callback(
                lambda result:
                self._on_goal_response(
                    command.command_id,
                    result,
                )
            )

        except Exception as error:

            self._finish_command(
                command.command_id,
                False,
                f"Goal dispatch failed: {error}",
            )

    def _on_goal_response(self, command_id, future):
        """Handle Nav2 goal acceptance."""

        try:
            goal_handle = future.result()

        except Exception as error:

            self._finish_command(
                command_id,
                False,
                f"Goal response failed: {error}",
            )
            return

        if not goal_handle.accepted:

            self._finish_command(
                command_id,
                False,
                "Nav2 rejected navigation goal",
            )
            return

        with self._lock:

            command = self._active_command

            if (
                command is None
                or command.command_id != command_id
            ):
                goal_handle.cancel_goal_async()
                return

            command.goal_handle = goal_handle

        self.get_logger().info(
            f"{self.robot_namespace}: "
            "Navigation goal accepted"
        )

        try:

            result_future = goal_handle.get_result_async()

            result_future.add_done_callback(
                lambda result:
                self._on_navigation_result(
                    command_id,
                    result,
                )
            )

        except Exception as error:

            self._finish_command(
                command_id,
                False,
                f"Result request failed: {error}",
            )
            return

        self._request_cancel_if_needed(command_id)

    def _on_navigation_result(self, command_id, future):
        """Handle terminal navigation status."""

        try:

            result = future.result()
            status = result.status

        except Exception as error:

            self._finish_command(
                command_id,
                False,
                f"Navigation result failed: {error}",
            )
            return

        with self._lock:

            command = self._active_command

            if (
                command is None
                or command.command_id != command_id
            ):
                return

            was_cancelled = command.cancel_requested

        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not was_cancelled
        )

        self._finish_command(
            command_id,
            succeeded,
            f"Nav2 terminal status: {status}",
        )

    def stop(self):
        """Request cancellation of the active Nav2 goal."""

        with self._lock:

            command = self._active_command

            if command is None:
                return

            command.cancel_requested = True

            command_id = command.command_id

        self.get_logger().info(
            f"{self.robot_namespace}: "
            "Navigation cancellation requested"
        )

        self._request_cancel_if_needed(command_id)

    def _request_cancel_if_needed(self, command_id):
        """Cancel an accepted goal if cancellation was requested."""

        with self._lock:

            command = self._active_command

            if (
                command is None
                or command.command_id != command_id
                or not command.cancel_requested
                or command.cancel_sent
                or command.goal_handle is None
            ):
                return

            command.cancel_sent = True

            goal_handle = command.goal_handle

        try:

            future = goal_handle.cancel_goal_async()

            future.add_done_callback(
                lambda result:
                self._on_cancel_response(
                    command_id,
                    result,
                )
            )

        except Exception as error:

            self.get_logger().error(
                f"{self.robot_namespace}: "
                f"Cancellation request failed: {error}"
            )

    def _on_cancel_response(self, command_id, future):
        """Log the outcome of a Nav2 cancellation request."""

        try:

            response = future.result()

            self.get_logger().info(
                f"{self.robot_namespace}: "
                f"Cancellation response for command "
                f"{command_id}: "
                f"{len(response.goals_canceling)} goal(s)"
            )

        except Exception as error:

            self.get_logger().error(
                f"{self.robot_namespace}: "
                f"Cancellation response failed: {error}"
            )

    def _finish_command(
        self,
        command_id: int,
        succeeded: bool,
        message: str,
    ):
        """Finish the current command and notify its owner."""

        with self._lock:

            command = self._active_command

            if (
                command is None
                or command.command_id != command_id
            ):
                return

            self._active_command = None

        if succeeded:
            self.get_logger().info(
                f"{self.robot_namespace}: {message}"
            )
        else:
            self.get_logger().warning(
                f"{self.robot_namespace}: {message}"
            )

        try:

            command.on_complete(
                succeeded,
                message,
            )

        except Exception as error:

            self.get_logger().error(
                f"{self.robot_namespace}: "
                f"Completion callback failed: {error}"
            )