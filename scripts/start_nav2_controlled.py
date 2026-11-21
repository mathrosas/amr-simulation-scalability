#!/usr/bin/env python3

"""Check Nav2 prerequisites and activate its managed lifecycle nodes."""

import argparse
import math
import re
import shlex
import subprocess
import sys
import time

from pathlib import Path


DEFAULT_COMPOSE_FILES = (
    "compose.yaml",
    "compose.visualization.yaml",
    "compose.navigation.yaml",
)

NAVIGATION_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "route_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
    "docking_server",
)


def run_inside(namespace):
    """Run readiness checks and lifecycle management inside ROS."""

    import rclpy

    from lifecycle_msgs.srv import GetState

    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time

    from tf2_ros import Buffer, TransformListener

    class StartupClient(Node):

        def __init__(self):

            super().__init__("amr_nav2_startup_check")

            self.set_parameters([
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    True,
                ),
            ])

            self.tf_buffer = Buffer()

            self.tf_listener = TransformListener(
                self.tf_buffer,
                self,
                spin_thread=False,
            )

        def check_clock(self, timeout=15.0):

            deadline = time.monotonic() + timeout
            first = None

            while time.monotonic() < deadline:

                rclpy.spin_once(
                    self,
                    timeout_sec=0.1,
                )

                now = self.get_clock().now().nanoseconds

                if now <= 0:
                    continue

                if first is None:
                    first = now

                elif now > first:

                    print(
                        "Simulation clock: advancing",
                        flush=True,
                    )

                    return

            raise RuntimeError(
                "Simulation clock did not advance"
            )

        def check_tf(
            self,
            target,
            source="base_link",
            timeout=15.0,
        ):

            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:

                rclpy.spin_once(
                    self,
                    timeout_sec=0.1,
                )

                if not self.tf_buffer.can_transform(
                    target,
                    source,
                    Time(),
                ):
                    continue

                transform = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    Time(),
                ).transform

                values = (
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                )

                quaternion_norm = sum(
                    value * value
                    for value in values[3:]
                )

                if not all(
                    math.isfinite(value)
                    for value in values
                ):
                    raise RuntimeError(
                        f"Invalid TF {target} <- {source}"
                    )

                if quaternion_norm < 1e-12:
                    raise RuntimeError(
                        f"Invalid TF rotation {target} <- {source}"
                    )

                print(
                    f"TF {target} <- {source}: valid",
                    flush=True,
                )

                return

            raise RuntimeError(
                f"TF unavailable: {target} <- {source}"
            )

        def request(
            self,
            srv_type,
            name,
            request,
            timeout=15.0,
        ):

            client = self.create_client(
                srv_type,
                name,
            )

            try:

                if not client.wait_for_service(
                    timeout_sec=timeout
                ):
                    raise RuntimeError(
                        f"Service unavailable: {name}"
                    )

                future = client.call_async(request)

                rclpy.spin_until_future_complete(
                    self,
                    future,
                    timeout_sec=timeout,
                )

                if (
                    not future.done()
                    or future.result() is None
                ):
                    raise RuntimeError(
                        f"Service call failed: {name}"
                    )

                return future.result()

            finally:

                self.destroy_client(client)

        def lifecycle_state(self, name):

            result = self.request(
                GetState,
                f"/{namespace}/{name}/get_state",
                GetState.Request(),
            )

            state = result.current_state

            print(
                f"/{namespace}/{name}: "
                f"{state.label} [{state.id}]",
                flush=True,
            )

            return state.id

        def states(self):

            return {
                name: self.lifecycle_state(name)
                for name in NAVIGATION_NODES
            }

        def activate(self, states):
            """Use the validated ROS CLI for lifecycle transitions."""

            values = list(states.values())

            if all(
                value == 3
                for value in values
            ):

                print(
                    "Navigation already active",
                    flush=True,
                )

                return

            if all(
                value == 1
                for value in values
            ):

                operation = "STARTUP"
                command = 0

            elif all(
                value in (2, 3)
                for value in values
            ):

                operation = "RESUME"
                command = 2

            else:

                raise RuntimeError(
                    "Unexpected navigation "
                    "lifecycle state mixture"
                )

            print(
                f"Requesting lifecycle {operation}...",
                flush=True,
            )

            service_name = (
                f"/{namespace}/"
                "lifecycle_manager_navigation/manage_nodes"
            )

            cli_command = [
                "ros2",
                "service",
                "call",
                service_name,
                "nav2_msgs/srv/ManageLifecycleNodes",
                f"{{command: {command}}}",
            ]

            try:

                result = subprocess.run(
                    cli_command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=105,
                    check=False,
                )

            except subprocess.TimeoutExpired as exc:

                raise RuntimeError(
                    f"Lifecycle {operation} timed out"
                ) from exc

            if (
                result.returncode != 0
                or (
                    "ManageLifecycleNodes_Response("
                    "success=True)"
                ) not in result.stdout
            ):

                raise RuntimeError(
                    f"Lifecycle {operation} failed:\n"
                    + result.stdout
                )

            print(
                f"Lifecycle {operation}: successful",
                flush=True,
            )

    rclpy.init(args=sys.argv)

    node = StartupClient()

    started = time.monotonic()

    try:

        print(
            "===== NAV2 CONTROLLED STARTUP =====",
            flush=True,
        )

        print(
            "\nChecking simulation clock...",
            flush=True,
        )

        node.check_clock()

        print(
            "\nChecking localization...",
            flush=True,
        )

        for name in ("map_server", "amcl"):

            if node.lifecycle_state(name) != 3:

                raise RuntimeError(
                    f"Localization node {name} "
                    "is not active"
                )

        print(
            "\nChecking transformations...",
            flush=True,
        )

        for frame in ("odom", "map"):

            node.check_tf(frame)

        print(
            "\nChecking navigation states...",
            flush=True,
        )

        node.activate(node.states())

        print(
            "\nVerifying final states...",
            flush=True,
        )

        if not all(
            value == 3
            for value in node.states().values()
        ):

            raise RuntimeError(
                "Not all navigation nodes are active"
            )

        elapsed = time.monotonic() - started

        print(
            "\n===== NAV2 STARTUP PASSED =====",
            flush=True,
        )

        print(
            f"Robot: {namespace}",
            flush=True,
        )

        print(
            f"Navigation nodes: "
            f"{len(NAVIGATION_NODES)} active",
            flush=True,
        )

        print(
            f"Elapsed wall time: {elapsed:.2f} seconds",
            flush=True,
        )

        return 0

    except (RuntimeError, KeyboardInterrupt) as exc:

        print(
            f"\nNAV2 STARTUP FAILED: {exc}",
            file=sys.stderr,
            flush=True,
        )

        return 1

    finally:

        node.destroy_node()
        rclpy.shutdown()


def main():

    parser = argparse.ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--namespace",
        default="j100_0000",
    )

    parser.add_argument(
        "--service",
        default="navigation",
    )

    parser.add_argument(
        "--compose-file",
        action="append",
        default=None,
    )

    parser.add_argument(
        "--inside",
        action="store_true",
        help=argparse.SUPPRESS,
    )

    args, _ = parser.parse_known_args()

    if not re.fullmatch(
        r"[A-Za-z][A-Za-z0-9_]*",
        args.namespace,
    ):

        parser.error("Invalid namespace")

    if args.inside:

        return run_inside(args.namespace)

    compose_files = (
        args.compose_file
        or DEFAULT_COMPOSE_FILES
    )

    command = [
        "docker",
        "compose",
    ]

    for file in compose_files:

        command.extend([
            "-f",
            file,
        ])

    ros_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "python3 - "
        f"--inside --namespace {shlex.quote(args.namespace)} "
        "--ros-args "
        f"-r /tf:=/{args.namespace}/tf "
        f"-r /tf_static:=/{args.namespace}/tf_static"
    )

    command.extend([
        "exec",
        "-T",
        args.service,
        "bash",
        "-lc",
        ros_command,
    ])

    try:

        result = subprocess.run(
            command,
            input=Path(__file__).read_text(
                encoding="utf-8"
            ),
            text=True,
            timeout=180,
            check=False,
        )

    except (
        FileNotFoundError,
        subprocess.TimeoutExpired,
    ) as exc:

        print(
            f"NAV2 STARTUP FAILED: {exc}",
            file=sys.stderr,
        )

        return 1

    return result.returncode


if __name__ == "__main__":

    raise SystemExit(main())