#!/usr/bin/env python3

"""Coordinate independent Nav2 action goals across a robot fleet."""

import argparse
import csv
import json
import math
import re
import sys
import time

from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from navigation_probe import (
    NavigationProbe,
    TERMINAL_STATES,
)

from fleet_navigation_probe import ACTIVE_FIELDS


SETTLING_DURATION = 2.0
ACCEPTANCE_TIMEOUT = 10.0

LINEAR_LIMIT = 0.005
ANGULAR_LIMIT = 0.01


def save_json(path, data):

    path.write_text(
        json.dumps(
            data,
            indent=2,
            allow_nan=False,
        ) + "\n"
    )


class ConcurrentNavigationProbe(NavigationProbe):

    def __init__(self, args, writer, coordinator):

        super().__init__(args, writer)

        self.coordinator = coordinator

    def tick(self):

        self.coordinator.tick()


class FleetNavigationCoordinator:

    def __init__(self, executor):

        self.executor = executor

        self.probes = {}

        self.actions = {}

    def add_probe(self, probe):

        robot = probe.args.namespace

        self.probes[robot] = probe

        self.executor.add_node(probe)

    def tick(self):

        self.executor.spin_once(
            timeout_sec=0.01
        )

        now = time.monotonic()

        for probe in self.probes.values():

            if now >= probe.next_sample_time:

                probe.write_sample()

                probe.next_sample_time = (
                    now + probe.sample_period
                )

    def verify_stationary(self):

        for robot, probe in self.probes.items():

            odom = probe.get_odom_state()

            pose = probe.get_map_pose()

            if odom is None or pose is None:

                raise RuntimeError(
                    f"{robot}: telemetry unavailable"
                )

            linear = odom["linear_x_mps"]

            angular = odom["angular_z_radps"]

            if (
                not math.isfinite(linear)
                or not math.isfinite(angular)
                or abs(linear) > LINEAR_LIMIT
                or abs(angular) > ANGULAR_LIMIT
            ):

                raise RuntimeError(
                    f"{robot}: robot is not stationary"
                )

    def prepare(self):

        for robot, probe in self.probes.items():

            print(
                f"Waiting for {robot}",
                flush=True,
            )

            probe.wait_ready()

        for probe in self.probes.values():

            probe.phase = "preflight"

        deadline = time.monotonic() + 1.0

        while time.monotonic() < deadline:

            self.tick()

            self.verify_stationary()

        for robot, probe in self.probes.items():

            probe.summary["initial_map_pose"] = (
                probe.get_map_pose()
            )

            probe.summary["initial_odometry"] = (
                probe.get_odom_state()
            )

        # Plan every route before commanding any robot.
        for robot, probe in self.probes.items():

            print(
                f"Planning route for {robot}",
                flush=True,
            )

            probe.compute_path()

            probe.phase = "waiting"

            self.verify_path_endpoint(probe)

        self.verify_stationary()

    def verify_path_endpoint(self, probe):

        path = (
            probe.args.output_dir
            / "planned_path.csv"
        )

        with path.open(newline="") as file:

            rows = list(csv.DictReader(file))

        if len(rows) < 2:

            raise RuntimeError(
                "Insufficient path poses"
            )

        endpoint = rows[-1]

        x = float(endpoint["x_m"])

        y = float(endpoint["y_m"])

        yaw = float(endpoint["yaw_rad"])

        position_error = math.hypot(
            x - probe.args.goal_x,
            y - probe.args.goal_y,
        )

        yaw_difference = yaw - probe.args.goal_yaw

        yaw_error = abs(
            math.atan2(
                math.sin(yaw_difference),
                math.cos(yaw_difference),
            )
        )

        if (
            not math.isfinite(position_error)
            or not math.isfinite(yaw_error)
            or position_error > 0.05
            or yaw_error > 0.05
        ):

            raise RuntimeError(
                f"{probe.args.namespace}: "
                "planned endpoint differs from goal"
            )

        probe.summary["planning"].update({
            "endpoint_position_error_m":
                position_error,
            "endpoint_yaw_error_rad":
                yaw_error,
        })

    def dispatch(self):

        # Construct every goal before dispatch.
        goals = {}

        for robot, probe in self.probes.items():

            goal = NavigateToPose.Goal()

            goal.pose = probe.make_goal_pose()

            goals[robot] = goal

        self.verify_stationary()

        # Send goals without waiting for acceptance
        # before dispatching the next robot.
        for robot, probe in self.probes.items():

            probe.phase = "navigation"

            record = probe.summary["navigation"]

            sent_ns = time.monotonic_ns()

            record["send_monotonic_ns"] = sent_ns

            state = {
                "goal_future": None,
                "handle": None,
                "result_future": None,
                "start_ns": sent_ns,
                "start_sim_ns":
                    probe.get_clock().now().nanoseconds,
                "terminal": False,
                "settle_deadline": None,
            }

            self.actions[robot] = state

            state["goal_future"] = (
                probe.navigation_client.send_goal_async(
                    goals[robot],
                    feedback_callback=(
                        probe.feedback_callback
                    ),
                )
            )

            print(
                f"Dispatched {robot}: "
                f"({probe.args.goal_x}, "
                f"{probe.args.goal_y})",
                flush=True,
            )

    def observe_actions(self):

        while True:

            self.tick()

            all_finished = True

            for robot, state in self.actions.items():

                probe = self.probes[robot]

                record = probe.summary["navigation"]

                now_ns = time.monotonic_ns()

                elapsed = (
                    now_ns - state["start_ns"]
                ) / 1e9

                if state["terminal"]:

                    if (
                        time.monotonic()
                        >= state["settle_deadline"]
                    ):

                        probe.phase = (
                            "idle_after_settling"
                        )

                        probe.summary["completed"] = True

                    else:

                        all_finished = False

                    continue

                all_finished = False

                if state["handle"] is None:

                    future = state["goal_future"]

                    if not future.done():

                        if elapsed > ACCEPTANCE_TIMEOUT:

                            raise TimeoutError(
                                f"{robot}: goal acceptance "
                                "timed out"
                            )

                        continue

                    handle = future.result()

                    state["handle"] = handle

                    record["accepted"] = bool(
                        handle.accepted
                    )

                    record["acceptance_wall_s"] = elapsed

                    if not handle.accepted:

                        raise RuntimeError(
                            f"{robot}: goal rejected"
                        )

                    state["result_future"] = (
                        handle.get_result_async()
                    )

                result_future = state["result_future"]

                if not result_future.done():

                    if elapsed > probe.args.goal_timeout:

                        raise TimeoutError(
                            f"{robot}: navigation timed out"
                        )

                    continue

                response = result_future.result()

                probe.record_action_result(
                    record,
                    response,
                )

                record[
                    "terminal_observed_monotonic_ns"
                ] = now_ns

                record["client_wall_duration_s"] = (
                    elapsed
                )

                record["client_sim_duration_s"] = (
                    (
                        probe.get_clock().now().nanoseconds
                        - state["start_sim_ns"]
                    ) / 1e9
                )

                state["terminal"] = (
                    response.status in TERMINAL_STATES
                )

                if (
                    response.status
                    != GoalStatus.STATUS_SUCCEEDED
                    or response.result.error_code != 0
                ):

                    raise RuntimeError(
                        f"{robot}: navigation failed"
                    )

                probe.phase = "settling"

                state["settle_deadline"] = (
                    time.monotonic()
                    + SETTLING_DURATION
                )

                print(
                    f"{robot}: navigation succeeded",
                    flush=True,
                )

            if all_finished:

                break

        self.verify_stationary()

    def cancel_outstanding(self):

        # The original probe cancellation methods
        # continue spinning the shared executor.
        for robot, state in self.actions.items():

            if state["terminal"]:

                continue

            probe = self.probes[robot]

            record = probe.summary["navigation"]

            try:

                handle = state["handle"]

                if handle is None:

                    future = state["goal_future"]

                    if future is None:

                        raise RuntimeError(
                            "Dispatch result unknown"
                        )

                    handle = probe.wait_future(
                        future,
                        5.0,
                        "outstanding goal response",
                    )

                    state["handle"] = handle

                record["accepted"] = bool(
                    handle.accepted
                )

                if not handle.accepted:

                    continue

                result_future = state["result_future"]

                if (
                    result_future is not None
                    and result_future.done()
                ):

                    response = result_future.result()

                    probe.record_action_result(
                        record,
                        response,
                    )

                    terminal = (
                        response.status
                        in TERMINAL_STATES
                    )

                    if terminal:

                        continue

                terminal = probe.cancel_action(
                    handle,
                    result_future,
                    record,
                )

                if not terminal:

                    probe.summary[
                        "requires_manual_stop"
                    ] = True

            except Exception as exc:

                record["cancellation_error"] = str(exc)

                probe.summary[
                    "requires_manual_stop"
                ] = True

    def summarize(self, execute, error):

        results = {}

        for robot, probe in self.probes.items():

            results[robot] = probe.summary

        planning_passed = (
            error is None
            and all(
                result["planning"].get("status")
                    == "SUCCEEDED"
                and result["planning"].get("error_code")
                    == 0
                and "endpoint_position_error_m"
                    in result["planning"]
                for result in results.values()
            )
        )

        send_times = []

        terminal_times = []

        for result in results.values():

            navigation = result["navigation"]

            if "send_monotonic_ns" in navigation:

                send_times.append(
                    navigation["send_monotonic_ns"]
                )

            if (
                "terminal_observed_monotonic_ns"
                in navigation
            ):

                terminal_times.append(
                    navigation[
                        "terminal_observed_monotonic_ns"
                    ]
                )

        dispatch_skew = None

        fleet_makespan = None

        common_overlap = None

        count = len(results)

        if len(send_times) == count:

            dispatch_skew = (
                max(send_times) - min(send_times)
            ) / 1e9

        if (
            len(send_times) == count
            and len(terminal_times) == count
        ):

            fleet_makespan = (
                max(terminal_times) - min(send_times)
            ) / 1e9

            common_overlap = max(
                0.0,
                (
                    min(terminal_times) - max(send_times)
                ) / 1e9,
            )

        execution_passed = None

        if execute:

            execution_passed = (
                planning_passed
                and common_overlap is not None
                and common_overlap > 0.0
                and all(
                    result["completed"]
                    and not result["requires_manual_stop"]
                    and result["stationary"] is True
                    and result["navigation"].get("status")
                        == "SUCCEEDED"
                    and result["navigation"].get("error_code")
                        == 0
                    for result in results.values()
                )
            )

        return {
            "robot_count": count,
            "execution_enabled": execute,
            "fleet_planning_passed": planning_passed,
            "simultaneous_navigation_passed":
                execution_passed,
            "dispatch_skew_s": dispatch_skew,
            "fleet_makespan_s": fleet_makespan,
            "common_action_overlap_s": common_overlap,
            "robot_results": results,
            "requires_manual_stop": any(
                result["requires_manual_stop"]
                for result in results.values()
            ),
            "error": error,
            "limitations": [
                "Preflight path requests are sequential",
                "Navigation goals are dispatched "
                "without waiting for prior acceptance",
                "Dispatch skew measures client calls, "
                "not DDS delivery or actuation skew",
                "Action overlap measures client-observed "
                "intervals, not simultaneous physical motion",
                "No centralized fleet traffic coordination",
                "No independent continuous ground truth",
                "Stationarity uses sampled odometry",
                "Shared executor introduces instrumentation "
                "workload",
                "Physical goal accuracy requires separate "
                "ground-truth analysis",
            ],
        }


def parse_args():

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--goal",
        nargs=4,
        action="append",
        required=True,
        metavar=("ROBOT", "X", "Y", "YAW"),
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

    if (
        not math.isfinite(args.goal_timeout)
        or args.goal_timeout <= 0.0
    ):

        parser.error("Invalid goal timeout")

    goals = {}

    for robot, x, y, yaw in args.goal:

        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*",
            robot,
        ):

            parser.error(
                f"Invalid namespace: {robot}"
            )

        if robot in goals:

            parser.error(
                f"Duplicate robot: {robot}"
            )

        try:

            coordinates = (
                float(x),
                float(y),
                float(yaw),
            )

        except ValueError:

            parser.error("Invalid goal coordinates")

        if not all(
            math.isfinite(value)
            for value in coordinates
        ):

            parser.error(
                "Goal coordinates must be finite"
            )

        goals[robot] = coordinates

    if len(goals) < 2:

        parser.error(
            "At least two robots are required"
        )

    args.goals = goals

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

    executor = SingleThreadedExecutor()

    coordinator = FleetNavigationCoordinator(
        executor
    )

    error = None

    summary = None

    exit_code = 0

    with ExitStack() as stack:

        try:

            for robot, goal in args.goals.items():

                directory = (
                    args.output_dir / robot
                )

                directory.mkdir()

                file = stack.enter_context(
                    (directory / "samples.csv").open(
                        "w",
                        newline="",
                    )
                )

                writer = csv.writer(
                    file,
                    lineterminator="\n",
                )

                writer.writerow(ACTIVE_FIELDS)

                robot_args = SimpleNamespace(
                    namespace=robot,
                    goal_x=goal[0],
                    goal_y=goal[1],
                    goal_yaw=goal[2],
                    goal_timeout=args.goal_timeout,
                    output_dir=directory,
                    execute=args.execute,
                )

                probe = ConcurrentNavigationProbe(
                    robot_args,
                    writer,
                    coordinator,
                )

                coordinator.add_probe(probe)

            coordinator.prepare()

            if args.execute:

                coordinator.dispatch()

                coordinator.observe_actions()

            else:

                # Planning-only mode never dispatches
                # a NavigateToPose action.
                for probe in coordinator.probes.values():

                    probe.phase = "post_planning"

                deadline = (
                    time.monotonic()
                    + SETTLING_DURATION
                )

                while time.monotonic() < deadline:

                    coordinator.tick()

                    coordinator.verify_stationary()

                for probe in coordinator.probes.values():

                    probe.summary["completed"] = True

        except (Exception, KeyboardInterrupt) as exc:

            error = str(exc) or type(exc).__name__

            exit_code = (
                130 if isinstance(
                    exc, KeyboardInterrupt
                ) else 1
            )

            print(
                f"Fleet error: {error}",
                file=sys.stderr,
                flush=True,
            )

            coordinator.cancel_outstanding()

        finally:

            try:

                for robot, probe in (
                    coordinator.probes.items()
                ):

                    probe.finalize()

                    if error is not None:

                        probe.summary[
                            "fleet_error"
                        ] = error

                    save_json(
                        probe.args.output_dir
                        / "summary.json",
                        probe.summary,
                    )

                summary = coordinator.summarize(
                    args.execute,
                    error,
                )

                save_json(
                    args.output_dir
                    / "fleet_summary.json",
                    summary,
                )

            finally:

                for probe in (
                    coordinator.probes.values()
                ):

                    executor.remove_node(probe)

                    probe.destroy_node()

                executor.shutdown()

                if rclpy.ok():

                    rclpy.shutdown()

    if summary is None:

        return 1

    print(
        json.dumps(
            summary,
            indent=2,
            allow_nan=False,
        )
    )

    if summary["requires_manual_stop"]:

        print(
            "WARNING: Action termination not confirmed. "
            "Stop fleet navigation.",
            file=sys.stderr,
        )

    passed = (
        summary["simultaneous_navigation_passed"]
        if args.execute
        else summary["fleet_planning_passed"]
    )

    if not passed and exit_code == 0:

        exit_code = 1

    return exit_code


if __name__ == "__main__":

    sys.exit(main())