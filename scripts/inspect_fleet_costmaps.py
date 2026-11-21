#!/usr/bin/env python3

"""Inspect live fleet costmaps without commanding robot motion."""

import argparse
import json
import math
import time

from collections import Counter
from functools import partial
from pathlib import Path

import yaml
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid


OBSERVATION_TIMEOUT = 30.0
GOAL_SCREENING_RADIUS = 0.35


def stamp_ns(stamp):

    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion_yaw(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FleetCostmapInspector(Node):

    def __init__(self, robots):

        super().__init__("amr_fleet_costmap_inspector")

        self.expected = [
            (robot, layer)
            for robot in robots
            for layer in ("local", "global")
        ]

        self.observations = {}
        self.costmap_subscriptions = []

        for robot, layer in self.expected:

            topic = (
                f"/{robot}/{layer}_costmap/costmap"
            )

            subscription = self.create_subscription(
                OccupancyGrid,
                topic,
                partial(
                    self.on_costmap,
                    robot,
                    layer,
                ),
                qos_profile_sensor_data,
            )

            self.costmap_subscriptions.append(
                subscription
            )

    def on_costmap(self, robot, layer, msg):

        key = (robot, layer)

        timestamp = stamp_ns(msg.header.stamp)

        if key not in self.observations:

            self.observations[key] = {
                "first_stamp_ns": timestamp,
                "last_stamp_ns": timestamp,
                "messages": 1,
                "advancing_stamps": 0,
                "message": msg,
            }

            return

        observation = self.observations[key]

        observation["messages"] += 1

        if timestamp > observation["last_stamp_ns"]:

            observation["advancing_stamps"] += 1

        observation["last_stamp_ns"] = timestamp
        observation["message"] = msg

    def observe(self):

        deadline = (
            time.monotonic() + OBSERVATION_TIMEOUT
        )

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if all(
                key in self.observations
                and self.observations[key][
                    "advancing_stamps"
                ] >= 1
                for key in self.expected
            ):
                return True

        return False


def summarize_costmap(observation, layer):

    msg = observation["message"]

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution

    origin = msg.info.origin

    counts = Counter(msg.data)

    expected_frame = (
        "odom" if layer == "local"
        else "map"
    )

    valid_values = all(
        -1 <= value <= 100
        for value in counts
    )

    valid_grid = (
        width > 0
        and height > 0
        and math.isfinite(resolution)
        and resolution > 0.0
        and len(msg.data) == width * height
        and valid_values
    )

    known_cells = len(msg.data) - counts[-1]

    passed = (
        valid_grid
        and msg.header.frame_id == expected_frame
        and observation["advancing_stamps"] >= 1
        and observation["last_stamp_ns"]
            > observation["first_stamp_ns"]
        and known_cells > 0
    )

    return {
        "frame_id": msg.header.frame_id,
        "width_cells": width,
        "height_cells": height,
        "resolution_m": resolution,
        "origin": {
            "x": origin.position.x,
            "y": origin.position.y,
            "yaw": quaternion_yaw(
                origin.orientation
            ),
        },
        "total_cells": len(msg.data),
        "free_cells": counts[0],
        "inflated_cells": sum(
            count
            for value, count in counts.items()
            if 1 <= value <= 98
        ),
        "inscribed_cells": counts[99],
        "lethal_cells": counts[100],
        "unknown_cells": counts[-1],
        "received_messages":
            observation["messages"],
        "advancing_timestamps":
            observation["advancing_stamps"],
        "first_stamp_ns":
            observation["first_stamp_ns"],
        "last_stamp_ns":
            observation["last_stamp_ns"],
        "costmap_passed": passed,
    }


def inspect_candidate(msg, x, y):

    info = msg.info

    if (
        msg.header.frame_id != "map"
        or info.resolution <= 0
        or len(msg.data) != info.width * info.height
    ):
        return {
            "goal": {"x": x, "y": y},
            "passed_screening": False,
            "reason": "Invalid global costmap",
        }

    yaw = quaternion_yaw(
        info.origin.orientation
    )

    dx = x - info.origin.position.x
    dy = y - info.origin.position.y

    grid_x = (
        math.cos(yaw) * dx
        + math.sin(yaw) * dy
    )

    grid_y = (
        -math.sin(yaw) * dx
        + math.cos(yaw) * dy
    )

    column = math.floor(
        grid_x / info.resolution
    )

    row = math.floor(
        grid_y / info.resolution
    )

    radius = GOAL_SCREENING_RADIUS

    min_column = math.floor(
        (grid_x - radius) / info.resolution
    )

    max_column = math.floor(
        (grid_x + radius) / info.resolution
    )

    min_row = math.floor(
        (grid_y - radius) / info.resolution
    )

    max_row = math.floor(
        (grid_y + radius) / info.resolution
    )

    result = {
        "goal": {"x": x, "y": y},
        "grid_column": column,
        "grid_row": row,
        "screening_radius_m": radius,
    }

    if (
        min_column < 0
        or min_row < 0
        or max_column >= info.width
        or max_row >= info.height
    ):

        result.update({
            "passed_screening": False,
            "reason": "Goal window outside costmap",
        })

        return result

    values = [
        msg.data[r * info.width + c]
        for r in range(min_row, max_row + 1)
        for c in range(min_column, max_column + 1)
    ]

    counts = Counter(values)

    center_cost = msg.data[
        row * info.width + column
    ]

    blocked_cells = (
        counts[-1]
        + counts[99]
        + counts[100]
    )

    result.update({
        "center_cost": center_cost,
        "maximum_window_cost": max(values),
        "window_cells": len(values),
        "unknown_cells": counts[-1],
        "inscribed_cells": counts[99],
        "lethal_cells": counts[100],
        "passed_screening": (
            blocked_cells == 0
        ),
    })

    return result


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
        "--candidate",
        nargs=3,
        action="append",
        default=[],
        metavar=("ROBOT", "X", "Y"),
    )

    args = parser.parse_args()

    config = yaml.safe_load(
        args.fleet_config.read_text()
    )

    robots = [
        robot["namespace"]
        for robot in config["robots"]
    ]

    if (
        not robots
        or len(set(robots)) != len(robots)
    ):
        parser.error("Invalid fleet namespaces")

    candidates = []

    for robot, x, y in args.candidate:

        x = float(x)
        y = float(y)

        if robot not in robots:
            parser.error(f"Unknown robot: {robot}")

        if not all(
            math.isfinite(value)
            for value in (x, y)
        ):
            parser.error("Invalid goal coordinates")

        candidates.append((robot, x, y))

    rclpy.init()

    node = FleetCostmapInspector(robots)

    try:
        completed = node.observe()

        results = {}

        for robot in robots:

            results[robot] = {}

            for layer in ("local", "global"):

                key = (robot, layer)

                observation = node.observations.get(
                    key
                )

                if observation is None:

                    results[robot][layer] = {
                        "costmap_passed": False,
                        "error": "No messages received",
                    }

                    continue

                results[robot][layer] = (
                    summarize_costmap(
                        observation,
                        layer,
                    )
                )

        goal_results = []

        for robot, x, y in candidates:

            observation = node.observations.get(
                (robot, "global")
            )

            if observation is None:

                result = {
                    "goal": {"x": x, "y": y},
                    "passed_screening": False,
                    "reason": "Global costmap unavailable",
                }

            else:

                result = inspect_candidate(
                    observation["message"],
                    x,
                    y,
                )

            result["robot"] = robot

            goal_results.append(result)

        passed = (
            completed
            and all(
                results[robot][layer]["costmap_passed"]
                for robot, layer in node.expected
            )
        )

        summary = {
            "robot_count": len(robots),
            "expected_costmaps": len(node.expected),
            "robot_results": results,
            "candidate_goals": goal_results,
            "fleet_costmap_validation_passed": passed,
            "limitations": [
                "Checks sampled costmap publications",
                "Does not measure costmap update latency",
                "Candidate screening is not path planning",
                "Goal screening uses a conservative square",
                "Costmaps may change before execution",
                "No collision-free motion is guaranteed",
            ],
        }

        print(
            json.dumps(
                summary,
                indent=2,
                allow_nan=False,
            )
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    if not passed:
        raise SystemExit(
            "Fleet costmap validation failed"
        )


if __name__ == "__main__":
    main()