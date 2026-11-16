#!/usr/bin/env python3
"""Monitor ROS 2 sensor and odometry message timing."""

import csv
import os
import sys
import time

from collections import defaultdict

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry


TOPICS = {
    "lidar": (
        LaserScan,
        "/j100_0000/sensors/lidar2d_0/scan",
    ),
    "odom_raw": (
        Odometry,
        "/j100_0000/platform/odom",
    ),
    "odom_filtered": (
        Odometry,
        "/j100_0000/platform/odom/filtered",
    ),
    "imu": (
        Imu,
        "/j100_0000/sensors/imu_0/data",
    ),
    "joint_states": (
        JointState,
        "/j100_0000/platform/joint_states",
    ),
}


def main():
    duration = float(os.getenv("MONITOR_DURATION_S", "10"))
    warmup = float(os.getenv("MONITOR_WARMUP_S", "3"))

    if duration <= 0 or warmup < 0:
        raise ValueError("Invalid monitoring duration or warm-up")

    rclpy.init()
    node = rclpy.create_node("amr_sensor_monitor")

    writer = csv.writer(sys.stdout, lineterminator="\n")

    writer.writerow([
        "stream",
        "topic",
        "received_utc_ns",
        "received_monotonic_ns",
        "header_stamp_ns",
        "arrival_interval_ms",
        "stamp_interval_ms",
    ])

    previous = {}
    counts = defaultdict(int)

    scheduled_start = os.getenv("MONITOR_START_UTC_NS")

    if scheduled_start is not None:
        start_utc_ns = int(scheduled_start)
    else:
        start_utc_ns = (
            time.time_ns()
            + int(warmup * 1_000_000_000)
        )

    end_utc_ns = (
        start_utc_ns
        + int(duration * 1_000_000_000)
    )

    def make_callback(stream, topic):

        def callback(msg):
            now_monotonic = time.monotonic_ns()
            now_utc = time.time_ns()

            if not (
                start_utc_ns <= now_utc < end_utc_ns
            ):
                return

            stamp = msg.header.stamp

            stamp_ns = (
                stamp.sec * 1_000_000_000
                + stamp.nanosec
            )

            previous_values = previous.get(stream)

            arrival_interval = ""
            stamp_interval = ""

            if previous_values is not None:
                previous_arrival, previous_stamp = previous_values

                arrival_interval = (
                    now_monotonic - previous_arrival
                ) / 1_000_000

                stamp_interval = (
                    stamp_ns - previous_stamp
                ) / 1_000_000

            writer.writerow([
                stream,
                topic,
                now_utc,
                now_monotonic,
                stamp_ns,
                arrival_interval,
                stamp_interval,
            ])

            previous[stream] = (
                now_monotonic,
                stamp_ns,
            )

            counts[stream] += 1

        return callback

    subscriptions = []

    for stream, (message_type, topic) in TOPICS.items():
        subscription = node.create_subscription(
            message_type,
            topic,
            make_callback(stream, topic),
            qos_profile_sensor_data,
        )

        subscriptions.append(subscription)

    try:
        while time.time_ns() < end_utc_ns:
            rclpy.spin_once(node, timeout_sec=0.05)

    finally:
        print(
            "\n===== SENSOR MONITOR RESULTS =====",
            file=sys.stderr,
        )

        for stream in TOPICS:
            print(
                f"{stream}: {counts[stream]} messages",
                file=sys.stderr,
            )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()