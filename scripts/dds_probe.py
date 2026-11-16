#!/usr/bin/env python3
"""Controlled same-host ROS 2 publisher/subscriber probe."""

import argparse
import csv
import struct
import sys
import time

import rclpy

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import UInt8MultiArray


END_SEQUENCE = (1 << 64) - 1

HEADER_FORMAT = "!QQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

TOPIC = "/benchmark/dds_probe"


def qos_profile():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def publish(args):
    rclpy.init()

    node = rclpy.create_node("dds_probe_publisher")

    publisher = node.create_publisher(
        UInt8MultiArray,
        args.topic,
        qos_profile(),
    )

    try:
        deadline = time.monotonic() + args.timeout

        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "No subscriber matched before timeout"
                )

            time.sleep(0.1)

        print(
            "Subscriber matched; starting transmission.",
            file=sys.stderr,
            flush=True,
        )

        time.sleep(0.25)

        payload = bytes(
            args.payload_bytes - HEADER_SIZE
        )

        period_ns = round(
            1_000_000_000 / args.rate
        )

        start_ns = (
            time.monotonic_ns()
            + 100_000_000
        )

        for sequence in range(args.count):
            target_ns = (
                start_ns + sequence * period_ns
            )

            remaining_ns = (
                target_ns - time.monotonic_ns()
            )

            if remaining_ns > 0:
                time.sleep(
                    remaining_ns / 1_000_000_000
                )

            timestamp_ns = time.monotonic_ns()

            message = UInt8MultiArray()

            message.data = list(
                struct.pack(
                    HEADER_FORMAT,
                    sequence,
                    timestamp_ns,
                ) + payload
            )

            publisher.publish(message)

        time.sleep(0.2)

        for _ in range(3):
            message = UInt8MultiArray()

            message.data = list(
                struct.pack(
                    HEADER_FORMAT,
                    END_SEQUENCE,
                    time.monotonic_ns(),
                )
            )

            publisher.publish(message)
            time.sleep(0.1)

        time.sleep(0.5)

        print(
            f"Sent {args.count} data messages "
            f"at target {args.rate:g} Hz, "
            f"payload {args.payload_bytes} bytes; "
            f"sent end markers.",
            file=sys.stderr,
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


def subscribe(args):
    rclpy.init()

    node = rclpy.create_node("dds_probe_subscriber")

    writer = csv.writer(
        sys.stdout,
        lineterminator="\n",
    )

    writer.writerow([
        "sequence",
        "sent_monotonic_ns",
        "received_monotonic_ns",
        "latency_us",
        "payload_bytes",
    ])

    received_sequences = []

    last_data_received_at = None
    end_seen = False

    def on_message(message):
        nonlocal end_seen
        nonlocal last_data_received_at

        received_ns = time.monotonic_ns()

        data = bytes(message.data)

        if len(data) < HEADER_SIZE:
            print(
                "Malformed message received",
                file=sys.stderr,
            )
            return

        sequence, sent_ns = struct.unpack(
            HEADER_FORMAT,
            data[:HEADER_SIZE],
        )

        if sequence == END_SEQUENCE:
            end_seen = True
            return

        received_sequences.append(sequence)

        last_data_received_at = time.monotonic()

        writer.writerow([
            sequence,
            sent_ns,
            received_ns,
            round(
                (received_ns - sent_ns) / 1000,
                3,
            ),
            len(data),
        ])

    subscription = node.create_subscription(
        UInt8MultiArray,
        args.topic,
        on_message,
        qos_profile(),
    )

    print(
        "Subscriber listening; "
        "start the publisher in terminal 2.",
        file=sys.stderr,
        flush=True,
    )

    try:
        deadline = time.monotonic() + args.timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if (
                end_seen
                and last_data_received_at is not None
            ):
                if (
                    time.monotonic()
                    - last_data_received_at
                    >= 1.0
                ):
                    break

    finally:
        node.destroy_node()
        rclpy.shutdown()

    unique = set(received_sequences)

    missing = len(
        set(range(args.count)) - unique
    )

    duplicates = (
        len(received_sequences) - len(unique)
    )

    reordered = sum(
        b < a
        for a, b in zip(
            received_sequences,
            received_sequences[1:],
        )
    )

    print(
        f"Received={len(received_sequences)} "
        f"unique={len(unique)} "
        f"missing={missing} "
        f"duplicates={duplicates} "
        f"reordered_pairs={reordered} "
        f"end_marker_seen={end_seen}",
        file=sys.stderr,
        flush=True,
    )

    if not end_seen:
        raise SystemExit(
            "Subscriber timeout before end marker; "
            "inspect the logs"
        )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "role",
        choices=["publisher", "subscriber"],
    )

    parser.add_argument(
        "--topic",
        default=TOPIC,
    )

    parser.add_argument(
        "--count",
        type=int,
        default=1000,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--payload-bytes",
        type=int,
        default=256,
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=90.0,
    )

    args = parser.parse_args()

    if (
        args.count <= 0
        or args.rate <= 0
        or args.payload_bytes < HEADER_SIZE
        or args.timeout <= 0
    ):
        parser.error(
            "Require count > 0, rate > 0, "
            "payload-bytes >= 16, timeout > 0"
        )

    if args.role == "publisher":
        publish(args)
    else:
        subscribe(args)


if __name__ == "__main__":
    main()