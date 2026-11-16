#!/usr/bin/env python3
"""Analyze ROS 2 application-level DDS probe results."""

import argparse
import csv
import math
import statistics

from collections import Counter
from pathlib import Path


def percentile(values, percentage):
    """Calculate an interpolated percentile."""

    ordered = sorted(values)

    position = (
        len(ordered) - 1
    ) * percentage / 100

    lower = math.floor(position)
    upper = math.ceil(position)

    fraction = position - lower

    return (
        ordered[lower] * (1 - fraction)
        + ordered[upper] * fraction
    )


def report_statistics(name, values, unit):
    if not values:
        print(f"{name}: insufficient data")
        return

    print(f"\n{name}")
    print(f"  Mean:    {statistics.mean(values):.3f} {unit}")
    print(f"  Median:  {statistics.median(values):.3f} {unit}")
    print(f"  P95:     {percentile(values, 95):.3f} {unit}")
    print(f"  P99:     {percentile(values, 99):.3f} {unit}")
    print(f"  Minimum: {min(values):.3f} {unit}")
    print(f"  Maximum: {max(values):.3f} {unit}")
    print(f"  Std dev: {statistics.pstdev(values):.3f} {unit}")


def intervals_ms(timestamps):
    return [
        (b - a) / 1_000_000
        for a, b in zip(
            timestamps,
            timestamps[1:],
        )
    ]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument("csv_file")

    parser.add_argument(
        "--expected-count",
        type=int,
        default=1000,
    )

    parser.add_argument(
        "--expected-payload",
        type=int,
        default=256,
    )

    args = parser.parse_args()

    if args.expected_count <= 0:
        parser.error("Expected count must be positive.")

    with Path(args.csv_file).open(newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise SystemExit("No messages recorded.")

    sequences = [
        int(row["sequence"])
        for row in rows
    ]

    sent = [
        int(row["sent_monotonic_ns"])
        for row in rows
    ]

    received = [
        int(row["received_monotonic_ns"])
        for row in rows
    ]

    sizes = [
        int(row["payload_bytes"])
        for row in rows
    ]

    latency_us = [
        (r - s) / 1000
        for s, r in zip(sent, received)
    ]

    expected = set(range(args.expected_count))
    observed = set(sequences)

    missing = len(expected - observed)

    duplicates = sum(
        count - 1
        for count in Counter(sequences).values()
    )

    reordered = sum(
        b < a
        for a, b in zip(
            sequences,
            sequences[1:],
        )
    )

    unexpected = len(observed - expected)

    negative_latencies = sum(
        value < 0
        for value in latency_us
    )

    payload_mismatches = sum(
        size != args.expected_payload
        for size in sizes
    )

    print("\n===== DDS COMMUNICATION ANALYSIS =====")

    print("\nMESSAGE DELIVERY")
    print(f"  Expected:          {args.expected_count}")
    print(f"  Received:          {len(rows)}")
    print(f"  Unique sequences:  {len(observed)}")
    print(f"  Missing sequences: {missing}")
    print(f"  Duplicates:        {duplicates}")
    print(f"  Reordered pairs:   {reordered}")
    print(f"  Unexpected IDs:    {unexpected}")
    print(f"  Negative latency:  {negative_latencies}")
    print(f"  Payload mismatches: {payload_mismatches}")

    report_statistics(
        "END-TO-END APPLICATION LATENCY",
        latency_us,
        "µs",
    )

    send_intervals = intervals_ms(sent)
    arrival_intervals = intervals_ms(received)

    report_statistics(
        "PUBLISHER INTERVALS",
        send_intervals,
        "ms",
    )

    report_statistics(
        "SUBSCRIBER ARRIVAL INTERVALS",
        arrival_intervals,
        "ms",
    )

    if len(sent) > 1:
        send_duration_s = (
            max(sent) - min(sent)
        ) / 1_000_000_000

        receive_duration_s = (
            max(received) - min(received)
        ) / 1_000_000_000

        print("\nTHROUGHPUT")

        if send_duration_s > 0:
            publication_rate = (
                len(sent) - 1
            ) / send_duration_s

            print(
                f"  Publication rate: "
                f"{publication_rate:.3f} Hz"
            )

        if receive_duration_s > 0:
            throughput = (
                sum(sizes) / receive_duration_s
            )

            print(
                f"  Application payload: "
                f"{throughput / 1000:.3f} kB/s"
            )

    passed = (
        missing == 0
        and duplicates == 0
        and reordered == 0
        and unexpected == 0
        and negative_latencies == 0
        and payload_mismatches == 0
        and len(rows) == args.expected_count
    )

    print("\n===== VALIDATION =====")
    print("PASS" if passed else "FAIL")

    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()