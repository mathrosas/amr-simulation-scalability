#!/usr/bin/env python3
"""Analyze ROS 2 sensor timing measurements."""

import argparse
import csv
import statistics
from collections import defaultdict


def calculate_intervals(values):
    return [
        (b - a) / 1_000_000
        for a, b in zip(values, values[1:])
    ]


def print_statistics(label, intervals):
    if not intervals:
        print(f"  {label}: insufficient data")
        return

    mean = statistics.mean(intervals)

    print(f"  {label}:")
    print(f"    Mean:       {mean:.3f} ms")
    print(f"    Median:     {statistics.median(intervals):.3f} ms")
    print(f"    Minimum:    {min(intervals):.3f} ms")
    print(f"    Maximum:    {max(intervals):.3f} ms")
    print(f"    Std dev:    {statistics.pstdev(intervals):.3f} ms")

    if mean > 0:
        print(f"    Frequency:  {1000 / mean:.3f} Hz")

    nonpositive = sum(value <= 0 for value in intervals)

    positive = [value for value in intervals if value > 0]

    if positive:
        median = statistics.median(positive)
        long_gaps = sum(
            value > 1.5 * median
            for value in positive
        )
    else:
        long_gaps = 0

    print(f"    Nonpositive intervals: {nonpositive}")
    print(f"    Long gaps (>1.5x median): {long_gaps}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    args = parser.parse_args()

    streams = defaultdict(list)

    with open(args.csv_file, newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            streams[row["stream"]].append(row)

    print("\n===== SENSOR TIMING ANALYSIS =====")

    for stream, rows in sorted(streams.items()):
        print(f"\n===== {stream.upper()} =====")
        print(f"Messages received: {len(rows)}")

        arrivals = [
            int(row["received_monotonic_ns"])
            for row in rows
        ]

        stamps = [
            int(row["header_stamp_ns"])
            for row in rows
        ]

        arrival_intervals = calculate_intervals(arrivals)
        stamp_intervals = calculate_intervals(stamps)

        print_statistics(
            "Arrival intervals",
            arrival_intervals,
        )

        print_statistics(
            "Header timestamp intervals",
            stamp_intervals,
        )


if __name__ == "__main__":
    main()