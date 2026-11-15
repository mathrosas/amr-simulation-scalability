#!/usr/bin/env python3

"""Collect Gazebo real-time factor measurements."""

import argparse
import csv
import re
import statistics
import subprocess
import sys

from datetime import datetime
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Collect Gazebo RTF measurements."
    )

    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Measurement duration in seconds (default: 30).",
    )

    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")

    print(f"Collecting RTF measurements for {args.duration} seconds...")

    command = [
        "docker",
        "compose",
        "exec",
        "-T",
        "simulation",
        "bash",
        "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            f"timeout {args.duration} "
            "gz topic -e -t /world/warehouse/stats"
        ),
    ]

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=args.duration + 20,
        )
    except subprocess.TimeoutExpired:
        sys.exit("ERROR: Gazebo statistics collection timed out.")

    # GNU timeout normally returns 124 when the duration expires.
    if result.returncode not in (0, 124):
        sys.exit(
            f"ERROR: Gazebo command failed:\n{result.stderr}"
        )

    pattern = r"(?m)^\s*real_time_factor:\s*([0-9.eE+-]+)"

    values = [
        float(value)
        for value in re.findall(pattern, result.stdout)
    ]

    if not values:
        sys.exit(
            "ERROR: No RTF measurements received. "
            "Check that Gazebo is running and unpaused."
        )

    # Create a unique filename for this run.
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    output_dir = Path("results/raw")
    output_dir.mkdir(parents=True, exist_ok=True)

    output_file = output_dir / f"rtf_1robot_{timestamp}.csv"

    with output_file.open("w", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")

        writer.writerow(["sample_index", "rtf"])

        for index, value in enumerate(values):
            writer.writerow([index, value])

    print("\n===== RTF RESULTS =====")
    print(f"Samples:   {len(values)}")
    print(f"Mean:      {statistics.mean(values):.6f}")
    print(f"Median:    {statistics.median(values):.6f}")
    print(f"Minimum:   {min(values):.6f}")
    print(f"Maximum:   {max(values):.6f}")
    print(f"Std. Dev.: {statistics.pstdev(values):.6f}")

    print(f"\nCSV saved: {output_file}")


if __name__ == "__main__":
    main()