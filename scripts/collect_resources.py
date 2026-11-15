#!/usr/bin/env python3

"""Collect Docker container and NVIDIA GPU resource metrics."""

import argparse
import csv
import json
import re
import statistics
import subprocess
import time

from datetime import datetime, timezone
from pathlib import Path


CONTAINER = "amr-scalability-simulation"


def run_command(command):
    """Execute a command and return its output."""
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=True,
        timeout=10,
    )
    return result.stdout.strip()


def memory_to_mib(value):
    """Convert Docker memory values to MiB."""
    match = re.match(
        r"([\d.]+)\s*(B|KiB|MiB|GiB|TiB|kB|MB|GB|TB)\b",
        value,
        re.IGNORECASE,
    )

    if not match:
        raise ValueError(f"Unsupported memory format: {value}")

    amount = float(match.group(1))
    unit = match.group(2)

    factors = {
        "B": 1 / 1048576,
        "KiB": 1 / 1024,
        "MiB": 1,
        "GiB": 1024,
        "TiB": 1048576,
        "kB": 1000 / 1048576,
        "MB": 1000000 / 1048576,
        "GB": 1000000000 / 1048576,
        "TB": 1000000000000 / 1048576,
    }

    return amount * factors[unit]


def docker_snapshot():
    """Read resource usage for the simulation container."""
    output = run_command([
        "docker",
        "stats",
        "--no-stream",
        "--format",
        "{{json .}}",
        CONTAINER,
    ])

    data = json.loads(output.splitlines()[0])

    return {
        "cpu_pct": float(data["CPUPerc"].rstrip("%")),
        "ram_mib": memory_to_mib(
            data["MemUsage"].split("/")[0].strip()
        ),
        "pids": int(data["PIDs"]),
    }


def gpu_snapshot():
    """Read system-wide NVIDIA GPU metrics."""
    output = run_command([
        "nvidia-smi",
        "--query-gpu="
        "utilization.gpu,"
        "utilization.memory,"
        "memory.used,"
        "temperature.gpu,"
        "power.draw",
        "--format=csv,noheader,nounits",
    ])

    values = next(csv.reader(output.splitlines()))

    def parse_value(value):
        try:
            return float(value.strip())
        except ValueError:
            return None

    return {
        "gpu_pct": parse_value(values[0]),
        "gpu_memory_pct": parse_value(values[1]),
        "vram_mib": parse_value(values[2]),
        "gpu_temp_c": parse_value(values[3]),
        "gpu_power_w": parse_value(values[4]),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Collect simulation resource metrics."
    )

    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--interval", type=float, default=2.0)

    args = parser.parse_args()

    if args.duration <= 0 or args.interval <= 0:
        parser.error("Duration and interval must be positive.")

    print("===== RESOURCE COLLECTION =====")
    print(f"Container: {CONTAINER}")
    print(f"Duration:  {args.duration} seconds")
    print(f"Interval:  {args.interval} seconds")
    print()

    rows = []
    start = time.monotonic()
    sample_count = max(1, int(args.duration / args.interval))

    try:
        for index in range(sample_count):
            target = start + index * args.interval

            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)

            docker = docker_snapshot()
            gpu = gpu_snapshot()

            row = {
                "timestamp_utc": datetime.now(
                    timezone.utc
                ).isoformat(timespec="milliseconds"),
                "elapsed_s": round(time.monotonic() - start, 3),
                **docker,
                **gpu,
            }

            rows.append(row)

            print(
                f"Sample {index + 1:02d}: "
                f"CPU={row['cpu_pct']:.1f}%  "
                f"RAM={row['ram_mib']:.1f} MiB  "
                f"GPU={row['gpu_pct']}%"
            )

    except (subprocess.SubprocessError, ValueError, KeyError) as error:
        parser.exit(1, f"ERROR: {error}\n")

    output_dir = Path("results/raw")
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    output_file = (
        output_dir / f"resources_1robot_{timestamp}.csv"
    )

    with output_file.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=rows[0].keys(),
            lineterminator="\n",
        )

        writer.writeheader()
        writer.writerows(rows)

    print("\n===== RESOURCE SUMMARY =====")

    for field, label, unit in [
        ("cpu_pct", "CPU", "%"),
        ("ram_mib", "RAM", "MiB"),
        ("gpu_pct", "GPU", "%"),
        ("vram_mib", "VRAM", "MiB"),
    ]:
        values = [
            row[field]
            for row in rows
            if row[field] is not None
        ]

        if not values:
            continue

        print(
            f"{label:5s} "
            f"Mean={statistics.mean(values):8.2f} {unit}  "
            f"Min={min(values):8.2f}  "
            f"Max={max(values):8.2f}"
        )

    print(f"\nSamples collected: {len(rows)}")
    print(f"CSV saved: {output_file}")


if __name__ == "__main__":
    main()