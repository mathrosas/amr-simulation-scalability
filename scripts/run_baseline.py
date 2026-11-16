#!/usr/bin/env python3
"""Collect Gazebo statistics and resource measurements in one run."""

import argparse
import csv
import json
import math
import re
import statistics
import subprocess
import threading
import time

from datetime import datetime, timezone
from pathlib import Path

from collect_resources import docker_snapshot, gpu_snapshot


STATS_TOPIC = "/world/warehouse/stats"


def utc_now():
    return datetime.now(timezone.utc).isoformat(
        timespec="milliseconds"
    )


def gazebo_time(message, name):
    match = re.search(
        rf"(?m)^{re.escape(name)}\s*\{{"
        rf"\s*sec:\s*(\d+)\s*nsec:\s*(\d+)",
        message,
    )

    if match is None:
        return None

    return (
        int(match.group(1))
        + int(match.group(2)) / 1_000_000_000
    )


def parse_stat(message):
    factor = re.search(
        r"(?m)^real_time_factor:\s*([0-9.eE+-]+)",
        message,
    )

    if factor is None:
        return None

    iterations = re.search(
        r"(?m)^iterations:\s*(\d+)",
        message,
    )

    return {
        "rtf": float(factor.group(1)),
        "sim_time_s": gazebo_time(message, "sim_time"),
        "gazebo_real_time_s": gazebo_time(
            message, "real_time"
        ),
        "iterations": (
            int(iterations.group(1))
            if iterations else None
        ),
    }


def read_gazebo(stdout, start_monotonic, samples):
    lines = []

    def collect():
        if not lines:
            return

        value = parse_stat("".join(lines))

        if value is not None:
            samples.append({
                "received_utc": utc_now(),
                "received_elapsed_s": round(
                    time.monotonic() - start_monotonic, 3
                ),
                **value,
            })

        lines.clear()

    for line in stdout:
        if line.strip():
            lines.append(line)
        else:
            collect()

    collect()


def save_csv(path, rows):
    if not rows:
        return

    with path.open("w", newline="") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=rows[0].keys(),
            lineterminator="\n",
        )

        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--duration", type=int, default=30
    )

    parser.add_argument(
        "--interval", type=float, default=2.0
    )

    args = parser.parse_args()

    if args.duration <= 0 or args.interval <= 0:
        parser.error(
            "Duration and interval must be positive."
        )

    run_id = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    output_dir = (
        Path("results/raw")
        / f"baseline_1robot_{run_id}"
    )

    output_dir.mkdir(
        parents=True, exist_ok=False
    )

    print(
        f"Experiment directory: {output_dir}",
        flush=True,
    )

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
            f"gz topic -e -t {STATS_TOPIC}"
        ),
    ]

    rtf_samples = []
    resource_samples = []

    start_utc = utc_now()
    start = time.monotonic()

    error = None

    with (
        output_dir / "gazebo_stderr.log"
    ).open("w") as gazebo_log:

        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=gazebo_log,
            text=True,
            bufsize=1,
        )

        reader = threading.Thread(
            target=read_gazebo,
            args=(
                process.stdout,
                start,
                rtf_samples,
            ),
            daemon=True,
        )

        reader.start()

        try:
            sample_count = math.ceil(
                args.duration / args.interval
            )

            for i in range(sample_count):
                target = (
                    start + i * args.interval
                )

                delay = (
                    target - time.monotonic()
                )

                if delay > 0:
                    time.sleep(delay)

                if (
                    time.monotonic() - start
                    >= args.duration
                ):
                    break

                docker_start = (
                    time.monotonic() - start
                )

                docker = docker_snapshot()

                docker_end = (
                    time.monotonic() - start
                )

                gpu_start = (
                    time.monotonic() - start
                )

                gpu = gpu_snapshot()

                gpu_end = (
                    time.monotonic() - start
                )

                resource_samples.append({
                    "received_utc": utc_now(),

                    "docker_start_elapsed_s":
                        round(docker_start, 3),

                    "docker_end_elapsed_s":
                        round(docker_end, 3),

                    "gpu_start_elapsed_s":
                        round(gpu_start, 3),

                    "gpu_end_elapsed_s":
                        round(gpu_end, 3),

                    **docker,
                    **gpu,
                })

                print(
                    f"Resources "
                    f"{len(resource_samples):02d}: "
                    f"CPU={docker['cpu_pct']:.1f}% "
                    f"RAM={docker['ram_mib']:.1f} MiB "
                    f"GPU={gpu['gpu_pct']}%",
                    flush=True,
                )

        except (
            subprocess.SubprocessError,
            ValueError,
            KeyError,
            IndexError,
        ) as exc:
            error = str(exc)

        finally:
            try:
                code = process.wait(
                    timeout=args.duration + 10
                )

            except subprocess.TimeoutExpired:
                process.kill()
                code = process.wait()

            reader.join(timeout=5)
            process.stdout.close()

    save_csv(
        output_dir / "rtf.csv",
        rtf_samples,
    )

    save_csv(
        output_dir / "resources.csv",
        resource_samples,
    )

    metadata = {
        "experiment": "001_single_robot_idle_gui",
        "started_utc": start_utc,
        "finished_utc": utc_now(),
        "requested_duration_s": args.duration,
        "resource_interval_s": args.interval,
        "robot_count": 1,
        "robot": "Clearpath Jackal J100",
        "lidar_enabled": True,
        "gui_enabled": True,
        "gazebo_stats_topic": STATS_TOPIC,
        "rtf_samples": len(rtf_samples),
        "resource_samples": len(resource_samples),
        "gazebo_command_exit_code": code,
        "resource_error": error,
    }

    (
        output_dir / "metadata.json"
    ).write_text(
        json.dumps(metadata, indent=2) + "\n"
    )

    print("\n===== BASELINE RESULTS =====")

    print(
        f"Gazebo RTF samples: {len(rtf_samples)}"
    )

    print(
        f"Resource samples:   {len(resource_samples)}"
    )

    if rtf_samples:
        values = [
            row["rtf"]
            for row in rtf_samples
        ]

        print(
            f"Mean RTF:           "
            f"{statistics.mean(values):.6f}"
        )

        print(
            f"Median RTF:         "
            f"{statistics.median(values):.6f}"
        )

        print(
            f"Minimum RTF:        "
            f"{min(values):.6f}"
        )

    print(f"Saved in: {output_dir}")

    if (
        error
        or code not in (0, 124)
        or not rtf_samples
        or not resource_samples
    ):
        raise SystemExit(
            f"Incomplete run: "
            f"resource_error={error}, "
            f"Gazebo exit={code}. "
            "Inspect metadata.json and "
            "gazebo_stderr.log."
        )


if __name__ == "__main__":
    main()