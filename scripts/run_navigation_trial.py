#!/usr/bin/env python3

"""Run a controlled single-robot navigation experiment."""

import argparse
import csv
import hashlib
import json
import math
import statistics
import subprocess
import threading
import time
import uuid

from pathlib import Path

from collect_resources import (
    run_command,
    memory_to_mib,
    gpu_snapshot,
)

from run_baseline import (
    read_gazebo,
    save_csv,
    utc_now,
)

from analyze_navigation_probe import (
    read_ground_truth,
)


COMPOSE = [
    "docker", "compose",
    "-f", "compose.yaml",
    "-f", "compose.visualization.yaml",
    "-f", "compose.navigation.yaml",
]

CONTAINERS = {
    "simulation": "amr-scalability-simulation",
    "localization": "amr-scalability-localization",
    "navigation": "amr-scalability-navigation",
}

VISUALIZATION = "amr-scalability-visualization"

NAMESPACE = "j100_0000"

CONFIG_FILES = [
    "compose.yaml",
    "compose.visualization.yaml",
    "compose.navigation.yaml",
    "config/jackal/robot.yaml",
    "config/navigation/j100/nav2.yaml",
    "docker/nav2/Dockerfile",
    "scripts/navigation_probe.py",
    "scripts/run_navigation_trial.py",
]


def configuration_hashes():
    return {
        name: hashlib.sha256(
            Path(name).read_bytes()
        ).hexdigest()
        for name in CONFIG_FILES
    }


def inspect_environment():
    output = run_command([
        "docker", "inspect",
        *CONTAINERS.values(),
        VISUALIZATION,
    ])

    containers = json.loads(output)

    information = {}

    for container in containers:
        name = container["Name"].lstrip("/")

        information[name] = {
            "running": container["State"]["Running"],
            "image_id": container["Image"],
            "started_at": container["State"]["StartedAt"],
        }

    for name in CONTAINERS.values():
        if not information[name]["running"]:
            raise RuntimeError(
                f"Required container not running: {name}"
            )

    if information[VISUALIZATION]["running"]:
        raise RuntimeError(
            "RViz2 must be stopped for controlled trials"
        )

    return information


def capture_ground_truth(path):
    result = subprocess.run(
        COMPOSE + [
            "exec", "-T", "simulation",
            "bash", "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                'gz model -m "j100_0000/robot" --pose'
            ),
        ],
        capture_output=True,
        text=True,
        timeout=20,
    )

    path.write_text(
        result.stdout + result.stderr
    )

    if result.returncode != 0:
        raise RuntimeError(
            "Ground-truth collection failed"
        )

    return read_ground_truth(path)


def resource_snapshot():
    output = run_command([
        "docker", "stats",
        "--no-stream",
        "--format", "{{json .}}",
        *CONTAINERS.values(),
    ])

    records = {}

    for line in output.splitlines():
        item = json.loads(line)
        records[item["Name"]] = item

    measurements = {}

    for service, name in CONTAINERS.items():
        item = records[name]

        measurements[f"{service}_cpu_pct"] = float(
            item["CPUPerc"].rstrip("%")
        )

        measurements[f"{service}_ram_mib"] = (
            memory_to_mib(
                item["MemUsage"].split("/")[0].strip()
            )
        )

        measurements[f"{service}_pids"] = int(
            item["PIDs"]
        )

    measurements.update(gpu_snapshot())

    return measurements


def collect_resources(
    stop_event,
    interval,
    start,
    samples,
    errors,
):
    target = time.monotonic()

    while not stop_event.is_set():
        try:
            sample_start = time.monotonic()

            measurements = resource_snapshot()

            sample_end = time.monotonic()

            samples.append({
                "received_utc": utc_now(),

                "start_elapsed_s":
                    sample_start - start,

                "end_elapsed_s":
                    sample_end - start,

                **measurements,
            })

            print(
                f"Resources {len(samples):03d}: "
                f"SIM={measurements['simulation_cpu_pct']:.1f}% "
                f"NAV={measurements['navigation_cpu_pct']:.1f}% "
                f"LOC={measurements['localization_cpu_pct']:.1f}% "
                f"GPU={measurements['gpu_pct']}%",
                flush=True,
            )

        except Exception as exc:
            errors["resources"] = str(exc)
            return

        target += interval

        delay = max(
            0.0,
            target - time.monotonic(),
        )

        stop_event.wait(delay)


def run_probe(remote_dir, args, output_dir):
    command = COMPOSE + [
        "exec", "-T", "navigation",
        "bash", "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            'exec python3 - "$@"'
        ),
        "_",
        "--namespace", NAMESPACE,
        "--goal-x", str(args.goal_x),
        "--goal-y", str(args.goal_y),
        "--goal-yaw", str(args.goal_yaw),
        "--goal-timeout", "120.0",
        "--output-dir", remote_dir,
        "--execute",
    ]

    with (
        Path("scripts/navigation_probe.py").open()
        as source,

        (output_dir / "probe.log").open("w")
        as log,
    ):
        result = subprocess.run(
            command,
            stdin=source,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=260,
        )

    return result.returncode


def copy_probe_files(remote_dir, output_dir, errors):
    for filename in [
        "planned_path.csv",
        "samples.csv",
        "summary.json",
    ]:
        result = subprocess.run(
            COMPOSE + [
                "cp",
                f"navigation:{remote_dir}/{filename}",
                str(output_dir / filename),
            ],
            capture_output=True,
            text=True,
            timeout=20,
        )

        if result.returncode != 0:
            errors[f"copy_{filename}"] = result.stderr


def capture_navigation_log(output_dir, started_utc):
    result = subprocess.run(
        [
            "docker", "logs",
            "--since", started_utc,
            CONTAINERS["navigation"],
        ],
        capture_output=True,
        text=True,
        timeout=20,
    )

    (output_dir / "navigation.log").write_text(
        result.stdout + result.stderr
    )


def compute_navigation_window(output_dir, start):
    sample_file = output_dir / "samples.csv"

    if not sample_file.exists():
        return None

    with sample_file.open(newline="") as file:
        timestamps = [
            int(row["monotonic_ns"]) / 1_000_000_000
            for row in csv.DictReader(file)
            if row["phase"] == "navigation"
        ]

    if len(timestamps) < 2:
        return None

    return {
        "start_elapsed_s": timestamps[0] - start,
        "end_elapsed_s": timestamps[-1] - start,
        "duration_s": timestamps[-1] - timestamps[0],
    }


def distribution(values):
    values = [
        value for value in values
        if value is not None
        and math.isfinite(value)
    ]

    if not values:
        return None

    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "stdev": (
            statistics.stdev(values)
            if len(values) > 1
            else None
        ),
    }


def summarize_resources(
    rtf_samples,
    resource_samples,
    window,
):
    if window is None:
        return None

    start = window["start_elapsed_s"]
    end = window["end_elapsed_s"]

    selected_rtf = [
        row for row in rtf_samples
        if start <= row["received_elapsed_s"] <= end
    ]

    selected_resources = [
        row for row in resource_samples
        if row["start_elapsed_s"] >= start
        and row["end_elapsed_s"] <= end
    ]

    fields = [
        f"{service}_{metric}"
        for service in CONTAINERS
        for metric in [
            "cpu_pct",
            "ram_mib",
            "pids",
        ]
    ] + [
        "gpu_pct",
        "gpu_memory_pct",
        "vram_mib",
        "gpu_temp_c",
        "gpu_power_w",
    ]

    return {
        "navigation_window": window,

        "rtf_sample_count": len(selected_rtf),

        "resource_sample_count":
            len(selected_resources),

        "gazebo_rtf": distribution([
            row["rtf"]
            for row in selected_rtf
        ]),

        "resources": {
            field: distribution([
                row.get(field)
                for row in selected_resources
            ])
            for field in fields
        },
    }


def stop_navigation():
    print(
        "Stopping navigation as a precaution.",
        flush=True,
    )

    subprocess.run(
        COMPOSE + [
            "stop", "-t", "10", "navigation"
        ],
        check=True,
        timeout=30,
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=-1.2874,
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=-3.1625,
    )

    parser.add_argument(
        "--goal-yaw",
        type=float,
        default=0.0,
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
    )

    args = parser.parse_args()

    if not all(
        math.isfinite(value)
        for value in [
            args.goal_x,
            args.goal_y,
            args.goal_yaw,
            args.interval,
        ]
    ):
        parser.error("Arguments must be finite")

    if args.interval <= 0:
        parser.error("Interval must be positive")

    environment = inspect_environment()

    hashes = configuration_hashes()

    output_dir = args.output_dir

    output_dir.mkdir(
        parents=True,
        exist_ok=False,
    )

    initial_pose = capture_ground_truth(
        output_dir / "ground_truth_initial.txt"
    )

    if (
        math.hypot(
            initial_pose["x"],
            initial_pose["y"],
        ) > 0.02

        or abs(initial_pose["yaw_rad"]) > 0.02
    ):
        raise RuntimeError(
            "Robot is not at the required initial pose"
        )

    identifier = uuid.uuid4().hex

    remote_dir = f"/tmp/navigation_trial_{identifier}"

    stop_file = f"/tmp/rtf_stop_{identifier}"

    errors = {}

    rtf_samples = []
    resource_samples = []

    stop_event = threading.Event()

    started_utc = utc_now()
    start = time.monotonic()

    probe_exit_code = None
    interrupted = False

    rtf_command = COMPOSE + [
        "exec", "-T", "simulation",
        "bash", "-lc",
        """
source /opt/ros/jazzy/setup.bash

stop_file="$1"

timeout 300s gz topic -e -t /world/warehouse/stats &
pid=$!

cleanup() {
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    rm -f "$stop_file"
}

trap cleanup EXIT

while kill -0 "$pid" 2>/dev/null &&
      [ ! -e "$stop_file" ]; do
    sleep 0.2
done
""",
        "_",
        stop_file,
    ]

    with (
        output_dir / "gazebo_stderr.log"
    ).open("w") as gazebo_log:

        rtf_process = subprocess.Popen(
            rtf_command,
            stdout=subprocess.PIPE,
            stderr=gazebo_log,
            text=True,
            bufsize=1,
        )

        rtf_thread = threading.Thread(
            target=read_gazebo,
            args=(
                rtf_process.stdout,
                start,
                rtf_samples,
            ),
            daemon=True,
        )

        resource_thread = threading.Thread(
            target=collect_resources,
            args=(
                stop_event,
                args.interval,
                start,
                resource_samples,
                errors,
            ),
        )

        rtf_thread.start()
        resource_thread.start()

        try:
            print(
                "Starting instrumentation warm-up",
                flush=True,
            )

            time.sleep(3.0)

            if errors or not rtf_samples:
                raise RuntimeError(
                    "Instrumentation did not initialize"
                )

            print(
                "Executing navigation experiment",
                flush=True,
            )

            probe_exit_code = run_probe(
                remote_dir,
                args,
                output_dir,
            )

            if probe_exit_code != 0:
                errors["probe"] = (
                    f"Exit code {probe_exit_code}"
                )

                stop_navigation()

        except KeyboardInterrupt:
            interrupted = True

            errors["probe"] = "Interrupted by user"

            stop_navigation()

        except Exception as exc:
            errors["trial"] = str(exc)

            if probe_exit_code is None:
                try:
                    stop_navigation()

                except Exception as stop_error:
                    errors["emergency_stop"] = str(
                        stop_error
                    )

        finally:
            stop_event.set()

            try:
                subprocess.run(
                    COMPOSE + [
                        "exec", "-T", "simulation",
                        "touch", stop_file,
                    ],
                    check=True,
                    timeout=10,
                )

                rtf_exit_code = rtf_process.wait(
                    timeout=10
                )

                if rtf_exit_code != 0:
                    errors["rtf_exit"] = rtf_exit_code

            except Exception as exc:
                errors["rtf_stop"] = str(exc)

                rtf_process.kill()
                rtf_process.wait()

            resource_thread.join()
            rtf_thread.join(timeout=5)

    save_csv(
        output_dir / "rtf.csv",
        rtf_samples,
    )

    save_csv(
        output_dir / "resources.csv",
        resource_samples,
    )

    try:
        capture_ground_truth(
            output_dir / "ground_truth_final.txt"
        )

        copy_probe_files(
            remote_dir,
            output_dir,
            errors,
        )

        capture_navigation_log(
            output_dir,
            started_utc,
        )

    except Exception as exc:
        errors["collection"] = str(exc)

    window = compute_navigation_window(
        output_dir,
        start,
    )

    resource_summary = summarize_resources(
        rtf_samples,
        resource_samples,
        window,
    )

    if resource_summary is not None:
        (
            output_dir / "resource_summary.json"
        ).write_text(
            json.dumps(
                resource_summary,
                indent=2,
                allow_nan=False,
            ) + "\n"
        )

    metadata = {
        "experiment": "single_robot_navigation",
        "started_utc": started_utc,
        "finished_utc": utc_now(),
        "robot_count": 1,
        "namespace": NAMESPACE,
        "world": "warehouse",
        "gazebo_gui_enabled": True,
        "rviz_enabled": False,
        "resource_interval_s": args.interval,
        "goal": {
            "x": args.goal_x,
            "y": args.goal_y,
            "yaw_rad": args.goal_yaw,
        },
        "environment": environment,
        "configuration_hashes": hashes,
        "initial_ground_truth": initial_pose,
        "probe_exit_code": probe_exit_code,
        "rtf_sample_count": len(rtf_samples),
        "resource_sample_count": len(resource_samples),
        "navigation_window": window,
        "errors": errors,
    }

    (
        output_dir / "metadata.json"
    ).write_text(
        json.dumps(metadata, indent=2) + "\n"
    )

    print("\n===== NAVIGATION TRIAL COMPLETE =====")

    print(f"Probe exit code: {probe_exit_code}")
    print(f"RTF samples: {len(rtf_samples)}")
    print(f"Resource samples: {len(resource_samples)}")
    print(f"Errors: {errors}")
    print(f"Saved in: {output_dir}")

    if interrupted:
        return 130

    if (
        errors
        or probe_exit_code != 0
        or not rtf_samples
        or not resource_samples
        or resource_summary is None
        or resource_summary["resource_sample_count"] == 0
        or resource_summary["rtf_sample_count"] == 0
    ):
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())