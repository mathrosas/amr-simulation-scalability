#!/usr/bin/env python3

"""Aggregate navigation resource measurements across trials."""

import argparse
import json
import statistics
from pathlib import Path


SERVICES = [
    "simulation",
    "navigation",
    "localization",
]

CONFIGURATION_KEYS = [
    "robot_count",
    "namespace",
    "world",
    "gazebo_gui_enabled",
    "rviz_enabled",
    "resource_interval_s",
    "goal",
    "configuration_hashes",
]


def load_json(path):
    return json.loads(path.read_text())


def distribution(values):
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


def image_ids(metadata):
    return {
        name: information["image_id"]
        for name, information
        in metadata["environment"].items()
    }


def verify_configuration(metadata_records):
    reference = metadata_records[0]
    reference_images = image_ids(reference)

    for index, metadata in enumerate(metadata_records):
        trial = index + 1

        for key in CONFIGURATION_KEYS:
            if metadata[key] != reference[key]:
                raise ValueError(
                    f"Trial {trial}: "
                    f"configuration mismatch in {key}"
                )

        if image_ids(metadata) != reference_images:
            raise ValueError(
                f"Trial {trial}: Docker image mismatch"
            )

        if metadata["errors"]:
            raise ValueError(
                f"Trial {trial}: runner errors recorded"
            )

        if metadata["probe_exit_code"] != 0:
            raise ValueError(
                f"Trial {trial}: probe execution failed"
            )

    restart_verified = {}

    for service in SERVICES:
        name = f"amr-scalability-{service}"

        timestamps = [
            metadata["environment"][name]["started_at"]
            for metadata in metadata_records
        ]

        restart_verified[service] = (
            len(set(timestamps))
            == len(metadata_records)
        )

        if not restart_verified[service]:
            raise ValueError(
                f"Container restart not verified: {service}"
            )

    return {
        "configuration_hashes_match": True,
        "image_ids_match": True,
        "configuration_values_match": True,
        "container_restarts_verified": restart_verified,
        "image_ids": reference_images,
        "configuration_hashes": reference[
            "configuration_hashes"
        ],
    }


def aggregate_measurements(records):
    means = [
        record["mean"]
        for record in records
    ]

    return {
        "trial_mean_statistics": distribution(means),
        "total_samples": sum(
            record["count"]
            for record in records
        ),
        "minimum_observed_sample": min(
            record["min"]
            for record in records
        ),
        "maximum_observed_sample": max(
            record["max"]
            for record in records
        ),
        "trial_means": means,
    }


def aggregate_resources(run_dirs):
    metadata_records = []
    resource_records = []
    run_records = []

    for run_dir in run_dirs:
        metadata = load_json(
            run_dir / "metadata.json"
        )

        resources = load_json(
            run_dir / "resource_summary.json"
        )

        if (
            resources["resource_sample_count"] <= 0
            or resources["rtf_sample_count"] <= 0
        ):
            raise ValueError(
                f"Incomplete resource dataset: {run_dir}"
            )

        metadata_records.append(metadata)
        resource_records.append(resources)

        measurements = resources["resources"]

        combined_cpu = sum(
            measurements[f"{service}_cpu_pct"]["mean"]
            for service in SERVICES
        )

        combined_ram = sum(
            measurements[f"{service}_ram_mib"]["mean"]
            for service in SERVICES
        )

        run_records.append({
            "trial": run_dir.name,
            "directory": str(run_dir),
            "resource_sample_count": resources[
                "resource_sample_count"
            ],
            "rtf_sample_count": resources[
                "rtf_sample_count"
            ],
            "mean_gazebo_rtf": resources[
                "gazebo_rtf"
            ]["mean"],
            "combined_mean_cpu_pct": combined_cpu,
            "combined_mean_ram_mib": combined_ram,
            "mean_gpu_pct": measurements[
                "gpu_pct"
            ]["mean"],
            "mean_vram_mib": measurements[
                "vram_mib"
            ]["mean"],
        })

    verification = verify_configuration(
        metadata_records
    )

    resource_fields = set(
        resource_records[0]["resources"]
    )

    for record in resource_records:
        if set(record["resources"]) != resource_fields:
            raise ValueError(
                "Resource metric mismatch between trials"
            )

    aggregated = {}

    for field in sorted(resource_fields):
        records = [
            record["resources"][field]
            for record in resource_records
        ]

        aggregated[field] = (
            aggregate_measurements(records)
            if all(
                record is not None
                for record in records
            )
            else None
        )

    return {
        "experiment": "single_robot_navigation_repeat",
        "experiment_count": len(run_dirs),
        "configuration_verification": verification,

        "total_resource_samples": sum(
            record["resource_sample_count"]
            for record in run_records
        ),

        "total_rtf_samples": sum(
            record["rtf_sample_count"]
            for record in run_records
        ),

        "gazebo_rtf": aggregate_measurements([
            record["gazebo_rtf"]
            for record in resource_records
        ]),

        "resources": aggregated,

        "combined_cpu_pct": distribution([
            record["combined_mean_cpu_pct"]
            for record in run_records
        ]),

        "combined_ram_mib": distribution([
            record["combined_mean_ram_mib"]
            for record in run_records
        ]),

        "runs": run_records,

        "limitations": [
            "Each trial receives equal statistical weight.",
            "Standard deviations describe between-trial "
            "variability in trial means.",
            "Observed sample extrema are not continuous peaks.",
            "GPU measurements are system-wide.",
            "Navigation CPU includes the navigation probe.",
            "Resource windows are aligned approximately "
            "using navigation telemetry timestamps.",
            "Configuration verification covers recorded "
            "file hashes, image IDs and metadata.",
            "Container restart verification uses Docker "
            "StartedAt timestamps.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "run_dirs",
        nargs="+",
        type=Path,
    )

    parser.add_argument(
        "--output",
        required=True,
        type=Path,
    )

    args = parser.parse_args()

    resolved = [
        path.resolve()
        for path in args.run_dirs
    ]

    if len(set(resolved)) != len(resolved):
        parser.error("Duplicate trial directories")

    result = aggregate_resources(args.run_dirs)

    output = json.dumps(
        result,
        indent=2,
        allow_nan=False,
    )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.output.write_text(output + "\n")

    print(output)


if __name__ == "__main__":
    main()