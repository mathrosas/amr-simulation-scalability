#!/usr/bin/env python3

"""Launch a configurable fleet of Clearpath robots in one Gazebo world."""

import math
import re
import subprocess

from pathlib import Path

import yaml

from ament_index_python.packages import (
    get_package_share_directory,
)

from clearpath_config.clearpath_config import (
    ClearpathConfig,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration


GENERATORS = [
    (
        "clearpath_generator_common",
        "generate_description",
    ),
    (
        "clearpath_generator_common",
        "generate_semantic_description",
    ),
    (
        "clearpath_generator_gz",
        "generate_launch",
    ),
    (
        "clearpath_generator_gz",
        "generate_param",
    ),
]


REQUIRED_GENERATED_FILES = [
    "robot.urdf.xacro",
    "platform/config/control.yaml",
    "platform/launch/platform-service.launch.py",
    "sensors/launch/sensors-service.launch.py",
]


def load_fleet(config_path):
    config_path = Path(config_path).resolve()

    with config_path.open() as file:
        fleet = yaml.safe_load(file)

    world = fleet["world"]
    robots = fleet["robots"]

    if not isinstance(world, str) or not world:
        raise ValueError("Invalid world name")

    if not isinstance(robots, list) or not robots:
        raise ValueError(
            "Fleet must contain at least one robot"
        )

    namespaces = set()
    serial_numbers = set()
    setup_paths = set()
    prepared_robots = []

    for robot in robots:
        namespace = robot["namespace"]

        if not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*",
            namespace,
        ):
            raise ValueError(
                f"Invalid namespace: {namespace}"
            )

        if namespace in namespaces:
            raise ValueError(
                f"Duplicate namespace: {namespace}"
            )

        namespaces.add(namespace)

        setup_path = (
            config_path.parent / robot["setup_path"]
        ).resolve()

        if setup_path in setup_paths:
            raise ValueError(
                f"Duplicate setup path: {setup_path}"
            )

        setup_paths.add(setup_path)

        robot_yaml = setup_path / "robot.yaml"

        if not robot_yaml.is_file():
            raise FileNotFoundError(robot_yaml)

        with robot_yaml.open() as file:
            robot_data = yaml.safe_load(file)

        serial_number = robot_data["serial_number"]

        if serial_number in serial_numbers:
            raise ValueError(
                f"Duplicate serial number: {serial_number}"
            )

        serial_numbers.add(serial_number)

        clearpath_config = ClearpathConfig(
            str(robot_yaml)
        )

        actual_namespace = (
            clearpath_config.system.namespace
        )

        if actual_namespace != namespace:
            raise ValueError(
                f"Namespace mismatch: "
                f"{namespace} != {actual_namespace}"
            )

        spawn = {
            key: float(robot["spawn"][key])
            for key in ("x", "y", "z", "yaw")
        }

        if not all(
            math.isfinite(value)
            for value in spawn.values()
        ):
            raise ValueError(
                f"Non-finite spawn pose: {namespace}"
            )

        if spawn["z"] < 0:
            raise ValueError(
                f"Negative spawn height: {namespace}"
            )

        prepared_robots.append({
            "namespace": namespace,
            "serial_number": serial_number,
            "setup_path": setup_path,
            "spawn": spawn,
        })

    return world, prepared_robots


def generate_robot_configuration(robot):
    namespace = robot["namespace"]
    setup_path = robot["setup_path"]

    print(
        f"\n===== GENERATING {namespace} =====",
        flush=True,
    )

    for package, executable in GENERATORS:
        print(
            f"{namespace}: {executable}",
            flush=True,
        )

        subprocess.run(
            [
                "ros2",
                "run",
                package,
                executable,
                "-s",
                str(setup_path),
            ],
            check=True,
            timeout=120,
        )

    for filename in REQUIRED_GENERATED_FILES:
        path = setup_path / filename

        if not path.is_file():
            raise FileNotFoundError(
                f"Generated file missing: {path}"
            )

    print(
        f"{namespace}: configuration generation passed",
        flush=True,
    )


def launch_setup(context):
    fleet_path = LaunchConfiguration(
        "fleet_config"
    ).perform(context)

    headless = LaunchConfiguration(
        "headless"
    ).perform(context)

    prepare_only = LaunchConfiguration(
        "prepare_only"
    ).perform(context)

    world, robots = load_fleet(fleet_path)

    print(
        f"\nFleet size: {len(robots)}",
        flush=True,
    )

    print(
        f"World: {world}",
        flush=True,
    )

    for robot in robots:
        generate_robot_configuration(robot)

    if prepare_only == "true":
        return [
            LogInfo(
                msg="Fleet configuration preparation completed"
            )
        ]

    clearpath_gz = Path(
        get_package_share_directory("clearpath_gz")
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                clearpath_gz
                / "launch"
                / "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "world": world,
            "headless": headless,
            "auto_start": "true",
            "use_sim_time": "true",
            "setup_path": str(
                robots[0]["setup_path"]
            ),
        }.items(),
    )

    actions = [
        LogInfo(
            msg=f"Starting one Gazebo world: {world}"
        ),
        gazebo_launch,
    ]

    for robot in robots:
        namespace = robot["namespace"]
        setup_path = robot["setup_path"]
        spawn = robot["spawn"]

        actions.append(
            LogInfo(
                msg=(
                    f"Spawning {namespace} at "
                    f"x={spawn['x']}, "
                    f"y={spawn['y']}, "
                    f"z={spawn['z']}, "
                    f"yaw={spawn['yaw']}"
                )
            )
        )

        actions.append(
            ExecuteProcess(
                name=f"spawn_{namespace}",
                cmd=[
                    "ros2",
                    "launch",
                    "clearpath_gz",
                    "robot_spawn.launch.py",
                    f"setup_path:={setup_path}",
                    f"world:={world}",
                    f"x:={spawn['x']}",
                    f"y:={spawn['y']}",
                    f"z:={spawn['z']}",
                    f"yaw:={spawn['yaw']}",
                    "generate:=false",
                    "use_sim_time:=true",
                    "rviz:=false",
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "fleet_config",
            default_value="/etc/clearpath/fleet.yaml",
            description="Path to the fleet configuration",
        ),

        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["true", "false"],
            description="Run without the Gazebo GUI",
        ),

        DeclareLaunchArgument(
            "prepare_only",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Generate robot configurations "
                "without starting Gazebo"
            ),
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])