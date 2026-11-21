#!/usr/bin/env python3

"""Launch independent localization stacks for a Clearpath fleet."""

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
    LogInfo,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration


def launch_setup(context):

    fleet_path = Path(
        LaunchConfiguration(
            "fleet_config"
        ).perform(context)
    ).resolve()

    with fleet_path.open() as file:
        fleet = yaml.safe_load(file)

    world = fleet["world"]
    robots = fleet["robots"]

    if not robots:
        raise ValueError("Fleet is empty")

    nav2_demos = Path(
        get_package_share_directory(
            "clearpath_nav2_demos"
        )
    )

    map_path = (
        nav2_demos / "maps" / f"{world}.yaml"
    )

    if not map_path.is_file():
        raise FileNotFoundError(map_path)

    actions = [
        LogInfo(
            msg=f"Fleet localization map: {map_path}"
        )
    ]

    namespaces = set()
    setup_paths = set()

    for robot in robots:

        namespace = robot["namespace"]

        if namespace in namespaces:
            raise ValueError(
                f"Duplicate namespace: {namespace}"
            )

        namespaces.add(namespace)

        setup_path = (
            fleet_path.parent
            / robot["setup_path"]
        ).resolve()

        if setup_path in setup_paths:
            raise ValueError(
                f"Duplicate setup path: {setup_path}"
            )

        setup_paths.add(setup_path)

        robot_yaml = setup_path / "robot.yaml"

        if not robot_yaml.is_file():
            raise FileNotFoundError(robot_yaml)

        config = ClearpathConfig(
            str(robot_yaml)
        )

        if config.system.namespace != namespace:
            raise ValueError(
                f"Namespace mismatch: {namespace}"
            )

        actions.append(
            LogInfo(
                msg=(
                    f"Starting localization for "
                    f"{namespace}"
                )
            )
        )

        actions.append(
            ExecuteProcess(
                name=f"localization_{namespace}",
                cmd=[
                    "ros2",
                    "launch",
                    "clearpath_nav2_demos",
                    "localization.launch.py",
                    f"setup_path:={setup_path}",
                    f"map:={map_path}",
                    "use_sim_time:=true",
                    "autostart:=true",
                    "use_composition:=false",
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
            description="Fleet configuration file",
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])