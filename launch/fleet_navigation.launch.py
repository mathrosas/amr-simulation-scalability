#!/usr/bin/env python3

"""Launch independent Nav2 stacks for a Clearpath fleet."""

import re

from pathlib import Path

import yaml

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

    robots = fleet["robots"]

    if not robots:
        raise ValueError("Fleet is empty")

    actions = []

    namespaces = set()
    setup_paths = set()

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
                msg=f"Starting Nav2 for {namespace}"
            )
        )

        actions.append(
            ExecuteProcess(
                name=f"navigation_{namespace}",
                cmd=[
                    "ros2",
                    "launch",
                    "clearpath_nav2_demos",
                    "nav2.launch.py",
                    f"setup_path:={setup_path}",
                    "use_sim_time:=true",
                    "slam:=false",
                    "autostart:=false",
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