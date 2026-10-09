#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    device_name_arg = launch_argument_creator.create(
        "device_name", default="/dev/ttyRobot-motordriver")
    # true のとき、プロセスが終了したら起動し直す
    respawn_drivers_arg = launch_argument_creator.create(
        "respawn_drivers", default="true")

    pkg_name = "mg_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch action group with ifconditions
    hardware_group = launch.actions.GroupAction(
        [
            Node(
                package=pkg_name,
                executable="motor_driver_node",
                name="motor_driver_node",
                output="screen",
                respawn=respawn_drivers_arg.launch_config,
                respawn_delay=2.0,
                remappings=[
                    ("cmd_vel", "cmd_vel"),
                ],
                parameters=[{
                    "motor_driver.device_name": device_name_arg.launch_config,
                    "motor_driver.wheel_pitch": 0.358,  # m
                    "motor_driver.max_speed": 1.0,  # m/s
                }],
            ),
        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            hardware_group,
        ]
    )
