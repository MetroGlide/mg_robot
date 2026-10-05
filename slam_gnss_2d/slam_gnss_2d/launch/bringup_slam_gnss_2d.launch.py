#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_gnss_2d')

    simulation = LaunchConfiguration('simulation')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_param = LaunchConfiguration('rviz_param')
    params_file = LaunchConfiguration('params_file')

    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value=EnvironmentVariable('SIMULATION', default_value='false'),
        description='Use simulation clock if true',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value=EnvironmentVariable('USE_RVIZ', default_value='false'),
        description='Whether to launch RViz2',
    )
    declare_rviz_param = DeclareLaunchArgument(
        'rviz_param',
        default_value='slam_gnss_2d.rviz',
        description='RViz config file name',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'slam_gnss_2d.yaml'),
        description='Path to parameter YAML file',
    )

    slam_node = Node(
        package='slam_gnss_2d',
        executable='slam_node',
        name='slam_gnss_2d_node',
        output='screen',
        parameters=[
            {'use_sim_time': simulation},
            params_file,
        ],
    )

    rviz_config_file = PathJoinSubstitution([pkg_dir, 'rviz', rviz_param])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        declare_simulation,
        declare_rviz,
        declare_rviz_param,
        declare_params_file,
        slam_node,
        rviz_node,
    ])
