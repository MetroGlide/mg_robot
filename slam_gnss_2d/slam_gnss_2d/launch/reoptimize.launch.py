#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_gnss_2d')

    input_dir = LaunchConfiguration('input_dir')
    bag_path = LaunchConfiguration('bag_path')
    params_file = LaunchConfiguration('params_file')
    save_dir = LaunchConfiguration('save_dir')

    declare_input_dir = DeclareLaunchArgument(
        'input_dir',
        description='Directory containing input pose_graph.json and gnss_transform.yaml',
    )
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Override path to the original ROS bag',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'slam_gnss_2d.yaml'),
        description='Path to the slam parameter YAML file',
    )
    declare_save_dir = DeclareLaunchArgument(
        'save_dir',
        default_value='',
        description='Directory to save the reoptimized map (can be same as input_dir)',
    )

    reoptimize_node = Node(
        package='slam_gnss_2d',
        executable='reoptimize_node',
        name='reoptimize_node',
        output='screen',
        parameters=[
            params_file,
            {
                'input_dir': input_dir,
                'bag_path': bag_path,
                'save_dir': save_dir,
            },
        ],
    )

    return LaunchDescription([
        declare_input_dir,
        declare_bag_path,
        declare_params_file,
        declare_save_dir,
        reoptimize_node,
    ])
