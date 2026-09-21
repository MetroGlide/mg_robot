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

    bag_path = LaunchConfiguration('bag_path')
    start_time = LaunchConfiguration('start_time')
    end_time = LaunchConfiguration('end_time')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_param = LaunchConfiguration('rviz_param')
    params_file = LaunchConfiguration('params_file')

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value=EnvironmentVariable('ROSBAG_FILE', default_value=''),
        description='Path to ROS2 bag directory',
    )
    declare_start_time = DeclareLaunchArgument(
        'start_time',
        default_value='0.0',
        description='Start time in elapsed seconds from rosbag start (0.0: from beginning)',
    )
    declare_end_time = DeclareLaunchArgument(
        'end_time',
        default_value='0.0',
        description='End time in elapsed seconds from rosbag start (0.0: until end of bag)',
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

    offline_node = Node(
        package='slam_gnss_2d',
        executable='slam_offline_node',
        name='slam_gnss_2d_offline_node',
        output='screen',
        parameters=[
            params_file,
            {
                'bag_path': bag_path,
                'start_time': start_time,
                'end_time': end_time,
            },
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
        declare_bag_path,
        declare_start_time,
        declare_end_time,
        declare_rviz,
        declare_rviz_param,
        declare_params_file,
        offline_node,
        rviz_node,
    ])
