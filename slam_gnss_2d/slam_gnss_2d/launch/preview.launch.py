#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_graph_file = LaunchConfiguration('pose_graph_file')

    declare_pose_graph_file = DeclareLaunchArgument(
        'pose_graph_file',
        description='Path to the pose_graph.json to preview',
    )

    preview_node = Node(
        package='slam_gnss_2d',
        executable='pose_graph_preview_node',
        name='pose_graph_preview_node',
        output='screen',
        parameters=[
            {
                'pose_graph_file': pose_graph_file,
            }
        ],
    )

    return LaunchDescription([
        declare_pose_graph_file,
        preview_node,
    ])
