#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    slam_pkg_share = get_package_share_directory('slam_gnss_2d')
    desc_pkg_share = get_package_share_directory('mg_description')
    drivers_pkg_share = get_package_share_directory('mg_drivers')

    launch_argument_creator = LaunchArgumentCreator()

    bag_path_arg = launch_argument_creator.create(
        'bag_path',
        default=EnvironmentVariable('ROSBAG_FILE', default_value=''),
        description='Path to ROS2 bag directory',
    )
    start_time_arg = launch_argument_creator.create(
        'start_time',
        default='0.0',
        description='Start time in elapsed seconds from rosbag start (0.0: from beginning)',
    )
    end_time_arg = launch_argument_creator.create(
        'end_time',
        default='0.0',
        description='End time in elapsed seconds from rosbag start (0.0: until end of bag)',
    )
    skip_rendering_arg = launch_argument_creator.create(
        'skip_intermediate_rendering',
        default='true',
        description='Skip intermediate rendering/visualization and render once at the end '
                    '(set false to watch the map in RViz while processing)',
    )
    launch_rviz_arg = launch_argument_creator.create(
        'rviz',
        default=EnvironmentVariable('USE_RVIZ', default_value='false'),
        description='Whether to launch RViz2',
    )
    rviz_param_arg = launch_argument_creator.create(
        'rviz_param',
        default='slam_gnss_2d.rviz',
        description='RViz config file name',
    )
    params_file_arg = launch_argument_creator.create(
        'params_file',
        default=os.path.join(slam_pkg_share, 'params', 'slam_gnss_2d.yaml'),
        description='Path to parameter YAML file',
    )

    mg_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={'simulation': 'false'}.items(),
    )

    bringup_postprocess_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drivers_pkg_share, 'launch', 'bringup_postprocess.launch.py'
            )
        ),
        launch_arguments={
            'simulation': 'false',
            'use_odom': 'true',
            'use_odom_tf': 'true',
            'use_lidar': 'false',
            'use_gps': 'false',
            'use_realsense': 'false',
        }.items(),
    )

    offline_node = Node(
        package='slam_gnss_2d',
        executable='slam_offline_node',
        name='slam_gnss_2d_offline_node',
        output='screen',
        parameters=[
            params_file_arg.launch_config,
            {
                'bag_path': bag_path_arg.launch_config,
                'start_time': start_time_arg.launch_config,
                'end_time': end_time_arg.launch_config,
                'skip_intermediate_rendering': ParameterValue(
                    skip_rendering_arg.launch_config, value_type=bool),
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [slam_pkg_share, 'rviz', rviz_param_arg.launch_config]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(launch_rviz_arg.launch_config),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            mg_description_launch,
            bringup_postprocess_launch,
            offline_node,
            rviz_node,
        ]
    )
