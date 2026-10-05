#!/usr/bin/env python3
"""rosbag リプレイで障害物検出の結果を確認するための launch ファイル。

深度画像から点群を復元し、障害物検出ノードと RViz を同時に起動する。
bag の配信は別端末の `make rosbag-replay` (--clock 付き) で行う。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    pkg_name = "mg_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    use_color_arg = launch_argument_creator.create(
        "use_color",
        default="false",
        description="カラー付き点群を復元するか (RGB 画像は RViz の Image 表示で確認できる)",
    )
    param_file_arg = launch_argument_creator.create(
        "param_file",
        default=os.path.join(pkg_share, "params", "obstacle_detection.yaml"),
        description="obstacle_detection_3d_node のパラメータYAML",
    )
    rviz_arg = launch_argument_creator.create(
        "rviz", default="true", description="RViz を起動するか"
    )
    rviz_config_arg = launch_argument_creator.create(
        "rviz_config",
        default=os.path.join(pkg_share, "rviz", "obstacle_detection_replay.rviz"),
        description="RViz の設定ファイル",
    )

    # 障害物検出 launch の購読トピックに合わせて復元点群を出力する
    depth_to_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "depth_to_pointcloud.launch.py")
        ),
        launch_arguments={
            "simulation": "true",
            "use_color": use_color_arg.launch_config,
            "points_topic": "/rs_d435i/depth/color/points",
        }.items(),
    )

    obstacle_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "obstacle_detection_3d.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "param_file": param_file_arg.launch_config,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_arg.launch_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(rviz_arg.launch_config),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            depth_to_pointcloud,
            obstacle_detection,
            rviz_node,
        ]
    )
