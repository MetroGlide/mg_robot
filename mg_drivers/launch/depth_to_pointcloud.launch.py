#!/usr/bin/env python3
"""デプス画像から点群（PointCloud2）を復元する launch ファイル。

rosbag 再生時に本 launch を起動することで、デプス画像とカメラ情報から
/camera/camera/depth/color/points をリアルタイム生成し、
障害物検知ノード (obstacle_detection_3d_node) や RViz に供給します。
"""

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    arg = LaunchArgumentCreator()

    simulation = arg.create(
        "simulation",
        default=EnvironmentVariable("SIMULATION"),
        description="シミュレーション時間 (use_sim_time) を使用するか",
    )
    use_color = arg.create(
        "use_color",
        default="true",
        description="カラー付き点群 (XYZRGB) を生成するか (false の場合は XYZ のみ)",
    )
    depth_image_topic = arg.create(
        "depth_image_topic",
        default="/camera/camera/depth/image_rect_raw",
        description="入力デプス画像トピック名",
    )
    depth_info_topic = arg.create(
        "depth_info_topic",
        default="/camera/camera/depth/camera_info",
        description="入力デプスカメラ情報トピック名",
    )
    rgb_image_topic = arg.create(
        "rgb_image_topic",
        default="/camera/camera/color/image_raw",
        description="入力カラー画像トピック名",
    )
    rgb_info_topic = arg.create(
        "rgb_info_topic",
        default="/camera/camera/color/camera_info",
        description="入力カラーカメラ情報トピック名",
    )
    points_topic = arg.create(
        "points_topic",
        default="/camera/camera/depth/color/points",
        description="出力点群トピック名",
    )
    depth_scale = arg.create(
        "depth_scale",
        default="0.001",
        description="デプス値のメートル変換スケール (16UC1 mm の場合は 0.001)",
    )
    stride = arg.create(
        "stride",
        default="1",
        description="画素サンプリング間隔 (1=全画素、2=半減)",
    )

    node = Node(
        package="mg_drivers",
        executable="depth_to_pointcloud_node.py",
        name="depth_to_pointcloud_node",
        output="screen",
        parameters=[{
            "use_sim_time": simulation.launch_config,
            "use_color": use_color.launch_config,
            "depth_scale": depth_scale.launch_config,
            "stride": stride.launch_config,
        }],
        remappings=[
            ("image_rect", depth_image_topic.launch_config),
            ("depth_camera_info", depth_info_topic.launch_config),
            ("image_color", rgb_image_topic.launch_config),
            ("color_camera_info", rgb_info_topic.launch_config),
            ("points", points_topic.launch_config),
        ],
    )

    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        node,
    ])
