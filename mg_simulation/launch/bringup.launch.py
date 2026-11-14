#!/usr/bin/env python3

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from mg_utils.launch_argument import LaunchArgumentCreator
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# センサ故障の注入で、ブリッジの出力を付け替えるトピック (fault_injector.py が中継する)。
# /tf は、オドメトリの誤差が真の odom->base で上書きされないよう、中継せずに捨てる
FAULT_TOPICS = ("/odom", "/scan_top_lidar", "/gps/fix", "/tf")


def bridge_actions(context, bridge_config):
    """ブリッジを作る。sensor_fault_file があれば、センサを /fault/ 以下へ出して注入ノードを起動する。"""
    fault_file = LaunchConfiguration("sensor_fault_file").perform(context)
    if not fault_file:
        return [Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
            output="screen",
        )]
    with open(bridge_config, encoding="utf-8") as f:
        entries = yaml.safe_load(f)
    for entry in entries:
        if entry["ros_topic_name"] in FAULT_TOPICS:
            entry["ros_topic_name"] = "/fault" + entry["ros_topic_name"]
    with tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        yaml.safe_dump(entries, f)
        fault_bridge_config = f.name
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["--ros-args", "-p", f"config_file:={fault_bridge_config}"],
            output="screen",
        ),
        ExecuteProcess(
            # シナリオがロボットの位置に合わせて故障を入れ切りできるよう、trigger を許す
            cmd=["python3", "/app/tools/scripts/fault_injector.py", fault_file, "--odom-out", "/odom",
                 "--allow-triggers"],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_sim = get_package_share_directory("mg_simulation")

    # default_world = os.path.join(pkg_sim, "worlds", "solar_farm.sdf")
    default_world = os.path.join(pkg_sim, "worlds", "warehouse.sdf")

    models_path = os.path.join(pkg_sim, "models")
    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path)

    arg = LaunchArgumentCreator()
    arg.create("world", default=default_world)
    arg.create("robot_name", default="mg")
    arg.create("spawn_x", default="0.0")
    arg.create("spawn_y", default="0.0")
    arg.create("spawn_z", default="0.05")
    arg.create("spawn_yaw", default="0.0")
    arg.create("headless", default="false")
    # センサ故障の定義 (tools/common/faults.py の書式)。空なら注入しない
    arg.create("sensor_fault_file", default="")
    arg.create(
        "publish_gazebo_tf",
        default=EnvironmentVariable(
            "PUBLISH_GAZEBO_TF", default_value="false"),
    )

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("mg_description"),
            "urdf",
            "mg_description.urdf.xacro",
        ]),
        " simulation:=true",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_gazebo_tf")),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [
            "-s -r ", LaunchConfiguration("world")]}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [
            "-r ", LaunchConfiguration("world")]}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("robot_name"),
            "-string", robot_description,
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    bridge_config = os.path.join(pkg_sim, "config", "bridge.yaml")
    bridge = OpaqueFunction(function=bridge_actions, args=[bridge_config])

    return LaunchDescription([
        set_env,
        *arg.get_created_declare_launch_args(),
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        spawn_robot,
        bridge,
    ])
