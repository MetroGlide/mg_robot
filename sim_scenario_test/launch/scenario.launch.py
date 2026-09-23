#!/usr/bin/env python3
"""起動済みのシミュレータ・ナビゲーションに対してシナリオを 1 本実行する (attach モード)。

シナリオ終了 (scenario_runner の終了) で launch 全体を終了する。
ロボット非依存のコアパッケージのため mg_utils は使わず標準の launch API で引数を定義する。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declares = [
        DeclareLaunchArgument("scenario_file", description="Path to scenario YAML"),
        DeclareLaunchArgument(
            "profile", default_value="", description="Profile name or path (overrides scenario)"),
        DeclareLaunchArgument(
            "result_file", default_value="", description="Path to write result JSON"),
    ]
    runner = Node(
        package="sim_scenario_test",
        executable="scenario_runner.py",
        parameters=[{
            "use_sim_time": True,
            "scenario_file": LaunchConfiguration("scenario_file"),
            "profile": LaunchConfiguration("profile"),
            "result_file": LaunchConfiguration("result_file"),
        }],
        output="screen",
    )
    return LaunchDescription([
        *declares,
        runner,
        RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown())])),
    ])
