#!/usr/bin/env python3
"""シミュレータ・ナビゲーションスタック・シナリオ実行ノードを 1 つの launch で起動する。

プロファイルの sim.launch / stack.launch を include し、scenario_runner の終了で全体を終了する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_scenario_test.loader import load_scenario
from sim_scenario_test.template import expand_all


def _include(spec, variables, where):
    path = os.path.join(get_package_share_directory(spec.package), spec.file)
    args = expand_all(spec.args, variables, where)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=args.items())


def _setup(context, *args, **kwargs):
    scenario_file = LaunchConfiguration("scenario_file").perform(context)
    profile_name = LaunchConfiguration("profile").perform(context)
    result_file = LaunchConfiguration("result_file").perform(context)
    scenario, profile = load_scenario(scenario_file, profile_name)

    variables = {
        "world": profile.world_vars(scenario.world, "scenario.world"),
        "headless": LaunchConfiguration("headless").perform(context),
    }
    actions = []
    if profile.sim.launch is not None:
        actions.append(_include(profile.sim.launch, variables, "profile.sim.launch.args"))
    if profile.stack is not None:
        actions.append(_include(profile.stack, variables, "profile.stack.args"))

    runner = Node(
        package="sim_scenario_test",
        executable="scenario_runner.py",
        parameters=[{
            "use_sim_time": True,
            "scenario_file": scenario_file,
            "profile": profile_name,
            "result_file": result_file,
        }],
        output="screen",
    )
    actions += [
        runner,
        RegisterEventHandler(OnProcessExit(
            target_action=runner, on_exit=[EmitEvent(event=Shutdown())])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario_file", description="Path to scenario YAML"),
        DeclareLaunchArgument("profile", default_value=""),
        DeclareLaunchArgument("result_file", default_value=""),
        DeclareLaunchArgument("headless", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
