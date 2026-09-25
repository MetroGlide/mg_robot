#!/usr/bin/env python3
"""シミュレータ・ナビゲーションスタック・シナリオ実行ノードを 1 つの launch で起動する。

プロファイルの sim.launch / stack.launch を include し、scenario_runner の終了で全体を終了する。
launch_stack:=false のときは stack を起動しない (別のマシンで起動済みのスタックに接続する)。
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
from launch_ros.parameter_descriptions import ParameterValue

from sim_scenario_test.loader import load_scenario
from sim_scenario_test.template import expand_all


def _include(spec, variables, where, overrides=None):
    path = os.path.join(get_package_share_directory(spec.package), spec.file)
    args = expand_all({**spec.args, **(overrides or {})}, variables, where)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=args.items())


def _setup(context, *args, **kwargs):
    scenario_file = LaunchConfiguration("scenario_file").perform(context)
    profile_name = LaunchConfiguration("profile").perform(context)
    result_file = LaunchConfiguration("result_file").perform(context)
    scenario, profile = load_scenario(scenario_file, profile_name)

    variables = profile.launch_variables(
        scenario.world, LaunchConfiguration("headless").perform(context), "scenario.world")
    actions = []
    if profile.sim.launch is not None:
        actions.append(_include(profile.sim.launch, variables, "profile.sim.launch.args"))
    # 実機PCなど別の場所でスタックを起動する場合 (--remote-stack) は include しない
    launch_stack = LaunchConfiguration("launch_stack").perform(context) == "true"
    if profile.stack is not None and launch_stack:
        actions.append(_include(
            profile.stack, variables, "profile.stack.args", scenario.stack_args))

    runner = Node(
        package="sim_scenario_test",
        executable="scenario_runner.py",
        parameters=[{
            "use_sim_time": True,
            "scenario_file": scenario_file,
            "profile": profile_name,
            "result_file": result_file,
            "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
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
        DeclareLaunchArgument("launch_stack", default_value="true"),
        DeclareLaunchArgument("seed", default_value="-1"),
        OpaqueFunction(function=_setup),
    ])
