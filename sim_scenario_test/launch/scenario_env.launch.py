#!/usr/bin/env python3
"""プロファイルのシミュレータとナビゲーションスタックだけを起動する (attach モード用の環境)。

scenario_runner は起動しない。起動したままにして、`scenario_cli.py run --attach` で
シナリオを繰り返し実行する (シナリオごとにシミュレータを起動し直さない)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from sim_scenario_test.launch_include import include_launch
from sim_scenario_test.profile import load_profile
from sim_scenario_test.registry import DEFAULT_REGISTRY


def _setup(context, *args, **kwargs):
    profile = load_profile(LaunchConfiguration("profile").perform(context), DEFAULT_REGISTRY)
    variables = profile.launch_variables(
        LaunchConfiguration("world").perform(context),
        LaunchConfiguration("headless").perform(context),
        "world")
    actions = []
    if profile.sim.launch is not None:
        actions.append(include_launch(profile.sim.launch, variables, "profile.sim.launch.args"))
    if profile.stack is not None:
        actions.append(include_launch(profile.stack, variables, "profile.stack.args"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", description="Profile name or path"),
        DeclareLaunchArgument("world", description="World name defined in the profile"),
        DeclareLaunchArgument("headless", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
