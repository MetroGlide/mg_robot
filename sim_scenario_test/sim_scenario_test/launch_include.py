"""プロファイルの launch 定義 (sim.launch / stack) を include するための launch 用ヘルパー。"""
from __future__ import annotations

import os
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from sim_scenario_test.profile import LaunchSpec
from sim_scenario_test.template import expand_all


def include_launch(
    spec: LaunchSpec,
    variables: Dict[str, object],
    where: str,
    overrides: Optional[Dict[str, str]] = None,
) -> IncludeLaunchDescription:
    """spec の引数を変数展開して include する。overrides は spec.args を上書きする。"""
    path = os.path.join(get_package_share_directory(spec.package), spec.file)
    args = expand_all({**spec.args, **(overrides or {})}, variables, where)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=args.items())
