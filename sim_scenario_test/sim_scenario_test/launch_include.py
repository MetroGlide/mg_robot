"""プロファイルの launch 定義 (sim.launch / stack) を include するための launch 用ヘルパー。"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import sim_scenario_test.builtin  # noqa: F401  組み込みのバックエンドを登録する
from sim_scenario_test.profile import LaunchSpec, Profile
from sim_scenario_test.registry import DEFAULT_REGISTRY
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


def backend_launch_actions(profile: Profile, variables: Dict[str, Dict]) -> List[Node]:
    """シミュレータのバックエンドが一緒に起動するノード (真の姿勢の中継など)。"""
    backend_cls = DEFAULT_REGISTRY.get("backend", profile.sim.backend, "profile.sim").impl
    return [
        Node(
            package=spec.package, executable=spec.executable, name=spec.name,
            arguments=list(spec.arguments), remappings=list(spec.remappings), output="log")
        for spec in backend_cls.companion_nodes(variables["world"]["sim_world"])
    ]
