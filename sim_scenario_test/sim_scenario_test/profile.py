"""ロボットプロファイル: ロボット固有の設定をシナリオ本体から分離して注入する。

プロファイルは名前またはファイルパスで指定する。名前の場合は ament リソース
`sim_scenario_test.profiles` に登録したパッケージの share/<pkg>/profiles/<name>.yaml を探す。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml
from ament_index_python import get_resources

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.geometry import Pose2DSpec
from sim_scenario_test.registry import Registry
from sim_scenario_test.spec import parse_spec

PROFILE_RESOURCE_TYPE = "sim_scenario_test.profiles"


@dataclass
class LaunchSpec:
    """起動する launch ファイル。file はパッケージの share ディレクトリからの相対パス。"""
    package: str
    file: str
    args: Dict[str, str] = field(default_factory=dict)


@dataclass
class SimSpec:
    backend: str = "gazebo_fortress"
    robot_entity: str = "robot"
    robot_spawn_z: float = 0.0
    # 走行中に sim 時間がこの壁時計秒数進まなければシミュレータ停止として ERROR にする
    clock_stall_sec: float = 30.0
    launch: Optional[LaunchSpec] = None


@dataclass
class FramesSpec:
    map: str = "map"
    base: str = "base_footprint"
    # シミュレータのワールド座標系における map 原点の姿勢
    map_in_world: Pose2DSpec = field(default_factory=Pose2DSpec)


@dataclass
class Nav2Spec:
    navigate_to_pose_action: str = "navigate_to_pose"
    initialpose_topic: str = "/initialpose"
    initialpose_cov_xy: float = 0.25
    initialpose_cov_yaw: float = 0.06853891945200942
    clear_costmap_services: List[str] = field(default_factory=lambda: [
        "/global_costmap/clear_entirely_global_costmap",
        "/local_costmap/clear_entirely_local_costmap",
    ])


@dataclass
class ReadinessSpec:
    timeout_sec: float = 120.0
    lifecycle_managers: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)


@dataclass
class Profile:
    name: str
    plugins: List[str] = field(default_factory=list)
    sim: SimSpec = field(default_factory=SimSpec)
    stack: Optional[LaunchSpec] = None
    frames: FramesSpec = field(default_factory=FramesSpec)
    nav2: Nav2Spec = field(default_factory=Nav2Spec)
    readiness: ReadinessSpec = field(default_factory=ReadinessSpec)
    # world 名 -> 変数 (sim_world, sdf, map, waypoints など任意)
    worlds: Dict[str, Dict[str, str]] = field(default_factory=dict)

    def world_vars(self, world: str, where: str) -> Dict[str, str]:
        if world not in self.worlds:
            raise ScenarioValidationError(
                f"{where}: world '{world}' is not defined in profile '{self.name}' "
                f"(available: {sorted(self.worlds)})")
        values = dict(self.worlds[world])
        values.setdefault("name", world)
        values.setdefault("sim_world", world)
        return values


def find_profile_path(name_or_path: str) -> str:
    if os.path.isfile(name_or_path):
        return name_or_path
    candidates = []
    for package, share_prefix in get_resources(PROFILE_RESOURCE_TYPE).items():
        path = os.path.join(
            share_prefix, "share", package, "profiles", f"{name_or_path}.yaml")
        candidates.append(path)
        if os.path.isfile(path):
            return path
    raise ScenarioValidationError(
        f"profile '{name_or_path}' not found (searched: {candidates or 'no registered packages'})")


def load_profile(name_or_path: str, registry: Registry) -> Profile:
    path = find_profile_path(name_or_path)
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    return parse_spec(Profile, raw, f"profile({path})", registry)
