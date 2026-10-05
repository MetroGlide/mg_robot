from __future__ import annotations

from typing import List

import yaml

from sim_scenario_test.registry import DEFAULT_REGISTRY, register_waypoint_format
from sim_scenario_test.spec import parse_spec
from sim_scenario_test.waypoints import IndexedPose, PoseListFile


@register_waypoint_format("poses")
def load_pose_list(path: str) -> List[IndexedPose]:
    """`waypoints: [{x, y, yaw, frame}]` 形式の汎用 waypoint ファイル。"""
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    parsed = parse_spec(PoseListFile, raw, path, DEFAULT_REGISTRY)
    return [IndexedPose(i, pose) for i, pose in enumerate(parsed.waypoints)]
