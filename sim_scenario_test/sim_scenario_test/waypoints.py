from __future__ import annotations

from dataclasses import dataclass
from typing import List

from sim_scenario_test.geometry import PoseSpec


@dataclass
class IndexedPose:
    """waypoint ファイルから読み込んだ姿勢。index はファイル上の番号。"""
    index: int
    pose: PoseSpec


@dataclass
class PoseListFile:
    """組み込み形式 `poses` のファイル構造: `waypoints: [{x, y, yaw, frame}]`。"""
    waypoints: List[PoseSpec]
