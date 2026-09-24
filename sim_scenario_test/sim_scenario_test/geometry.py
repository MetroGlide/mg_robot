from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

Frame = Literal["map", "world", "robot"]


@dataclass(frozen=True)
class Pose:
    """解決済みの 2.5D 姿勢 (x, y, z, yaw)。どの座標系かは利用側が管理する。"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def compose(self, other: "Pose") -> "Pose":
        """self を基準とした相対姿勢 other を self の親座標系へ変換する。"""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose(
            x=self.x + other.x * c - other.y * s,
            y=self.y + other.x * s + other.y * c,
            z=self.z + other.z,
            yaw=normalize_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose":
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose(
            x=-(self.x * c + self.y * s),
            y=-(-self.x * s + self.y * c),
            z=-self.z,
            yaw=normalize_angle(-self.yaw),
        )

    def distance_xy(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class PoseSpec:
    """シナリオ上の姿勢指定。

    frame:
      map   : ナビゲーションの地図座標 (デフォルト)
      world : シミュレータのワールド座標
      robot : 現在のロボット姿勢を基準とした相対座標
    """
    frame: Frame = "map"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def as_pose(self) -> Pose:
        return Pose(self.x, self.y, self.z, self.yaw)


@dataclass
class Pose2DSpec:
    """座標系を持たない姿勢指定 (プロファイルの座標変換定義などに使う)。"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_pose(self) -> Pose:
        return Pose(self.x, self.y, 0.0, self.yaw)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float):
    """(x, y, z, w) を返す。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
