"""復旧の候補を作る・評価するときに共通の計算。"""
import math
from typing import Optional, Tuple

import numpy as np

from ..geometry import Pose2, compose, inverse
from ..grid import scan_points_in_map
from ..interfaces import RecoveryContext


def predict_from_anchor(context: RecoveryContext, motion: str) -> Optional[Tuple[Pose2, np.ndarray, float]]:
    """疑いが始まる前の、最も新しいアンカーに、その後の動きを足した今の姿勢と共分散、動いた距離 [m]。

    共分散は、アンカーの時点のものだけ。動きの誤差の増え方は、呼び出し側が足す。アンカーがなければ None。
    """
    if context.anchors is None:
        return None
    snapshot = context.snapshot
    anchor = (context.anchors.latest_before(context.suspect_since) if context.suspect_since is not None
              else context.anchors.latest())
    now = snapshot.motion_at(motion, snapshot.stamp)
    if anchor is None or now is None:
        return None
    relative = compose(inverse(anchor.motion), now)
    pose = compose(anchor.pose.pose, relative)
    return pose, np.array(anchor.pose.cov, dtype=float), math.hypot(relative[0], relative[1])


def grow_covariance(
    cov: np.ndarray, distance: float, rotation: float, std_per_m: float, std_per_rad: float,
    min_std_m: float, min_std_yaw: float,
) -> np.ndarray:
    """動いた距離・回転に比例して、位置と向きの分散を増やす。下限の標準偏差は min_*。"""
    grown = np.array(cov, dtype=float, copy=True)
    grown[0, 0] = max(grown[0, 0] + (std_per_m * distance) ** 2, min_std_m ** 2)
    grown[1, 1] = max(grown[1, 1] + (std_per_m * distance) ** 2, min_std_m ** 2)
    grown[2, 2] = max(grown[2, 2] + (std_per_rad * rotation) ** 2, min_std_yaw ** 2)
    return grown


def thin(points: np.ndarray, max_points: int) -> np.ndarray:
    if points.shape[0] <= max_points:
        return points
    return points[np.linspace(0, points.shape[0] - 1, max_points).astype(int)]


def base_points(
    context: RecoveryContext, reference, motion: str, min_distance_m: float, max_points: int,
) -> Optional[np.ndarray]:
    """今の base の座標のスキャンの点 (N, 2)。

    スキャンのバッファの走行距離が min_distance_m に届いていれば、動きでまとめた直近のスキャン
    (1 枚より広い範囲。似た場所の取り違えに強い)。届かなければ最新の 1 枚。どちらもなければ None。
    """
    snapshot = context.snapshot
    scans = context.scans
    if scans is not None and scans.travelled() >= min_distance_m:
        now = snapshot.motion_at(motion, snapshot.stamp)
        if now is not None:
            return thin(scans.points_in((0.0, 0.0, 0.0), now), max_points)
    scan = reference.scan
    if scan is None or scan.sensor_pose is None:
        return None
    return scan_points_in_map(
        scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
        (0.0, 0.0, 0.0), scan.sensor_pose, max_points)
