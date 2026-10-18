"""判定で共通に使う計算。"""
import math
from typing import Optional, Sequence

import numpy as np

from ..geometry import Pose2
from ..grid import scan_points_in_map
from ..interfaces import SnapshotLike
from ..snapshot import ReferenceData


def position_std(cov: np.ndarray) -> float:
    """位置の標準偏差 (x, y の分散の大きいほう)。"""
    return math.sqrt(max(float(cov[0, 0]), float(cov[1, 1]), 0.0))


def covariance_metric(metric: str, cov: np.ndarray) -> float:
    """x, y の 2x2 の共分散の大きさ。"""
    xy = np.asarray(cov, dtype=float)[:2, :2]
    if metric == 'trace_xy':
        return float(xy[0, 0] + xy[1, 1])
    if metric == 'determinant_xy':
        return float(np.linalg.det(xy))
    if metric == 'max_eigenvalue_xy':
        return float(np.max(np.linalg.eigvalsh(0.5 * (xy + xy.T))))
    raise ValueError(f'unknown covariance metric: {metric}')


COVARIANCE_METRICS: Sequence[str] = ('trace_xy', 'determinant_xy', 'max_eigenvalue_xy')


def scan_points(
    snapshot: SnapshotLike, reference_name: str, reference: ReferenceData, pose: Pose2,
    max_points: int, max_range: float,
) -> Optional[np.ndarray]:
    """reference のスキャンを、pose (map->base) で地図の座標の点 (N, 2) にする。同じ周期では使い回す。"""
    scan = reference.scan
    if scan is None or scan.sensor_pose is None:
        return None
    key = ('scan_points', reference_name, scan.stamp, tuple(round(v, 6) for v in pose), max_points, max_range)
    return snapshot.cached(key, lambda: scan_points_in_map(
        scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
        min(scan.range_max, max_range), pose, scan.sensor_pose, max_points))
