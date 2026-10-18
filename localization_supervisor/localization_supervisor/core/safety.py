"""安全の信号: 指定した姿勢の footprint が、計画用地図の致死セル (占有) にかかっているか (ROS に依存しない)。

自己位置の判定とは別に計算する。自己位置が正常なのに致死セルにかかっていれば、走行側で脱出する。

  fraction : footprint の中の標本の点のうち、致死セルにある割合
  depth_m  : 致死セルにある点から、最も近い致死でないセルまでの距離の最大 (食い込みの深さ)
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from .geometry import Pose2
from .grid import OccupancyMap, transform_points


@dataclass
class SafetyConfig:
    pose: str = ''
    map: str = ''
    footprint: List[Tuple[float, float]] = field(default_factory=list)
    max_age_sec: float = 3.0
    # 食い込みの深さを測る範囲 (footprint の外接円からの距離) [m]。これより深いときは、この値になる
    depth_range_m: float = 2.0


@dataclass
class SafetyResult:
    in_lethal: bool
    fraction: float
    depth_m: float


def inside_polygon(points_xy: np.ndarray, polygon: Sequence[Tuple[float, float]]) -> np.ndarray:
    """点 (N, 2) が多角形の中にあるか (偶奇則)。"""
    x, y = points_xy[:, 0], points_xy[:, 1]
    inside = np.zeros(points_xy.shape[0], dtype=bool)
    count = len(polygon)
    for i in range(count):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % count]
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_at = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (x < x_at)
    return inside


def footprint_samples(polygon: Sequence[Tuple[float, float]], spacing: float) -> np.ndarray:
    """footprint の中と辺の上の、spacing 間隔の標本の点 (base の座標、(N, 2))。"""
    poly = np.asarray(polygon, dtype=float)
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    xs = np.arange(lo[0], hi[0] + 1e-9, spacing)
    ys = np.arange(lo[1], hi[1] + 1e-9, spacing)
    grid = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
    parts = [grid[inside_polygon(grid, polygon)]]
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        n = max(int(math.ceil(float(np.hypot(*(b - a))) / spacing)), 1)
        t = np.arange(n)[:, None] / n
        parts.append(a + (b - a) * t)
    return np.concatenate(parts, axis=0)


class FootprintChecker:
    """footprint の標本の点を、地図の解像度ごとに作り直して使い回す。"""

    def __init__(self, config: SafetyConfig) -> None:
        self.config = config
        self._samples: Optional[np.ndarray] = None
        self._resolution: Optional[float] = None
        poly = np.asarray(config.footprint, dtype=float)
        self._radius = float(np.hypot(poly[:, 0], poly[:, 1]).max()) if len(poly) else 0.0

    def samples(self, resolution: float) -> np.ndarray:
        if self._samples is None or self._resolution != resolution:
            self._samples = footprint_samples(self.config.footprint, resolution * 0.5)
            self._resolution = resolution
        return self._samples

    def check(self, occupancy: OccupancyMap, pose: Pose2) -> SafetyResult:
        points = transform_points(pose, self.samples(occupancy.resolution))
        lethal = occupancy.occupied_at(points)
        if not lethal.any():
            return SafetyResult(False, 0.0, 0.0)
        return SafetyResult(True, float(lethal.mean()), self._depth(occupancy, pose, points[lethal]))

    def _depth(self, occupancy: OccupancyMap, pose: Pose2, lethal_points: np.ndarray) -> float:
        """致死セルにある点から、最も近い致死でないセルまでの距離の最大。"""
        res = occupancy.resolution
        reach = self._radius + self.config.depth_range_m
        height, width = occupancy.data.shape
        i0 = max(int((pose[0] - reach - occupancy.origin_x) / res), 0)
        i1 = min(int((pose[0] + reach - occupancy.origin_x) / res) + 1, width)
        j0 = max(int((pose[1] - reach - occupancy.origin_y) / res), 0)
        j1 = min(int((pose[1] + reach - occupancy.origin_y) / res) + 1, height)
        if i1 <= i0 or j1 <= j0:
            return self.config.depth_range_m
        window = occupancy.data[j0:j1, i0:i1] >= occupancy.occupied_threshold
        if window.all():
            return self.config.depth_range_m
        distance = ndimage.distance_transform_edt(window) * res
        ix = np.floor((lethal_points[:, 0] - occupancy.origin_x) / res).astype(int) - i0
        iy = np.floor((lethal_points[:, 1] - occupancy.origin_y) / res).astype(int) - j0
        keep = (ix >= 0) & (ix < window.shape[1]) & (iy >= 0) & (iy < window.shape[0])
        if not keep.any():
            return self.config.depth_range_m
        return min(float(distance[iy[keep], ix[keep]].max()), self.config.depth_range_m)
