"""スキャンと地図の一致を調べる計算 (ROS に依存しない)。

ある姿勢でスキャンを地図に重ね、スキャンの点が占有セルの近くにあるかどうかを見る。
AMCL の共分散 (自己申告) とは独立な、自己位置の妥当性の指標になる。
巨大な地図でも扱えるよう、姿勢の周りの局所だけの距離場をそのつど作る。
"""
import math
from typing import Optional, Tuple

import numpy as np
from scipy import ndimage

from .checks import Pose2, compose


class OccupancyMap:
    """nav_msgs/OccupancyGrid 相当の占有格子 (行優先、原点は左下)。"""

    def __init__(
        self, data: np.ndarray, resolution: float, origin_x: float, origin_y: float,
        occupied_threshold: int = 65,
    ) -> None:
        self.data = data
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.occupied_threshold = occupied_threshold


class LocalDistanceField:
    """姿勢の周りの、最も近い占有セルまでの距離 [m] の格子。"""

    def __init__(self, distance: np.ndarray, origin_x: float, origin_y: float, resolution: float) -> None:
        self.distance = distance
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution

    def lookup(self, points_xy: np.ndarray) -> np.ndarray:
        """点 (N, 2) の距離を返す。格子の外の点は nan。"""
        ix = np.floor((points_xy[:, 0] - self.origin_x) / self.resolution).astype(int)
        iy = np.floor((points_xy[:, 1] - self.origin_y) / self.resolution).astype(int)
        height, width = self.distance.shape
        inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        result = np.full(points_xy.shape[0], np.nan)
        result[inside] = self.distance[iy[inside], ix[inside]]
        return result


def build_local_distance_field(
    occupancy: OccupancyMap, center_x: float, center_y: float, radius: float, min_occupied_cells: int = 50,
) -> Optional[LocalDistanceField]:
    """中心の周り (一辺 2*radius の正方形) の距離場を作る。占有セルが少なければ None。"""
    res = occupancy.resolution
    height, width = occupancy.data.shape
    i0 = max(int((center_x - radius - occupancy.origin_x) / res), 0)
    i1 = min(int((center_x + radius - occupancy.origin_x) / res) + 1, width)
    j0 = max(int((center_y - radius - occupancy.origin_y) / res), 0)
    j1 = min(int((center_y + radius - occupancy.origin_y) / res) + 1, height)
    if i1 <= i0 or j1 <= j0:
        return None
    occupied = occupancy.data[j0:j1, i0:i1] >= occupancy.occupied_threshold
    if int(occupied.sum()) < min_occupied_cells:
        return None
    distance = ndimage.distance_transform_edt(~occupied) * res
    return LocalDistanceField(
        distance, occupancy.origin_x + i0 * res, occupancy.origin_y + j0 * res, res)


def scan_points_in_map(
    ranges: np.ndarray, angle_min: float, angle_increment: float,
    range_min: float, range_max: float,
    map_base: Pose2, base_lidar: Pose2, max_points: int = 180,
) -> np.ndarray:
    """LaserScan の有効な点を、map 座標の (N, 2) にする。max_points 点になるよう等間隔に間引く。

    map_base: 推定した base の姿勢 (map->base)。base_lidar: base->lidar (静的 TF)。
    """
    angles = angle_min + angle_increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    ranges, angles = ranges[valid], angles[valid]
    if ranges.size == 0:
        return np.zeros((0, 2))
    if ranges.size > max_points:
        keep = np.linspace(0, ranges.size - 1, max_points).astype(int)
        ranges, angles = ranges[keep], angles[keep]
    map_lidar = compose(map_base, base_lidar)
    c, s = math.cos(map_lidar[2]), math.sin(map_lidar[2])
    px = ranges * np.cos(angles)
    py = ranges * np.sin(angles)
    return np.stack([map_lidar[0] + c * px - s * py, map_lidar[1] + s * px + c * py], axis=1)


def match_ratio(
    field: LocalDistanceField, points_xy: np.ndarray, tolerance: float, min_points: int = 30,
) -> Optional[Tuple[float, float]]:
    """点のうち、占有セルから tolerance [m] 以内にある割合と、距離の平均を返す。

    距離場の外の点は数えない。数えられた点が min_points 未満なら None (判断できない)。
    """
    distances = field.lookup(points_xy)
    distances = distances[np.isfinite(distances)]
    if distances.size < min_points:
        return None
    return float(np.mean(distances <= tolerance)), float(np.mean(distances))
