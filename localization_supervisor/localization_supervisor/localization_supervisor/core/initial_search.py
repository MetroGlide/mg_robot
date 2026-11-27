"""GNSS などのおおよその位置の周りで、スキャンを地図に向きの全周で照合して、初期姿勢を探す (ROS に依存しない)。

おおよその位置は、地図とのずれ (場所で変わる) を含み、向きも定まらない。その周りで今のスキャンを地図に照合し、

  - はっきり合う姿勢が 1 つ見つかれば (matched)、その姿勢を小さい共分散で使う
  - 合う姿勢がない・似た一致の姿勢が離れて並ぶ・点や地図が足りない (low_ratio / ambiguous / too_few_points / no_map)
    ときは、おおよその位置を広い共分散で使う (推定器に広く探させる)
"""
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .geometry import Pose2
from .grid import OccupancyMap, build_local_distance_field
from .interfaces import SearchWindow
from .matcher.correlative import CorrelativeMatcher
from .plugins import resolve_params


@dataclass
class InitialSearchConfig:
    search_range_m: float = 4.0
    # 起動の直後は向きが定まらないので、向きは全周を探す
    search_yaw_rad: float = math.pi
    step_xy_m: float = 0.5
    step_yaw_rad: float = 0.1
    # 距離場を作る範囲は、探す範囲にこれを足したもの。これより遠い点は照合に使わない
    scan_radius_m: float = 15.0
    # 線の太い地図では、広い許容距離だと少しずれた姿勢も合ってしまい、見分けられない
    tolerance_m: float = 0.15
    # 粗い格子の許容距離と、絞り込む候補の数 (CorrelativeMatcher の coarse_tolerance_m・coarse_candidates)
    coarse_tolerance_m: float = 0.4
    coarse_candidates: int = 8
    # 点を地図の外へ追い出す姿勢を、残った点だけで合ったとみなさない
    outside_is_miss: bool = True
    min_points: int = 30
    min_ratio: float = 0.5
    ambiguity_margin: float = 0.1
    separation_m: float = 1.0
    matched_std_m: float = 0.3
    matched_std_yaw_rad: float = 0.1
    fallback_std_m: float = 2.0
    fallback_std_yaw_rad: float = 0.5


@dataclass
class InitialSearchResult:
    pose: Pose2
    std_m: float
    std_yaw_rad: float
    # matched / low_ratio / ambiguous / too_few_points / no_map
    reason: str
    ratio: float = 0.0
    margin: float = 0.0


def search_initial_pose(
    occupancy: Optional[OccupancyMap], points_base: Optional[np.ndarray], center: Pose2, cfg: InitialSearchConfig,
) -> InitialSearchResult:
    """center (おおよその姿勢) の周りで、base 座標のスキャンの点を地図に照合して、初期姿勢と共分散を決める。"""
    def fallback(reason: str, ratio: float = 0.0, margin: float = 0.0) -> InitialSearchResult:
        return InitialSearchResult(center, cfg.fallback_std_m, cfg.fallback_std_yaw_rad, reason, ratio, margin)

    if occupancy is None:
        return fallback('no_map')
    if points_base is None:
        return fallback('too_few_points')
    points_base = points_base[np.hypot(points_base[:, 0], points_base[:, 1]) <= cfg.scan_radius_m]
    if points_base.shape[0] < cfg.min_points:
        return fallback('too_few_points')
    field = build_local_distance_field(
        occupancy, center[0], center[1], cfg.search_range_m + cfg.scan_radius_m)
    if field is None:
        return fallback('no_map')
    matcher = CorrelativeMatcher('initial_search', resolve_params(CorrelativeMatcher, {
        'tolerance_m': cfg.tolerance_m, 'min_points': cfg.min_points, 'separation_m': cfg.separation_m,
        'coarse_tolerance_m': cfg.coarse_tolerance_m, 'coarse_candidates': cfg.coarse_candidates,
        'outside_is_miss': cfg.outside_is_miss},
        'initial_search'))
    window = SearchWindow(cfg.search_range_m, cfg.step_xy_m, cfg.search_yaw_rad, cfg.step_yaw_rad)
    peaks = matcher.match(field, points_base, center, window, max_peaks=2)
    if not peaks:
        return fallback('too_few_points')
    best = peaks[0]
    margin = best.ratio - peaks[1].ratio if len(peaks) > 1 else best.ratio
    if best.ratio < cfg.min_ratio:
        return fallback('low_ratio', best.ratio, margin)
    if len(peaks) > 1 and margin < cfg.ambiguity_margin:
        return fallback('ambiguous', best.ratio, margin)
    return InitialSearchResult(best.pose, cfg.matched_std_m, cfg.matched_std_yaw_rad, 'matched', best.ratio, margin)


def covariance_6x6(std_m: float, std_yaw_rad: float) -> list:
    """PoseWithCovariance の共分散 (6x6、行優先)。x・y・yaw だけを埋める。"""
    cov = [0.0] * 36
    cov[0] = cov[7] = std_m * std_m
    cov[35] = std_yaw_rad * std_yaw_rad
    return cov
