"""直近の一定距離のスキャンを、動きの情報源の姿勢とともに残す (ROS に依存しない)。

スキャンは、動きが keyframe_distance_m か keyframe_angle_rad を超えたときだけ残し (間引き)、
残したスキャンの間の走行距離の合計が window_distance_m を超えたら、古いものから捨てる。
残したスキャンを、動きで今の base の座標にまとめれば、1 枚より広い範囲の局所地図になる。
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np

from .geometry import Pose2, compose, inverse, wrap_angle
from .grid import scan_points_in_map, transform_points
from .interfaces import ScanSample


@dataclass
class BufferedScan:
    stamp: float
    motion: Pose2         # スキャンの時刻の、動きの情報源の座標での base の姿勢
    points: np.ndarray    # base の座標の点 (N, 2)
    distance: float       # 最初に残したスキャンからの走行距離 [m]


@dataclass
class ScanBufferConfig:
    """監督が復旧のために残すスキャンのバッファ (判定の submap_map のバッファとは別)。"""
    reference: str = ''
    motion: str = ''
    keyframe_distance_m: float = 0.5
    keyframe_angle_rad: float = 0.2
    window_distance_m: float = 20.0
    max_points_per_scan: int = 90
    max_range_m: float = 25.0

    def make(self) -> 'ScanBuffer':
        return ScanBuffer(self.keyframe_distance_m, self.keyframe_angle_rad, self.window_distance_m,
                          self.max_points_per_scan, self.max_range_m)


class ScanBuffer:
    def __init__(
        self, keyframe_distance_m: float, keyframe_angle_rad: float, window_distance_m: float,
        max_points_per_scan: int, max_range_m: float,
    ) -> None:
        self.keyframe_distance_m = keyframe_distance_m
        self.keyframe_angle_rad = keyframe_angle_rad
        self.window_distance_m = window_distance_m
        self.max_points_per_scan = max_points_per_scan
        self.max_range_m = max_range_m
        self._scans: Deque[BufferedScan] = deque()

    def __len__(self) -> int:
        return len(self._scans)

    def clear(self) -> None:
        self._scans.clear()

    def latest(self) -> Optional[BufferedScan]:
        return self._scans[-1] if self._scans else None

    def travelled(self) -> float:
        """残しているスキャンの間の走行距離 [m]。"""
        if not self._scans:
            return 0.0
        return self._scans[-1].distance - self._scans[0].distance

    def add(self, scan: ScanSample, motion: Pose2) -> bool:
        """キーフレームの条件を満たせば残して True を返す。"""
        if scan.sensor_pose is None:
            return False
        last = self.latest()
        distance = 0.0
        if last is not None:
            if scan.stamp < last.stamp:
                self.clear()
                last = None
            else:
                step = math.hypot(motion[0] - last.motion[0], motion[1] - last.motion[1])
                turn = abs(wrap_angle(motion[2] - last.motion[2]))
                if step < self.keyframe_distance_m and turn < self.keyframe_angle_rad:
                    return False
                distance = last.distance + step
        points = scan_points_in_map(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
            min(scan.range_max, self.max_range_m), (0.0, 0.0, 0.0), scan.sensor_pose, self.max_points_per_scan)
        self._scans.append(BufferedScan(scan.stamp, motion, points, distance))
        while len(self._scans) > 1 and distance - self._scans[0].distance > self.window_distance_m:
            self._scans.popleft()
        return True

    def points_in(self, map_base: Pose2, motion: Pose2) -> np.ndarray:
        """残したスキャンの点を、今の base の姿勢 (map_base) と、そのときの動き (motion) で地図の座標にする。"""
        if not self._scans:
            return np.zeros((0, 2))
        now_inv = inverse(motion)
        parts = [transform_points(compose(map_base, compose(now_inv, s.motion)), s.points) for s in self._scans]
        return np.concatenate(parts, axis=0)
