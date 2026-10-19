"""中核のテストで使う、人工の入力を作る道具。"""
import math

import numpy as np

from localization_supervisor.core.geometry import compose
from localization_supervisor.core.grid import OccupancyMap
from localization_supervisor.core.interfaces import (
    Detector, Evidence, MotionSample, ParamSpec, PoseSample, ScanSample, SourceKind, Status)

ROOM_HALF = 9.95


# 外部のプラグインの例 (`supervisor_test_helpers:クラス` で読み込む)

class AlwaysNormal(Detector):
    SOURCES = {'pose': SourceKind.POSE}
    PARAMS = {'gain': ParamSpec(float, 1.0)}

    def evaluate(self, snapshot):
        return Evidence(Status.NORMAL)


class NeedsExtra(Detector):
    EXTRAS = ('wheel_slip',)

    def evaluate(self, snapshot):
        return Evidence(Status.UNKNOWN)


class NotAPlugin:
    pass


def cov(std_xy=0.1, std_yaw=0.05):
    return np.diag([std_xy ** 2, std_xy ** 2, std_yaw ** 2])


def pose_sample(t, pose, std_xy=0.1):
    return PoseSample(t, tuple(pose), cov(std_xy))


def wall_map():
    """20 m 四方の、周囲が壁の地図 (0.1 m 解像度、原点 (-10, -10))。"""
    size = 200
    data = np.zeros((size, size), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    # 部屋の形が対称にならないよう、柱を置く
    data[150:160, 40:50] = 100
    return OccupancyMap(data, 0.1, -10.0, -10.0)


def room_scan(t, pose, n=360, max_range=30.0):
    """wall_map の部屋の中の pose から見たスキャン (センサは base と同じ位置)。柱は無視する。"""
    angles = -math.pi + 2.0 * math.pi * np.arange(n) / n
    ranges = np.full(n, np.inf)
    for i, a in enumerate(angles):
        world = pose[2] + a
        c, s = math.cos(world), math.sin(world)
        candidates = []
        if abs(c) > 1e-9:
            candidates += [(ROOM_HALF - pose[0]) / c, (-ROOM_HALF - pose[0]) / c]
        if abs(s) > 1e-9:
            candidates += [(ROOM_HALF - pose[1]) / s, (-ROOM_HALF - pose[1]) / s]
        candidates = [d for d in candidates if d > 0]
        if candidates:
            ranges[i] = min(candidates)
    ranges[ranges > max_range] = np.inf
    return ScanSample(t, ranges, -math.pi, 2.0 * math.pi / n, 0.1, max_range, (0.0, 0.0, 0.0))


class Drive:
    """odom 座標で一定の速度で動くロボット。map->odom は固定。"""

    def __init__(self, map_odom=(1.0, -2.0, 0.3), speed=0.5, yaw_rate=0.05, start=(0.0, 0.0, 0.0)):
        self.map_odom = map_odom
        self.speed = speed
        self.yaw_rate = yaw_rate
        self.start = start

    def odom(self, t):
        yaw = self.start[2] + self.yaw_rate * t
        if abs(self.yaw_rate) < 1e-9:
            return (self.start[0] + self.speed * t * math.cos(yaw),
                    self.start[1] + self.speed * t * math.sin(yaw), yaw)
        r = self.speed / self.yaw_rate
        return (self.start[0] + r * (math.sin(yaw) - math.sin(self.start[2])),
                self.start[1] - r * (math.cos(yaw) - math.cos(self.start[2])), yaw)

    def map(self, t):
        return compose(self.map_odom, self.odom(t))

    def motion(self, t):
        return MotionSample(t, self.odom(t))
