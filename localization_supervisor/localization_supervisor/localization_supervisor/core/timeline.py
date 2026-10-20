"""時刻の付いた標本の履歴と、標本を共通の時刻に移す計算 (ROS に依存しない)。

情報源の標本は届く時刻がそろわない。比べるときは、動きの情報源 (motion) の履歴で、
各標本を共通の時刻 (Snapshot の時刻) に移してから比べる。

  map->base(t_to) = map->base(t_from) * [odom->base(t_from)^-1 * odom->base(t_to)]

動きの情報源は優先の順に並べ、両方の時刻を含む最初のものを使う。どれも含まなければ、姿勢を時刻どおりにそろえられない。
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Generic, List, Optional, Sequence, Tuple, TypeVar

import numpy as np

from .geometry import Pose2, compose, inverse, wrap_angle
from .interfaces import MotionSample, PoseSample

T = TypeVar('T', PoseSample, MotionSample)


class History(Generic[T]):
    """時刻の順の標本の履歴。horizon_sec より古いものは捨てる。"""

    def __init__(self, horizon_sec: float) -> None:
        self.horizon_sec = horizon_sec
        self._samples: Deque[T] = deque()

    def add(self, sample: T) -> None:
        # 時刻が戻ったら (再生のやり直し、シミュレーション時刻の巻き戻し) 履歴を捨てる
        if self._samples and sample.stamp < self._samples[-1].stamp:
            self._samples.clear()
        self._samples.append(sample)
        while self._samples and sample.stamp - self._samples[0].stamp > self.horizon_sec:
            self._samples.popleft()

    def latest(self) -> Optional[T]:
        return self._samples[-1] if self._samples else None

    def samples(self) -> List[T]:
        return list(self._samples)

    def __len__(self) -> int:
        return len(self._samples)

    def bracket(self, stamp: float) -> Optional[Tuple[T, T]]:
        """stamp を挟む 2 つの標本。範囲の外なら None。"""
        samples = self._samples
        if not samples or stamp < samples[0].stamp or stamp > samples[-1].stamp:
            return None
        lo, hi = 0, len(samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if samples[mid].stamp <= stamp:
                lo = mid
            else:
                hi = mid
        return samples[lo], samples[hi]


def interpolate_pose(a: Pose2, b: Pose2, ratio: float) -> Pose2:
    return (a[0] + (b[0] - a[0]) * ratio, a[1] + (b[1] - a[1]) * ratio,
            wrap_angle(a[2] + wrap_angle(b[2] - a[2]) * ratio))


class MotionHistory(History[MotionSample]):
    def __init__(self, horizon_sec: float, max_extrapolation_sec: float = 0.0) -> None:
        super().__init__(horizon_sec)
        self.max_extrapolation_sec = max_extrapolation_sec

    def at(self, stamp: float) -> Optional[Pose2]:
        """stamp の姿勢 (前後の標本から線形補間)。最新より max_extrapolation_sec 以内の新しさなら最新の値。"""
        latest = self.latest()
        if latest is None:
            return None
        if latest.stamp < stamp <= latest.stamp + self.max_extrapolation_sec:
            return latest.pose
        pair = self.bracket(stamp)
        if pair is None:
            return None
        a, b = pair
        if b.stamp <= a.stamp:
            return a.pose
        return interpolate_pose(a.pose, b.pose, (stamp - a.stamp) / (b.stamp - a.stamp))


@dataclass
class AlignmentConfig:
    # 移した距離・回転に比例して足す、位置・yaw の標準偏差 (動きの情報源の誤差)
    std_per_m: float = 0.02
    std_per_rad: float = 0.02


@dataclass
class Motion:
    """2 つの時刻の間の動き。delta は t_from の base から見た t_to の base の姿勢。"""
    delta: Pose2
    distance: float
    rotation: float
    source: str


class Timeline:
    """動きの情報源 (優先の順) で、時刻の間の動きを求める。"""

    def __init__(self, motions: Sequence[Tuple[str, MotionHistory]], alignment: AlignmentConfig) -> None:
        self._motions = list(motions)
        self.alignment = alignment

    def motion(self, t_from: float, t_to: float) -> Optional[Motion]:
        for name, history in self._motions:
            a = history.at(t_from)
            b = history.at(t_to)
            if a is None or b is None:
                continue
            delta = compose(inverse(a), b)
            return Motion(delta, math.hypot(delta[0], delta[1]), abs(delta[2]), name)
        return None

    def align(self, sample: PoseSample, stamp: float) -> Optional[PoseSample]:
        """sample を stamp の時刻に移す。移せなければ None。"""
        if sample.stamp == stamp:
            return sample
        motion = self.motion(sample.stamp, stamp)
        if motion is None:
            return None
        cov = np.array(sample.cov, dtype=float, copy=True)
        var_xy = (self.alignment.std_per_m * motion.distance) ** 2
        cov[0, 0] += var_xy
        cov[1, 1] += var_xy
        cov[2, 2] += (self.alignment.std_per_rad * motion.rotation) ** 2
        return PoseSample(stamp, compose(sample.pose, motion.delta), cov)
