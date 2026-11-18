"""初期姿勢を決めるまでの照合の段取り (ROS に依存しない)。

起動の直後は GNSS の向きが定まらず、止まって推論する時間もない。走りながら (遠隔操作でも自律でも)
GNSS の位置の周りで照合を繰り返し、はっきり合った姿勢で初期姿勢を決める。

  - 最初の照合で合わなければ、GNSS の位置を広い共分散で 1 回だけ出す (AMCL に粒子を広く撒かせる)
  - GNSS の位置が retry_distance_m 動くごとに照合し直し、合えば (広い共分散の後でも) 上書きする
  - 外から初期姿勢 (/initialpose) が与えられたら、それに従い、GNSS による初期化をやめる
"""
import math
from typing import Optional, Tuple

from localization_supervisor.core.geometry import Pose2, wrap_angle


class InitAttempts:
    def __init__(self, retry_distance_m: float):
        self.retry_distance_m = retry_distance_m
        self.reset()

    def reset(self) -> None:
        self.decided = False
        self.external = False
        self.fallback_published = False
        self.attempts = 0
        self._last_xy: Optional[Tuple[float, float]] = None

    def should_attempt(self, xy: Tuple[float, float]) -> bool:
        """GNSS の位置 xy で照合するか。決まった後と、前の照合から動いていないときはしない。"""
        if self.decided:
            return False
        if self._last_xy is None:
            return True
        return math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1]) >= self.retry_distance_m

    def record(self, xy: Tuple[float, float], matched: bool) -> bool:
        """照合の結果を記録し、GNSS の位置を広い共分散で出すべきかを返す (最初に合わなかったときだけ)。"""
        self.attempts += 1
        self._last_xy = xy
        if matched:
            self.decided = True
            return False
        if self.fallback_published:
            return False
        self.fallback_published = True
        return True

    def take_external(self) -> None:
        """外から初期姿勢が与えられた。以後、GNSS による初期化はしない。"""
        self.decided = True
        self.external = True


def relative(a: Pose2, b: Pose2) -> Pose2:
    """a から見た b (a^-1 * b)。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap_angle(b[2] - a[2]))


def compose(a: Pose2, b: Pose2) -> Pose2:
    """a * b。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def propagate(pose_at_scan: Pose2, odom_at_scan: Pose2, odom_now: Pose2) -> Pose2:
    """スキャンの時刻の地図上の姿勢を、オドメトリで今の時刻へ進める。

    照合に時間がかかる間も走り続けるので、スキャンの時刻の姿勢のまま出すと、その間に進んだ分だけずれる。
    """
    return compose(pose_at_scan, relative(odom_at_scan, odom_now))
