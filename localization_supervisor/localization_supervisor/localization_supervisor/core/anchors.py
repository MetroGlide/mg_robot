"""アンカー: 自己位置が信頼できた時点の、主の姿勢と動きの組 (ROS に依存しない)。

復旧では、疑いが始まった時刻より前の最も新しいアンカーに、その後の動きを足して候補を作る。
残すのは時間 (max_age_sec) と走行距離 (max_distance_m) の上限まで。
"""
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

from .geometry import Pose2
from .interfaces import PoseSample


@dataclass
class Anchor:
    stamp: float
    pose: PoseSample   # 主の姿勢 (stamp の時刻に移したもの)
    motion: Pose2      # stamp の時刻の、動きの情報源の姿勢
    odometer: float    # 残し始めてからの走行距離 [m] (この時点)


@dataclass
class AnchorConfig:
    pose: str = ''
    motion: str = ''
    min_interval_sec: float = 1.0
    max_age_sec: float = 120.0
    max_distance_m: float = 100.0


class AnchorStore:
    def __init__(self, config: AnchorConfig) -> None:
        self.config = config
        self._anchors: Deque[Anchor] = deque()

    def clear(self) -> None:
        self._anchors.clear()

    def __len__(self) -> int:
        return len(self._anchors)

    def add(self, anchor: Anchor) -> bool:
        """前のアンカーから min_interval_sec 以上空いていれば残す。残したら True。"""
        last = self._anchors[-1] if self._anchors else None
        if last is not None and anchor.stamp < last.stamp:
            self._anchors.clear()
            last = None
        if last is not None and anchor.stamp - last.stamp < self.config.min_interval_sec:
            return False
        self._anchors.append(anchor)
        return True

    def prune(self, now: float, odometer: float) -> None:
        cfg = self.config
        while self._anchors and (
                now - self._anchors[0].stamp > cfg.max_age_sec
                or odometer - self._anchors[0].odometer > cfg.max_distance_m):
            self._anchors.popleft()

    def latest_before(self, stamp: float) -> Optional[Anchor]:
        chosen = None
        for anchor in self._anchors:
            if anchor.stamp < stamp:
                chosen = anchor
            else:
                break
        return chosen

    def latest(self) -> Optional[Anchor]:
        return self._anchors[-1] if self._anchors else None
