"""走行中に位置が進まない (詰まっている) 時間を測る"""
from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Optional, Tuple


@dataclass(frozen=True)
class ProgressStatus:
    stalled_sec: float
    number_of_recoveries: int


class ProgressMonitor:
    """最後に min_progress_m 以上動いてからの経過時間を測る。

    BT はリカバリーの試行回数を無制限にしているので、障害物が居座ると Nav2 は失敗を返さず
    回避行動をくり返す。それを外から気づけるようにするためのもの。
    """

    def __init__(self, min_progress_m: float):
        self._min_progress_m = min_progress_m
        self._anchor: Optional[Tuple[float, float]] = None
        self._anchor_time: Optional[float] = None
        self._number_of_recoveries = 0

    def reset(self, now: float) -> None:
        """新しいゴールを送ったときに呼ぶ。"""
        self._anchor = None
        self._anchor_time = now
        self._number_of_recoveries = 0

    def update(self, x: float, y: float, number_of_recoveries: int, now: float) -> None:
        """Nav2 のフィードバックごとに呼ぶ。"""
        self._number_of_recoveries = number_of_recoveries
        if (self._anchor is None
                or hypot(x - self._anchor[0], y - self._anchor[1]) >= self._min_progress_m):
            self._anchor = (x, y)
            self._anchor_time = now

    def status(self, now: float) -> ProgressStatus:
        stalled = 0.0 if self._anchor_time is None else max(0.0, now - self._anchor_time)
        return ProgressStatus(stalled, self._number_of_recoveries)
