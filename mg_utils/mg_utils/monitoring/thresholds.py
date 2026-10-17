"""値のしきい値による判定と、移動平均。ROS に依存しない。

CPU 使用率や GNSS の精度のように、値の大きさで診断のレベルを決めるノードが共通で使う。
"""
from collections import deque
from typing import Deque

from mg_utils.monitoring.health import ERROR, OK, WARN


class MovingAverage:
    """直近 window 個の値の平均 (瞬間的な値で診断のレベルが揺れないようにする)。"""

    def __init__(self, window: int) -> None:
        if window < 1:
            raise ValueError(f'window は 1 以上: {window}')
        self._values: Deque[float] = deque(maxlen=window)

    def add(self, value: float) -> float:
        self._values.append(value)
        return self.value

    @property
    def value(self) -> float:
        if not self._values:
            return 0.0
        return sum(self._values) / len(self._values)


def level_for_high(value: float, warn: float, error: float) -> str:
    """値が大きいほど悪いものの判定。warn 以上で warn、error 以上で error。"""
    if warn > error:
        raise ValueError(f'warn ({warn}) は error ({error}) 以下にする')
    if value >= error:
        return ERROR
    if value >= warn:
        return WARN
    return OK


def worst(*levels: str) -> str:
    """レベル (ok / warn / error) のうち、最も悪いもの。"""
    order = (OK, WARN, ERROR)
    return max(levels, key=order.index)
