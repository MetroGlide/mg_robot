"""トピックの鮮度と受信頻度を判定する。ROS に依存しない (時刻は呼び出し側が渡す)。

- RateEstimator: 直近のメッセージの間隔から、受信頻度 [Hz] を推定する
- ActivityMonitor: 1 つのトピックの途絶を判定し (復帰の保持つき)、受信頻度も持つ
- TickGuard: 監視ノード自身の周期の遅れを検知する

driver_watchdog_node (反応つきの監視) と topic_monitor_node (表示だけの監視) が、同じ判定を使う。
"""
from collections import deque
from typing import Deque, Optional


class RateEstimator:
    """直近 window 個のメッセージの間隔から、受信頻度 [Hz] を推定する。"""

    def __init__(self, window: int = 10) -> None:
        self._stamps: Deque[float] = deque(maxlen=window)

    def tick(self, now: float) -> None:
        self._stamps.append(now)

    def hz(self, now: float, max_age_sec: Optional[float] = None) -> float:
        """受信頻度。max_age_sec より長く途絶えていれば 0 (最後の頻度を出し続けない)。"""
        if len(self._stamps) < 2:
            return 0.0
        if max_age_sec is not None and now - self._stamps[-1] > max_age_sec:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self._stamps) - 1) / span


class ActivityMonitor:
    """1 つのトピックの鮮度を見て、異常 (faulted) を判定する。

    - 最後の受信から stale_timeout_sec を超えたら異常にする
    - 異常から戻るには、recover_hold_sec のあいだ正常が続く必要がある (ばたつきを防ぐ)
    - 起動直後は startup_grace_sec のあいだ、初回のメッセージが無くても異常にしない
    - armed_on_first_message が true のときは、初回のメッセージを受け取るまで監視しない
    - ok=False のメッセージ (接続状態が false など) は、途絶と同じく異常にする
    """

    def __init__(
        self,
        stale_timeout_sec: float,
        recover_hold_sec: float,
        start_time: float,
        startup_grace_sec: float = 0.0,
        armed_on_first_message: bool = False,
        rate_window: int = 10,
    ) -> None:
        self._stale_timeout = stale_timeout_sec
        self._recover_hold = recover_hold_sec
        self._start_time = start_time
        self._startup_grace = startup_grace_sec
        self._armed_on_first = armed_on_first_message
        self._rate = RateEstimator(rate_window)
        self._last_rx: Optional[float] = None
        self._ok = True
        self._faulted = False
        self._good_since: Optional[float] = None

    @property
    def faulted(self) -> bool:
        return self._faulted

    @property
    def disconnected(self) -> bool:
        """最後のメッセージが ok=False だった"""
        return not self._ok

    @property
    def received(self) -> bool:
        return self._last_rx is not None

    def on_message(self, now: float, ok: bool = True) -> None:
        self._last_rx = now
        self._ok = ok
        self._rate.tick(now)

    def forgive(self, now: float) -> None:
        """監視側の遅れで、受信の記録が古く見えているとき、受信時刻を今に進める"""
        if self._last_rx is not None:
            self._last_rx = max(self._last_rx, now)

    def stale_sec(self, now: float) -> float:
        """途絶している時間。猶予の間と、監視前は 0"""
        if self._last_rx is None:
            if self._armed_on_first:
                return 0.0
            return max(0.0, now - (self._start_time + self._startup_grace))
        return max(0.0, now - self._last_rx)

    def hz(self, now: float) -> float:
        """受信頻度。途絶 (stale_timeout_sec 超え) していれば 0"""
        return self._rate.hz(now, self._stale_timeout)

    def update(self, now: float) -> bool:
        """判定を更新し、異常かどうかを返す。何度呼んでもよい (時刻が進んだ分だけ状態が進む)"""
        bad = self.stale_sec(now) > self._stale_timeout or not self._ok
        if bad:
            self._faulted = True
            self._good_since = None
        elif self._faulted:
            if self._good_since is None:
                self._good_since = now
            if now - self._good_since >= self._recover_hold:
                self._faulted = False
                self._good_since = None
        return self._faulted


class TickGuard:
    """周期処理の間隔が max_gap_sec を超えたら、遅れたとみなす (CPU の詰まりなど)"""

    def __init__(self, max_gap_sec: float) -> None:
        self._max_gap = max_gap_sec
        self._last: Optional[float] = None

    def check(self, now: float) -> bool:
        lagged = self._last is not None and now - self._last > self._max_gap
        self._last = now
        return lagged
