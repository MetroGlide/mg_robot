"""2 つの姿勢の情報源の残差から、a の分散にかける倍率を求める (ROS に依存しない)。

自己位置が信頼できる周期に、a と b の位置の差の二乗 Δ² と、両方の分散を残す。
a の自己申告の分散が正しければ、Δ² の平均は 2 自由度 × (a の分散 + b の分散 + 時刻のずれの分散) になる。
実際に Δ² の平均がそれより大きければ、a の実際の誤差は自己申告より大きい。そのとき、a の分散にかける倍率 S を
`S = (Δ² の平均 / 2 − b の分散 − 時刻のずれの分散) / a の分散` とする (これで平均が期待値に一致する)。1 以上、上限以下に収める。
地図と GNSS のずれは場所ごとに違う (地図は SLAM から作るため) ので、一定のバイアスとしては補正せず、信頼度だけを下げる。

観測する a の分散は、受け取る側が倍率をかけた後の値になる。そこで、前の周期に出した倍率で割って、
倍率をかける前の分散に戻して残す (受け取る側がかけていなければ、倍率は上限に向かうだけで害はない)。
信頼できない周期は残さず、倍率を保つ。倍率は急に変えない (gain の割合で近づける)。
"""
from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple

# 位置 (x, y) の自由度
DOF = 2.0


@dataclass
class ResidualConfig:
    # 倍率をかけたい情報源と、比べる相手
    a: str = ''
    b: str = ''
    # 統計に使う直近の時間 [s]
    window_sec: float = 60.0
    # 倍率を更新するのに要る標本の数
    min_samples: int = 20
    # これより古い標本は使わない [s]
    max_age_sec: float = 3.0
    # 時刻のずれなどによる位置の不確かさ [m]
    extra_std_m: float = 0.3
    # 1 回の更新で、求めた倍率へ近づく割合 (小さいほどゆっくり)
    gain: float = 0.2
    # 倍率の上限
    max_scale: float = 100.0
    # 倍率がこれ以上の状態を「大きい」としてイベントで知らせる (これの 0.8 倍を下回ると戻ったとする)
    notify_scale: float = 2.0


@dataclass
class ResidualStats:
    count: int = 0
    # 今の倍率で正規化した Δ² の平均 (自己申告が正しければ 2 に近い)
    mean_d2: float = 0.0
    # a の分散にかける倍率 (1 以上)
    scale: float = 1.0


class ResidualTracker:
    def __init__(self, config: ResidualConfig) -> None:
        self.config = config
        self.scale = 1.0
        # (時刻, Δ², 倍率をかける前の a の分散, b の分散 + 時刻のずれの分散)
        self._samples: Deque[Tuple[float, float, float, float]] = deque()
        self._fresh = False

    def clear(self) -> None:
        """標本と倍率を捨てる (観測する分散の意味が変わるとき)。"""
        self._samples.clear()
        self._fresh = False
        self.scale = 1.0

    def add(self, stamp: float, delta2: float, var_a: float, var_other: float) -> None:
        """var_a は、受け取る側が倍率をかけた後の分散 (add の中で今の倍率で割る)。"""
        if self._samples and stamp < self._samples[-1][0]:
            self._samples.clear()
        self._samples.append((stamp, delta2, var_a / self.scale, var_other))
        self._fresh = True

    def update(self, now: float) -> ResidualStats:
        """標本の統計を求め、新しい標本があったときだけ倍率を更新する。"""
        cfg = self.config
        while self._samples and now - self._samples[0][0] > cfg.window_sec:
            self._samples.popleft()
        count = len(self._samples)
        if count == 0:
            return ResidualStats(0, 0.0, self.scale)
        mean_delta2 = sum(s[1] for s in self._samples) / count
        mean_var_a = max(sum(s[2] for s in self._samples) / count, 1e-9)
        mean_var_other = sum(s[3] for s in self._samples) / count
        if self._fresh and count >= cfg.min_samples:
            target = (mean_delta2 / DOF - mean_var_other) / mean_var_a
            target = min(max(target, 1.0), cfg.max_scale)
            self.scale += cfg.gain * (target - self.scale)
        self._fresh = False
        mean_d2 = mean_delta2 / (mean_var_a * self.scale + mean_var_other)
        return ResidualStats(count, mean_d2, self.scale)
