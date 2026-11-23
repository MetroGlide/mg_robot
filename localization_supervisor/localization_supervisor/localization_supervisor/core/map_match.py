"""姿勢の候補を地図と照合し、離れた仮説のうち地図に最もよく合うものに決める (ROS に依存しない)。

照合した後の姿勢どうしが近い候補は同じ仮説にまとめ、離れた仮説の最良 (次点) との一致率の差が足りなければ決めない。
照合の一致率の山は近い所でほぼ平らなので、これは離れた仮説を見分けるためのもので、細かい位置合わせには使わない。
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np

from .geometry import Pose2, separated
from .interfaces import SearchWindow
from .matcher.correlative import CorrelativeMatcher


@dataclass
class MapMatchConfig:
    # 決めるのに要る一致率の下限
    min_ratio: float = 0.35
    # 決めるのに要る、離れた次点との一致率の差。候補を入力のまま採るのは、最良との差がこれ未満のとき
    margin: float = 0.1
    # 別の仮説とみなす、照合した後の姿勢どうしの位置・向きの差
    separation_m: float = 1.0
    separation_yaw_rad: float = 0.2
    # 狭い範囲 (細かい格子で探す) と、その刻み
    narrow_xy_m: float = 0.5
    narrow_yaw_rad: float = 0.15
    step_xy_m: float = 0.1
    step_yaw_rad: float = 0.05
    # 候補ごとの探す範囲の上限
    cap_xy_m: float = 2.0
    cap_yaw_rad: float = 0.55
    # 狭い範囲を超えるときの粗い格子の刻み
    wide_step_xy_m: float = 0.25
    wide_step_yaw_rad: float = 0.1
    # 一致とみなす距離 (細かい格子 / 粗い格子) と、粗い格子から絞り込む候補の数
    tolerance_m: float = 0.3
    coarse_tolerance_m: float = 0.5
    coarse_candidates: int = 5
    # 照合に使う点の数の上限と、距離場の半径
    max_points: int = 400
    field_radius_m: float = 25.0


@dataclass
class CandidatePose:
    """照合の候補。range は、その姿勢の周りを探す範囲 (狭い範囲と上限で挟む)。"""
    pose: Pose2
    range_xy_m: float
    range_yaw_rad: float


@dataclass
class MatchDecision:
    """照合の結果。best が None なら決まらない (reason にわけ)。"""
    best: Optional[str]
    pose: Optional[Pose2]
    reason: str
    # 最良と同じ仮説の候補と、そのうち入力の姿勢のまま採ってよい候補
    same: List[str] = field(default_factory=list)
    agree: List[str] = field(default_factory=list)
    margin: Optional[float] = None
    ratios: Dict[str, float] = field(default_factory=dict)
    matched: Dict[str, Pose2] = field(default_factory=dict)


FieldFor = Callable[[float, float, float], object]


class MapMatcher:
    def __init__(self, config: MapMatchConfig) -> None:
        self.config = config
        params = {key: spec.default for key, spec in CorrelativeMatcher.PARAMS.items()}
        params['tolerance_m'] = config.tolerance_m
        self._narrow = CorrelativeMatcher('map_match', params)
        # 粗い格子では本当の姿勢から刻みの半分ずれるので、許容距離を広げて多めに拾い、細かい格子で絞り込む
        self._wide = CorrelativeMatcher('map_match_wide', dict(
            params, coarse_tolerance_m=config.coarse_tolerance_m, coarse_candidates=config.coarse_candidates))

    def window(self, range_xy: float, range_yaw: float) -> Tuple[CorrelativeMatcher, SearchWindow]:
        """探す範囲 (狭い範囲と上限で挟む) と、それに合う照合器。狭い範囲を超えるときは粗い格子で探す。"""
        c = self.config
        range_xy = min(max(range_xy, c.narrow_xy_m), c.cap_xy_m)
        range_yaw = min(max(range_yaw, c.narrow_yaw_rad), c.cap_yaw_rad)
        if range_xy <= c.narrow_xy_m and range_yaw <= c.narrow_yaw_rad:
            return self._narrow, SearchWindow(range_xy, c.step_xy_m, range_yaw, c.step_yaw_rad)
        return self._wide, SearchWindow(range_xy, c.wide_step_xy_m, range_yaw, c.wide_step_yaw_rad)

    def thin(self, points_base: np.ndarray) -> np.ndarray:
        """照合に使う点を、上限の数まで等間隔に間引く。"""
        if points_base.shape[0] <= self.config.max_points:
            return points_base
        keep = np.linspace(0, points_base.shape[0] - 1, self.config.max_points).astype(int)
        return points_base[keep]

    def separated(self, a: Pose2, b: Pose2) -> bool:
        c = self.config
        return separated(a, b, c.separation_m - 1e-9, c.separation_yaw_rad - 1e-9)

    def decide(
        self, field_for: FieldFor, points_base: np.ndarray, candidates: Mapping[str, CandidatePose],
    ) -> MatchDecision:
        """候補ごとに照合し、最良の仮説に決める。field_for(x, y, radius) は姿勢の周りの距離場 (なければ None)。"""
        c = self.config
        points = self.thin(points_base)
        scored: List[Tuple[float, str, Pose2]] = []
        for name, candidate in candidates.items():
            x, y, yaw = candidate.pose
            distance_field = field_for(x, y, c.field_radius_m)
            if distance_field is None:
                continue
            matcher, window = self.window(candidate.range_xy_m, candidate.range_yaw_rad)
            peaks = matcher.match(distance_field, points, (float(x), float(y), float(yaw)), window)
            if peaks:
                scored.append((peaks[0].ratio, name, peaks[0].pose))
        scored.sort(key=lambda s: -s[0])
        ratios = {name: ratio for ratio, name, _ in scored}
        matched = {name: pose for _, name, pose in scored}
        if not scored:
            return MatchDecision(None, None, 'no_match', ratios=ratios, matched=matched)
        best = scored[0]
        if best[0] < c.min_ratio:
            return MatchDecision(None, None, 'low_ratio', ratios=ratios, matched=matched)
        same = [s for s in scored if not self.separated(s[2], best[2])]
        rival = next((s for s in scored if self.separated(s[2], best[2])), None)
        margin = best[0] - rival[0] if rival is not None else None
        if margin is not None and margin < c.margin:
            return MatchDecision(None, None, 'ambiguous', margin=margin, ratios=ratios, matched=matched)
        # 同じ仮説でも、一致率が最良より差の閾値以上低い候補は、入力の姿勢が山の頂上から外れている
        # (照合で範囲の端まで動いて止まった)。入力の姿勢のまま採ってよいのは、差の閾値の内側の候補だけ
        agree = [s[1] for s in same if best[0] - s[0] < c.margin]
        return MatchDecision(best[1], best[2], 'decided', same=[s[1] for s in same], agree=agree, margin=margin,
                             ratios=ratios, matched=matched)
