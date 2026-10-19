"""復旧の候補を評価するもの。値は 0〜1 (大きいほど良い)。"""
import math
from typing import Optional

from ..geometry import wrap_angle
from ..grid import match_ratio, transform_points
from ..interfaces import Candidate, CandidateScorer, ParamSpec, RecoveryContext, SourceKind
from ..safety import FootprintChecker
from .common import base_points, predict_from_anchor


class ScanMatch(CandidateScorer):
    """候補の姿勢でスキャンを地図に重ねたときの一致率 (占有セルから tolerance_m 以内の点の割合)。

    スキャンのバッファが min_distance_m に届いていれば、動きでまとめた直近のスキャンを使い (似た場所の
    取り違えに強い)、届かなければ最新の 1 枚を使う。
    """
    SOURCES = {'reference': SourceKind.REFERENCE, 'motion': SourceKind.MOTION}
    PARAMS = {
        'tolerance_m': ParamSpec(float, 0.3),
        'radius_m': ParamSpec(float, 25.0),
        'min_points': ParamSpec(int, 30),
        'max_points': ParamSpec(int, 400),
        'min_distance_m': ParamSpec(float, 3.0),
    }

    def score(self, candidate: Candidate, context: RecoveryContext) -> Optional[float]:
        p = self.params
        snapshot = context.snapshot
        reference = snapshot.reference(self.sources['reference'])
        if reference.occupancy is None:
            return None
        points = base_points(context, reference, self.sources['motion'], p['min_distance_m'], p['max_points'])
        if points is None:
            return None
        field = reference.fields.around(
            reference.occupancy, candidate.pose[0], candidate.pose[1], p['radius_m'])
        if field is None:
            return None
        result = match_ratio(field, transform_points(candidate.pose, points), p['tolerance_m'], p['min_points'])
        return None if result is None else result[0]


class FootprintFree(CandidateScorer):
    """候補の姿勢の footprint が、地図 (計画用地図) の占有セルにかからなければ 1、かかれば 0。
    footprint は安全の信号の設定 (safety.footprint) と同じ。"""
    SOURCES = {'reference': SourceKind.REFERENCE}
    REQUIRES = ('safety',)

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self._checker: Optional[FootprintChecker] = None

    def score(self, candidate: Candidate, context: RecoveryContext) -> Optional[float]:
        occupancy = context.snapshot.reference(self.sources['reference']).occupancy
        if occupancy is None or context.safety is None:
            return None
        if self._checker is None:
            self._checker = FootprintChecker(context.safety)
        return 0.0 if self._checker.check(occupancy, candidate.pose).in_lethal else 1.0


class AnchorAgreement(CandidateScorer):
    """候補が、疑いが始まる前のアンカーに動きを足した姿勢(推測航法の予測)と近いほど 1 に近い (max_distance_m で 0)。
    スキャンの照合が決めきれない(廊下で進行方向の位置が定まらないなど)とき、動きの事前情報で選ぶ。
    ロボットが運ばれたときは予測が外れるので、重みを小さくして、スキャンの一致を優先させる。"""
    SOURCES = {'motion': SourceKind.MOTION}
    REQUIRES = ('anchors',)
    PARAMS = {
        'max_distance_m': ParamSpec(float, 5.0),
        'yaw_weight_m': ParamSpec(float, 1.0),
    }

    def score(self, candidate: Candidate, context: RecoveryContext) -> Optional[float]:
        p = self.params
        predicted = predict_from_anchor(context, self.sources['motion'])
        if predicted is None:
            return None
        pose = predicted[0]
        distance = math.hypot(pose[0] - candidate.pose[0], pose[1] - candidate.pose[1])
        distance += p['yaw_weight_m'] * abs(wrap_angle(pose[2] - candidate.pose[2]))
        return max(0.0, 1.0 - distance / p['max_distance_m'])


class SourceAgreement(CandidateScorer):
    """候補が、指定した姿勢の情報源の今の姿勢と近いほど 1 に近い (max_distance_m で 0)。
    向きは、yaw_weight_m [m/rad] を掛けて距離に足す。"""
    SOURCES = {'pose': SourceKind.POSE}
    PARAMS = {
        'max_distance_m': ParamSpec(float, 5.0),
        'max_age_sec': ParamSpec(float, 2.0),
        'yaw_weight_m': ParamSpec(float, 1.0),
    }

    def score(self, candidate: Candidate, context: RecoveryContext) -> Optional[float]:
        p = self.params
        aligned = context.snapshot.pose_aligned(self.sources['pose'], p['max_age_sec'])
        if aligned is None:
            return None
        pose = aligned[0].pose
        distance = math.hypot(pose[0] - candidate.pose[0], pose[1] - candidate.pose[1])
        distance += p['yaw_weight_m'] * abs(wrap_angle(pose[2] - candidate.pose[2]))
        return max(0.0, 1.0 - distance / p['max_distance_m'])
