"""復旧の候補を作るもの。"""
import math
import time
from typing import List

import numpy as np

from ..interfaces import (
    Candidate, CandidateGenerator, ParamSpec, RecoveryContext, SearchWindow, SourceKind)
from ..matcher.correlative import window_size
from .common import base_points, grow_covariance, predict_from_anchor


class PoseSource(CandidateGenerator):
    """指定した姿勢の情報源の今の姿勢を候補にする。

    位置の標準偏差が max_std_m 以下の標本が consecutive 個続くまでは作らない (精度の良い測位が続くのを待つ)。
    向きの標準偏差が yaw_std_rad より大きいとき (向きを持たない測位) は、yaw_std_rad にする。
    """
    SOURCES = {'pose': SourceKind.POSE}
    PARAMS = {
        'max_std_m': ParamSpec(float, 1.0),
        'consecutive': ParamSpec(int, 2),
        'max_age_sec': ParamSpec(float, 2.0),
        'min_std_m': ParamSpec(float, 0.3),
        'yaw_std_rad': ParamSpec(float, 0.5),
    }

    def generate(self, context: RecoveryContext) -> List[Candidate]:
        p = self.params
        name = self.sources['pose']
        snapshot = context.snapshot
        aligned = snapshot.pose_aligned(name, p['max_age_sec'])
        if aligned is None:
            return []
        recent = snapshot.pose_history(name).samples()[-p['consecutive']:]
        if len(recent) < p['consecutive']:
            return []
        for sample in recent:
            if math.sqrt(max(float(sample.cov[0, 0]), float(sample.cov[1, 1]), 0.0)) > p['max_std_m']:
                return []
        sample = aligned[0]
        cov = np.array(sample.cov, dtype=float, copy=True)
        cov[0, 0] = max(cov[0, 0], p['min_std_m'] ** 2)
        cov[1, 1] = max(cov[1, 1], p['min_std_m'] ** 2)
        cov[2, 2] = min(cov[2, 2], p['yaw_std_rad'] ** 2) if cov[2, 2] > 0.0 else p['yaw_std_rad'] ** 2
        return [Candidate(sample.pose, cov, self.name)]


class AnchorMotion(CandidateGenerator):
    """疑いが始まる前のアンカーに、その後の動きを足した姿勢を候補にする (推測航法)。

    動きの誤差は、動いた距離に比例して共分散に足す。アンカーがなければ作らない。
    """
    SOURCES = {'motion': SourceKind.MOTION}
    REQUIRES = ('anchors',)
    PARAMS = {
        'std_per_m': ParamSpec(float, 0.05),
        'std_per_rad': ParamSpec(float, 0.05),
        'min_std_m': ParamSpec(float, 0.3),
        'min_std_yaw_rad': ParamSpec(float, 0.05),
    }

    def generate(self, context: RecoveryContext) -> List[Candidate]:
        p = self.params
        predicted = predict_from_anchor(context, self.sources['motion'])
        if predicted is None:
            return []
        pose, cov, distance = predicted
        cov = grow_covariance(cov, distance, 0.0, p['std_per_m'], p['std_per_rad'], p['min_std_m'],
                              p['min_std_yaw_rad'])
        return [Candidate(pose, cov, self.name, info={'distance_m': distance})]


class SubmapMatch(CandidateGenerator):
    """アンカーに動きを足した姿勢の周りで、直近のスキャン (動きでまとめた局所地図) を地図に照合し、
    一致の良い互いに離れた姿勢を最大 peaks 個、候補にする。

    探す範囲は、予測の共分散の sigmas 倍 (min_range_m〜max_range_m)。スキャンのバッファが
    min_distance_m に届かなければ、最新の 1 枚で照合する。
    """
    SOURCES = {'motion': SourceKind.MOTION, 'reference': SourceKind.REFERENCE}
    REQUIRES = ('anchors',)
    PARAMS = {
        'peaks': ParamSpec(int, 3),
        'sigmas': ParamSpec(float, 3.0),
        'min_range_m': ParamSpec(float, 1.0),
        'max_range_m': ParamSpec(float, 6.0),
        'step_m': ParamSpec(float, 0.3),
        'max_yaw_rad': ParamSpec(float, 0.3),
        'yaw_step_rad': ParamSpec(float, 0.05),
        'min_distance_m': ParamSpec(float, 3.0),
        'max_points': ParamSpec(int, 400),
        'radius_m': ParamSpec(float, 25.0),
        'std_per_m': ParamSpec(float, 0.05),
        'std_per_rad': ParamSpec(float, 0.05),
        'result_std_m': ParamSpec(float, 0.3),
        'result_std_yaw_rad': ParamSpec(float, 0.1),
    }

    def generate(self, context: RecoveryContext) -> List[Candidate]:
        p = self.params
        if context.matcher is None:
            return []
        predicted = predict_from_anchor(context, self.sources['motion'])
        reference = context.snapshot.reference(self.sources['reference'])
        if predicted is None or reference.occupancy is None:
            return []
        center, cov, distance = predicted
        cov = grow_covariance(cov, distance, 0.0, p['std_per_m'], p['std_per_rad'], 0.0, 0.0)
        points = base_points(context, reference, self.sources['motion'], p['min_distance_m'], p['max_points'])
        if points is None:
            return []
        field = reference.fields.around(reference.occupancy, center[0], center[1], p['radius_m'])
        if field is None:
            return []
        range_xy, range_yaw = window_size(cov, p['sigmas'], p['min_range_m'], p['max_range_m'])
        window = SearchWindow(range_xy, p['step_m'], min(range_yaw, p['max_yaw_rad']), p['yaw_step_rad'])
        started = time.monotonic()
        peaks = context.matcher.match(field, points, center, window, p['peaks'])
        elapsed = time.monotonic() - started
        result_cov = np.diag([p['result_std_m'] ** 2, p['result_std_m'] ** 2, p['result_std_yaw_rad'] ** 2])
        return [Candidate(
            peak.pose, result_cov.copy(), self.name,
            info={'ratio': peak.ratio, 'rank': float(rank), 'search_m': range_xy, 'elapsed_sec': elapsed,
                  'points': float(points.shape[0])})
            for rank, peak in enumerate(peaks)]
