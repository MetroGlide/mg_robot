"""2 つの姿勢の情報源の位置の食い違い。"""
import math

import numpy as np

from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind
from .common import position_std


class PoseConsistency(Detector):
    """a と b の位置を共通の時刻に移して比べる。

    metric が mahalanobis のときは、両方の共分散と extra_std_m を足した 2 自由度のマハラノビス距離の二乗、
    distance のときは位置の差 [m] を threshold と比べる。
    標準偏差が a_max_std_m / b_max_std_m を超える標本は使わない (0 は制限なし)。
    """
    SOURCES = {'a': SourceKind.POSE, 'b': SourceKind.POSE}
    PARAMS = {
        'metric': ParamSpec(str, 'mahalanobis', 'mahalanobis | distance'),
        'threshold': ParamSpec(float, None, 'これを超えたら異常 (mahalanobis は d^2、distance は m)'),
        'max_age_sec': ParamSpec(float, 3.0, 'これより古い標本は使わない'),
        'a_max_std_m': ParamSpec(float, 0.0),
        'b_max_std_m': ParamSpec(float, 0.0),
        'extra_std_m': ParamSpec(float, 0.0, '時刻のずれなどによる位置の不確かさ'),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        if self.params['metric'] not in ('mahalanobis', 'distance'):
            raise ValueError(f'{name}: metric must be mahalanobis or distance')

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        a = snapshot.pose_aligned(self.sources['a'], p['max_age_sec'])
        b = snapshot.pose_aligned(self.sources['b'], p['max_age_sec'])
        if a is None or b is None:
            return Evidence.unknown()
        a_sample, b_sample = a[0], b[0]
        for sample, limit in ((a_sample, p['a_max_std_m']), (b_sample, p['b_max_std_m'])):
            if limit > 0.0 and position_std(sample.cov) > limit:
                return Evidence.unknown()
        d = np.array([a_sample.pose[0] - b_sample.pose[0], a_sample.pose[1] - b_sample.pose[1]])
        distance = float(math.hypot(d[0], d[1]))
        if p['metric'] == 'distance':
            return Evidence.from_score(distance / p['threshold'], value=distance, distance=distance)
        cov = (np.asarray(a_sample.cov, dtype=float)[:2, :2] + np.asarray(b_sample.cov, dtype=float)[:2, :2]
               + np.eye(2) * (p['extra_std_m'] ** 2 + 1e-6))
        d2 = float(d @ np.linalg.solve(cov, d))
        return Evidence.from_score(d2 / p['threshold'], value=d2, distance=distance)
