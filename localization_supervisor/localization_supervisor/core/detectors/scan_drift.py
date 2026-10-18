"""姿勢の情報源の姿勢から、スキャンを地図にずらしたほうが合う向きと量が、続いて同じ向きか (少しずつずれる場合)。"""
import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from ..grid import best_offset, search_offsets
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind
from .common import scan_points


class ScanDrift(Detector):
    """新しいスキャンごとに、pose の姿勢の周り (±search_range_m) で、スキャンが地図に最もよく合うずらし量
    (地図の座標の dx, dy) を求め、直近 window_sec の間の並びを見る。

      drift_m     : ずらし量の平均のベクトルの長さ
      consistency : ずらし量の平均の長さ / 長さの平均 (1 に近いほど、同じ向きにそろっている)
      growth      : 平均の向きに射影したずらし量の、時間に対する傾き [m/s]

    score は drift_m · consistency / threshold_m。min_growth_m_per_s を正にすると、growth がそれ未満のときは
    増えていく傾向がないので異常としない (地図の一部の位置合わせのずれのように、場所で決まる一定のずれを除く)。
    ずらしても一致した割合の増え方が min_gain に届かないときは、ずらし量を 0 とする (既定は常に採る。
    ばらつくずらし量は平均すると打ち消し合い、consistency も下がる)。
    """
    SOURCES = {'pose': SourceKind.POSE, 'reference': SourceKind.REFERENCE}
    PARAMS = {
        'max_age_sec': ParamSpec(float, 3.0, '姿勢の標本の古さの上限'),
        'scan_max_age_sec': ParamSpec(float, 2.0),
        'radius_m': ParamSpec(float, 25.0, '距離場を作る範囲 (と、使う点の距離の上限)'),
        'tolerance_m': ParamSpec(float, 0.2),
        'search_range_m': ParamSpec(float, 1.0),
        'search_step_m': ParamSpec(float, 0.1),
        'min_gain': ParamSpec(float, 0.0, 'ずらし量を採る、一致した割合の増え方の下限'),
        'max_points': ParamSpec(int, 180),
        'min_points': ParamSpec(int, 30),
        'window_sec': ParamSpec(float, 30.0),
        'min_samples': ParamSpec(int, 10),
        'threshold_m': ParamSpec(float, 0.5),
        'min_growth_m_per_s': ParamSpec(float, 0.0, '0 なら傾きを見ない'),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self._offsets = search_offsets(self.params['search_range_m'], self.params['search_step_m'])
        self.reset()

    def reset(self) -> None:
        self._history: Deque[Tuple[float, float, float]] = deque()
        self._last_scan: Optional[float] = None

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        reference_name = self.sources['reference']
        reference = snapshot.reference(reference_name)
        scan = reference.scan
        if reference.occupancy is None or scan is None or snapshot.stamp - scan.stamp > p['scan_max_age_sec']:
            return Evidence.unknown()
        if scan.stamp != self._last_scan:
            self._last_scan = scan.stamp
            offset = self._offset(snapshot, reference_name, reference, scan.stamp)
            if offset is not None:
                self._history.append((scan.stamp, offset[0], offset[1]))
        while self._history and scan.stamp - self._history[0][0] > p['window_sec']:
            self._history.popleft()
        if len(self._history) < p['min_samples']:
            return Evidence.unknown()
        samples = np.asarray(self._history)
        offsets = samples[:, 1:3]
        mean = offsets.mean(axis=0)
        drift = float(math.hypot(mean[0], mean[1]))
        spread = float(np.hypot(offsets[:, 0], offsets[:, 1]).mean())
        consistency = drift / spread if spread > 1e-9 else 0.0
        growth = 0.0
        if drift > 1e-9:
            along = offsets @ (mean / drift)
            times = samples[:, 0] - samples[0, 0]
            if float(np.ptp(times)) > 0.0:
                growth = float(np.polyfit(times, along, 1)[0])
        score = drift * consistency / p['threshold_m']
        if p['min_growth_m_per_s'] > 0.0 and growth < p['min_growth_m_per_s']:
            score = min(score, 1.0)
        return Evidence.from_score(
            score, drift_m=drift, consistency=consistency, growth_m_per_s=growth,
            dx=float(mean[0]), dy=float(mean[1]))

    def _offset(self, snapshot, reference_name, reference, stamp) -> Optional[Tuple[float, float]]:
        p = self.params
        aligned = snapshot.pose_aligned(self.sources['pose'], p['max_age_sec'], stamp=stamp)
        if aligned is None:
            return None
        pose = aligned[0].pose
        field = reference.fields.around(reference.occupancy, pose[0], pose[1], p['radius_m'])
        if field is None:
            return None
        points = scan_points(snapshot, reference_name, reference, pose, p['max_points'], p['radius_m'])
        if points is None:
            return None
        result = best_offset(field, points, p['tolerance_m'], (pose[0], pose[1]), self._offsets,
                             min_points=p['min_points'])
        if result is None:
            return None
        base, best, (dx, dy, _) = result
        if best - base < p['min_gain']:
            return 0.0, 0.0
        return dx, dy
