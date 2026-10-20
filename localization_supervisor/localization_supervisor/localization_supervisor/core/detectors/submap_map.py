"""直近の一定距離のスキャンを動きで 1 枚の局所地図にまとめ、今の姿勢で地図に照合する。"""
from typing import Optional

from ..grid import best_offset, search_offsets
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind, Status
from ..scan_buffer import ScanBuffer
from .common import Baseline

# 一致率が 0 のときに score が無限大にならないための下限
MIN_RATIO = 1e-3


class SubmapMap(Detector):
    """スキャンを、動きの情報源 (motion) の姿勢とともに間引いて残し (ScanBuffer)、interval_sec ごとに、
    残したスキャンを pose の姿勢 (最新のスキャンの時刻に移したもの) で地図に重ねる。次のどちらかなら異常。

      - 一致率が、正常なときの中央値の ratio_factor 倍 (下限 ratio_min) 未満
      - 姿勢の周り (±search_range_m、±search_yaw_rad) でずらしたほうが、一致率が gain_threshold 以上増える

    1 枚のスキャンより広い範囲を見るので、似た場所の取り違え (推定器が自信を持って間違える状態) に強い。
    残したスキャンの走行距離が min_distance_m に届くまでと、照合しない周期は、判定できない。
    動きの情報源の誤差 (すべり) も局所地図を崩すので、疑いは pose だけにかける。
    """
    SOURCES = {'pose': SourceKind.POSE, 'motion': SourceKind.MOTION, 'reference': SourceKind.REFERENCE}
    SUBJECTS = ('pose',)
    PARAMS = {
        'max_age_sec': ParamSpec(float, 3.0, '姿勢の標本の古さの上限'),
        'interval_sec': ParamSpec(float, 5.0, '照合する間隔'),
        'keyframe_distance_m': ParamSpec(float, 0.5),
        'keyframe_angle_rad': ParamSpec(float, 0.2),
        'window_distance_m': ParamSpec(float, 15.0, '残すスキャンの走行距離'),
        'min_distance_m': ParamSpec(float, 5.0, '照合に要る走行距離'),
        'max_points_per_scan': ParamSpec(int, 90),
        'radius_m': ParamSpec(float, 25.0, '距離場を作る範囲 (と、使う点の距離の上限)'),
        'tolerance_m': ParamSpec(float, 0.3),
        'ratio_min': ParamSpec(float, 0.1),
        'ratio_factor': ParamSpec(float, 0.5),
        'baseline_min_samples': ParamSpec(int, 10),
        'baseline_window': ParamSpec(int, 60),
        'gain_threshold': ParamSpec(float, 1.0, '1.0 以上なら、利得では異常としない'),
        'search_range_m': ParamSpec(float, 1.0),
        'search_step_m': ParamSpec(float, 0.25),
        'search_yaw_rad': ParamSpec(float, 0.05),
        'search_yaw_step_rad': ParamSpec(float, 0.025),
        'min_points': ParamSpec(int, 100),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        p = self.params
        self._offsets = search_offsets(p['search_range_m'], p['search_step_m'])
        steps = int(round(p['search_yaw_rad'] / p['search_yaw_step_rad'])) if p['search_yaw_step_rad'] > 0 else 0
        self._yaws = [k * p['search_yaw_step_rad'] for k in range(-steps, steps + 1)]
        self.buffer = ScanBuffer(p['keyframe_distance_m'], p['keyframe_angle_rad'], p['window_distance_m'],
                                 p['max_points_per_scan'], p['radius_m'])
        self.reset()

    def reset(self) -> None:
        self.buffer.clear()
        self._ratios = Baseline(self.params['baseline_window'], self.params['baseline_min_samples'])
        self._last_scan: Optional[float] = None
        self._last_match: Optional[float] = None

    def ratio_threshold(self) -> float:
        p = self.params
        baseline = self._ratios.median()
        return p['ratio_min'] if baseline is None else max(p['ratio_min'], p['ratio_factor'] * baseline)

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        reference = snapshot.reference(self.sources['reference'])
        scan = reference.scan
        if scan is not None and scan.stamp != self._last_scan:
            self._last_scan = scan.stamp
            motion = snapshot.motion_at(self.sources['motion'], scan.stamp)
            if motion is not None:
                self.buffer.add(scan, motion)
        latest = self.buffer.latest()
        if (reference.occupancy is None or latest is None or self.buffer.travelled() < p['min_distance_m']
                or snapshot.stamp - latest.stamp > p['interval_sec']):
            return Evidence.unknown()
        if self._last_match is not None and snapshot.stamp - self._last_match < p['interval_sec']:
            return Evidence.unknown()
        aligned = snapshot.pose_aligned(self.sources['pose'], p['max_age_sec'], stamp=latest.stamp)
        if aligned is None:
            return Evidence.unknown()
        pose = aligned[0].pose
        field = reference.fields.around(reference.occupancy, pose[0], pose[1], p['radius_m'])
        if field is None:
            return Evidence.unknown()
        points = self.buffer.points_in(pose, latest.motion)
        result = best_offset(field, points, p['tolerance_m'], (pose[0], pose[1]), self._offsets, self._yaws,
                             p['min_points'])
        if result is None:
            return Evidence.unknown()
        self._last_match = snapshot.stamp
        ratio, gain = result.base, result.gain
        dx, dy, dyaw = result.shift
        threshold = self.ratio_threshold()
        score = max(threshold / max(ratio, MIN_RATIO), gain / p['gain_threshold'])
        return Evidence.from_score(
            score, ratio=ratio, gain=gain, ratio_threshold=threshold, dx=dx, dy=dy, dyaw=dyaw,
            points=float(points.shape[0]), distance_m=float(self.buffer.travelled()))

    def after_step(self, evidence: Evidence, normal: bool) -> None:
        if normal and evidence.status == Status.NORMAL:
            self._ratios.add(evidence.values['ratio'])
