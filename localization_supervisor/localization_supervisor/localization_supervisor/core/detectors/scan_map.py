"""姿勢の情報源の姿勢で、スキャンを地図に重ねたときの一致。"""
from ..grid import best_offset, search_offsets
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind, Status
from .common import Baseline, scan_points

# 一致率が 0 のときに score が無限大にならないための下限
MIN_RATIO = 1e-3


class ScanMap(Detector):
    """pose の姿勢 (スキャンの時刻に移したもの) でスキャンを地図に重ね、占有セルから tolerance_m 以内の点の
    割合 (一致率) を見る。次のどちらかに当てはまれば異常。

      - 一致率が、正常なときの中央値の ratio_factor 倍 (下限 ratio_min) 未満
      - 姿勢の周り (±search_range_m) でずらしたほうが、一致率が gain_threshold 以上増える

    一致率の絶対値は地図の質やセンサの特性で決まるので、正常なとき (状態が NORMAL で、この判定も正常) の
    値を基準にする。最初の baseline_min_samples 個までは下限だけを使う。
    """
    SOURCES = {'pose': SourceKind.POSE, 'reference': SourceKind.REFERENCE}
    PARAMS = {
        'max_age_sec': ParamSpec(float, 3.0, '姿勢の標本の古さの上限'),
        'scan_max_age_sec': ParamSpec(float, 2.0),
        'radius_m': ParamSpec(float, 25.0, '距離場を作る範囲 (と、使う点の距離の上限)'),
        'tolerance_m': ParamSpec(float, 0.3),
        'ratio_min': ParamSpec(float, 0.1),
        'ratio_factor': ParamSpec(float, 0.5),
        'baseline_min_samples': ParamSpec(int, 20),
        'baseline_window': ParamSpec(int, 120),
        'gain_threshold': ParamSpec(float, 1.0, '1.0 以上なら、利得では異常としない'),
        'search_range_m': ParamSpec(float, 1.0),
        'search_step_m': ParamSpec(float, 0.25),
        'max_points': ParamSpec(int, 180),
        'min_points': ParamSpec(int, 30),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self._offsets = search_offsets(self.params['search_range_m'], self.params['search_step_m'])
        self.reset()

    def reset(self) -> None:
        self._ratios = Baseline(self.params['baseline_window'], self.params['baseline_min_samples'])

    def ratio_threshold(self) -> float:
        p = self.params
        baseline = self._ratios.median()
        return p['ratio_min'] if baseline is None else max(p['ratio_min'], p['ratio_factor'] * baseline)

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        reference_name = self.sources['reference']
        reference = snapshot.reference(reference_name)
        scan = reference.scan
        if reference.occupancy is None or scan is None or snapshot.stamp - scan.stamp > p['scan_max_age_sec']:
            return Evidence.unknown()
        aligned = snapshot.pose_aligned(self.sources['pose'], p['max_age_sec'], stamp=scan.stamp)
        if aligned is None:
            return Evidence.unknown()
        pose = aligned[0].pose
        field = reference.fields.around(reference.occupancy, pose[0], pose[1], p['radius_m'])
        if field is None:
            return Evidence.unknown()
        points = scan_points(snapshot, reference_name, reference, pose, p['max_points'], p['radius_m'])
        if points is None:
            return Evidence.unknown()
        result = best_offset(
            field, points, p['tolerance_m'], (pose[0], pose[1]), self._offsets, min_points=p['min_points'])
        if result is None:
            return Evidence.unknown()
        ratio, gain = result.base, result.gain
        threshold = self.ratio_threshold()
        score = max(threshold / max(ratio, MIN_RATIO), gain / p['gain_threshold'])
        return Evidence.from_score(score, ratio=ratio, gain=gain, ratio_threshold=threshold)

    def after_step(self, evidence: Evidence, normal: bool) -> None:
        if normal and evidence.status == Status.NORMAL:
            self._ratios.add(evidence.values['ratio'])
