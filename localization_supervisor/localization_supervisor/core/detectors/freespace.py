"""姿勢の情報源の姿勢で、スキャンの光線が地図の占有セルを突き抜けているか (見えるはずの壁が見えない)。"""
from ..grid import ray_penetration
from ..geometry import compose
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind, Status
from .common import Baseline, scan_points


class Freespace(Detector):
    """pose の姿勢 (スキャンの時刻に移したもの) でスキャンを地図に重ね、センサから端点までの光線が、
    占有セルを penetration_m より長く通っている光線の割合 (突き抜けの割合) を見る。地図の壁は 1〜2 セルの
    厚さしかないことが多いので、penetration_m は小さくする (既定は占有セルを 1 つでも通れば突き抜け)。

    人や車などの地図に無いものは光線を短くするだけなので、突き抜けにはならない。姿勢がずれると、
    地図の壁の向こうに端点が来るので、突き抜けが増える。端点の近く (margin_m) は数えない。
    割合が、ratio_max と、正常なときの中央値の ratio_factor 倍の大きいほうを超えたら異常。
    """
    SOURCES = {'pose': SourceKind.POSE, 'reference': SourceKind.REFERENCE}
    PARAMS = {
        'max_age_sec': ParamSpec(float, 3.0, '姿勢の標本の古さの上限'),
        'scan_max_age_sec': ParamSpec(float, 2.0),
        'max_range_m': ParamSpec(float, 25.0, 'これより遠い点は使わない'),
        'max_points': ParamSpec(int, 180),
        'min_points': ParamSpec(int, 30),
        'margin_m': ParamSpec(float, 0.3, '端点からこの距離の手前までを調べる'),
        'penetration_m': ParamSpec(float, 0.0, '占有セルをこの長さより長く通る光線を突き抜けとする'),
        'ratio_max': ParamSpec(float, 0.1),
        'ratio_factor': ParamSpec(float, 3.0),
        'baseline_min_samples': ParamSpec(int, 20),
        'baseline_window': ParamSpec(int, 120),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        self.reset()

    def reset(self) -> None:
        self._ratios = Baseline(self.params['baseline_window'], self.params['baseline_min_samples'])

    def ratio_threshold(self) -> float:
        p = self.params
        baseline = self._ratios.median()
        return p['ratio_max'] if baseline is None else max(p['ratio_max'], p['ratio_factor'] * baseline)

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        reference_name = self.sources['reference']
        reference = snapshot.reference(reference_name)
        scan = reference.scan
        if (reference.occupancy is None or scan is None or scan.sensor_pose is None
                or snapshot.stamp - scan.stamp > p['scan_max_age_sec']):
            return Evidence.unknown()
        aligned = snapshot.pose_aligned(self.sources['pose'], p['max_age_sec'], stamp=scan.stamp)
        if aligned is None:
            return Evidence.unknown()
        pose = aligned[0].pose
        points = scan_points(snapshot, reference_name, reference, pose, p['max_points'], p['max_range_m'])
        if points is None or points.shape[0] < p['min_points']:
            return Evidence.unknown()
        sensor = compose(pose, scan.sensor_pose)
        penetration = ray_penetration(reference.occupancy, (sensor[0], sensor[1]), points, p['margin_m'])
        ratio = float((penetration > p['penetration_m']).mean())
        threshold = self.ratio_threshold()
        return Evidence.from_score(ratio / threshold, ratio=ratio, ratio_threshold=threshold)

    def after_step(self, evidence: Evidence, normal: bool) -> None:
        if normal and evidence.status == Status.NORMAL:
            self._ratios.add(evidence.values['ratio'])
