"""姿勢の情報源が自分で出す共分散の大きさ。"""
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind
from .common import COVARIANCE_METRICS, covariance_metric


class Covariance(Detector):
    """x, y の共分散の大きさ (metric) を threshold と比べる。

    自己申告の値なので、推定器が「自信を持って間違えている」状態は検知できない。
    """
    SOURCES = {'pose': SourceKind.POSE}
    PARAMS = {
        'metric': ParamSpec(str, 'trace_xy', ' | '.join(COVARIANCE_METRICS)),
        'threshold': ParamSpec(float, None),
        'max_age_sec': ParamSpec(float, 3.0),
    }

    def __init__(self, name, params, sources) -> None:
        super().__init__(name, params, sources)
        if self.params['metric'] not in COVARIANCE_METRICS:
            raise ValueError(f'{name}: metric must be one of {", ".join(COVARIANCE_METRICS)}')

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        sample = snapshot.pose(self.sources['pose'], self.params['max_age_sec'])
        if sample is None:
            return Evidence.unknown()
        value = covariance_metric(self.params['metric'], sample.cov)
        return Evidence.from_score(value / self.params['threshold'], value=value)
