"""姿勢の仮説の集合が割れているか。"""
from ..geometry import separated
from ..interfaces import Detector, Evidence, ParamSpec, SnapshotLike, SourceKind


class HypothesisSplit(Detector):
    """最も重い仮説から離れた仮説のうち、最も重いものの重みを threshold と比べる。

    推定器 (AMCL など) の粒子が、離れた姿勢の 2 つ以上の仮説に割れているときに反応する。割れていても、
    推定器が正しい方を選んでいることは多いので、異常の証拠としては弱い。疑義は推定器の姿勢 (pose) にかける。
    """
    SOURCES = {'pose': SourceKind.POSE, 'hypotheses': SourceKind.HYPOTHESES}
    SUBJECTS = ('pose',)
    PARAMS = {
        'threshold': ParamSpec(float, 0.2, '離れた仮説の重みがこれを超えたら異常'),
        'separation_m': ParamSpec(float, 1.0),
        'separation_rad': ParamSpec(float, 0.5),
        'max_age_sec': ParamSpec(float, 3.0),
    }

    def evaluate(self, snapshot: SnapshotLike) -> Evidence:
        p = self.params
        sample = snapshot.hypotheses(self.sources['hypotheses'], p['max_age_sec'])
        if sample is None or not sample.hypotheses:
            return Evidence.unknown()
        top = sample.hypotheses[0]
        second = max((h.weight for h in sample.hypotheses[1:]
                      if separated(h.pose, top.pose, p['separation_m'], p['separation_rad'])), default=0.0)
        return Evidence.from_score(second / p['threshold'], second_weight=second, top_weight=top.weight,
                                   modes=float(len(sample.hypotheses)))
