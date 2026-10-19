"""情報源の分散にかける倍率を出す対処。

監督が求めた倍率 (中核の StepResult.applied_scales) を、周期ごとに配信する。倍率は、設定の residuals の統計
(a の実際の誤差が自己申告の何倍か) から求め、a の情報源を切り離しているあいだは上限になる。
受け取る側 (例: GNSS のブリッジ) が、その情報源の分散にかけて、信頼度を下げる。
受け取る側は、一定時間届かなければ倍率を 1 に戻す (監督が止まっても、弱いままにしない)。
"""
from typing import Any, Collection, Dict, Iterable, Mapping

from std_msgs.msg import Float64

from ...core.interfaces import ParamSpec
from ...core.supervisor import StepResult
from .base import Actuator


class VarianceScale(Actuator):
    PARAMS = {
        'topic': ParamSpec(str, None, '倍率を配信するトピック (std_msgs/Float64)'),
        'residual': ParamSpec(str, None, '設定の residuals の名前'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._pub = node.create_publisher(Float64, self.params['topic'], 10)

    def apply(self, result: StepResult) -> None:
        self._publish(result.applied_scales.get(self.params['residual'], 1.0))

    def release(self) -> None:
        self._publish(1.0)

    def _publish(self, scale: float) -> None:
        message = Float64()
        message.data = float(scale)
        self._pub.publish(message)


def delivered_scales(
    actuators: Iterable[Actuator], disabled: Collection[str], scales: Mapping[str, float],
) -> Dict[str, float]:
    """受け取る側に実際に届く倍率。対処 variance_scale が無い、またはプロファイルで無効なら、倍率は届かない (1)。
    割り戻しは、届いた倍率だけで行う。"""
    active = {a.params['residual'] for a in actuators if isinstance(a, VarianceScale) and a.name not in disabled}
    return {name: (scale if name in active else 1.0) for name, scale in scales.items()}
