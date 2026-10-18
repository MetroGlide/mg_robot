"""情報源の分散にかける倍率を出す対処。

設定の residuals の統計 (a の実際の誤差が自己申告の何倍か) から倍率を求めて、周期ごとに配信する。
受け取る側 (例: GNSS のブリッジ) が、その情報源の分散にかけて、信頼度を下げる。
blame がその情報源のときは、倍率を上限にして、ほぼ使わないようにする。
受け取る側は、一定時間届かなければ倍率を 1 に戻す (監督が止まっても、弱いままにしない)。
"""
from typing import Any, Mapping

from std_msgs.msg import Float64

from ...core.interfaces import ParamSpec
from ...core.supervisor import StepResult
from .base import Actuator


class VarianceScale(Actuator):
    PARAMS = {
        'topic': ParamSpec(str, None, '倍率を配信するトピック (std_msgs/Float64)'),
        'residual': ParamSpec(str, None, '設定の residuals の名前'),
        'blame': ParamSpec(str, '', 'この情報源が blame のとき、倍率を max_scale にする (空なら使わない)'),
        'max_scale': ParamSpec(float, 100.0),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._pub = node.create_publisher(Float64, self.params['topic'], 10)

    def apply(self, result: StepResult) -> None:
        p = self.params
        stats = result.residuals.get(p['residual'])
        scale = 1.0 if stats is None else stats.scale
        if p['blame'] and result.blame == p['blame']:
            scale = p['max_scale']
        message = Float64()
        message.data = float(min(max(scale, 1.0), p['max_scale']))
        self._pub.publish(message)
