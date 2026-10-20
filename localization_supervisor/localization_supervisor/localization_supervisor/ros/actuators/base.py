"""対処の約束: 監督の状態に応じて、外に働きかける (ゲートの開閉、一時停止の要求、サービスの呼び出しなど)。"""
from typing import Any, Mapping

from ...core.interfaces import ParamSpec, Plugin
from ...core.supervisor import StepResult


class Actuator(Plugin):
    """apply を実装する。node は rclpy の Node。周期ごとに、周期の結果 (状態・blame・指示) を受け取る。
    状態に応じた働きかけ (ゲート・一時停止) は、毎周期の結果から決めて、変わったときだけ送るとよい
    (失敗しても次の周期でやり直せる)。"""
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params)
        self.node = node

    def apply(self, result: StepResult) -> None:
        raise NotImplementedError
