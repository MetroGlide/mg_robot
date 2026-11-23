"""一時停止の要求の出力の約束: 走行を止めてほしいかを、走行の側 (シーケンサなど) に伝える。"""
from typing import Any, Mapping

from ...core.interfaces import ParamSpec, Plugin


class HoldOutput(Plugin):
    """apply を実装する。node は rclpy の Node。周期ごとに呼ばれるので、後から起動した受け手にも届くよう、
    止めてほしい間は出し続けるとよい。"""
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params)
        self.node = node

    def apply(self, active: bool, reason: str) -> None:
        raise NotImplementedError
