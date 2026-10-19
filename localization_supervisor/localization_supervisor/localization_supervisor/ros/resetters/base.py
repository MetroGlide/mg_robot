"""再初期化の送り先の約束: 選んだ姿勢を、推定器に送る。"""
from typing import Any, Mapping

from ...core.interfaces import Candidate, ParamSpec, Plugin
from ..own_messages import OwnMessages


class PoseResetter(Plugin):
    """reset を実装する。node は rclpy の Node。own に、送ったメッセージの目印を入れる
    (監督が自分の送ったものを、外からの再初期化と取り違えないため)。"""
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(self, name: str, params: Mapping[str, Any], node: Any, own: OwnMessages) -> None:
        super().__init__(name, params)
        self.node = node
        self.own = own

    def reset(self, candidate: Candidate) -> None:
        """candidate の姿勢と共分散で推定器を初期化し直す。"""
        raise NotImplementedError
