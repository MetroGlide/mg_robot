"""AMCL の出力を EKF に入れるかどうかの調停 (ROS に依存しない)。

要求元 (ウェイポイント、監督ノードなど) ごとに「入れてよいか」を持ち、
すべての要求元が入れてよいときだけ AMCL の出力を通す。
"""

from typing import Dict, Optional


class GateArbiter:
    def __init__(self, requesters):
        self._wants: Dict[str, bool] = {name: True for name in requesters}
        self._applied: Optional[bool] = None

    def request(self, requester: str, attach: bool) -> None:
        if requester not in self._wants:
            raise KeyError(f'unknown requester: {requester}')
        self._wants[requester] = bool(attach)

    @property
    def desired(self) -> bool:
        return all(self._wants.values())

    @property
    def holders(self):
        """AMCL の出力を止めている要求元。"""
        return sorted(name for name, attach in self._wants.items() if not attach)

    def needs_apply(self) -> bool:
        """ゲートにまだ反映していない (または反映に失敗した) 値があるか。"""
        return self._applied != self.desired

    def mark_applied(self, attach: bool) -> None:
        self._applied = attach

    def mark_unknown(self) -> None:
        """ゲートのノードが再起動したかもしれないときなど、反映済みかどうか分からなくする。"""
        self._applied = None
