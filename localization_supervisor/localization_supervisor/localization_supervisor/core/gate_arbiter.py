"""ゲートの開閉の調停 (ROS に依存しない)。

要求元 (走行の手順、監督ノードなど) ごとに「開けてよいか」を持ち、
すべての要求元が開けてよいときだけゲートを開く。
"""

from typing import Dict, List, Optional, Tuple


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
        """ゲートを閉じている要求元。"""
        return sorted(name for name, attach in self._wants.items() if not attach)

    @property
    def applied(self) -> bool:
        """要求元の意図がゲートに反映済みか。反映前・失敗後・不明のときは False。"""
        return self._applied is not None and self._applied == self.desired

    def needs_apply(self) -> bool:
        """ゲートにまだ反映していない (または反映に失敗した) 値があるか。"""
        return self._applied != self.desired

    def mark_applied(self, attach: bool) -> None:
        self._applied = attach

    def mark_unknown(self) -> None:
        """ゲートのノードが再起動したかもしれないときなど、反映済みかどうか分からなくする。"""
        self._applied = None


def gate_health(
    desired: bool, applied: bool, holders: List[str],
) -> Tuple[str, str, Dict[str, str]]:
    """診断 `gate` の内容 (レベル 'ok' / 'warn'、メッセージ、values)。

    - ゲートに反映済みなら ok (止めている要求元がいても、意図どおりなので ok)
    - 要求がゲートに反映できていない (ゲートのノードがいない、失敗した) あいだは warn
    """
    state = 'attached' if desired else f'detached (held by: {", ".join(holders)})'
    values = {'desired': str(desired), 'applied': str(applied), 'holders': ','.join(holders)}
    if applied:
        return 'ok', state, values
    return 'warn', f'not applied: {state}', values
