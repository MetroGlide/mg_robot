"""自己位置推定の監視ノードが出す診断 `localization` の内容。

監視ノードの種類によらず、診断が同じ形になるようにする。ROS に依存しない。
"""
from typing import Dict, Tuple

from .state_machine import State

LEVEL_BY_STATE = {
    State.NORMAL: 'ok',
    State.DEGRADED: 'error',
}


def localization_health(
    state: int, attached: bool, last_event: str, values: Dict[str, float],
) -> Tuple[str, str, Dict[str, str]]:
    """(レベル 'ok' / 'warn' / 'error', メッセージ, values) を返す。

    NORMAL は ok、DEGRADED (復旧できていない) は error、それ以外 (確認中・切り離し・復旧中) は warn。
    メッセージは状態の名前。values の判定値は、使えなかったもの (負の値) を出さない。
    """
    level = LEVEL_BY_STATE.get(State(state), 'warn')
    out = {'amcl_attached': str(attached), 'last_event': last_event}
    for key, value in values.items():
        if value >= 0.0:
            out[key] = f'{value:.3f}'
    return level, State(state).name, out
