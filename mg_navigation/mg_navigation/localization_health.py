"""自己位置推定の監視ノード (localization_supervisor と amcl_watchdog_node) が出す診断の内容。

どちらのノードが動いていても、診断 `<ノード名>: localization` が同じ形になるようにする。ROS に依存しない。
"""
from typing import Dict, Tuple

from mg_navigation.localization_supervisor.state_machine import State

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


def watchdog_state(in_recovery: bool, recovery_failed: bool, consecutive_anomalies: int) -> State:
    """amcl_watchdog_node の状態を、監督ノードと同じ状態の名前に対応づける。

    - 再初期化中は RECOVERING
    - 再初期化に失敗し、まだ戻っていなければ DEGRADED
    - 異常を検知し始めた (まだ再初期化に至らない) なら SUSPECT
    - それ以外は NORMAL
    """
    if in_recovery:
        return State.RECOVERING
    if recovery_failed:
        return State.DEGRADED
    if consecutive_anomalies > 0:
        return State.SUSPECT
    return State.NORMAL
