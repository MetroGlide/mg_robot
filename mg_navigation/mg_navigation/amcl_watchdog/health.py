"""amcl_watchdog_node の状態を、監督ノード (localization_supervisor) と同じ状態の名前に対応づける。"""
from localization_supervisor.core.state_machine import State


def watchdog_state(in_recovery: bool, recovery_failed: bool, consecutive_anomalies: int) -> State:
    """watchdog の内部状態から、配信する状態を決める。

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
