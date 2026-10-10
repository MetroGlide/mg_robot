"""ActivityMonitor の判定から、診断に出す内容 (レベル・メッセージ・values) を作る。ROS に依存しない。

watchdog と topic_monitor が、途絶と受信頻度を同じキー・同じメッセージで出すための共通の判定。
diagnostic_updater のタスクへの詰め替えは diagnostic_task.py にある。
"""
from dataclasses import dataclass
from typing import Dict

from mg_utils.monitoring.activity import ActivityMonitor

OK = 'ok'
WARN = 'warn'
ERROR = 'error'
FAULT_LEVELS = (WARN, ERROR)


@dataclass
class TopicHealth:
    level: str
    message: str
    values: Dict[str, str]


def evaluate_topic_health(
    monitor: ActivityMonitor,
    now: float,
    expected_hz: float = 0.0,
    hz_warn_ratio: float = 0.5,
    fault_level: str = ERROR,
) -> TopicHealth:
    """トピックの鮮度と受信頻度の診断の内容を作る。monitor の判定も更新する。

    - 途絶 (または ok=False を受信) したら fault_level (既定 error)
    - 初回のメッセージを待っている間は ok
    - 受信頻度が expected_hz × hz_warn_ratio を下回ったら warn (expected_hz が 0 なら調べない)

    values: hz、expected_hz (指定時のみ)、stale_sec。
    """
    if fault_level not in FAULT_LEVELS:
        raise ValueError(f'fault_level は {FAULT_LEVELS} のいずれか: {fault_level}')
    faulted = monitor.update(now)
    hz = monitor.hz(now)
    stale = monitor.stale_sec(now)

    if faulted:
        level = fault_level
        message = 'reports disconnected' if monitor.disconnected else f'no data for {stale:.1f} s'
    elif not monitor.received:
        level, message = OK, 'waiting for first message'
    elif expected_hz > 0.0 and hz < expected_hz * hz_warn_ratio:
        level = WARN
        message = f'low rate: {hz:.1f} Hz (expected {expected_hz:.1f})'
    else:
        level = OK
        message = f'{hz:.1f} Hz' if expected_hz > 0.0 else 'ok'

    values = {'hz': f'{hz:.2f}'}
    if expected_hz > 0.0:
        values['expected_hz'] = f'{expected_hz:.2f}'
    values['stale_sec'] = f'{stale:.1f}'
    return TopicHealth(level, message, values)
