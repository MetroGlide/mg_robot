"""evaluate_topic_health の結果を、diagnostic_updater のタスクとして出す。"""
from typing import Callable, Dict, Optional

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask

from mg_utils.monitoring.activity import ActivityMonitor
from mg_utils.monitoring.health import ERROR, evaluate_topic_health

_LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class TopicHealthTask(DiagnosticTask):
    """トピックの鮮度と受信頻度の診断。判定の内容は evaluate_topic_health にある。

    clock は monitor に渡す時刻と同じ時計 (time.monotonic)。
    hardware_id を指定すると、Updater 全体の hardware_id の代わりに、この診断の hardware_id にする
    (機器ごとの診断を、1 つのノードから出すため)。
    extra_values で、ノード固有の values (再起動の回数など) を足せる。
    """

    def __init__(
        self,
        name: str,
        monitor: ActivityMonitor,
        clock: Callable[[], float],
        expected_hz: float = 0.0,
        hz_warn_ratio: float = 0.5,
        fault_level: str = ERROR,
        hardware_id: str = '',
        extra_values: Optional[Callable[[], Dict[str, str]]] = None,
    ) -> None:
        super().__init__(name)
        self._monitor = monitor
        self._clock = clock
        self._expected_hz = expected_hz
        self._hz_warn_ratio = hz_warn_ratio
        self._fault_level = fault_level
        self._hardware_id = hardware_id
        self._extra_values = extra_values

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        health = evaluate_topic_health(
            self._monitor, self._clock(), self._expected_hz, self._hz_warn_ratio,
            self._fault_level)
        if self._hardware_id:
            stat.hardware_id = self._hardware_id
        stat.summary(_LEVELS[health.level], health.message)
        for key, value in health.values.items():
            stat.add(key, value)
        if self._extra_values is not None:
            for key, value in self._extra_values().items():
                stat.add(key, value)
        return stat
