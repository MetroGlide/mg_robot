"""状態 (StatusView) を、diagnostic_updater のタスクとして出す。"""
from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask

from ..core.status import StatusView, localization_health

_LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class LocalizationTask(DiagnosticTask):
    """診断 (名前は name)。snapshot は直近の状態を返す (まだ無ければ None)。"""

    def __init__(self, name: str, snapshot: Callable[[], Optional[StatusView]]) -> None:
        super().__init__(name)
        self._snapshot = snapshot

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        view = self._snapshot()
        if view is None:
            stat.summary(DiagnosticStatus.WARN, 'no status yet')
            return stat
        level, message, values = localization_health(view)
        stat.summary(_LEVELS[level], message)
        for key, value in values.items():
            stat.add(key, value)
        return stat
