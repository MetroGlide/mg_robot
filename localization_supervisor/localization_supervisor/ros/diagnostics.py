"""localization_health の結果を、diagnostic_updater のタスクとして出す。"""
from typing import Callable, Dict, Tuple

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask

from ..core.health import localization_health

_LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class LocalizationTask(DiagnosticTask):
    """診断 `localization`。snapshot は (状態, AMCL を EKF に入れているか, 直近のできごと, 判定値) を返す。"""

    def __init__(self, snapshot: Callable[[], Tuple[int, bool, str, Dict[str, float]]]) -> None:
        super().__init__('localization')
        self._snapshot = snapshot

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        level, message, values = localization_health(*self._snapshot())
        stat.summary(_LEVELS[level], message)
        for key, value in values.items():
            stat.add(key, value)
        return stat
