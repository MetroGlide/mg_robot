"""診断のレベルの変化を検出する。ROS に依存しない。

event_relay_node が、集約された診断 (/diagnostics_agg) の変化を health.level_changed のイベントにする。
"""
from dataclasses import dataclass
from typing import Dict, Optional

LEVEL_NAMES = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
NONE = 'NONE'


def level_name(level: int) -> str:
    return LEVEL_NAMES.get(level, str(level))


@dataclass(frozen=True)
class LevelChange:
    name: str
    from_level: str
    to_level: str
    message: str


class LevelChangeTracker:
    """名前ごとのレベルを覚え、変わったときに LevelChange を返す。

    - 最初の startup_grace_sec のあいだは、覚えるだけで変化として返さない (スタックの起動中の
      ERROR や STALE をイベントにしない)
    - 猶予のあと初めて見た名前は、OK 以外のときだけ from=NONE の変化として返す
    """

    def __init__(self, start_time: float, startup_grace_sec: float) -> None:
        self._grace_until = start_time + startup_grace_sec
        self._levels: Dict[str, int] = {}

    def update(self, now: float, name: str, level: int, message: str) -> Optional[LevelChange]:
        previous = self._levels.get(name)
        self._levels[name] = level
        if now < self._grace_until:
            return None
        if previous is None:
            if level == 0:
                return None
            return LevelChange(name, NONE, level_name(level), message)
        if previous == level:
            return None
        return LevelChange(name, level_name(previous), level_name(level), message)
