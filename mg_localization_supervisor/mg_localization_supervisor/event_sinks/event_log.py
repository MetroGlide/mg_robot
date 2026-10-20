"""監督のイベントを、このプロジェクトのイベントログ (mg_utils.event_log、doc/monitoring.md の規約) で出す。

設定 (supervision.yaml) の events.sink に `mg_localization_supervisor.event_sinks.event_log:EventLogSink` と書く。
"""
from typing import Any, Mapping

from localization_supervisor.ros.event_sinks import EventSink
from mg_utils.event_log import EventLog


class EventLogSink(EventSink):
    def __init__(self, name: str, params: Mapping[str, Any], logger: Any) -> None:
        super().__init__(name, params, logger)
        self._log = EventLog(logger)

    def emit(self, name: str, level: str, fields: Mapping[str, Any]) -> None:
        if level == 'error':
            self._log.error(name, **fields)
        elif level == 'warn':
            self._log.warn(name, **fields)
        else:
            self._log.info(name, **fields)
