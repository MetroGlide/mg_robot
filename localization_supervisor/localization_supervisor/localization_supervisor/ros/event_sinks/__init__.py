"""イベントの出力。設定の events.sink の type に、登録名か `パッケージ.モジュール:クラス` を書く。"""
from typing import Any, Mapping

from ...core.config import ConfigError
from ...core.plugins import PluginError, resolve, resolve_params
from .base import EventSink
from .logfmt import LogfmtEventSink

STANDARD_EVENT_SINKS = {
    'logfmt': LogfmtEventSink,
}

__all__ = ['EventSink', 'LogfmtEventSink', 'STANDARD_EVENT_SINKS', 'build_event_sink']


def build_event_sink(spec: Mapping[str, Any], logger: Any) -> EventSink:
    """events.sink の辞書 ({type, その他はパラメータ}) から作る。合わなければ ConfigError。"""
    raw = dict(spec)
    type_name = raw.pop('type')
    try:
        cls = resolve(type_name, STANDARD_EVENT_SINKS, EventSink, 'events.sink')
        params = resolve_params(cls, raw, 'events.sink')
    except PluginError as error:
        raise ConfigError(str(error)) from error
    return cls(type_name, params, logger)
