"""イベントの出力の約束: 名前 (`<domain>.<event>`)・レベル・値を書き出す。"""
from typing import Any, Mapping

from ...core.interfaces import ParamSpec, Plugin

LEVELS = ('info', 'warn', 'error')


class EventSink(Plugin):
    """emit を実装する。logger はノードのロガー (`node.get_logger()`)。"""
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(self, name: str, params: Mapping[str, Any], logger: Any) -> None:
        super().__init__(name, params)
        self.logger = logger

    def emit(self, name: str, level: str, fields: Mapping[str, Any]) -> None:
        """level は info / warn / error。fields のキーはイベントの書式に従う。"""
        raise NotImplementedError

    def info(self, name: str, /, **fields: Any) -> None:
        self.emit(name, 'info', fields)

    def warn(self, name: str, /, **fields: Any) -> None:
        self.emit(name, 'warn', fields)

    def error(self, name: str, /, **fields: Any) -> None:
        self.emit(name, 'error', fields)
