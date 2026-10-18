"""イベントを、ノードのロガーに logfmt 書式の 1 行で出す。

    evt=<domain>.<event> key=value key="空白を含む値" ...

    LogfmtEventSink(node.get_logger()).warn('localization.state_changed', from_='NORMAL', to='SUSPECT')
"""
import re
from typing import Any

EVENT_KEY = 'evt'

_NAME_RE = re.compile(r'^[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*$')
_KEY_RE = re.compile(r'^[a-z][a-z0-9_]*$')
_NEEDS_QUOTE_RE = re.compile(r'[\s"=\\]')
_ESCAPES = {'\\': '\\\\', '"': '\\"', '\n': '\\n', '\r': '\\r', '\t': '\\t'}


def format_value(value: Any) -> str:
    """値を logfmt の値の文字列にする。空白・`"`・`=`・`\\` を含む値と空文字列は `"` で囲む。"""
    if isinstance(value, bool):
        text = 'true' if value else 'false'
    elif isinstance(value, float):
        text = format(value, '.6g')
    else:
        text = str(value)
    if text == '' or _NEEDS_QUOTE_RE.search(text):
        return '"' + ''.join(_ESCAPES.get(c, c) for c in text) + '"'
    return text


def format_event(name: str, /, **fields: Any) -> str:
    """`evt=<name> key=value ...` の本文を作る。

    name は `<domain>.<event>` (小文字・数字・`_`)。キーの末尾の `_` は取り除く (`from_` → `from`)。
    値が None のフィールドは出さない。
    """
    if not _NAME_RE.match(name):
        raise ValueError(f'イベント名は <domain>.<event> の形にする: {name!r}')
    parts = [f'{EVENT_KEY}={name}']
    for key, value in fields.items():
        key = key.rstrip('_')
        if not _KEY_RE.match(key):
            raise ValueError(f'イベントのキーは小文字・数字・_ だけにする: {key!r}')
        if key == EVENT_KEY:
            raise ValueError(f'イベントのキーに {EVENT_KEY!r} は使えない')
        if value is None:
            continue
        parts.append(f'{key}={format_value(value)}')
    return ' '.join(parts)


class LogfmtEventSink:
    """ロガーにイベントを出す。logger は rclpy のロガー (`node.get_logger()`)。"""

    def __init__(self, logger: Any) -> None:
        self._logger = logger

    def info(self, name: str, /, **fields: Any) -> None:
        self._logger.info(format_event(name, **fields))

    def warn(self, name: str, /, **fields: Any) -> None:
        self._logger.warning(format_event(name, **fields))

    def error(self, name: str, /, **fields: Any) -> None:
        self._logger.error(format_event(name, **fields))
