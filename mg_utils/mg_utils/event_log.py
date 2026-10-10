"""イベントログ。ノードのロガーに logfmt 書式の 1 行を出す。書式の規約は doc/monitoring.md。

    evt=<domain>.<event> key=value key="空白を含む値" ...

    EventLog(node.get_logger()).info('sequencer.state_changed', from_='IDLE', to='NAVIGATING')

書式を作る側 (format_event) と読む側 (parse_event) を同じモジュールに置き、規則を 1 か所にする。
"""
import re
from typing import Any, Dict, Optional, Tuple

EVENT_KEY = 'evt'

_NAME_RE = re.compile(r'^[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*$')
_KEY_RE = re.compile(r'^[a-z][a-z0-9_]*$')
_NEEDS_QUOTE_RE = re.compile(r'[\s"=\\]')
_ESCAPES = {'\\': '\\\\', '"': '\\"', '\n': '\\n', '\r': '\\r', '\t': '\\t'}
_UNESCAPES = {'\\': '\\', '"': '"', 'n': '\n', 'r': '\r', 't': '\t'}
_PAIR_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=("(?:[^"\\]|\\.)*"|\S*)')


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

    name は `<domain>.<event>` (小文字・数字・`_`)。位置専用の引数なので、`name=` のフィールドも渡せる。
    キーの末尾の `_` は取り除く
    (`from_` → `from`。Python の予約語を避けるため)。値が None のフィールドは出さない。
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


def parse_event(message: str) -> Optional[Tuple[str, Dict[str, str]]]:
    """メッセージの本文がイベントなら (名前, フィールド) を返す。イベントでなければ None。

    イベントは、本文の先頭が `evt=` で始まるもの。値は文字列のまま返す。
    """
    if not message.startswith(f'{EVENT_KEY}='):
        return None
    fields: Dict[str, str] = {}
    for match in _PAIR_RE.finditer(message):
        key, raw = match.groups()
        if raw.startswith('"'):
            raw = _unquote(raw[1:-1])
        fields[key] = raw
    name = fields.pop(EVENT_KEY)
    return name, fields


def _unquote(body: str) -> str:
    out = []
    i = 0
    while i < len(body):
        c = body[i]
        if c == '\\' and i + 1 < len(body):
            out.append(_UNESCAPES.get(body[i + 1], body[i + 1]))
            i += 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


class EventLog:
    """ロガーにイベントを出す。logger は rclpy のロガー (`node.get_logger()`)。"""

    def __init__(self, logger: Any) -> None:
        self._logger = logger

    def info(self, name: str, /, **fields: Any) -> None:
        self._logger.info(format_event(name, **fields))

    def warn(self, name: str, /, **fields: Any) -> None:
        self._logger.warning(format_event(name, **fields))

    def error(self, name: str, /, **fields: Any) -> None:
        self._logger.error(format_event(name, **fields))
