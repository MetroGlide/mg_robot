"""イベントログ (evt=<domain>.<event> key=value ...) を集めて、時刻順に並べる。ROS に依存しない。

入力は次の 3 種類。どれも、1 行の本文に `[LEVEL] [<epoch>] [<ノード名>]: evt=...` を含む。
  - make log-collector が書き出したログ: `<ISO 時刻> <本文>` (~/ros2_data/logs/<サービス>/<日付>.log)
  - make logs-export のログ: `<コンテナ名>  | <ISO 時刻> <本文>` (docker compose logs --timestamps)
  - rosbag の /rosout・/events (rcl_interfaces/Log)

並べる時刻は、本文の epoch (ノードが記録した時刻) を優先する。無ければ行頭の ISO 時刻を使う。
"""
import csv
import io
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone, tzinfo
from typing import Dict, Iterable, List, Optional
from zoneinfo import ZoneInfo

from mg_utils.event_log import parse_event

LEVELS = ('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL')
# rcl_interfaces/Log の level
ROS_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

_ISO_PREFIX_RE = re.compile(
    r'^(?:\S+\s+\|\s+)?'
    r'(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})?)\s+(.*)$')
_ROS_CONSOLE_RE = re.compile(
    r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\]\s+\[(\d+(?:\.\d+)?)\]\s+\[([^\]]*)\]:\s+(.*)$')


@dataclass(frozen=True)
class EventRecord:
    time: datetime
    level: str
    node: str
    name: str
    fields: Dict[str, str] = field(default_factory=dict)
    source: str = ''

    @property
    def domain(self) -> str:
        return self.name.split('.', 1)[0]


def parse_iso(text: str) -> datetime:
    """ISO 8601 の時刻。タイムゾーンが無ければ UTC とみなす。"""
    if text.endswith('Z'):
        text = text[:-1] + '+00:00'
    value = datetime.fromisoformat(_trim_fraction(text))
    return value if value.tzinfo is not None else value.replace(tzinfo=timezone.utc)


def _trim_fraction(text: str) -> str:
    """fromisoformat (Python 3.10) が受け付ける 6 桁以内の小数にする (docker は 9 桁を出す)。"""
    return re.sub(r'(\.\d{6})\d+', r'\1', text)


def _record(time: datetime, level: str, node: str, body: str, source: str) -> Optional[EventRecord]:
    parsed = parse_event(body)
    if parsed is None:
        return None
    name, fields = parsed
    return EventRecord(time, level, node, name, fields, source)


def parse_log_line(line: str, source: str = '') -> Optional[EventRecord]:
    """ログの 1 行がイベントなら EventRecord を返す。イベントでなければ None。"""
    line = line.rstrip('\n')
    iso_time = None
    match = _ISO_PREFIX_RE.match(line)
    if match:
        iso_time, line = parse_iso(match.group(1)), match.group(2)
    console = _ROS_CONSOLE_RE.match(line)
    if console:
        level, epoch, node, body = console.groups()
        time = datetime.fromtimestamp(float(epoch), tz=timezone.utc)
    else:
        level, node, body, time = 'INFO', '', line, iso_time
    if time is None:
        return None
    return _record(time, level, node, body, source)


def parse_ros_log(stamp_sec: float, level: int, node: str, msg: str, source: str = '') -> Optional[EventRecord]:
    """rcl_interfaces/Log の 1 件。"""
    return _record(
        datetime.fromtimestamp(stamp_sec, tz=timezone.utc), ROS_LEVELS.get(level, str(level)),
        node, msg, source)


def parse_lines(lines: Iterable[str], source: str = '') -> List[EventRecord]:
    records = []
    for line in lines:
        record = parse_log_line(line, source)
        if record is not None:
            records.append(record)
    return records


def merge(records: Iterable[EventRecord]) -> List[EventRecord]:
    """時刻順に並べ、重複 (同じ時刻・ノード・本文) を 1 つにする。

    同じイベントが、docker のログと /events の両方に入っていても、1 回だけ出る。
    """
    seen = set()
    out = []
    for record in sorted(records, key=lambda r: r.time):
        key = (record.time, record.node, record.name, tuple(sorted(record.fields.items())))
        if key in seen:
            continue
        seen.add(key)
        out.append(record)
    return out


def filter_records(
    records: Iterable[EventRecord],
    since: Optional[datetime] = None,
    until: Optional[datetime] = None,
    min_level: Optional[str] = None,
    domains: Optional[Iterable[str]] = None,
    sources: Optional[Iterable[str]] = None,
) -> List[EventRecord]:
    domain_set = set(domains) if domains else None
    source_set = set(sources) if sources else None
    min_rank = LEVELS.index(min_level) if min_level else 0
    out = []
    for record in records:
        if since is not None and record.time < since:
            continue
        if until is not None and record.time > until:
            continue
        # 知らないレベルの文字列は、絞り込みで落とさない
        rank = LEVELS.index(record.level) if record.level in LEVELS else len(LEVELS)
        if rank < min_rank:
            continue
        if domain_set is not None and record.domain not in domain_set:
            continue
        if source_set is not None and record.source not in source_set:
            continue
        out.append(record)
    return out


def resolve_tz(text: Optional[str]) -> Optional[tzinfo]:
    """`UTC`、`+09:00`、`Asia/Tokyo` のような指定。None なら実行環境のローカル。"""
    if not text or text == 'local':
        return None
    if text.upper() == 'UTC':
        return timezone.utc
    match = re.fullmatch(r'([+-])(\d{2}):?(\d{2})', text)
    if match:
        delta = timedelta(hours=int(match.group(2)), minutes=int(match.group(3)))
        return timezone(delta if match.group(1) == '+' else -delta)
    return ZoneInfo(text)


def format_time(time: datetime, tz: Optional[tzinfo]) -> str:
    return time.astimezone(tz).isoformat(timespec='milliseconds')


def _fields_text(fields: Dict[str, str]) -> str:
    return ' '.join(f'{k}={v}' for k, v in fields.items())


def to_markdown(records: List[EventRecord], tz: Optional[tzinfo] = None) -> str:
    lines = ['| 時刻 | レベル | ソース | ノード | イベント | 内容 |', '| :--- | :--- | :--- | :--- | :--- | :--- |']
    for r in records:
        content = _fields_text(r.fields).replace('|', '\\|')
        lines.append(
            f'| {format_time(r.time, tz)} | {r.level} | {r.source} | {r.node} | {r.name} | {content} |')
    return '\n'.join(lines) + '\n'


def to_csv(records: List[EventRecord], tz: Optional[tzinfo] = None) -> str:
    out = io.StringIO()
    writer = csv.writer(out, lineterminator='\n')
    writer.writerow(['time', 'level', 'source', 'node', 'event', 'fields'])
    for r in records:
        writer.writerow([format_time(r.time, tz), r.level, r.source, r.node, r.name, _fields_text(r.fields)])
    return out.getvalue()
