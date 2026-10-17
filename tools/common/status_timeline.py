"""診断のレベルの遷移と、イベントログの集計。ROS に依存しない。

rosbag_summary.py が、/diagnostics_agg (または /diagnostics) と /rosout・/events から、
「いつ、何が、どのレベルになったか」を要約するのに使う。
"""
import datetime
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

LEVEL_NAMES = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def level_value(level: Any) -> int:
    """DiagnosticStatus.level は、Python では長さ 1 の bytes。int ならそのまま。"""
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, 'little')
    return int(level)


def level_name(level: int) -> str:
    return LEVEL_NAMES.get(level, str(level))


class LevelTimeline:
    """名前ごとのレベルの遷移と、各レベルにいた時間を数える。

    遷移の一覧には、診断ごとに最初の max_per_name 回だけを残す (1 秒ごとに行き来する診断で、
    他の診断の遷移が埋もれないように)。回数は by_name の transitions に全部数える。
    """

    def __init__(self, max_transitions: int = 200, max_per_name: int = 5) -> None:
        self._max_transitions = max_transitions
        self._max_per_name = max_per_name
        self._counts: Dict[str, int] = {}
        self._last: Dict[str, Tuple[int, int]] = {}      # name -> (stamp_ns, level)
        self._seconds: Dict[str, Counter] = {}            # name -> {level: seconds}
        self._worst: Dict[str, int] = {}
        self.transitions: List[Dict[str, Any]] = []
        self.dropped_transitions = 0

    def update(self, stamp_ns: int, name: str, level: int, message: str = '') -> None:
        previous = self._last.get(name)
        self._worst[name] = max(self._worst.get(name, 0), level)
        if previous is not None:
            self._seconds.setdefault(name, Counter())[previous[1]] += (stamp_ns - previous[0]) / 1e9
            if previous[1] != level:
                self._add_transition(stamp_ns, name, previous[1], level, message)
        self._last[name] = (stamp_ns, level)

    def _add_transition(self, stamp_ns: int, name: str, before: int, after: int, message: str) -> None:
        count = self._counts.get(name, 0) + 1
        self._counts[name] = count
        if count > self._max_per_name or len(self.transitions) >= self._max_transitions:
            self.dropped_transitions += 1
            return
        self.transitions.append({
            'stamp_ns': stamp_ns, 'name': name, 'from': level_name(before),
            'to': level_name(after), 'message': message})

    def summary(self) -> Dict[str, Any]:
        by_name = {}
        for name, (_, final) in self._last.items():
            seconds = self._seconds.get(name, Counter())
            by_name[name] = {
                'final': level_name(final),
                'worst': level_name(self._worst[name]),
                'transitions': self._counts.get(name, 0),
                'warn_s': round(float(seconds[1]), 1),
                'error_s': round(float(seconds[2]), 1),
                'stale_s': round(float(seconds[3]), 1),
            }
        return {
            'transitions': self.transitions,
            'dropped_transitions': self.dropped_transitions,
            'by_name': by_name,
        }


class EventSummary:
    """イベント (evt=...) の数と、WARN 以上のイベントの一覧。/rosout と /events の重複は 1 回にする。"""

    def __init__(self, max_notable: int = 100) -> None:
        self._max_notable = max_notable
        self._seen = set()
        self.counts: Counter = Counter()
        self.notable: List[Dict[str, Any]] = []
        self.dropped_notable = 0

    def add(self, stamp_ns: int, level: str, node: str, name: str, fields: Dict[str, str]) -> None:
        key = (stamp_ns, node, name, tuple(sorted(fields.items())))
        if key in self._seen:
            return
        self._seen.add(key)
        self.counts[name] += 1
        if level in ('WARN', 'ERROR', 'FATAL'):
            if len(self.notable) >= self._max_notable:
                self.dropped_notable += 1
                return
            self.notable.append({
                'stamp_ns': stamp_ns, 'level': level, 'node': node, 'name': name, 'fields': fields})

    def summary(self) -> Dict[str, Any]:
        return {
            'counts': dict(self.counts.most_common()),
            'notable': self.notable,
            'dropped_notable': self.dropped_notable,
        }


def pick_timeline(agg: Optional[LevelTimeline], raw: Optional[LevelTimeline]) -> Optional[LevelTimeline]:
    """集約した診断 (/diagnostics_agg) があればそれを、無ければ生の診断 (/diagnostics) を使う。"""
    if agg is not None and agg.summary()['by_name']:
        return agg
    if raw is not None and raw.summary()['by_name']:
        return raw
    return None


def clock_text(stamp_ns: int) -> str:
    """bag の時刻を、ローカル時刻の時分秒に直す。"""
    return datetime.datetime.fromtimestamp(stamp_ns / 1e9).strftime("%H:%M:%S.%f")[:-3]


def fields_text(fields: Dict[str, str]) -> str:
    return " ".join(f"{k}={v}" for k, v in fields.items())


def has_status_section(diag: Dict[str, Any]) -> bool:
    """状態とイベントの節に出す内容があるか (レベルの遷移か、イベントがある)。"""
    timeline = diag.get("timeline") or {}
    events = diag.get("events") or {}
    return bool(timeline.get("transitions") or events.get("counts"))


def status_section_lines(diag: Dict[str, Any], markdown: bool) -> List[str]:
    """診断のレベルの遷移と、イベントログの集計 (端末と Markdown で共通)。"""
    lines: List[str] = []
    timeline = diag.get("timeline") or {}
    events = diag.get("events") or {}

    # OK 以外になったことのある診断
    bad = {n: v for n, v in (timeline.get("by_name") or {}).items() if v["worst"] != "OK"}
    if bad:
        source = diag.get("timeline_source")
        lines.append(f"診断 ({source}) のうち、OK 以外になったもの:")
        if markdown:
            lines += ["", "| 診断 | 最後 | 最悪 | 変化 [回] | WARN [s] | ERROR [s] | STALE [s] |", "| :--- | :---: | :---: | ---: | :---: | :---: | :---: |"]
        for name, v in sorted(bad.items(), key=lambda kv: -(kv[1]["error_s"] + kv[1]["stale_s"]))[:20]:
            if markdown:
                lines.append(
                    f"| `{name}` | {v['final']} | {v['worst']} | {v['transitions']} | "
                    f"{v['warn_s']} | {v['error_s']} | {v['stale_s']} |")
            else:
                lines.append(
                    f"  - {name}: 最後={v['final']} 最悪={v['worst']} 変化={v['transitions']}回 "
                    f"WARN={v['warn_s']}s ERROR={v['error_s']}s STALE={v['stale_s']}s")
        lines.append("")

    transitions = timeline.get("transitions") or []
    if transitions:
        lines.append("レベルの変化 (古い順。診断ごとに最初の数回だけ。回数は上の一覧):")
        if markdown:
            lines += ["", "| 時刻 | 診断 | 変化 | メッセージ |", "| :--- | :--- | :--- | :--- |"]
        for t in transitions[:30]:
            change = f"{t['from']} → {t['to']}"
            if markdown:
                lines.append(f"| {clock_text(t['stamp_ns'])} | `{t['name']}` | {change} | {t['message']} |")
            else:
                lines.append(f"  - {clock_text(t['stamp_ns'])} {t['name']}: {change} {t['message']}".rstrip())
        omitted = len(transitions) - 30 + timeline.get("dropped_transitions", 0)
        if omitted > 0:
            lines.append(f"  ...ほか {omitted} 件")
        lines.append("")

    counts = events.get("counts") or {}
    if counts:
        lines.append("イベントの数 (evt=...):")
        top = list(counts.items())[:20]
        if markdown:
            lines += ["", "| イベント | 件数 |", "| :--- | ---: |"] + [f"| `{n}` | {c} |" for n, c in top]
        else:
            lines.append("  " + " | ".join(f"{n}: {c}" for n, c in top))
        lines.append("")

    notable = events.get("notable") or []
    if notable:
        lines.append("WARN 以上のイベント (古い順):")
        if markdown:
            lines += ["", "| 時刻 | レベル | ノード | イベント | 内容 |", "| :--- | :---: | :--- | :--- | :--- |"]
        for e in notable[:30]:
            if markdown:
                content = fields_text(e["fields"]).replace("|", "\\|")
                lines.append(f"| {clock_text(e['stamp_ns'])} | {e['level']} | {e['node']} | `{e['name']}` | {content} |")
            else:
                lines.append(
                    f"  - {clock_text(e['stamp_ns'])} [{e['level']}] {e['node']} {e['name']} {fields_text(e['fields'])}")
        omitted = len(notable) - 30 + events.get("dropped_notable", 0)
        if omitted > 0:
            lines.append(f"  ...ほか {omitted} 件")
        lines.append("")
    return lines
