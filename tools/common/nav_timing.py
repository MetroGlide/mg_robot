"""走行の時間的な健全性の集計。ROS に依存しない。

rosbag_summary.py が、/tf と /rosout・/events から次を要約するのに使う。

- TF の打刻の差: odom→base を受け取った時点で、その打刻が最新の map→odom の打刻より何秒新しいか。
  正のとき、map で姿勢を引くノード (RPP など) は、その時刻の map→odom が届くまで待つ
  (待ちの上限は各ノードの transform_tolerance)。0 以下なら待たない。
- Nav2 の主な警告の件数 (周期の遅れ、TF の例外、BT のサービスの時間切れなど)
- シーケンサの状態 (activity) ごとの時間
"""
import re
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# (キー, 表示名, ログの本文に含まれる文字列)
NAV_WARNINGS: List[Tuple[str, str, str]] = [
    ('control_loop_missed', '制御の周期の遅れ (controller_server)', 'Control loop missed its desired rate'),
    ('controller_tf_exception', '制御の TF の例外 (その周期は速度 0)', 'Exception in transformPose'),
    ('controller_zero_poses', '近くに経路の点がない (その周期は速度 0)', 'Resulting plan has 0 poses'),
    ('controller_collision_ahead', 'RPP の前方の衝突の予測', 'detected collision ahead'),
    ('controller_patience', '制御の失敗の許容時間の超過', 'Controller patience exceeded'),
    ('planner_loop_missed', '計画の周期の遅れ (planner_server)', 'Planner loop missed its desired rate'),
]
_SERVICE_TIMEOUT = re.compile(r'timed out while executing service call to (\S+?)\.?$')
_HALT_NO_RESULT = re.compile(r'Failed to get result for (\S+) in node halt')
_BEHAVIOR_COLLISION = re.compile(r'Collision Ahead - Exiting (\w+)')

ACTIVITY_EVENT = 'sequencer.activity_changed'


def _stats(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {'count': 0}
    arr = np.asarray(values, dtype=np.float64)
    p50, p95, p99 = np.percentile(arr, [50, 95, 99])
    return {'count': int(arr.size), 'p50': float(p50), 'p95': float(p95), 'p99': float(p99),
            'max': float(arr.max()), 'positive_ratio': float(np.mean(arr > 0.0))}


class NavTimingSummary:
    def __init__(self, map_frame: str = 'map', odom_frame: str = 'odom',
                 base_frame: str = 'base_footprint') -> None:
        self._map_odom = (map_frame, odom_frame)
        self._odom_base = (odom_frame, base_frame)
        self._last_map_odom: Optional[float] = None
        self._gaps: List[float] = []
        self.warnings: Counter = Counter()
        self._activity: Optional[str] = None
        self._activity_since: Optional[int] = None
        self._activity_seconds: Counter = Counter()
        self._activity_entries: Counter = Counter()
        self._last_ns: Optional[int] = None

    def process_tf(self, parent: str, child: str, stamp_s: float) -> None:
        pair = (parent, child)
        if pair == self._map_odom:
            self._last_map_odom = stamp_s
        elif pair == self._odom_base and self._last_map_odom is not None:
            self._gaps.append(stamp_s - self._last_map_odom)

    def process_log(self, text: str) -> None:
        for key, _, needle in NAV_WARNINGS:
            if needle in text:
                self.warnings[key] += 1
                return
        for pattern, prefix in ((_SERVICE_TIMEOUT, 'service_timeout'), (_HALT_NO_RESULT, 'halt_no_result'),
                                (_BEHAVIOR_COLLISION, 'behavior_collision')):
            m = pattern.search(text)
            if m:
                self.warnings[f'{prefix}:{m.group(1)}'] += 1
                return

    def process_event(self, stamp_ns: int, name: str, fields: Dict[str, str]) -> None:
        """イベントの時刻順に渡す。/rosout と /events の重複 (同じ状態への変化) は数えない。"""
        self._last_ns = stamp_ns if self._last_ns is None else max(self._last_ns, stamp_ns)
        if name != ACTIVITY_EVENT:
            return
        to = fields.get('to')
        if not to or to == self._activity:
            return
        self._close_activity(stamp_ns)
        self._activity, self._activity_since = to, stamp_ns
        self._activity_entries[to] += 1

    def _close_activity(self, stamp_ns: int) -> None:
        if self._activity is not None and self._activity_since is not None:
            self._activity_seconds[self._activity] += max(stamp_ns - self._activity_since, 0) / 1e9

    def summary(self, end_ns: Optional[int] = None) -> Dict[str, Any]:
        end = end_ns if end_ns is not None else self._last_ns
        seconds = Counter(self._activity_seconds)
        if self._activity is not None and self._activity_since is not None and end is not None:
            seconds[self._activity] += max(end - self._activity_since, 0) / 1e9
        return {
            'tf_gap_s': _stats(self._gaps),
            'warnings': dict(self.warnings.most_common()),
            'activity': [{'name': k, 'seconds': round(v, 1), 'entries': self._activity_entries[k]}
                         for k, v in seconds.most_common()],
        }


def has_nav_timing_section(nav: Dict[str, Any]) -> bool:
    return bool(nav.get('tf_gap_s', {}).get('count') or nav.get('warnings') or nav.get('activity'))


def _warning_label(key: str) -> str:
    for k, label, _ in NAV_WARNINGS:
        if k == key:
            return label
    prefix, _, target = key.partition(':')
    return {
        'service_timeout': f'BT のサービスの時間切れ ({target})',
        'halt_no_result': f'BT の中断で結果を受け取れず ({target})',
        'behavior_collision': f'リカバリーの衝突の予測による中止 ({target})',
    }.get(prefix, key)


def nav_timing_section_lines(nav: Dict[str, Any], duration_s: float, markdown: bool) -> List[str]:
    """TF の打刻の差・Nav2 の警告・シーケンサの状態の時間 (端末と Markdown で共通)。"""
    lines: List[str] = []
    gap = nav.get('tf_gap_s') or {}
    if gap.get('count'):
        lines.append('TF の打刻の差 (odom→base の打刻 − 最新の map→odom の打刻) [s]。'
                     '正の回は、map で姿勢を引くノード (RPP など) が map→odom の到着を待つ:')
        row = (f"p50={gap['p50']:.3f} p95={gap['p95']:.3f} p99={gap['p99']:.3f} "
               f"max={gap['max']:.3f} 正の割合={gap['positive_ratio'] * 100:.1f}% (n={gap['count']})")
        lines += (['', f'- {row}', ''] if markdown else [f'  {row}', ''])

    warnings = nav.get('warnings') or {}
    if warnings:
        per_min = 60.0 / duration_s if duration_s > 0 else 0.0
        lines.append('Nav2 の主な警告:')
        if markdown:
            lines += ['', '| 警告 | 件数 | 1 分あたり |', '| :--- | ---: | ---: |']
        for key, count in warnings.items():
            if markdown:
                lines.append(f'| {_warning_label(key)} | {count} | {count * per_min:.1f} |')
            else:
                lines.append(f'  - {_warning_label(key)}: {count} 件 ({count * per_min:.1f} 件/分)')
        lines.append('')

    activity = nav.get('activity') or []
    if activity:
        lines.append('シーケンサの状態ごとの時間 (最初の状態の変化から):')
        if markdown:
            lines += ['', '| 状態 | 時間 [s] | 回数 |', '| :--- | ---: | ---: |']
        for a in activity:
            if markdown:
                lines.append(f"| `{a['name']}` | {a['seconds']:.0f} | {a['entries']} |")
            else:
                lines.append(f"  - {a['name']}: {a['seconds']:.0f} s ({a['entries']} 回)")
        lines.append('')
    return lines
