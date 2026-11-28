"""活動の code ごとの状態ランプの意味 (config/activity_lamps.yaml) の読み込みと検査。ROS に依存しない。

ランプの意味は、人がどうすればよいか (response) と、動いているか (motion) の 2 つ。
Web UI と MMI は、この 2 つから自分の見た目 (色・点灯 / 点滅) を決める。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable

import yaml

# 値は mg_msgs/msg/NavigationActivity.msg の LAMP_* の定数と合わせる
RESPONSES = frozenset(('normal', 'watch', 'act', 'operator', 'start', 'none'))
MOTIONS = frozenset(('moving', 'waiting', 'handling'))


@dataclass(frozen=True)
class LampMeaning:
    response: str
    motion: str
    description: str


def parse_lamp_table(raw: object, codes: Iterable[str]) -> Dict[str, LampMeaning]:
    """YAML を読んだ値から表を作る。codes (活動の code の全部) と食い違えば ValueError。"""
    if not isinstance(raw, dict) or not isinstance(raw.get('lamps'), dict):
        raise ValueError('activity_lamps: lamps の辞書がない')
    table: Dict[str, LampMeaning] = {}
    for code, entry in raw['lamps'].items():
        if not isinstance(entry, dict):
            raise ValueError(f'activity_lamps: {code} が辞書でない')
        response = entry.get('response')
        motion = entry.get('motion')
        if response not in RESPONSES:
            raise ValueError(f'activity_lamps: {code} の response が不明: {response!r}')
        if motion not in MOTIONS:
            raise ValueError(f'activity_lamps: {code} の motion が不明: {motion!r}')
        table[str(code)] = LampMeaning(response, motion, str(entry.get('description', '')))
    expected = set(codes)
    if set(table) != expected:
        raise ValueError(
            f'activity_lamps: 活動の code と食い違う: 表にない {sorted(expected - set(table))}, '
            f'活動にない {sorted(set(table) - expected)}')
    return table


def load_lamp_table(path: str, codes: Iterable[str]) -> Dict[str, LampMeaning]:
    with open(path, encoding='utf-8') as f:
        return parse_lamp_table(yaml.safe_load(f), codes)
