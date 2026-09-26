"""自己位置推定の評価で注入する故障の定義 (YAML) を読み込む共通モジュール。

評価ツール (eval_localization.py) と故障注入ノード (fault_injector.py) の両方が使う。
時刻はすべて再生開始からの経過秒で表す。

例:
  faults:
    - {type: kidnap,    at: 120.0, offset: {x: 3.0, y: 0.0, yaw: 0.5}}
    - {type: gnss_bias, start: 200.0, end: 230.0, bias: {x: 2.0, y: 0.0}}
    - {type: gnss_drop, start: 300.0, end: 360.0}
    - {type: odom_scale, start: 400.0, end: 420.0, v: 1.0, w: 1.3}
    - {type: scan_drop, start: 500.0, end: 520.0, sector_deg: [-90.0, 90.0]}
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml

# 種類ごとの必須パラメータ。kidnap は瞬間的、それ以外は区間 [start, end] で作用する
REQUIRED_PARAMS: Dict[str, List[str]] = {
    "kidnap": ["offset"],
    "gnss_bias": ["bias"],
    "gnss_drop": [],
    "odom_scale": ["v", "w"],
    "scan_drop": ["sector_deg"],
}
INSTANT_TYPES = {"kidnap"}


@dataclass
class Fault:
    type: str
    start: float
    end: float
    params: Dict[str, Any] = field(default_factory=dict)

    @property
    def is_instant(self) -> bool:
        return self.type in INSTANT_TYPES


def parse_faults(data: Dict[str, Any]) -> List[Fault]:
    """辞書 ({"faults": [...]}) から Fault のリスト (開始時刻順) を作る。"""
    faults: List[Fault] = []
    for i, raw in enumerate(data.get("faults", [])):
        ftype = raw.get("type")
        if ftype not in REQUIRED_PARAMS:
            raise ValueError(f"faults[{i}]: 未知の type です: {ftype}")
        params = {k: v for k, v in raw.items() if k not in ("type", "at", "start", "end")}
        for key in REQUIRED_PARAMS[ftype]:
            if key not in params:
                raise ValueError(f"faults[{i}] ({ftype}): 必須パラメータ '{key}' がありません")
        if ftype in INSTANT_TYPES:
            if "at" not in raw:
                raise ValueError(f"faults[{i}] ({ftype}): 'at' が必要です")
            start = end = float(raw["at"])
        else:
            if "start" not in raw or "end" not in raw:
                raise ValueError(f"faults[{i}] ({ftype}): 'start' と 'end' が必要です")
            start, end = float(raw["start"]), float(raw["end"])
            if end <= start:
                raise ValueError(f"faults[{i}] ({ftype}): end は start より後にしてください")
        faults.append(Fault(type=ftype, start=start, end=end, params=params))
    faults.sort(key=lambda f: f.start)
    return faults


def load_faults(path: str) -> List[Fault]:
    with open(path, "r", encoding="utf-8") as f:
        return parse_faults(yaml.safe_load(f) or {})
