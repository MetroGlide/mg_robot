"""シナリオで使える型 (action / trigger / driver 等) の登録簿。

プラグインはモジュールの import 時にデコレータで型を登録する。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from sim_scenario_test.errors import ScenarioValidationError

KINDS = (
    "action",
    "trigger",
    "expectation",
    "monitor",
    "driver",
    "backend",
    "waypoint_format",
)


@dataclass(frozen=True)
class TypeEntry:
    kind: str
    name: str
    spec_cls: Optional[type]
    impl: Any
    doc: str


class Registry:
    def __init__(self):
        self._entries: Dict[str, Dict[str, TypeEntry]] = {k: {} for k in KINDS}

    def register(self, kind: str, name: str, spec_cls: Optional[type], impl: Any) -> None:
        if kind not in self._entries:
            raise ValueError(f"unknown kind '{kind}'")
        if name in self._entries[kind]:
            raise ValueError(f"{kind} '{name}' is already registered")
        doc = (getattr(impl, "__doc__", None) or "").strip().splitlines()
        self._entries[kind][name] = TypeEntry(
            kind, name, spec_cls, impl, doc[0] if doc else "")

    def get(self, kind: str, name: str, where: str) -> TypeEntry:
        entry = self._entries[kind].get(name)
        if entry is None:
            raise ScenarioValidationError(
                f"{where}: unknown {kind} '{name}' (available: {self.names(kind)})")
        return entry

    def names(self, kind: str) -> List[str]:
        return sorted(self._entries[kind])

    def entries(self, kind: str) -> List[TypeEntry]:
        return [self._entries[kind][n] for n in self.names(kind)]


DEFAULT_REGISTRY = Registry()


def _decorator(kind: str, name: str, spec: Optional[type]) -> Callable:
    def wrap(impl):
        DEFAULT_REGISTRY.register(kind, name, spec, impl)
        return impl
    return wrap


def register_action(name: str, spec: Optional[type] = None) -> Callable:
    """関数 fn(ctx, spec, stop_event) を action として登録する。"""
    return _decorator("action", name, spec)


def register_trigger(name: str, spec: Optional[type] = None) -> Callable:
    """クラス cls(ctx, spec) (poll() -> bool を持つ) を trigger として登録する。"""
    return _decorator("trigger", name, spec)


def register_expectation(name: str, spec: Optional[type] = None) -> Callable:
    """関数 fn(ctx, spec, outcome) -> CheckResult を expectation として登録する。"""
    return _decorator("expectation", name, spec)


def register_monitor(name: str, spec: Optional[type] = None) -> Callable:
    """クラス cls(ctx, spec) (start/stop/result を持つ) を monitor として登録する。"""
    return _decorator("monitor", name, spec)


def register_driver(name: str, spec: Optional[type] = None) -> Callable:
    """RunDriver のサブクラスを走行ドライバとして登録する。"""
    return _decorator("driver", name, spec)


def register_backend(name: str) -> Callable:
    """SimulationBackend のサブクラスをシミュレータバックエンドとして登録する。"""
    return _decorator("backend", name, None)


def register_waypoint_format(name: str) -> Callable:
    """関数 fn(path) -> List[IndexedPose] を waypoint ファイル形式として登録する。"""
    return _decorator("waypoint_format", name, None)
