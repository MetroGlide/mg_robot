"""dataclass 定義に基づく厳格な YAML パーサ。

未知のキー・型不一致・必須項目の欠落はすべて ScenarioValidationError とする。
プラグインは dataclass を定義するだけで同じ検証を受けられる。
"""
from __future__ import annotations

import dataclasses
import typing
from dataclasses import dataclass
from typing import Any, Dict, List, Union

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.registry import Registry, TypeEntry

_NONE_TYPE = type(None)


@dataclass
class Call:
    """登録済みの型の呼び出し (例: `- spawn: {obstacle: box}`)。"""
    KIND = ""
    name: str
    spec: Any
    entry: TypeEntry
    where: str


class ActionCall(Call):
    KIND = "action"


class TriggerCall(Call):
    KIND = "trigger"


class ExpectationCall(Call):
    KIND = "expectation"


class MonitorCall(Call):
    KIND = "monitor"


class DriverCall(Call):
    KIND = "driver"


def parse_spec(cls: type, raw: Any, where: str, registry: Registry) -> Any:
    """raw (YAML 由来の値) を dataclass cls に変換する。"""
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ScenarioValidationError(f"{where}: mapping expected, got {raw!r}")
    hints = typing.get_type_hints(cls)
    fields = {f.name: f for f in dataclasses.fields(cls) if f.init}
    unknown = sorted(set(raw) - set(fields))
    if unknown:
        raise ScenarioValidationError(
            f"{where}: unknown key(s) {unknown} (allowed: {sorted(fields)})")

    kwargs = {}
    for name, field in fields.items():
        if name in raw:
            kwargs[name] = _convert(hints[name], raw[name], f"{where}.{name}", registry)
        elif (field.default is dataclasses.MISSING
              and field.default_factory is dataclasses.MISSING):
            raise ScenarioValidationError(f"{where}: '{name}' is required")
    obj = cls(**kwargs)
    validate = getattr(obj, "validate", None)
    if callable(validate):
        validate(where)
    return obj


def parse_call(call_cls: type, raw: Any, where: str, registry: Registry) -> Call:
    """`{type_name: params}` または `type_name` を Call に変換する。"""
    if isinstance(raw, str):
        name, params = raw, None
    elif isinstance(raw, dict) and len(raw) == 1:
        name, params = next(iter(raw.items()))
    else:
        raise ScenarioValidationError(
            f"{where}: expected '{{<{call_cls.KIND}_type>: {{...}}}}', got {raw!r}")
    entry = registry.get(call_cls.KIND, str(name), where)
    where = f"{where}.{name}"
    spec = parse_spec(entry.spec_cls, params, where, registry) if entry.spec_cls else None
    if entry.spec_cls is None and params not in (None, {}):
        raise ScenarioValidationError(f"{where}: takes no parameters")
    return call_cls(name=str(name), spec=spec, entry=entry, where=where)


def _convert(tp: Any, value: Any, where: str, registry: Registry) -> Any:
    origin = typing.get_origin(tp)
    args = typing.get_args(tp)

    if tp is Any:
        return value
    if origin is Union:
        non_none = [a for a in args if a is not _NONE_TYPE]
        if value is None and len(non_none) < len(args):
            return None
        if len(non_none) == 1:
            return _convert(non_none[0], value, where, registry)
        raise ScenarioValidationError(f"{where}: unsupported union type {tp}")
    if origin is typing.Literal:
        if value not in args:
            raise ScenarioValidationError(
                f"{where}: must be one of {list(args)}, got {value!r}")
        return value
    if origin in (list, List):
        if value is None:
            return []
        if not isinstance(value, list):
            raise ScenarioValidationError(f"{where}: list expected, got {value!r}")
        return [_convert(args[0], v, f"{where}[{i}]", registry)
                for i, v in enumerate(value)]
    if origin in (dict, Dict):
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise ScenarioValidationError(f"{where}: mapping expected, got {value!r}")
        key_tp, val_tp = args
        return {
            _convert(key_tp, k, f"{where}.<key {k!r}>", registry):
                _convert(val_tp, v, f"{where}.{k}", registry)
            for k, v in value.items()
        }
    if isinstance(tp, type) and issubclass(tp, Call):
        return parse_call(tp, value, where, registry)
    if dataclasses.is_dataclass(tp):
        return parse_spec(tp, value, where, registry)
    if tp is bool:
        if not isinstance(value, bool):
            raise ScenarioValidationError(f"{where}: bool expected, got {value!r}")
        return value
    if tp is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ScenarioValidationError(f"{where}: int expected, got {value!r}")
        return value
    if tp is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ScenarioValidationError(f"{where}: number expected, got {value!r}")
        return float(value)
    if tp is str:
        if not isinstance(value, str):
            raise ScenarioValidationError(f"{where}: string expected, got {value!r}")
        return value
    raise ScenarioValidationError(f"{where}: unsupported field type {tp}")
