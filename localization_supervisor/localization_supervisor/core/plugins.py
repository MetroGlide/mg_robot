"""プラグインの読み込み。

標準のものは登録名 (例: pose_consistency) で、外部のものは `パッケージ.モジュール:クラス` で指定する。
"""
import importlib
from typing import Any, Dict, Mapping, Type, TypeVar

from .interfaces import ParamSpec, Plugin

P = TypeVar('P', bound=type)


class PluginError(ValueError):
    pass


def resolve(type_name: str, registry: Mapping[str, P], base: type, where: str) -> P:
    """登録名、または `module:Class` からクラスを得る。base の派生でなければ PluginError。"""
    if ':' in type_name:
        module_name, _, class_name = type_name.partition(':')
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as error:
            raise PluginError(f'{where}: cannot import {module_name} ({error})') from error
        cls = getattr(module, class_name, None)
        if cls is None:
            raise PluginError(f'{where}: {module_name} has no {class_name}')
    else:
        cls = registry.get(type_name)
        if cls is None:
            raise PluginError(f'{where}: unknown type {type_name!r} (standard: {", ".join(sorted(registry))})')
    if not (isinstance(cls, type) and issubclass(cls, base)):
        raise PluginError(f'{where}: {type_name} is not a {base.__name__}')
    return cls


def _matches(spec: ParamSpec, value: Any) -> bool:
    if spec.type is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if spec.type is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if spec.type is list:
        return isinstance(value, (list, tuple))
    return isinstance(value, spec.type)


def resolve_params(cls: Type[Plugin], raw: Mapping[str, Any], where: str) -> Dict[str, Any]:
    """宣言 (cls.PARAMS) にそって、型と必須を検証し、既定値を補う。宣言にない名前は誤り。"""
    raw = dict(raw or {})
    unknown = sorted(set(raw) - set(cls.PARAMS))
    if unknown:
        raise PluginError(f'{where}: unknown params {unknown} (known: {sorted(cls.PARAMS)})')
    params: Dict[str, Any] = {}
    for name, spec in cls.PARAMS.items():
        if name in raw:
            value = raw[name]
            if not _matches(spec, value):
                raise PluginError(f'{where}.{name}: expected {spec.type.__name__}, got {value!r}')
            params[name] = float(value) if spec.type is float else value
        elif spec.default is None:
            raise PluginError(f'{where}.{name}: required')
        else:
            params[name] = spec.default
    return params
