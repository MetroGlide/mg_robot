"""入力のアダプタの組み立て: 設定の sources.<名前>.input からアダプタを作る。"""
from typing import Dict

from ...core.config import ConfigError, SupervisorConfig
from ...core.plugins import PluginError, resolve, resolve_params
from ...core.snapshot import SampleStore
from .base import AdapterContext, SourceAdapter, Subscription
from .standard import STANDARD_ADAPTERS

__all__ = ['AdapterContext', 'SourceAdapter', 'Subscription', 'STANDARD_ADAPTERS', 'build_adapters']


def build_adapters(
    config: SupervisorConfig, store: SampleStore, context: AdapterContext,
) -> Dict[str, SourceAdapter]:
    """情報源ごとのアダプタ。input.type が合わない、種類を作れない、パラメータが合わなければ ConfigError。"""
    adapters: Dict[str, SourceAdapter] = {}
    for name, source in config.sources.items():
        where = f'sources.{name}.input'
        raw = dict(source.input)
        type_name = raw.pop('type', None)
        if not isinstance(type_name, str):
            raise ConfigError(f'{where}.type: required')
        try:
            cls = resolve(type_name, STANDARD_ADAPTERS, SourceAdapter, where)
            params = resolve_params(cls, raw, where)
        except PluginError as error:
            raise ConfigError(str(error)) from error
        if source.kind not in cls.KINDS:
            raise ConfigError(f'{where}: {type_name} cannot make a {source.kind.value} source')
        try:
            adapters[name] = cls(name, source.kind, params, store, context)
        except ValueError as error:
            raise ConfigError(f'{where}: {error}') from error
    return adapters
