"""対処の組み立て: 設定の recovery.actuators から対処を作る。"""
from typing import Any, List, Mapping

from ...core.config import ConfigError
from ...core.plugins import PluginError, resolve, resolve_params
from .base import Actuator
from .standard import Gate, ServiceCall

STANDARD_ACTUATORS = {
    'gate': Gate,
    'service_call': ServiceCall,
}

__all__ = ['Actuator', 'STANDARD_ACTUATORS', 'build_actuators']


def build_actuators(items: List[Mapping[str, Any]], node: Any) -> List[Actuator]:
    """items は {type, name?, params?}。type が合わない、パラメータが合わなければ ConfigError。"""
    actuators = []
    for i, item in enumerate(items):
        where = f'recovery.actuators[{i}]'
        try:
            cls = resolve(item['type'], STANDARD_ACTUATORS, Actuator, where)
            params = resolve_params(cls, item.get('params') or {}, f'{where}.params')
            actuators.append(cls(item.get('name', item['type']), params, node))
        except PluginError as error:
            raise ConfigError(str(error)) from error
        except ValueError as error:
            raise ConfigError(f'{where}: {error}') from error
    return actuators
