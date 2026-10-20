"""対処の組み立て: 設定の recovery.actuators から対処を作る。"""
from typing import Any, Collection, List, Mapping, Set

from ...core.plugins import ConfigError, build
from .base import Actuator
from .standard import Gate, ServiceCall
from .variance_scale import VarianceScale

STANDARD_ACTUATORS = {
    'gate': Gate,
    'service_call': ServiceCall,
    'variance_scale': VarianceScale,
}

__all__ = ['Actuator', 'STANDARD_ACTUATORS', 'apply_actuators', 'build_actuators']


def apply_actuators(actuators: List[Actuator], disabled: Collection[str], released: Set[str], result: Any) -> None:
    """プロファイルで無効にした対処 (disabled の名前) は働かせない。無効になった周期に、一度だけ release する。
    released は、release 済みの名前 (呼び出し側が持ち続ける)。"""
    for actuator in actuators:
        if actuator.name in disabled:
            if actuator.name not in released:
                released.add(actuator.name)
                actuator.release()
            continue
        released.discard(actuator.name)
        actuator.apply(result)


def build_actuators(items: List[Mapping[str, Any]], node: Any) -> List[Actuator]:
    """items は {type, name?, params?}。type が合わない、パラメータが合わなければ ConfigError。"""
    actuators = []
    for i, item in enumerate(items):
        where = f'recovery.actuators[{i}]'
        cls, params = build(item['type'], STANDARD_ACTUATORS, Actuator, item.get('params'), where, f'{where}.params')
        try:
            actuators.append(cls(item.get('name', item['type']), params, node))
        except ValueError as error:
            raise ConfigError(f'{where}: {error}') from error
    return actuators
