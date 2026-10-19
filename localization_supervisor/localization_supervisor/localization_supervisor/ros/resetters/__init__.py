"""再初期化の送り先の組み立て: 設定の recovery.resetters から送り先を作る。"""
from typing import Any, List, Mapping

from ...core.config import ConfigError
from ...core.plugins import PluginError, resolve, resolve_params
from ..own_messages import OwnMessages
from .base import PoseResetter
from .standard import Amcl, RobotLocalization

STANDARD_RESETTERS = {
    'amcl': Amcl,
    'robot_localization': RobotLocalization,
}

__all__ = ['PoseResetter', 'STANDARD_RESETTERS', 'build_resetters']


def build_resetters(items: List[Mapping[str, Any]], node: Any, own: OwnMessages) -> List[PoseResetter]:
    """items は {type, name?, params?}。type が合わない、パラメータが合わなければ ConfigError。"""
    resetters = []
    for i, item in enumerate(items):
        where = f'recovery.resetters[{i}]'
        try:
            cls = resolve(item['type'], STANDARD_RESETTERS, PoseResetter, where)
            params = resolve_params(cls, item.get('params') or {}, f'{where}.params')
        except PluginError as error:
            raise ConfigError(str(error)) from error
        resetters.append(cls(item.get('name', item['type']), params, node, own))
    return resetters
