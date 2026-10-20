"""再初期化の送り先の組み立て: 設定の recovery.resetters から送り先を作る。"""
from typing import Any, List, Mapping

from ...core.plugins import build
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
        cls, params = build(
            item['type'], STANDARD_RESETTERS, PoseResetter, item.get('params'), where, f'{where}.params')
        resetters.append(cls(item.get('name', item['type']), params, node, own))
    return resetters
