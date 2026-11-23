"""門のノードの一時停止の要求の出力。設定の hold_output.type に `パッケージ.モジュール:クラス` を書く。"""
from typing import Any, Mapping, Optional

from ...core.plugins import build
from .base import HoldOutput

STANDARD_HOLD_OUTPUTS: Mapping[str, type] = {}

__all__ = ['HoldOutput', 'STANDARD_HOLD_OUTPUTS', 'build_hold_output']


def build_hold_output(spec: Mapping[str, Any], node: Any) -> Optional[HoldOutput]:
    """hold_output の辞書 ({type, params?})。空なら None (止める要求を出さない)。合わなければ ConfigError。"""
    if not spec:
        return None
    cls, params = build(spec['type'], STANDARD_HOLD_OUTPUTS, HoldOutput, spec.get('params'), 'hold_output',
                        'hold_output.params')
    return cls(spec.get('name', spec['type']), params, node)
