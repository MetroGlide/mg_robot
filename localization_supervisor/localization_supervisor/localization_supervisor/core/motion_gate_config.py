"""門のノードの設定ファイル (YAML) の読み込みと検証 (ROS に依存しない)。

情報源 (sources) と閉じる理由の部品 (closers) の書き方は、監督の設定 (doc/parameters.md) と同じ。
"""
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping

import yaml

from .config import (
    OutputConfig, SourceConfig, _check_keys, _dataclass_from, _mapping, _number, _parse_sources, _plugin_list,
    parse_gate_parts)
from .interfaces import CONTRACT_VERSION
from .motion_gate import MotionGateConfig
from .plugins import ConfigError

TOP_LEVEL_KEYS = {
    'version', 'period_sec', 'timeline', 'sources', 'gate', 'closers', 'map_match', 'scan_buffer', 'resetters',
    'gate_service', 'hold_output', 'manual_pose_topic', 'events', 'output',
}
RESETTER_ROLES = {'estimator', 'fused'}


@dataclass
class GateNodeConfig:
    gate: MotionGateConfig
    sources: Dict[str, SourceConfig]
    period_sec: float = 0.2
    horizon_sec: float = 30.0
    max_extrapolation_sec: float = 0.5
    # 置き直しの送り先 (推定器 / 融合)。ROS 側のプラグインが読む
    resetters: Dict[str, List[Dict[str, Any]]] = field(default_factory=dict)
    gate_service: str = ''
    hold_output: Dict[str, Any] = field(default_factory=dict)
    manual_pose_topic: str = ''
    event_prefix: str = 'localization'
    event_sink: Dict[str, Any] = field(default_factory=lambda: {'type': 'logfmt'})
    output: OutputConfig = field(default_factory=OutputConfig)


def _string(raw: Mapping[str, Any], key: str) -> str:
    value = raw.get(key, '')
    if not isinstance(value, str):
        raise ConfigError(f'{key}: expected a string')
    return value


def parse_gate_config(raw: Mapping[str, Any]) -> GateNodeConfig:
    raw = _mapping(raw, 'config')
    _check_keys(raw, TOP_LEVEL_KEYS, 'config')
    if raw.get('version') != CONTRACT_VERSION:
        raise ConfigError(f'version: expected {CONTRACT_VERSION}, got {raw.get("version")!r}')
    sources = _parse_sources(_mapping(raw.get('sources'), 'sources'))

    gate = parse_gate_parts(
        _mapping(raw.get('gate'), 'gate'), _mapping(raw.get('closers'), 'closers'),
        _mapping(raw.get('map_match'), 'map_match'), _mapping(raw.get('scan_buffer'), 'scan_buffer'), sources)

    resetters_raw = _mapping(raw.get('resetters'), 'resetters')
    _check_keys(resetters_raw, RESETTER_ROLES, 'resetters')
    resetters = {role: _plugin_list(resetters_raw.get(role), f'resetters.{role}') for role in RESETTER_ROLES}
    if not resetters['estimator'] or not resetters['fused']:
        raise ConfigError('resetters: estimator and fused are required')

    hold_output = _mapping(raw.get('hold_output'), 'hold_output')
    if hold_output and not isinstance(hold_output.get('type'), str):
        raise ConfigError('hold_output.type: required')
    timeline = _mapping(raw.get('timeline'), 'timeline')
    _check_keys(timeline, {'horizon_sec', 'max_extrapolation_sec'}, 'timeline')
    events = _mapping(raw.get('events'), 'events')
    _check_keys(events, {'prefix', 'sink'}, 'events')
    sink = _mapping(events.get('sink', {'type': 'logfmt'}), 'events.sink')
    if not isinstance(sink.get('type'), str):
        raise ConfigError('events.sink.type: required')
    period_sec = _number(raw, 'period_sec', 0.2, 'config')
    if period_sec <= 0.0:
        raise ConfigError('period_sec: must be positive')
    gate_service = _string(raw, 'gate_service')
    if not gate_service:
        raise ConfigError('gate_service: required')

    return GateNodeConfig(
        gate=gate, sources=sources, period_sec=period_sec,
        horizon_sec=_number(timeline, 'horizon_sec', 30.0, 'timeline'),
        max_extrapolation_sec=_number(timeline, 'max_extrapolation_sec', 0.5, 'timeline'),
        resetters=resetters, gate_service=gate_service, hold_output=hold_output,
        manual_pose_topic=_string(raw, 'manual_pose_topic'),
        event_prefix=events.get('prefix', 'localization'), event_sink=sink,
        output=_dataclass_from(OutputConfig, _mapping(raw.get('output'), 'output'), 'output'))


def load_gate_config(path: str) -> GateNodeConfig:
    try:
        with open(path, encoding='utf-8') as f:
            raw = yaml.safe_load(f)
    except OSError as error:
        raise ConfigError(f'cannot read {path} ({error})') from error
    return parse_gate_config(_mapping(raw, path))
