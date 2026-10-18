"""設定ファイル (YAML) のスキーマ・読み込み・検証。ROS に依存しない。

ノードもオフライン評価も、同じファイルをこのモジュールで読む。合わないところがあれば ConfigError で落とす。
スキーマの版は CONTRACT_VERSION (interfaces.py)。書き方は doc/parameters.md。

プロファイルは、基の設定に重ねる部分の設定 (辞書は再帰的に重ね、それ以外は置き換える)。
enabled_detectors を書くと、そこに並べた判定のインスタンスだけを有効にする。
"""
import copy
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional

import yaml

from .anchors import AnchorConfig
from .detectors import STANDARD_DETECTORS
from .fusion import STANDARD_FUSIONS
from .interfaces import CONTRACT_VERSION, Detector, FusionStrategy, SourceKind
from .plugins import PluginError, resolve, resolve_params
from .safety import SafetyConfig
from .state_machine import MachineConfig
from .timeline import AlignmentConfig


class ConfigError(ValueError):
    pass


TOP_LEVEL_KEYS = {
    'version', 'period_sec', 'sources', 'timeline', 'detectors', 'enabled_detectors', 'fusion', 'trust',
    'state_machine', 'recovery', 'anchors', 'safety', 'grace', 'external_reinit', 'events', 'output',
    'extras', 'profiles',
}


@dataclass
class OutputConfig:
    """ノードの出力の名前。"""
    status_topic: str = '/localization/status'
    diagnostic_name: str = 'localization'
    hardware_id: str = 'localization'
DETECTOR_KEYS = {'type', 'sources', 'params', 'weight', 'allowance', 'decay', 'max_score', 'enabled', 'options'}


@dataclass
class SourceConfig:
    name: str
    kind: SourceKind
    derived_from: List[str] = field(default_factory=list)
    input: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DetectorConfig:
    name: str
    type: str
    cls: type
    sources: Dict[str, str]
    params: Dict[str, Any]
    weight: float = 1.0
    allowance: float = 1.0
    decay: float = 1.0
    max_score: float = 5.0
    enabled: bool = True
    options: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SupervisorConfig:
    sources: Dict[str, SourceConfig]
    detectors: Dict[str, DetectorConfig]
    fusion_type: str
    fusion_cls: type
    fusion_params: Dict[str, Any]
    motion_priority: List[str] = field(default_factory=list)
    horizon_sec: float = 30.0
    max_extrapolation_sec: float = 0.5
    alignment: AlignmentConfig = field(default_factory=AlignmentConfig)
    trust_require_normal: List[str] = field(default_factory=list)
    machine: MachineConfig = field(default_factory=MachineConfig)
    convergence: List[str] = field(default_factory=list)
    anchors: Optional[AnchorConfig] = None
    safety: Optional[SafetyConfig] = None
    period_sec: float = 1.0
    startup_grace_sec: float = 0.0
    grace_after_recovery_sec: float = 0.0
    grace_after_reset_sec: float = 0.0
    external_reinit_topics: List[str] = field(default_factory=list)
    event_prefix: str = 'localization'
    event_sink: Dict[str, Any] = field(default_factory=lambda: {'type': 'logfmt'})
    output: OutputConfig = field(default_factory=OutputConfig)
    extras: List[str] = field(default_factory=list)
    profile: str = ''
    profiles: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    raw: Dict[str, Any] = field(default_factory=dict)

    def kinds(self) -> Dict[str, SourceKind]:
        return {name: source.kind for name, source in self.sources.items()}

    def independent_sources(self) -> List[str]:
        return [name for name, source in self.sources.items()
                if not source.derived_from and source.kind != SourceKind.REFERENCE]

    def derived_sources(self) -> Dict[str, List[str]]:
        return {name: list(source.derived_from) for name, source in self.sources.items() if source.derived_from}

    def enabled_detectors(self) -> Dict[str, DetectorConfig]:
        return {name: d for name, d in self.detectors.items() if d.enabled}

    def with_profile(self, profile: str) -> 'SupervisorConfig':
        """プロファイルを重ねた設定。'' は基の設定。"""
        return parse_config(self.raw, profile=profile, profiles=self.profiles)


def merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    """override を base に重ねる。type が変わる辞書 (別のプラグイン) は、重ねずに置き換える。"""
    result = copy.deepcopy(dict(base))
    for key, value in override.items():
        if (isinstance(value, Mapping) and isinstance(result.get(key), Mapping)
                and value.get('type', result[key].get('type')) == result[key].get('type')):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _mapping(value: Any, where: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ConfigError(f'{where}: expected a mapping')
    return dict(value)


def _check_keys(raw: Mapping[str, Any], allowed: set, where: str) -> None:
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ConfigError(f'{where}: unknown keys {unknown} (known: {sorted(allowed)})')


def _number(raw: Mapping[str, Any], key: str, default: float, where: str) -> float:
    value = raw.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f'{where}.{key}: expected a number, got {value!r}')
    return float(value)


def _names(raw: Any, where: str) -> List[str]:
    if raw is None:
        return []
    if not isinstance(raw, list) or not all(isinstance(v, str) for v in raw):
        raise ConfigError(f'{where}: expected a list of names')
    return list(raw)


def _parse_sources(raw: Mapping[str, Any]) -> Dict[str, SourceConfig]:
    sources: Dict[str, SourceConfig] = {}
    if not raw:
        raise ConfigError('sources: at least one source is required')
    for name, body in raw.items():
        where = f'sources.{name}'
        body = _mapping(body, where)
        _check_keys(body, {'kind', 'derived_from', 'input'}, where)
        try:
            kind = SourceKind(body.get('kind'))
        except ValueError as error:
            raise ConfigError(f'{where}.kind: expected one of {[k.value for k in SourceKind]}') from error
        sources[name] = SourceConfig(
            name, kind, _names(body.get('derived_from'), f'{where}.derived_from'),
            _mapping(body.get('input'), f'{where}.input'))
    for name, source in sources.items():
        for parent in source.derived_from:
            if parent not in sources:
                raise ConfigError(f'sources.{name}.derived_from: unknown source {parent!r}')
            if sources[parent].derived_from:
                raise ConfigError(f'sources.{name}.derived_from: {parent!r} is itself derived')
    return sources


def _parse_detector(
    name: str, body: Mapping[str, Any], sources: Mapping[str, SourceConfig], extras: List[str],
) -> DetectorConfig:
    where = f'detectors.{name}'
    body = _mapping(body, where)
    _check_keys(body, DETECTOR_KEYS, where)
    type_name = body.get('type')
    if not isinstance(type_name, str):
        raise ConfigError(f'{where}.type: required')
    try:
        cls = resolve(type_name, STANDARD_DETECTORS, Detector, where)
        params = resolve_params(cls, _mapping(body.get('params'), f'{where}.params'), f'{where}.params')
    except PluginError as error:
        raise ConfigError(str(error)) from error
    bound = _mapping(body.get('sources'), f'{where}.sources')
    if set(bound) != set(cls.SOURCES):
        raise ConfigError(f'{where}.sources: expected arguments {sorted(cls.SOURCES)}, got {sorted(bound)}')
    for arg, kind in cls.SOURCES.items():
        source = sources.get(bound[arg])
        if source is None:
            raise ConfigError(f'{where}.sources.{arg}: unknown source {bound[arg]!r}')
        if source.kind != kind:
            raise ConfigError(
                f'{where}.sources.{arg}: {bound[arg]!r} is {source.kind.value}, expected {kind.value}')
    missing = [e for e in cls.EXTRAS if e not in extras]
    if missing:
        raise ConfigError(f'{where}: extras {missing} are not provided (extras: {extras})')
    enabled = body.get('enabled', True)
    if not isinstance(enabled, bool):
        raise ConfigError(f'{where}.enabled: expected true or false')
    detector = DetectorConfig(
        name=name, type=type_name, cls=cls, sources=dict(bound), params=params,
        weight=_number(body, 'weight', 1.0, where), allowance=_number(body, 'allowance', 1.0, where),
        decay=_number(body, 'decay', 1.0, where), max_score=_number(body, 'max_score', 5.0, where),
        enabled=enabled, options=_mapping(body.get('options'), f'{where}.options'))
    try:
        cls(name, params, bound)
    except (ValueError, TypeError) as error:
        raise ConfigError(f'{where}: {error}') from error
    return detector


def _dataclass_from(cls: type, raw: Mapping[str, Any], where: str) -> Any:
    defaults = cls()
    _check_keys(raw, set(vars(defaults)), where)
    values = {}
    for key, default in vars(defaults).items():
        value = raw.get(key, default)
        if isinstance(default, bool):
            if not isinstance(value, bool):
                raise ConfigError(f'{where}.{key}: expected true or false')
        elif isinstance(default, int):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigError(f'{where}.{key}: expected an integer')
        elif isinstance(default, float):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(f'{where}.{key}: expected a number')
            value = float(value)
        elif isinstance(default, str) and not isinstance(value, str):
            raise ConfigError(f'{where}.{key}: expected a string')
        values[key] = value
    return cls(**values)


def _parse_safety(raw: Mapping[str, Any], sources: Mapping[str, SourceConfig]) -> SafetyConfig:
    _check_keys(raw, {'pose', 'map', 'footprint', 'max_age_sec', 'depth_range_m'}, 'safety')
    pose, map_name = raw.get('pose'), raw.get('map')
    if sources.get(pose) is None or sources[pose].kind != SourceKind.POSE:
        raise ConfigError(f'safety.pose: {pose!r} is not a pose source')
    if sources.get(map_name) is None or sources[map_name].kind != SourceKind.REFERENCE:
        raise ConfigError(f'safety.map: {map_name!r} is not a reference source')
    footprint = raw.get('footprint')
    if (not isinstance(footprint, list) or len(footprint) < 3
            or not all(isinstance(p, list) and len(p) == 2
                       and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)
                       for p in footprint)):
        raise ConfigError('safety.footprint: expected a list of at least 3 [x, y] points')
    return SafetyConfig(
        pose=pose, map=map_name, footprint=[(float(x), float(y)) for x, y in footprint],
        max_age_sec=_number(raw, 'max_age_sec', SafetyConfig.max_age_sec, 'safety'),
        depth_range_m=_number(raw, 'depth_range_m', SafetyConfig.depth_range_m, 'safety'))


def _detector_names(names: List[str], detectors: Mapping[str, DetectorConfig], where: str) -> List[str]:
    for name in names:
        detector = detectors.get(name)
        if detector is None:
            raise ConfigError(f'{where}: unknown detector {name!r}')
    return [name for name in names if detectors[name].enabled]


def parse_config(
    raw: Mapping[str, Any], profile: str = '', profiles: Optional[Mapping[str, Mapping[str, Any]]] = None,
) -> SupervisorConfig:
    """辞書から設定を作る。profiles はプロファイルの名前と、重ねる部分の設定。"""
    raw = _mapping(raw, 'config')
    profiles = dict(profiles or {})
    merged = dict(raw)
    if profile:
        if profile not in profiles:
            raise ConfigError(f'profile {profile!r} is not defined (profiles: {sorted(profiles)})')
        override = _mapping(profiles[profile], f'profiles.{profile}')
        if 'profiles' in override or 'version' in override:
            raise ConfigError(f'profiles.{profile}: cannot override version or profiles')
        merged = merge(raw, override)
    _check_keys(merged, TOP_LEVEL_KEYS, 'config')
    if merged.get('version') != CONTRACT_VERSION:
        raise ConfigError(f'version: expected {CONTRACT_VERSION}, got {merged.get("version")!r}')

    sources = _parse_sources(_mapping(merged.get('sources'), 'sources'))
    extras = _names(merged.get('extras'), 'extras')

    timeline = _mapping(merged.get('timeline'), 'timeline')
    _check_keys(timeline, {'motion_priority', 'horizon_sec', 'max_extrapolation_sec',
                           'alignment_std_per_m', 'alignment_std_per_rad'}, 'timeline')
    motion_priority = _names(timeline.get('motion_priority'), 'timeline.motion_priority')
    for name in motion_priority:
        if name not in sources or sources[name].kind != SourceKind.MOTION:
            raise ConfigError(f'timeline.motion_priority: {name!r} is not a motion source')

    detectors = {name: _parse_detector(name, body, sources, extras)
                 for name, body in _mapping(merged.get('detectors'), 'detectors').items()}
    if 'enabled_detectors' in merged:
        only = _names(merged['enabled_detectors'], 'enabled_detectors')
        for name in only:
            if name not in detectors:
                raise ConfigError(f'enabled_detectors: unknown detector {name!r}')
        for name, detector in detectors.items():
            detector.enabled = name in only

    fusion = _mapping(merged.get('fusion'), 'fusion')
    _check_keys(fusion, {'type', 'params'}, 'fusion')
    fusion_type = fusion.get('type', 'cusum')
    try:
        fusion_cls = resolve(fusion_type, STANDARD_FUSIONS, FusionStrategy, 'fusion')
        fusion_params = resolve_params(fusion_cls, _mapping(fusion.get('params'), 'fusion.params'), 'fusion.params')
    except PluginError as error:
        raise ConfigError(str(error)) from error

    trust = _mapping(merged.get('trust'), 'trust')
    _check_keys(trust, {'require_normal'}, 'trust')
    recovery = _mapping(merged.get('recovery'), 'recovery')
    _check_keys(recovery, {'convergence'}, 'recovery')

    anchors = None
    if merged.get('anchors') is not None:
        anchors = _dataclass_from(AnchorConfig, _mapping(merged['anchors'], 'anchors'), 'anchors')
        if sources.get(anchors.pose) is None or sources[anchors.pose].kind != SourceKind.POSE:
            raise ConfigError(f'anchors.pose: {anchors.pose!r} is not a pose source')
        if sources.get(anchors.motion) is None or sources[anchors.motion].kind != SourceKind.MOTION:
            raise ConfigError(f'anchors.motion: {anchors.motion!r} is not a motion source')

    safety = None
    if merged.get('safety') is not None:
        safety = _parse_safety(_mapping(merged['safety'], 'safety'), sources)

    grace = _mapping(merged.get('grace'), 'grace')
    _check_keys(grace, {'startup_sec', 'after_recovery_sec', 'after_reset_sec'}, 'grace')
    external = _mapping(merged.get('external_reinit'), 'external_reinit')
    _check_keys(external, {'topics'}, 'external_reinit')
    events = _mapping(merged.get('events'), 'events')
    _check_keys(events, {'prefix', 'sink'}, 'events')
    prefix = events.get('prefix', 'localization')
    if not isinstance(prefix, str) or not prefix:
        raise ConfigError('events.prefix: expected a non-empty string')
    sink = _mapping(events.get('sink', {'type': 'logfmt'}), 'events.sink')
    if not isinstance(sink.get('type'), str):
        raise ConfigError('events.sink.type: required')
    period_sec = _number(merged, 'period_sec', 1.0, 'config')
    if period_sec <= 0.0:
        raise ConfigError('period_sec: must be positive')

    return SupervisorConfig(
        sources=sources, detectors=detectors, fusion_type=fusion_type, fusion_cls=fusion_cls,
        fusion_params=fusion_params, motion_priority=motion_priority,
        horizon_sec=_number(timeline, 'horizon_sec', 30.0, 'timeline'),
        max_extrapolation_sec=_number(timeline, 'max_extrapolation_sec', 0.5, 'timeline'),
        alignment=AlignmentConfig(
            _number(timeline, 'alignment_std_per_m', AlignmentConfig.std_per_m, 'timeline'),
            _number(timeline, 'alignment_std_per_rad', AlignmentConfig.std_per_rad, 'timeline')),
        trust_require_normal=_detector_names(
            _names(trust.get('require_normal'), 'trust.require_normal'), detectors, 'trust.require_normal'),
        machine=_dataclass_from(MachineConfig, _mapping(merged.get('state_machine'), 'state_machine'),
                                'state_machine'),
        convergence=_detector_names(
            _names(recovery.get('convergence'), 'recovery.convergence'), detectors, 'recovery.convergence'),
        anchors=anchors, safety=safety, period_sec=period_sec,
        startup_grace_sec=_number(grace, 'startup_sec', 0.0, 'grace'),
        grace_after_recovery_sec=_number(grace, 'after_recovery_sec', 0.0, 'grace'),
        grace_after_reset_sec=_number(grace, 'after_reset_sec', 0.0, 'grace'),
        external_reinit_topics=_names(external.get('topics'), 'external_reinit.topics'),
        event_prefix=prefix, event_sink=sink,
        output=_dataclass_from(OutputConfig, _mapping(merged.get('output'), 'output'), 'output'),
        extras=extras, profile=profile, profiles=profiles,
        raw={k: v for k, v in raw.items() if k != 'profiles'})


def load_profiles(raw: Mapping[str, Any], base_dir: str) -> Dict[str, Dict[str, Any]]:
    """profiles の値 (ファイルのパス、または辞書) を読む。相対パスは設定ファイルの場所から。"""
    profiles: Dict[str, Dict[str, Any]] = {}
    for name, value in _mapping(raw, 'profiles').items():
        if isinstance(value, str):
            path = value if os.path.isabs(value) else os.path.join(base_dir, value)
            try:
                with open(path, encoding='utf-8') as f:
                    profiles[name] = _mapping(yaml.safe_load(f), f'profiles.{name} ({path})')
            except OSError as error:
                raise ConfigError(f'profiles.{name}: cannot read {path} ({error})') from error
        else:
            profiles[name] = _mapping(value, f'profiles.{name}')
    return profiles


def load_config(path: str, profile: str = '') -> SupervisorConfig:
    """設定ファイルを読み、すべてのプロファイルも検証したうえで、profile を重ねた設定を返す。"""
    try:
        with open(path, encoding='utf-8') as f:
            raw = _mapping(yaml.safe_load(f), path)
    except OSError as error:
        raise ConfigError(f'cannot read {path} ({error})') from error
    profiles = load_profiles(raw.get('profiles'), os.path.dirname(os.path.abspath(path)))
    for name in profiles:
        parse_config(raw, profile=name, profiles=profiles)
    return parse_config(raw, profile=profile, profiles=profiles)
