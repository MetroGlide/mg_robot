"""設定ファイル (YAML) のスキーマ・読み込み・検証。ROS に依存しない。

ノードもオフライン評価も、同じファイルをこのモジュールで読む。合わないところがあれば ConfigError で落とす。
スキーマの版は CONTRACT_VERSION (interfaces.py)。書き方は doc/parameters.md。

プロファイルは、基本設定に重ねる部分の設定 (辞書は再帰的に重ね、それ以外は置き換える)。
enabled_detectors を書くと、そこに並べた判定のインスタンスだけを有効にする。
"""
import copy
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

import yaml

from .anchors import AnchorConfig
from .detectors import STANDARD_DETECTORS
from .fusion import STANDARD_FUSIONS
from .interfaces import (
    CONTRACT_VERSION, CandidateGenerator, CandidateScorer, Detector, FusionStrategy, Matcher, SourceKind)
from .matcher import STANDARD_MATCHERS
from .plugins import ConfigError, build
from .recovery import STANDARD_GENERATORS, STANDARD_SCORERS
from .recovery.planner import GeneratorConfig, ScorerConfig, SelectionConfig
from .residuals import ResidualConfig
from .safety import SafetyConfig
from .scan_buffer import ScanBufferConfig
from .state_machine import MachineConfig
from .timeline import AlignmentConfig


TOP_LEVEL_KEYS = {
    'version', 'period_sec', 'sources', 'timeline', 'detectors', 'enabled_detectors', 'fusion', 'trust',
    'state_machine', 'recovery', 'anchors', 'safety', 'residuals', 'grace', 'external_reinit', 'events', 'output',
    'profiles',
}


@dataclass
class OutputConfig:
    """ノードの出力の名前。"""
    status_topic: str = '/localization/status'
    diagnostic_name: str = 'localization'
    hardware_id: str = 'localization'


DETECTOR_KEYS = {
    'type', 'sources', 'params', 'weight', 'allowance', 'decay', 'max_score', 'enabled', 'options', 'vouch'}


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
    # 正常と判定できたとき、正しいと示す情報源 (この判定の疑いの対象のうち)
    vouch: List[str] = field(default_factory=list)


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
    generators: Dict[str, GeneratorConfig] = field(default_factory=dict)
    scorers: Dict[str, ScorerConfig] = field(default_factory=dict)
    selection: SelectionConfig = field(default_factory=SelectionConfig)
    matcher_cls: Optional[type] = None
    matcher_params: Dict[str, Any] = field(default_factory=dict)
    scan_buffer: Optional[ScanBufferConfig] = None
    # 推定器への送り先と、対処の設定 (ROS 側のプラグインが読む。形だけここで確かめる)
    resetters: List[Dict[str, Any]] = field(default_factory=list)
    actuators: List[Dict[str, Any]] = field(default_factory=list)
    anchors: Optional[AnchorConfig] = None
    safety: Optional[SafetyConfig] = None
    residuals: Dict[str, ResidualConfig] = field(default_factory=dict)
    period_sec: float = 1.0
    startup_grace_sec: float = 0.0
    grace_after_recovery_sec: float = 0.0
    grace_after_reset_sec: float = 0.0
    external_reinit_topics: List[str] = field(default_factory=list)
    event_prefix: str = 'localization'
    event_sink: Dict[str, Any] = field(default_factory=lambda: {'type': 'logfmt'})
    output: OutputConfig = field(default_factory=OutputConfig)
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
        """プロファイルを重ねた設定。'' は基本設定。"""
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


def _source(sources: Mapping[str, SourceConfig], name: Any, kind: SourceKind, where: str) -> None:
    """name が kind の情報源でなければ ConfigError。"""
    source = sources.get(name)
    if source is None or source.kind != kind:
        raise ConfigError(f'{where}: {name!r} is not a {kind.value} source')


def _enabled(body: Mapping[str, Any], where: str) -> bool:
    enabled = body.get('enabled', True)
    if not isinstance(enabled, bool):
        raise ConfigError(f'{where}.enabled: expected true or false')
    return enabled


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


def _bind_plugin(
    where: str, body: Mapping[str, Any], base: type, registry: Mapping[str, type],
    sources: Mapping[str, SourceConfig],
) -> Tuple[str, type, Dict[str, Any], Dict[str, str]]:
    """type・params・sources を検証して、(type, クラス, パラメータ, 引数に結んだ情報源) を返す。"""
    type_name = body.get('type')
    if not isinstance(type_name, str):
        raise ConfigError(f'{where}.type: required')
    cls, params = build(
        type_name, registry, base, _mapping(body.get('params'), f'{where}.params'), where, f'{where}.params')
    declared = getattr(cls, 'SOURCES', {})
    bound = _mapping(body.get('sources'), f'{where}.sources')
    if set(bound) != set(declared):
        raise ConfigError(f'{where}.sources: expected arguments {sorted(declared)}, got {sorted(bound)}')
    for arg, kind in declared.items():
        source = sources.get(bound[arg])
        if source is None:
            raise ConfigError(f'{where}.sources.{arg}: unknown source {bound[arg]!r}')
        if source.kind != kind:
            raise ConfigError(
                f'{where}.sources.{arg}: {bound[arg]!r} is {source.kind.value}, expected {kind.value}')
    return type_name, cls, params, bound


def _parse_detector(
    name: str, body: Mapping[str, Any], sources: Mapping[str, SourceConfig], anchors: Optional[AnchorConfig],
    safety: Optional[SafetyConfig],
) -> DetectorConfig:
    where = f'detectors.{name}'
    body = _mapping(body, where)
    _check_keys(body, DETECTOR_KEYS, where)
    type_name, cls, params, bound = _bind_plugin(where, body, Detector, STANDARD_DETECTORS, sources)
    _check_requires(where, cls, anchors, safety, bound)
    enabled = _enabled(body, where)
    detector = DetectorConfig(
        name=name, type=type_name, cls=cls, sources=dict(bound), params=params,
        weight=_number(body, 'weight', 1.0, where), allowance=_number(body, 'allowance', 1.0, where),
        decay=_number(body, 'decay', 1.0, where), max_score=_number(body, 'max_score', 5.0, where),
        enabled=enabled, options=_mapping(body.get('options'), f'{where}.options'),
        vouch=_names(body.get('vouch'), f'{where}.vouch'))
    try:
        subjects = cls(name, params, bound).subjects()
    except (ValueError, TypeError) as error:
        raise ConfigError(f'{where}: {error}') from error
    for source in detector.vouch:
        if source not in subjects:
            raise ConfigError(f'{where}.vouch: {source!r} is not one of the sources this detector suspects {subjects}')
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
    _source(sources, pose, SourceKind.POSE, 'safety.pose')
    _source(sources, map_name, SourceKind.REFERENCE, 'safety.map')
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


RECOVERY_KEYS = {'convergence', 'matcher', 'scan_buffer', 'generators', 'scorers', 'selection', 'resetters',
                 'actuators'}
GENERATOR_KEYS = {'type', 'sources', 'params', 'enabled'}
SCORER_KEYS = {'type', 'sources', 'params', 'weight', 'veto_below', 'enabled'}


def _check_requires(where: str, cls: type, anchors: Optional[AnchorConfig], safety: Optional[SafetyConfig],
                    bound: Mapping[str, str]) -> None:
    available = {'anchors': anchors is not None, 'safety': safety is not None}
    for required in cls.REQUIRES:
        if not available[required]:
            raise ConfigError(f'{where}: requires {required!r} to be configured')
    if 'anchors' in cls.REQUIRES and anchors is not None and 'motion' in bound and bound['motion'] != anchors.motion:
        raise ConfigError(
            f'{where}.sources.motion: {bound["motion"]!r} differs from anchors.motion {anchors.motion!r}')


def _parse_recovery(
    recovery: Mapping[str, Any], sources: Mapping[str, SourceConfig], anchors: Optional[AnchorConfig],
    safety: Optional[SafetyConfig],
) -> Dict[str, Any]:
    """recovery の convergence 以外の項目 (候補を作るもの・評価するもの・選び方・照合・スキャンのバッファ・
    送り先・対処)。SupervisorConfig の同名のフィールドに入れる辞書を返す。"""
    generators: Dict[str, GeneratorConfig] = {}
    for name, body in _mapping(recovery.get('generators'), 'recovery.generators').items():
        where = f'recovery.generators.{name}'
        body = _mapping(body, where)
        _check_keys(body, GENERATOR_KEYS, where)
        type_name, cls, params, bound = _bind_plugin(where, body, CandidateGenerator, STANDARD_GENERATORS, sources)
        _check_requires(where, cls, anchors, safety, bound)
        generators[name] = GeneratorConfig(name, type_name, cls, bound, params, _enabled(body, where))
    scorers: Dict[str, ScorerConfig] = {}
    for name, body in _mapping(recovery.get('scorers'), 'recovery.scorers').items():
        where = f'recovery.scorers.{name}'
        body = _mapping(body, where)
        _check_keys(body, SCORER_KEYS, where)
        type_name, cls, params, bound = _bind_plugin(where, body, CandidateScorer, STANDARD_SCORERS, sources)
        _check_requires(where, cls, anchors, safety, bound)
        veto = body.get('veto_below')
        scorers[name] = ScorerConfig(
            name, type_name, cls, bound, params, weight=_number(body, 'weight', 1.0, where),
            veto_below=None if veto is None else _number(body, 'veto_below', 0.0, where),
            enabled=_enabled(body, where))
    for name, config in {**generators, **scorers}.items():
        try:
            config.cls(name, config.params, config.sources)
        except (ValueError, TypeError) as error:
            raise ConfigError(f'recovery.{name}: {error}') from error
    generators = {name: g for name, g in generators.items() if g.enabled}
    scorers = {name: s for name, s in scorers.items() if s.enabled}

    selection = _dataclass_from(SelectionConfig, _mapping(recovery.get('selection'), 'recovery.selection'),
                                'recovery.selection')

    matcher = _mapping(recovery.get('matcher'), 'recovery.matcher')
    _check_keys(matcher, {'type', 'params'}, 'recovery.matcher')
    matcher_cls, matcher_params = build(
        matcher.get('type', 'correlative'), STANDARD_MATCHERS, Matcher,
        _mapping(matcher.get('params'), 'recovery.matcher.params'), 'recovery.matcher', 'recovery.matcher.params')

    scan_buffer = None
    if recovery.get('scan_buffer') is not None:
        scan_buffer = _dataclass_from(
            ScanBufferConfig, _mapping(recovery['scan_buffer'], 'recovery.scan_buffer'), 'recovery.scan_buffer')
        _source(sources, scan_buffer.reference, SourceKind.REFERENCE, 'recovery.scan_buffer.reference')
        _source(sources, scan_buffer.motion, SourceKind.MOTION, 'recovery.scan_buffer.motion')

    resetters = _plugin_list(recovery.get('resetters'), 'recovery.resetters')
    actuators = _plugin_list(recovery.get('actuators'), 'recovery.actuators')
    return {'generators': generators, 'scorers': scorers, 'selection': selection, 'matcher_cls': matcher_cls,
            'matcher_params': matcher_params, 'scan_buffer': scan_buffer, 'resetters': resetters,
            'actuators': actuators}


def _plugin_list(raw: Any, where: str) -> List[Dict[str, Any]]:
    """type を持つ辞書の並び (中身は、使う側のプラグインが検証する)。"""
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ConfigError(f'{where}: expected a list')
    result = []
    for i, item in enumerate(raw):
        item = _mapping(item, f'{where}[{i}]')
        if not isinstance(item.get('type'), str):
            raise ConfigError(f'{where}[{i}].type: required')
        result.append(item)
    return result


def _parse_residuals(raw: Mapping[str, Any], sources: Mapping[str, SourceConfig]) -> Dict[str, ResidualConfig]:
    residuals: Dict[str, ResidualConfig] = {}
    for name, body in raw.items():
        where = f'residuals.{name}'
        residual = _dataclass_from(ResidualConfig, _mapping(body, where), where)
        for key in ('a', 'b'):
            _source(sources, getattr(residual, key), SourceKind.POSE, f'{where}.{key}')
        if residual.a == residual.b:
            raise ConfigError(f'{where}: a and b must differ')
        residuals[name] = residual
    return residuals


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

    timeline = _mapping(merged.get('timeline'), 'timeline')
    _check_keys(timeline, {'motion_priority', 'horizon_sec', 'max_extrapolation_sec',
                           'alignment_std_per_m', 'alignment_std_per_rad'}, 'timeline')
    motion_priority = _names(timeline.get('motion_priority'), 'timeline.motion_priority')
    for name in motion_priority:
        _source(sources, name, SourceKind.MOTION, 'timeline.motion_priority')

    anchors = None
    if merged.get('anchors') is not None:
        anchors = _dataclass_from(AnchorConfig, _mapping(merged['anchors'], 'anchors'), 'anchors')
        _source(sources, anchors.pose, SourceKind.POSE, 'anchors.pose')
        _source(sources, anchors.motion, SourceKind.MOTION, 'anchors.motion')

    safety = None
    if merged.get('safety') is not None:
        safety = _parse_safety(_mapping(merged['safety'], 'safety'), sources)

    detectors = {name: _parse_detector(name, body, sources, anchors, safety)
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
    fusion_cls, fusion_params = build(
        fusion_type, STANDARD_FUSIONS, FusionStrategy, _mapping(fusion.get('params'), 'fusion.params'), 'fusion',
        'fusion.params')

    trust = _mapping(merged.get('trust'), 'trust')
    _check_keys(trust, {'require_normal'}, 'trust')
    recovery = _mapping(merged.get('recovery'), 'recovery')
    _check_keys(recovery, RECOVERY_KEYS, 'recovery')

    recovery_parts = _parse_recovery(recovery, sources, anchors, safety)
    machine = _dataclass_from(MachineConfig, _mapping(merged.get('state_machine'), 'state_machine'), 'state_machine')
    if machine.recovery_enabled and not recovery_parts['generators']:
        raise ConfigError('state_machine.recovery_enabled: requires recovery.generators')
    machine.isolate_only = _names(machine.isolate_only, 'state_machine.isolate_only')
    for name in machine.isolate_only:
        if name not in sources:
            raise ConfigError(f'state_machine.isolate_only: unknown source {name!r}')

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
        machine=machine,
        convergence=_detector_names(
            _names(recovery.get('convergence'), 'recovery.convergence'), detectors, 'recovery.convergence'),
        anchors=anchors, safety=safety, period_sec=period_sec, **recovery_parts,
        residuals=_parse_residuals(_mapping(merged.get('residuals'), 'residuals'), sources),
        startup_grace_sec=_number(grace, 'startup_sec', 0.0, 'grace'),
        grace_after_recovery_sec=_number(grace, 'after_recovery_sec', 0.0, 'grace'),
        grace_after_reset_sec=_number(grace, 'after_reset_sec', 0.0, 'grace'),
        external_reinit_topics=_names(external.get('topics'), 'external_reinit.topics'),
        event_prefix=prefix, event_sink=sink,
        output=_dataclass_from(OutputConfig, _mapping(merged.get('output'), 'output'), 'output'),
        profile=profile, profiles=profiles,
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
