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
from .conduct import Action as ConductAction, ConductConfig, Degree, Situation
from .detectors import STANDARD_DETECTORS
from .fusion import STANDARD_FUSIONS
from .initial_search import InitialSearchConfig
from .interfaces import (
    CONTRACT_VERSION, CandidateGenerator, CandidateScorer, Detector, FusionStrategy, Matcher, SourceKind)
from .map_match import MapMatchConfig
from .matcher import STANDARD_MATCHERS
from .motion_gate import CloserConfig, GateSettings, MotionGateConfig
from .plugins import ConfigError, build
from .recovery import STANDARD_GENERATORS, STANDARD_SCORERS
from .recovery.planner import GeneratorConfig, ObserveConfig, ScorerConfig, SelectionConfig
from .residuals import ResidualConfig
from .safety import SafetyConfig
from .scan_buffer import ScanBufferConfig
from .snapshot import HoldConfig
from .state_machine import MachineConfig
from .timeline import AlignmentConfig


TOP_LEVEL_KEYS = {
    'version', 'period_sec', 'sources', 'timeline', 'detectors', 'enabled_detectors', 'fusion', 'trust',
    'state_machine', 'recovery', 'anchors', 'safety', 'residuals', 'grace', 'external_reinit', 'events', 'output',
    'profiles', 'gate', 'conduct',
}
GATE_KEYS = {
    'enabled', 'observe_only', 'period_sec', 'settings', 'closers', 'map_match', 'scan_buffer', 'initial_search',
    'manual_pose_topic'}
# 閉じる理由の部品に、判定の書き方に足すキー
GATE_CLOSER_KEYS = {'close_after_sec', 'active_in_grace'}
# 置き直しの送り先の役: 推定器 (門の置き直しの 'all' で送る) と融合 ('all' と 'fused' で送る)
RESETTER_ROLES = ('estimator', 'fused')


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
    hold: Optional[HoldConfig] = None


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
    # all: convergence のすべてが正常で収束とする。any: どれか 1 つが正常で収束とする
    convergence_mode: str = 'all'
    generators: Dict[str, GeneratorConfig] = field(default_factory=dict)
    scorers: Dict[str, ScorerConfig] = field(default_factory=dict)
    selection: SelectionConfig = field(default_factory=SelectionConfig)
    observe: Optional[ObserveConfig] = None
    matcher_cls: Optional[type] = None
    matcher_params: Dict[str, Any] = field(default_factory=dict)
    scan_buffer: Optional[ScanBufferConfig] = None
    # 推定器への送り先と、対処の設定 (ROS 側のプラグインが読む。形だけここで確かめる)
    resetters: List[Dict[str, Any]] = field(default_factory=list)
    actuators: List[Dict[str, Any]] = field(default_factory=list)
    anchors: Optional[AnchorConfig] = None
    safety: Optional[SafetyConfig] = None
    residuals: Dict[str, ResidualConfig] = field(default_factory=dict)
    # 門 (推定器の姿勢を融合に入れる前に確かめる。gate.enabled のときだけ)。周期 gate_period_sec ごとに回し、
    # 判定・総合・状態遷移は period_sec ごとに回す
    gate: Optional[MotionGateConfig] = None
    gate_period_sec: float = 0.2
    # 走り方 (状況 × 程度 → 減速。門があるときだけ使う)
    conduct: Optional[ConductConfig] = None
    # 手動の初期姿勢のトピック (門があるときにノードが購読する)
    manual_pose_topic: str = ''
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

    def holds(self) -> Dict[str, HoldConfig]:
        return {name: source.hold for name, source in self.sources.items() if source.hold is not None}

    def independent_sources(self) -> List[str]:
        return [name for name, source in self.sources.items()
                if not source.derived_from and source.kind in (SourceKind.POSE, SourceKind.MOTION)]

    def derived_sources(self) -> Dict[str, List[str]]:
        return {name: list(source.derived_from) for name, source in self.sources.items() if source.derived_from}

    def enabled_detectors(self) -> Dict[str, DetectorConfig]:
        return {name: d for name, d in self.detectors.items() if d.enabled}

    def with_profile(self, profile: str) -> 'SupervisorConfig':
        """プロファイルを重ねた設定。'' は基本設定。カンマで区切ると、左から順に重ねる。"""
        return parse_config(self.raw, profile=profile, profiles=self.profiles)

    def with_gate(self, enabled: bool, observe_only: bool = False) -> 'SupervisorConfig':
        """門 (gate 節) を有効 / 無効にした設定。observe_only なら判定と照合だけ (門も監督も何もしない)。
        プロファイルはそのまま。"""
        raw = copy.deepcopy(self.raw)
        if (enabled or observe_only) and 'gate' not in raw:
            raise ConfigError('gate: the config has no gate section')
        if 'gate' in raw:
            raw['gate'] = dict(_mapping(raw['gate'], 'gate'), enabled=enabled or observe_only,
                               observe_only=observe_only)
        return parse_config(raw, profile=self.profile, profiles=self.profiles)


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
        _check_keys(body, {'kind', 'derived_from', 'input', 'hold'}, where)
        try:
            kind = SourceKind(body.get('kind'))
        except ValueError as error:
            raise ConfigError(f'{where}.kind: expected one of {[k.value for k in SourceKind]}') from error
        sources[name] = SourceConfig(
            name, kind, _names(body.get('derived_from'), f'{where}.derived_from'),
            _mapping(body.get('input'), f'{where}.input'), _parse_hold(body.get('hold'), kind, f'{where}.hold'))
    for name, source in sources.items():
        for parent in source.derived_from:
            if parent not in sources:
                raise ConfigError(f'sources.{name}.derived_from: unknown source {parent!r}')
            if sources[parent].derived_from:
                raise ConfigError(f'sources.{name}.derived_from: {parent!r} is itself derived')
    return sources


def _parse_hold(raw: Any, kind: SourceKind, where: str) -> Optional[HoldConfig]:
    if raw is None:
        return None
    if kind != SourceKind.POSE:
        raise ConfigError(f'{where}: only for pose sources')
    raw = _mapping(raw, where)
    _check_keys(raw, {'distance_m', 'rotation_rad'}, where)
    if 'distance_m' not in raw:
        raise ConfigError(f'{where}.distance_m: required')
    return HoldConfig(
        _number(raw, 'distance_m', 0.0, where), _number(raw, 'rotation_rad', HoldConfig.rotation_rad, where))


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


def _parse_conduct(raw: Mapping[str, Any]) -> ConductConfig:
    """conduct 節。floors と table は既定に重ねる (プロファイルで升目ごとに差し替えられる)。"""
    raw = dict(raw)
    floors_raw = _mapping(raw.pop('floors', None), 'conduct.floors')
    table_raw = _mapping(raw.pop('table', None), 'conduct.table')
    config = _dataclass_from(ConductConfig, raw, 'conduct')
    situations = {s.value for s in Situation if s != Situation.NORMAL}
    _check_keys(floors_raw, situations, 'conduct.floors')
    for key, value in floors_raw.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0.0:
            raise ConfigError(f'conduct.floors.{key}: expected a non-negative number')
        config.floors[key] = float(value)
    _check_keys(table_raw, situations, 'conduct.table')
    degrees = {d.name.lower() for d in Degree if d != Degree.NONE}
    actions = {a.value for a in ConductAction}
    for situation, row in table_raw.items():
        row = _mapping(row, f'conduct.table.{situation}')
        _check_keys(row, degrees, f'conduct.table.{situation}')
        for degree, action in row.items():
            if action not in actions:
                raise ConfigError(f'conduct.table.{situation}.{degree}: expected one of {sorted(actions)}')
            config.table[situation][degree] = action
    if not 0.0 < config.medium_ratio <= config.heavy_ratio:
        raise ConfigError('conduct: expected 0 < medium_ratio <= heavy_ratio')
    if config.slow_speed_mps <= 0.0:
        raise ConfigError('conduct.slow_speed_mps: must be positive')
    return config


def parse_gate_parts(
    settings_raw: Mapping[str, Any], closers_raw: Mapping[str, Any], map_match_raw: Mapping[str, Any],
    buffer_raw: Mapping[str, Any], sources: Mapping[str, SourceConfig], where: str = '',
    initial_raw: Optional[Mapping[str, Any]] = None,
) -> MotionGateConfig:
    """門の設定 (設定・閉じる理由の部品・照合・スキャンの重ね・全周の照合 (initial_raw が None ならしない))。
    where は場所の前置き (例 'gate.')。"""
    settings = _dataclass_from(GateSettings, settings_raw, f'{where}settings' if where else 'gate')
    for key, kind in (('estimator', SourceKind.POSE), ('fused', SourceKind.POSE), ('motion', SourceKind.MOTION),
                      ('reference', SourceKind.REFERENCE)):
        _source(sources, getattr(settings, key), kind, f'{where}settings.{key}' if where else f'gate.{key}')
    if settings.gnss:
        _source(sources, settings.gnss, SourceKind.POSE, f'{where}settings.gnss' if where else 'gate.gnss')

    closers = []
    for name, body in closers_raw.items():
        at = f'{where}closers.{name}'
        body = _mapping(body, at)
        close_after = _number(body, 'close_after_sec', 0.0, at)
        active_in_grace = body.get('active_in_grace', False)
        if not isinstance(active_in_grace, bool):
            raise ConfigError(f'{at}.active_in_grace: expected true or false')
        enabled = _enabled(body, at)
        detector = _parse_detector(name, {k: v for k, v in body.items() if k not in GATE_CLOSER_KEYS}, sources,
                                   None, None)
        if enabled:
            closers.append(CloserConfig(name, detector.cls(name, detector.params, detector.sources), close_after,
                                        active_in_grace, detector.type, dict(detector.sources)))
    if not closers:
        raise ConfigError(f'{where}closers: at least one enabled closer is required')

    map_match = _dataclass_from(MapMatchConfig, map_match_raw, f'{where}map_match')
    _check_keys(buffer_raw, set(vars(ScanBufferConfig())) - {'reference', 'motion'}, f'{where}scan_buffer')
    scan_buffer = _dataclass_from(
        ScanBufferConfig, dict(buffer_raw, reference=settings.reference, motion=settings.motion),
        f'{where}scan_buffer')
    initial = None
    if initial_raw is not None:
        initial = _dataclass_from(InitialSearchConfig, initial_raw, f'{where}initial_search')
        if not settings.gnss:
            raise ConfigError(f'{where}initial_search: requires settings.gnss (the search is around the GNSS position)')
    return MotionGateConfig(settings, closers, map_match, scan_buffer,
                            {name: source.kind for name, source in sources.items()}, initial)


def _parse_gate(
    raw: Mapping[str, Any], sources: Mapping[str, SourceConfig], detectors: Mapping[str, DetectorConfig],
    resetters: List[Dict[str, Any]],
) -> Tuple[Optional[MotionGateConfig], float, str]:
    """gate 節。(門の設定 (無効なら None), 周期, 手動の初期姿勢のトピック)。"""
    _check_keys(raw, GATE_KEYS, 'gate')
    period_sec = _number(raw, 'period_sec', 0.2, 'gate')
    if period_sec <= 0.0:
        raise ConfigError('gate.period_sec: must be positive')
    manual = raw.get('manual_pose_topic', '')
    if not isinstance(manual, str):
        raise ConfigError('gate.manual_pose_topic: expected a string')
    enabled = raw.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ConfigError('gate.enabled: expected true or false')
    observe_only = raw.get('observe_only', False)
    if not isinstance(observe_only, bool):
        raise ConfigError('gate.observe_only: expected true or false')
    if not raw or not enabled:
        return None, period_sec, manual
    gate = parse_gate_parts(
        _mapping(raw.get('settings'), 'gate.settings'), _mapping(raw.get('closers'), 'gate.closers'),
        _mapping(raw.get('map_match'), 'gate.map_match'), _mapping(raw.get('scan_buffer'), 'gate.scan_buffer'),
        sources, 'gate.',
        None if raw.get('initial_search') is None else _mapping(raw['initial_search'], 'gate.initial_search'))
    for closer in gate.closers:
        if closer.name in detectors:
            raise ConfigError(f'gate.closers.{closer.name}: the name is used by a detector (status shows both)')
    roles = {item.get('role', 'estimator') for item in resetters}
    if not set(RESETTER_ROLES) <= roles:
        raise ConfigError('gate: recovery.resetters requires both roles (role: estimator and role: fused)')
    gate.observe_only = observe_only
    return gate, period_sec, manual


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


RECOVERY_KEYS = {'convergence', 'convergence_mode', 'matcher', 'scan_buffer', 'generators', 'scorers', 'selection', 'observe',
                 'resetters', 'actuators'}
CONVERGENCE_MODES = {'all', 'any'}
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
    observe = None
    if recovery.get('observe') is not None:
        observe = _dataclass_from(
            ObserveConfig, _mapping(recovery['observe'], 'recovery.observe'), 'recovery.observe')
        _source(sources, observe.motion, SourceKind.MOTION, 'recovery.observe.motion')

    matcher =_mapping(recovery.get('matcher'), 'recovery.matcher')
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
    for i, item in enumerate(resetters):
        if item.get('role', 'estimator') not in RESETTER_ROLES:
            raise ConfigError(f'recovery.resetters[{i}].role: expected one of {list(RESETTER_ROLES)}')
    actuators = _plugin_list(recovery.get('actuators'), 'recovery.actuators')
    return {'generators': generators, 'scorers': scorers, 'selection': selection, 'observe': observe,
            'matcher_cls': matcher_cls,
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


def _convergence_mode(value: Any) -> str:
    if value not in CONVERGENCE_MODES:
        raise ConfigError(f'recovery.convergence_mode: expected one of {sorted(CONVERGENCE_MODES)}, got {value!r}')
    return value


def _detector_names(names: List[str], detectors: Mapping[str, DetectorConfig], where: str) -> List[str]:
    for name in names:
        detector = detectors.get(name)
        if detector is None:
            raise ConfigError(f'{where}: unknown detector {name!r}')
    return [name for name in names if detectors[name].enabled]


def profile_names(profile: str) -> List[str]:
    """プロファイルの指定を名前の列にする。カンマで区切ると、左から順に重ねる。'' は基本設定。"""
    names = [name.strip() for name in profile.split(',')] if profile else []
    if any(not name for name in names):
        raise ConfigError(f'profile {profile!r}: empty name')
    if len(set(names)) != len(names):
        raise ConfigError(f'profile {profile!r}: duplicated name')
    return names


def parse_config(
    raw: Mapping[str, Any], profile: str = '', profiles: Optional[Mapping[str, Mapping[str, Any]]] = None,
) -> SupervisorConfig:
    """辞書から設定を作る。profiles はプロファイルの名前と、重ねる部分の設定。"""
    raw = _mapping(raw, 'config')
    profiles = dict(profiles or {})
    merged = dict(raw)
    for name in profile_names(profile):
        if name not in profiles:
            raise ConfigError(f'profile {name!r} is not defined (profiles: {sorted(profiles)})')
        override = _mapping(profiles[name], f'profiles.{name}')
        if 'profiles' in override or 'version' in override or 'gate' in override:
            raise ConfigError(f'profiles.{name}: cannot override version, profiles or gate '
                              '(the gate is built at startup)')
        merged = merge(merged, override)
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
    gate, gate_period_sec, manual_pose_topic = _parse_gate(
        _mapping(merged.get('gate'), 'gate'), sources, detectors, recovery_parts['resetters'])
    if gate is not None and gate_period_sec > period_sec:
        raise ConfigError('gate.period_sec: must not exceed period_sec')
    if gate is not None and gate.observe_only:
        # 判定だけのときは、監督も何もしない (プロファイルが復旧を有効にしても、通知だけにする)
        machine.recovery_enabled = False

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
        convergence_mode=_convergence_mode(recovery.get('convergence_mode', 'all')),
        anchors=anchors, safety=safety, period_sec=period_sec, **recovery_parts,
        gate=gate, gate_period_sec=gate_period_sec, manual_pose_topic=manual_pose_topic,
        conduct=_parse_conduct(_mapping(merged['conduct'], 'conduct')) if 'conduct' in merged else None,
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
