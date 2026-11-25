"""状態のメッセージ (LocalizationStatus) と診断の中身を、周期の結果から作る (ROS に依存しない)。

ノードは StatusView をメッセージに写すだけにする。監督以外の監視 (watchdog など) も、StatusView を作れば
同じ形のメッセージと診断を出せる。
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .config import SupervisorConfig
from .interfaces import Status
from .motion_gate import GateState, GateStep, MotionGateConfig
from .state_machine import State
from .supervisor import StepResult

# LocalizationCheck.status の値
CHECK_STATUS = {Status.NORMAL: 0, Status.ANOMALY: 1, Status.UNKNOWN: 2}

LEVEL_BY_STATE = {
    State.NORMAL: 'ok',
    State.DEGRADED: 'error',
}


@dataclass
class CheckView:
    name: str
    detector: str
    sources: List[str]
    status: Status
    score: float = 0.0
    accumulated: float = 0.0
    values: Dict[str, float] = field(default_factory=dict)


@dataclass
class StatusView:
    state: State
    blame: str = ''
    contaminated: List[str] = field(default_factory=list)
    isolated_sources: List[str] = field(default_factory=list)
    hold_requested: bool = False
    fusion_level: float = 0.0
    trusted: bool = False
    in_grace: bool = False
    footprint_in_lethal: bool = False
    lethal_depth_m: float = -1.0
    situation: str = ''
    degree: int = 0
    possible_error_m: float = 0.0
    clearance_m: float = -1.0
    speed_limit: float = 0.0
    verify_id: int = 0
    verify_result: str = ''
    profile: str = ''
    last_event: str = ''
    checks: List[CheckView] = field(default_factory=list)


def status_view(result: StepResult, config: SupervisorConfig, profile: str) -> StatusView:
    """監督の周期の結果の状態。門があるときは、監督の状態と門の状態 (gate_state) の重い方を出し、
    門の閉じる理由の部品も判定の並びに足す (門が閉じている間に、走行側が正常と見て動かないため)。"""
    checks = []
    for name, evidence in result.evidences.items():
        detector = config.detectors[name]
        checks.append(CheckView(
            name=name, detector=detector.type, sources=list(detector.sources.values()), status=evidence.status,
            score=0.0 if evidence.status == Status.UNKNOWN else evidence.score,
            accumulated=result.fusion.accumulated.get(name, 0.0), values=dict(evidence.values)))
    safety = result.safety
    state = result.state
    isolated = list(result.isolated_sources)
    trusted = result.trusted
    in_grace = result.in_grace
    if result.gate is not None and config.gate is not None:
        state = max(state, gate_state(result.gate))
        checks.extend(_closer_checks(result.gate, config.gate))
        estimator = config.gate.settings.estimator
        if not result.gate.gate_open and estimator not in isolated:
            isolated.append(estimator)
        trusted = trusted and result.gate.state == GateState.OPEN and not result.gate.in_grace
        in_grace = in_grace or result.gate.in_grace or result.gate.state == GateState.UNCONFIRMED
    return StatusView(
        state=state, blame=result.fusion.blame, contaminated=list(result.fusion.contaminated),
        isolated_sources=isolated, hold_requested=result.hold_requested,
        fusion_level=result.fusion.level, trusted=trusted, in_grace=in_grace,
        footprint_in_lethal=safety is not None and safety.in_lethal,
        lethal_depth_m=-1.0 if safety is None else safety.depth_m,
        profile=profile, last_event=result.last_event, checks=checks, **_conduct_fields(result),
        **_verify_fields(result))


def _verify_fields(result: StepResult) -> dict:
    if result.gate is None or result.gate.last_verify is None:
        return {}
    number, outcome = result.gate.last_verify
    return {'verify_id': number, 'verify_result': outcome}


def _conduct_fields(result: StepResult) -> dict:
    conduct = result.conduct
    if conduct is None:
        return {}
    return {
        'situation': conduct.situation.value, 'degree': int(conduct.degree),
        'possible_error_m': conduct.possible_error_m,
        'clearance_m': -1.0 if conduct.clearance_m is None else conduct.clearance_m,
        'speed_limit': conduct.speed_limit_mps}


GATE_STATES = {
    GateState.UNCONFIRMED: State.NORMAL,
    GateState.OPEN: State.NORMAL,
    GateState.CLOSED: State.ISOLATED,
    GateState.CORRECTING: State.RECOVERING,
}


def gate_state(step: GateStep) -> State:
    """門の状態を監督の状態に写す。確かめる前と開いている間は NORMAL、閉じている間は ISOLATED、止めて戻す間は
    RECOVERING、閉じたまま決まらず止めている間は DEGRADED。"""
    if step.state == GateState.CLOSED and step.hold:
        return State.DEGRADED
    return GATE_STATES[step.state]


def _closer_checks(step: GateStep, config: MotionGateConfig) -> List[CheckView]:
    checks = []
    for closer in config.closers:
        evidence = step.evidences.get(closer.name)
        if evidence is None:
            continue
        checks.append(CheckView(
            name=closer.name, detector=closer.type, sources=list(closer.sources.values()), status=evidence.status,
            score=0.0 if evidence.status == Status.UNKNOWN else evidence.score, values=dict(evidence.values)))
    return checks


def gate_status_view(step: GateStep, config: MotionGateConfig, last_event: str) -> StatusView:
    """門のノードの周期の結果を、監督と同じ形の状態にする。

    確かめる前と門が開いている間は NORMAL、閉じている間は ISOLATED、止めて戻す間は RECOVERING、
    閉じたまま決まらず止めている間は DEGRADED。安全の信号は出さない (lethal_depth_m は -1)。
    """
    state = gate_state(step)
    closed = step.state in (GateState.CLOSED, GateState.CORRECTING)
    checks = _closer_checks(step, config)
    return StatusView(
        state=state, isolated_sources=[config.settings.estimator] if closed else [], hold_requested=step.hold,
        trusted=step.state == GateState.OPEN and not step.in_grace,
        in_grace=step.in_grace or step.state == GateState.UNCONFIRMED, last_event=last_event, checks=checks)


def localization_health(view: StatusView) -> Tuple[str, str, Dict[str, str]]:
    """診断の (レベル 'ok' / 'warn' / 'error', メッセージ, 値)。

    NORMAL は ok、DEGRADED (異常を確認した、または復旧できていない) は error、それ以外は warn。
    値は、状態の要約と、判定のインスタンスごとの "<状態> <score>"。
    """
    level = LEVEL_BY_STATE.get(view.state, 'warn')
    message = view.state.name
    if view.blame:
        message += f' (blame: {view.blame})'
    values = {
        'blame': view.blame,
        'contaminated': ','.join(view.contaminated),
        'hold_requested': str(view.hold_requested),
        'fusion_level': f'{view.fusion_level:.3f}',
        'in_grace': str(view.in_grace),
        'footprint_in_lethal': str(view.footprint_in_lethal),
        'profile': view.profile,
        'last_event': view.last_event,
    }
    if view.situation:
        values.update({
            'situation': view.situation, 'degree': str(view.degree),
            'clearance_m': f'{view.clearance_m:.2f}', 'speed_limit': f'{view.speed_limit:.2f}'})
    for check in view.checks:
        text = check.status.value
        if check.status != Status.UNKNOWN:
            text += f' {check.score:.3f}'
        values[f'check.{check.name}'] = text
    return level, message, values
