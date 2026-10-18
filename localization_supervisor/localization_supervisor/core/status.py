"""状態のメッセージ (LocalizationStatus) と診断の中身を、周期の結果から作る (ROS に依存しない)。

ノードは StatusView をメッセージに写すだけにする。監督以外の監視 (watchdog など) も、StatusView を作れば
同じ形のメッセージと診断を出せる。
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .config import SupervisorConfig
from .interfaces import Status
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
    profile: str = ''
    last_event: str = ''
    checks: List[CheckView] = field(default_factory=list)


def status_view(result: StepResult, config: SupervisorConfig, profile: str) -> StatusView:
    checks = []
    for name, evidence in result.evidences.items():
        detector = config.detectors[name]
        checks.append(CheckView(
            name=name, detector=detector.type, sources=list(detector.sources.values()), status=evidence.status,
            score=0.0 if evidence.status == Status.UNKNOWN else evidence.score,
            accumulated=result.fusion.accumulated.get(name, 0.0), values=dict(evidence.values)))
    safety = result.safety
    return StatusView(
        state=result.state, blame=result.fusion.blame, contaminated=list(result.fusion.contaminated),
        isolated_sources=list(result.isolated_sources), hold_requested=result.hold_requested,
        fusion_level=result.fusion.level, trusted=result.trusted, in_grace=result.in_grace,
        footprint_in_lethal=safety is not None and safety.in_lethal,
        lethal_depth_m=-1.0 if safety is None else safety.depth_m,
        profile=profile, last_event=result.last_event, checks=checks)


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
    for check in view.checks:
        text = check.status.value
        if check.status != Status.UNKNOWN:
            text += f' {check.score:.3f}'
        values[f'check.{check.name}'] = text
    return level, message, values
