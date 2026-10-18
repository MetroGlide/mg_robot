"""判定・総合・状態遷移・アンカーを束ねる (ROS に依存しない)。

ノードもオフライン評価も、周期ごとに Supervisor.step(snapshot) を呼ぶ。

  1. 有効な判定のインスタンスごとに Evidence を求める
  2. 総合する (猶予の間は累積を進めない)
  3. 状態遷移。復旧の収束の確認には、recovery.convergence のインスタンスを使う
  4. 信頼できる周期ならアンカーを残す
  5. イベントを作る
"""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .anchors import Anchor, AnchorStore
from .config import SupervisorConfig
from .geometry import Pose2
from .interfaces import Detector, Evidence, FusionResult, FusionStrategy, InstanceInfo, Status
from .snapshot import SampleStore, Snapshot
from .state_machine import Action, Decision, State, StateMachine


@dataclass
class Event:
    """イベント。name は前置きを含む (例: localization.state_changed)。level は info / warn / error。"""
    name: str
    level: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class StepResult:
    stamp: float
    state: State
    decision: Decision
    fusion: FusionResult
    evidences: Dict[str, Evidence]
    trusted: bool
    in_grace: bool
    hold_requested: bool
    events: List[Event] = field(default_factory=list)


def make_store(config: SupervisorConfig) -> SampleStore:
    return SampleStore(
        config.kinds(), tuple(config.motion_priority), config.horizon_sec,
        config.max_extrapolation_sec, config.alignment)


class Supervisor:
    def __init__(self, config: SupervisorConfig) -> None:
        self._build(config)
        self._grace_until: Optional[float] = None
        self._suspect_since: Optional[float] = None
        self._odometer = 0.0
        self._last_motion: Optional[Pose2] = None

    def _build(self, config: SupervisorConfig) -> None:
        self.config = config
        enabled = config.enabled_detectors()
        self.detectors: Dict[str, Detector] = {
            name: d.cls(name, d.params, d.sources) for name, d in enabled.items()}
        infos = [InstanceInfo(
            name=name, subjects=self.detectors[name].subjects(), sources=list(d.sources.values()),
            weight=d.weight, allowance=d.allowance, decay=d.decay, max_score=d.max_score, options=d.options)
            for name, d in enabled.items()]
        self.fusion: FusionStrategy = config.fusion_cls(
            config.fusion_type, config.fusion_params, infos,
            config.independent_sources(), config.derived_sources())
        self.machine = StateMachine(config.machine)
        self.anchors = AnchorStore(config.anchors) if config.anchors is not None else None
        self._weights = {name: d.weight for name, d in enabled.items()}

    @property
    def state(self) -> State:
        return self.machine.state

    def set_profile(self, profile: str) -> None:
        """プロファイルを切り替える。判定・総合・状態遷移・アンカーは作り直す (状態は NORMAL から)。"""
        self._build(self.config.with_profile(profile))
        self._suspect_since = None

    def set_weight(self, name: str, weight: float) -> None:
        self.fusion.instances[name].weight = weight

    def reset(self) -> None:
        """累積・判定の内部状態・アンカーを捨てる (外からの再初期化、地図の切り替え)。"""
        self.fusion.reset()
        for detector in self.detectors.values():
            detector.reset()
        if self.anchors is not None:
            self.anchors.clear()

    def start_grace(self, now: float, duration: float) -> None:
        self._grace_until = max(self._grace_until or now, now + duration)

    def step(self, snapshot: Snapshot) -> StepResult:
        now = snapshot.stamp
        if self._grace_until is None:
            self._grace_until = now + self.config.startup_grace_sec
        in_grace = now < self._grace_until

        evidences = {name: detector.evaluate(snapshot) for name, detector in self.detectors.items()}
        fusion = self.fusion.update(evidences, active=not in_grace)
        state_before = self.machine.state
        decision = self.machine.step(now, fusion, self._converged(evidences))
        state = decision.state

        if state_before == State.NORMAL and state == State.SUSPECT:
            self._suspect_since = now
        if state == State.NORMAL and state_before != State.NORMAL:
            self._suspect_since = None
            self.fusion.reset()
        if Action.REINIT in decision.actions:
            self.fusion.reset()
        if Action.RESTORE in decision.actions and self.config.grace_after_recovery_sec > 0.0:
            self.start_grace(now, self.config.grace_after_recovery_sec)

        normal = state == State.NORMAL
        for name, detector in self.detectors.items():
            detector.after_step(evidences[name], normal)

        trusted = normal and not in_grace and self._trusted(evidences, fusion)
        self._update_anchors(snapshot, trusted)

        events = self._events(state_before, decision, fusion)
        return StepResult(
            stamp=now, state=state, decision=decision, fusion=fusion, evidences=evidences,
            trusted=trusted, in_grace=in_grace, hold_requested=self.machine.hold_requested, events=events)

    @property
    def suspect_since(self) -> Optional[float]:
        return self._suspect_since

    def _converged(self, evidences: Dict[str, Evidence]) -> Optional[bool]:
        """recovery.convergence のインスタンスが、すべて正常なら True、どれかが異常なら False。"""
        statuses = [evidences[name].status for name in self.config.convergence if name in evidences]
        if any(s == Status.ANOMALY for s in statuses):
            return False
        if statuses and all(s == Status.NORMAL for s in statuses):
            return True
        return None

    def _trusted(self, evidences: Dict[str, Evidence], fusion: FusionResult) -> bool:
        if fusion.level > 0.0 or fusion.anomaly:
            return False
        required = [evidences[name].status for name in self.config.trust_require_normal if name in evidences]
        return all(s == Status.NORMAL for s in required)

    def _update_anchors(self, snapshot: Snapshot, trusted: bool) -> None:
        cfg = self.config.anchors
        if self.anchors is None or cfg is None:
            return
        motion = snapshot.motion_at(cfg.motion, snapshot.stamp)
        if motion is not None:
            if self._last_motion is not None:
                self._odometer += math.hypot(motion[0] - self._last_motion[0], motion[1] - self._last_motion[1])
            self._last_motion = motion
        self.anchors.prune(snapshot.stamp, self._odometer)
        if not trusted or motion is None:
            return
        aligned = snapshot.pose_aligned(cfg.pose)
        if aligned is None:
            return
        self.anchors.add(Anchor(snapshot.stamp, aligned[0], motion, self._odometer))

    def _events(self, before: State, decision: Decision, fusion: FusionResult) -> List[Event]:
        prefix = self.config.event_prefix
        events: List[Event] = []
        if decision.state != before:
            level = 'info' if decision.state == State.NORMAL else (
                'error' if decision.state == State.DEGRADED else 'warn')
            events.append(Event(f'{prefix}.state_changed', level, {
                'from': before.name, 'to': decision.state.name, 'reason': decision.event,
                'blame': decision.blame or fusion.blame, 'fusion_level': round(fusion.level, 3)}))
        return events
