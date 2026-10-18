"""判定・総合・状態遷移・アンカー・安全の信号を束ねる (ROS に依存しない)。

ノードもオフライン評価も、周期ごとに Supervisor.step(snapshot) を呼ぶ。

  1. 判定が使う地図が替わっていたら、判定の内部状態・累積・アンカーを捨てて猶予を置く
  2. 有効な判定のインスタンスごとに Evidence を求める
  3. 総合する (猶予の間は累積を進めない)
  4. 状態遷移。復旧の収束の確認には、recovery.convergence のインスタンスを使う
  5. 信頼できる周期ならアンカーを残す
  6. 安全の信号 (footprint が計画用地図の致死セルにかかっているか) を求める
  7. イベントを作る

外からの再初期化 (人による上書き) は external_reinit で知らせる。プロファイル・判定の有効・無効・重みは、
走行中に set_profile・set_disabled・set_weight で変えられる。
"""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Set

from .anchors import Anchor, AnchorStore
from .config import SupervisorConfig
from .geometry import Pose2
from .interfaces import (
    Candidate, Detector, Evidence, FusionResult, FusionStrategy, InstanceInfo, RecoveryContext, SourceKind, Status)
from .recovery.planner import RecoveryPlan, RecoveryPlanner
from .safety import FootprintChecker, SafetyResult
from .scan_buffer import ScanBuffer
from .snapshot import SampleStore, Snapshot
from .state_machine import Action, Decision, State, StateMachine


@dataclass
class Event:
    """イベント。name は前置きを含む (例: localization.state_changed)。level は info / warn / error。"""
    name: str
    level: str
    fields: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Directive:
    """推定器や周りへの働きかけの指示 (ROS 側の対処と送り先が実行する)。

      isolate : blame の情報源を切り離す
      reinit  : candidate の姿勢で推定器を初期化し直す
      restore : 切り離したものを戻す
    """
    kind: str
    blame: str = ''
    candidate: Optional[Candidate] = None


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
    safety: Optional[SafetyResult] = None
    isolated_sources: List[str] = field(default_factory=list)
    last_event: str = ''
    events: List[Event] = field(default_factory=list)
    directives: List[Directive] = field(default_factory=list)
    plan: Optional[RecoveryPlan] = None
    # 異常を確認してから戻るまで保持する blame (切り離す対象を決める)
    blame: str = ''


def make_store(config: SupervisorConfig) -> SampleStore:
    return SampleStore(
        config.kinds(), tuple(config.motion_priority), config.horizon_sec,
        config.max_extrapolation_sec, config.alignment)


class Supervisor:
    def __init__(self, config: SupervisorConfig) -> None:
        self._base = config
        self._profile = config.profile
        self._disabled: Set[str] = set()
        self._weights: Dict[str, float] = {}
        self._grace_until: Optional[float] = None
        self._suspect_since: Optional[float] = None
        self._odometer = 0.0
        self._last_motion: Optional[Pose2] = None
        self._map_versions: Dict[str, int] = {}
        self._in_lethal = False
        self._last_event = ''
        self._pending: List[Event] = []
        self._last_scan: Optional[float] = None
        self._reinit_pending = False
        self._next_plan = 0.0
        self._plan_attempts = 0
        self._isolated_blame = ''
        self._check_status: Dict[str, Status] = {}
        self._build(config)

    def _build(self, config: SupervisorConfig) -> None:
        """設定に、無効にした判定と重みの指定を重ねて作る。"""
        self.config = config
        enabled = {name: d for name, d in config.enabled_detectors().items() if name not in self._disabled}
        self.detectors: Dict[str, Detector] = {
            name: d.cls(name, d.params, d.sources) for name, d in enabled.items()}
        infos = [InstanceInfo(
            name=name, subjects=self.detectors[name].subjects(), sources=list(d.sources.values()),
            weight=self._weights.get(name, d.weight), allowance=d.allowance, decay=d.decay,
            max_score=d.max_score, options=d.options)
            for name, d in enabled.items()]
        self.fusion: FusionStrategy = config.fusion_cls(
            config.fusion_type, config.fusion_params, infos,
            config.independent_sources(), config.derived_sources())
        self.machine = StateMachine(config.machine)
        self.anchors = AnchorStore(config.anchors) if config.anchors is not None else None
        self.safety = FootprintChecker(config.safety) if config.safety is not None else None
        self.scans: Optional[ScanBuffer] = config.scan_buffer.make() if config.scan_buffer is not None else None
        self.planner = self._make_planner(config)
        self._references = sorted({
            source for d in enabled.values() for source in d.sources.values()
            if config.sources[source].kind == SourceKind.REFERENCE})
        self._suspect_since = None
        self._reinit_pending = False
        self._check_status = {}

    @staticmethod
    def _make_planner(config: SupervisorConfig) -> Optional[RecoveryPlanner]:
        if not config.generators:
            return None
        return RecoveryPlanner(
            [g.cls(name, g.params, g.sources) for name, g in config.generators.items()],
            [s.cls(name, s.params, s.sources) for name, s in config.scorers.items()],
            list(config.scorers.values()), config.selection, config.matcher_cls('matcher', config.matcher_params))

    def _rebuild(self) -> None:
        self._build(self._base.with_profile(self._profile))

    @property
    def state(self) -> State:
        return self.machine.state

    @property
    def profile(self) -> str:
        return self._profile

    @property
    def suspect_since(self) -> Optional[float]:
        return self._suspect_since

    def set_profile(self, profile: str) -> None:
        """プロファイルを切り替える ('' は基の設定)。判定・総合・状態遷移・アンカーは作り直す (状態は NORMAL から)。"""
        if profile and profile not in self._base.profiles:
            raise ValueError(f'unknown profile {profile!r} (profiles: {sorted(self._base.profiles)})')
        before = self._profile
        self._profile = profile
        self._rebuild()
        self._emit('profile_changed', 'info', {'from': before, 'to': profile})

    def set_disabled(self, names: Iterable[str]) -> None:
        """名前を並べた判定のインスタンスを無効にする (プロファイルの上に重ねる)。作り直す。"""
        names = set(names)
        unknown = sorted(names - set(self._base.detectors))
        if unknown:
            raise ValueError(f'unknown detectors {unknown}')
        if names != self._disabled:
            self._disabled = names
            self._rebuild()
            self._emit('detectors_changed', 'info', {'disabled': ','.join(sorted(names))})

    def set_weight(self, name: str, weight: Optional[float]) -> None:
        """判定のインスタンスの重み (None は設定の値に戻す)。作り直さずに効く。"""
        if name not in self._base.detectors:
            raise ValueError(f'unknown detector {name!r}')
        if weight is None:
            self._weights.pop(name, None)
            configured = self.config.detectors.get(name)
            weight = configured.weight if configured is not None else 1.0
        else:
            self._weights[name] = weight
        if name in self.fusion.instances:
            self.fusion.instances[name].weight = weight

    def reset(self) -> None:
        """累積・判定の内部状態・アンカーを捨てる。"""
        self.fusion.reset()
        for detector in self.detectors.values():
            detector.reset()
        if self.anchors is not None:
            self.anchors.clear()
        if self.scans is not None:
            self.scans.clear()
        if self.planner is not None:
            self.planner.reset()
        self._reinit_pending = False

    def start_grace(self, now: float, duration: float) -> None:
        self._grace_until = max(self._grace_until or now, now + duration)

    def external_reinit(self, now: float, topic: str) -> None:
        """外から推定器が初期化し直された (人による上書き)。復旧をやめて、状態を NORMAL に戻し、捨てて猶予を置く。"""
        before = self.machine.state
        self.machine = StateMachine(self.config.machine)
        self._suspect_since = None
        self.reset()
        self.start_grace(now, self.config.grace_after_reset_sec)
        self._emit('external_reinit', 'warn', {'topic': topic, 'state': before.name})

    def step(self, snapshot: Snapshot) -> StepResult:
        now = snapshot.stamp
        if self._grace_until is None:
            self._grace_until = now + self.config.startup_grace_sec
        self._check_maps(snapshot)
        self._update_scans(snapshot)
        in_grace = now < self._grace_until

        evidences = {name: detector.evaluate(snapshot) for name, detector in self.detectors.items()}
        self._check_events(evidences)
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
            self._reinit_pending = True
            self._next_plan = now
            self._plan_attempts = 0
        if Action.RESTORE in decision.actions and self.config.grace_after_recovery_sec > 0.0:
            self.start_grace(now, self.config.grace_after_recovery_sec)
        if decision.event:
            self._last_event = decision.event
        directives = self._directives(decision)
        plan = self._recover(snapshot, state, directives)

        normal = state == State.NORMAL
        for name, detector in self.detectors.items():
            detector.after_step(evidences[name], normal)

        trusted = normal and not in_grace and self._trusted(evidences, fusion)
        self._update_anchors(snapshot, trusted)
        safety = self._check_safety(snapshot)

        self._state_events(state_before, decision, fusion)
        events, self._pending = self._pending, []
        return StepResult(
            stamp=now, state=state, decision=decision, fusion=fusion, evidences=evidences,
            trusted=trusted, in_grace=in_grace, hold_requested=self.machine.hold_requested, safety=safety,
            isolated_sources=self._isolated(), last_event=self._last_event, events=events,
            directives=directives, plan=plan, blame=self.machine.blame)

    def _check_events(self, evidences: Dict[str, Evidence]) -> None:
        """判定のインスタンスが、正常と異常の間で変わったとき (判定できない周期は飛ばす) にイベントを出す。
        異常の原因を、あとから値で確かめられるようにする。"""
        for name, evidence in evidences.items():
            if evidence.status == Status.UNKNOWN:
                continue
            previous = self._check_status.get(name)
            self._check_status[name] = evidence.status
            if previous is None and evidence.status == Status.NORMAL or previous == evidence.status:
                continue
            fields: Dict[str, Any] = {'check': name, 'to': evidence.status.value, 'score': round(evidence.score, 3)}
            fields.update({f'v_{key}': round(value, 3) for key, value in evidence.values.items()})
            self._emit('check_changed', 'info', fields)

    def _directives(self, decision: Decision) -> List[Directive]:
        directives = []
        if Action.ISOLATE in decision.actions:
            self._isolated_blame = decision.blame
            directives.append(Directive('isolate', decision.blame))
        if Action.RESTORE in decision.actions:
            directives.append(Directive('restore', self._isolated_blame))
            self._isolated_blame = ''
        return directives

    def _update_scans(self, snapshot: Snapshot) -> None:
        cfg = self.config.scan_buffer
        if self.scans is None or cfg is None:
            return
        scan = snapshot.reference(cfg.reference).scan
        if scan is None or scan.stamp == self._last_scan:
            return
        self._last_scan = scan.stamp
        motion = snapshot.motion_at(cfg.motion, scan.stamp)
        if motion is not None:
            self.scans.add(scan, motion)

    def _recover(self, snapshot: Snapshot, state: State, directives: List[Directive]) -> Optional[RecoveryPlan]:
        """再初期化が要るあいだ、retry_sec ごとに候補を計画する。選べたら reinit の指示を出す。"""
        now = snapshot.stamp
        if state != State.RECOVERING:
            self._reinit_pending = False
        if not self._reinit_pending or now < self._next_plan:
            return None
        if self.planner is None:
            self._reinit_pending = False
            self._emit('reinit_skipped', 'error', {'reason': 'no_generators'})
            return None
        self._plan_attempts += 1
        self._next_plan = now + self.config.selection.retry_sec
        context = RecoveryContext(
            snapshot=snapshot, anchors=self.anchors, suspect_since=self._suspect_since, scans=self.scans,
            safety=self.config.safety)
        plan = self.planner.plan(context)
        self._plan_events(plan)
        if plan.chosen is not None:
            directives.append(Directive('reinit', self.machine.blame, plan.chosen))
            self._after_reinit(now)
        return plan

    def _plan_events(self, plan: RecoveryPlan) -> None:
        for candidate in plan.candidates:
            fields: Dict[str, Any] = {
                'origin': candidate.origin, **_pose_fields(candidate.pose),
                'total': '' if candidate.total is None else round(candidate.total, 3),
                'vetoed_by': candidate.vetoed_by}
            fields.update({f'score_{k}': round(v, 3) for k, v in candidate.scores.items()})
            self._emit('recovery_candidate', 'info', fields)
        common = {'attempt': self._plan_attempts, 'candidates': len(plan.candidates),
                  'elapsed_ms': round(plan.elapsed_sec * 1000.0),
                  'generators': ','.join(f'{name}:{status}' for name, status in plan.generators.items())}
        if plan.chosen is None:
            self._emit('reinit_skipped', 'warn', {'reason': plan.reason, **common})
        else:
            self._emit('reinit_requested', 'warn', {
                'origin': plan.chosen.origin, **_pose_fields(plan.chosen.pose), 'reason': plan.reason,
                'margin': round(plan.margin, 3), 'blame': self.machine.blame, **common})

    def _after_reinit(self, now: float) -> None:
        """推定器を初期化し直す指示を出した後。姿勢が飛ぶので、累積・判定の内部状態を捨てて猶予を置く。
        アンカーとスキャンのバッファは、再試行のために残す。"""
        self._reinit_pending = False
        self.machine.note_reinit()
        self.fusion.reset()
        for detector in self.detectors.values():
            detector.reset()
        self.start_grace(now, self.config.grace_after_reset_sec)

    def _emit(self, event: str, level: str, fields: Dict[str, Any]) -> None:
        self._pending.append(Event(f'{self.config.event_prefix}.{event}', level, fields))
        self._last_event = event

    def _check_maps(self, snapshot: Snapshot) -> None:
        """判定が使う地図が替わったら (最初に届いたときは除く)、捨てて猶予を置く。"""
        changed = []
        for name in self._references:
            version = snapshot.reference(name).map_version
            previous = self._map_versions.get(name, 0)
            if previous and version != previous:
                changed.append(name)
            self._map_versions[name] = version
        if changed:
            self.reset()
            self.start_grace(snapshot.stamp, self.config.grace_after_reset_sec)
            self._emit('map_changed', 'info', {'sources': ','.join(changed)})

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

    def _isolated(self) -> List[str]:
        blame = self.machine.blame
        if self.machine.hold_requested and blame and blame != 'unknown':
            return [blame]
        return []

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

    def _check_safety(self, snapshot: Snapshot) -> Optional[SafetyResult]:
        cfg = self.config.safety
        if self.safety is None or cfg is None:
            return None
        occupancy = snapshot.reference(cfg.map).occupancy
        aligned = snapshot.pose_aligned(cfg.pose, cfg.max_age_sec)
        if occupancy is None or aligned is None:
            return None
        result = self.safety.check(occupancy, aligned[0].pose)
        if result.in_lethal != self._in_lethal:
            self._in_lethal = result.in_lethal
            self._emit('lethal', 'warn' if result.in_lethal else 'info', {
                'in_lethal': result.in_lethal, 'fraction': round(result.fraction, 3),
                'depth_m': round(result.depth_m, 3)})
        return result

    def _state_events(self, before: State, decision: Decision, fusion: FusionResult) -> None:
        if decision.state == before:
            return
        level = 'info' if decision.state == State.NORMAL else (
            'error' if decision.state == State.DEGRADED else 'warn')
        self._pending.append(Event(f'{self.config.event_prefix}.state_changed', level, {
            'from': before.name, 'to': decision.state.name, 'reason': decision.event,
            'blame': decision.blame or fusion.blame, 'fusion_level': round(fusion.level, 3)}))


def _pose_fields(pose: Pose2) -> Dict[str, float]:
    return {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 3)}
