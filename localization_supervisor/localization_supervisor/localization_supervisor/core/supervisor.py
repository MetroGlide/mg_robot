"""判定・総合・状態遷移・アンカー・安全の信号を束ねる (ROS に依存しない)。

ノードもオフライン評価も、周期ごとに Supervisor.step(snapshot) を呼ぶ。

  1. 判定が使う地図が替わっていたら、判定の内部状態・累積・アンカーを捨てて猶予を置く
  2. 有効な判定のインスタンスごとに Evidence を求める
  3. 総合する (猶予の間は累積を進めない)
  4. 状態遷移。復旧の収束の確認には、recovery.convergence のインスタンスを使う
  5. 信頼できる周期ならアンカーを残す
  6. 安全の信号 (footprint が計画用地図の致死セルにかかっているか) を求める
  7. イベントを作る

外からの再初期化 (人による上書き) は external_reinit で知らせる。プロファイルは、走行中に set_profile で変えられる。

門 (設定の gate、core/motion_gate.py) があるときは、step を門の周期で呼ぶ。判定・総合・状態遷移 (上の 1〜7) は
period_sec ごとに回し、門は毎回回す。門との分担:
  - 置き直しと、推定器を融合に入れるかの門、一時停止は、門が決める。監督の復旧は、再初期化の指示で門を閉じさせ
    (force_close)、候補を作るものの候補を足して、門の照合の規則で決める (監督の候補の評価と選び方は使わない)
  - 門が置き直したら、監督は累積・判定の内部状態を捨てて猶予を置く (置き直しによる飛びを異常と見ない)
  - 監督が GNSS を疑っている間は、門は GNSS を使う閉じる理由の部品と GNSS の候補を使わない
  - 外からの再初期化 (/initialpose など) は見ない。外からの推定器の飛びは、門が閉じる理由の部品で判定する
  - 観測を増やす指示 (observe) は出さない。決まらないときは、門が止めて照合し、解いて少し走らせる
"""
import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .anchors import Anchor, AnchorStore
from .conduct import Action as ConductAction, Conduct, ConductInputs, ConductPlanner, Situation
from .config import SupervisorConfig
from .detectors.common import position_std
from .geometry import Pose2
from .interfaces import (
    Candidate, Detector, Evidence, FusionResult, FusionStrategy, InstanceInfo, PoseSample, RecoveryContext,
    SourceKind, Status)
from .map_match import CandidatePose, MatchDecision
from .motion_gate import GateState, GateStep, MatchRequest, MotionGate
from .recovery.planner import RecoveryPlan, RecoveryPlanner
from .residuals import ResidualStats, ResidualTracker
from .safety import FootprintChecker, SafetyResult
from .scan_buffer import ScanBuffer
from .sensor_sets import ALL as ALL_SENSORS, SensorSetTracker
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
      observe : 止まったままでは決められないので、観測を増やす動作 (その場の首振りなど) をしてほしい

    targets は reinit の送り先: all (推定器と融合)、fused (融合だけ。門が推定器を採って、融合だけを直すとき)。
    """
    kind: str
    blame: str = ''
    candidate: Optional[Candidate] = None
    targets: str = 'all'


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
    # 残差の統計 (設定の residuals ごと)
    residuals: Dict[str, ResidualStats] = field(default_factory=dict)
    # 受け取る側に出す、分散の倍率 (設定の residuals ごと。残差の a の分散にかける。1 以上、max_scale 以下)
    applied_scales: Dict[str, float] = field(default_factory=dict)
    # 一時停止の理由 (門があるときの門の理由。correction / undecided)
    hold_reason: str = ''
    # 門の周期の結果 (門がなければ None)
    gate: Optional[GateStep] = None
    # 走り方 (状況・程度・速度の上限。設定の conduct と門がなければ None)
    conduct: Optional[Conduct] = None
    # 今のセンサの組 (空は全部。設定の sensor_sets がなければ空)
    sensor_set: str = ''


def make_store(config: SupervisorConfig, track_scales: bool = True) -> SampleStore:
    """標本の置き場。track_scales なら、設定の residuals で a に指定した情報源の標本の分散から、
    出した倍率を割り戻す (受け取る側が倍率をかける実機。記録を読むオフライン評価は、倍率がかかっていないので
    False)。倍率の記録は note_scale で行う。"""
    store = SampleStore(
        config.kinds(), tuple(config.motion_priority), config.horizon_sec,
        config.max_extrapolation_sec, config.alignment, config.holds())
    if track_scales:
        for residual in config.residuals.values():
            store.track_scale(residual.a, residual.scale_delay_sec)
    return store


class Supervisor:
    def __init__(self, config: SupervisorConfig) -> None:
        self._base = config
        self._profile = config.profile
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
        # 観測を増やす求め (recovery.observe): 最後に進んだ (再初期化した・収束を確認した) 時刻、最後に求めた時刻、
        # 最後の再初期化からの動き (走行距離 [m]、回転 [rad]。NORMAL に戻るまで残す)
        self._progress_at = 0.0
        self._observe_at: Optional[float] = None
        self._since_reinit: Optional[List[float]] = None
        self._observe_motion: Optional[Pose2] = None
        # 門 (プロファイルで変えない。状態を保つため、プロファイルを変えても作り直さない)
        self.gate: Optional[MotionGate] = MotionGate(config.gate) if config.gate is not None else None
        # 走り方は、確かめた姿勢を門が持つので、門があるときだけ
        self.conduct: Optional[ConductPlanner] = (
            ConductPlanner(config.conduct) if config.conduct is not None and self.gate is not None else None)
        self._clearance: Optional[float] = None
        self.sensor_sets: Optional[SensorSetTracker] = (
            SensorSetTracker(config.sensor_sets, {n: c.kind for n, c in config.sources.items()})
            if config.sensor_sets else None)
        self._last: Optional[StepResult] = None
        self._next_supervise = 0.0
        # 監督の復旧を門に任せて、門が決めるのを待っている。切り離しの間 (再初期化を求める前) に門が決めた
        self._awaiting_gate = False
        self._gate_decided = False
        self._build(config)

    def _build(self, config: SupervisorConfig) -> None:
        """設定から、判定・総合・状態遷移・アンカーなどを作る。"""
        self.config = config
        enabled = config.enabled_detectors()
        self.detectors: Dict[str, Detector] = {
            name: d.cls(name, d.params, d.sources) for name, d in enabled.items()}
        infos = [InstanceInfo(
            name=name, subjects=self.detectors[name].subjects(), sources=list(d.sources.values()),
            weight=d.weight, allowance=d.allowance, decay=d.decay,
            max_score=d.max_score, options=d.options, vouches=d.vouch)
            for name, d in enabled.items()]
        self.fusion: FusionStrategy = config.fusion_cls(
            config.fusion_type, config.fusion_params, infos,
            config.independent_sources(), config.derived_sources())
        self.machine = StateMachine(config.machine)
        self.anchors = AnchorStore(config.anchors) if config.anchors is not None else None
        self.safety = FootprintChecker(config.safety) if config.safety is not None else None
        self.scans: Optional[ScanBuffer] = config.scan_buffer.make() if config.scan_buffer is not None else None
        self.residuals = {name: ResidualTracker(cfg) for name, cfg in config.residuals.items()}
        self._residual_high: Dict[str, bool] = {name: False for name in config.residuals}
        # 切り離した GNSS を、門が確かめた姿勢と合うまで門と復旧に使わせない (門の設定 gnss_hold_after_distrust)
        self._gnss_held = False
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
        """プロファイルを切り替える ('' は基本設定)。判定・総合・状態遷移・アンカーは作り直す (状態は NORMAL から)。"""
        if profile and profile not in self._base.profiles:
            raise ValueError(f'unknown profile {profile!r} (profiles: {sorted(self._base.profiles)})')
        before = self._profile
        self._profile = profile
        self._rebuild()
        self._emit('profile_changed', 'info', {'from': before, 'to': profile})

    def reset(self) -> None:
        """累積・判定の内部状態・アンカーを捨てる。"""
        self.fusion.reset()
        for detector in self.detectors.values():
            detector.reset()
        if self.anchors is not None:
            self.anchors.clear()
        if self.scans is not None:
            self.scans.clear()
        for tracker in self.residuals.values():
            tracker.clear()
        if self.planner is not None:
            self.planner.reset()
        self._reinit_pending = False
        self._since_reinit = None

    def start_grace(self, now: float, duration: float) -> None:
        self._grace_until = max(self._grace_until or now, now + duration)

    def external_reinit(self, now: float, topic: str) -> None:
        """外から推定器が初期化し直された (人による上書き)。復旧をやめて、状態を NORMAL に戻し、捨てて猶予を置く。

        今のプロファイルの external_reinit.topics にないトピックからのものと、門があるときは無視する。
        """
        if self.gate is not None or topic not in self.config.external_reinit_topics:
            return
        before = self.machine.state
        self.machine = StateMachine(self.config.machine)
        self._suspect_since = None
        self.reset()
        self.start_grace(now, self.config.grace_after_reset_sec)
        self._emit('external_reinit', 'warn', {'topic': topic, 'state': before.name})

    def request_verify(self, now: float) -> Optional[int]:
        """走行側からの確かめの要求 (門があるときだけ)。受け付ければ番号を返す。"""
        return None if self.gate is None else self.gate.request_verify(now)

    def manual_pose(self, pose: Pose2) -> None:
        """手動の初期姿勢 (門があるときだけ。門がその周りを照合してから置き直す)。"""
        if self.gate is not None:
            self.gate.manual_pose(pose)

    def step(self, snapshot: Snapshot, match: Optional[Tuple[MatchRequest, MatchDecision]] = None) -> StepResult:
        """周期の処理。門があるときは、門の照合の結果 (前の周期の request を照合したもの) を match に渡す。"""
        if self.gate is None:
            return self._supervise(snapshot)
        now = snapshot.stamp
        fresh = self._last is None or now >= self._next_supervise
        if fresh:
            result = self._supervise(snapshot)
            # 門の周期の揺らぎで 1 周期遅れないよう、門の周期の半分だけ早めに許す
            self._next_supervise = now + self.config.period_sec - 0.5 * self.config.gate_period_sec
            self._last = result
        else:
            result = dataclasses.replace(self._last, stamp=now, events=[], directives=[], plan=None)
        gate_step = self.gate.step(snapshot, match, self._gate_distrust())
        self._apply_gate(now, result, gate_step)
        self._release_held_gnss(snapshot, gate_step, result)
        if self.conduct is not None:
            result.conduct = self._update_conduct(now, gate_step, result, fresh)
        return result

    def _update_sensor_set(self, snapshot: Snapshot) -> str:
        """情報源の欠けから今のセンサの組を決め、変わればイベントを出す。"""
        if self.sensor_sets is None:
            return ALL_SENSORS
        current, changed = self.sensor_sets.update(snapshot)
        if changed is not None:
            before, after = changed
            self._emit('sensor_set_changed', 'warn' if after else 'info',
                       {'from': before or 'all', 'to': after or 'all'})
        return current

    def _update_conduct(self, now: float, gate_step: GateStep, result: StepResult, fresh: bool) -> Conduct:
        """監督と門の状態から状況を決め、走り方を決める。復旧が無効 (通知だけ・判定だけ) なら、減速は出さない。

        正常に戻すのは、門が地図との照合で確かめ、その後の監督の新しい判定がどれも異常でないとき (地図が現場と食い違うと、
        照合は誤った姿勢も確かめるので、GNSS などとの食い違いが残る間は戻さない)。
        """
        cfg = self.conduct.config
        state = self.machine.state
        since = self.gate.since_confirmed_m()
        if gate_step.state == GateState.UNCONFIRMED:
            situation = Situation.UNINITIALIZED
        elif state in (State.ISOLATED, State.RECOVERING) and self.machine.hold_requested:
            situation = Situation.RECOVERING
        elif state == State.DEGRADED or self.gate.closed_for_sec(now) >= cfg.undecided_after_sec:
            situation = Situation.UNDECIDED
        elif state != State.NORMAL or gate_step.state != GateState.OPEN:
            situation = Situation.SUSPECT
        elif since is not None and since >= cfg.unobserved_after_m:
            situation = Situation.UNOBSERVED
        else:
            situation = Situation.NORMAL
        confirmed = any(event.name in ('pose_confirmed', 'gate_opened') for event in gate_step.events)
        contradicted = any(e.status == Status.ANOMALY for e in result.evidences.values()) if fresh else None
        conduct = self.conduct.update(ConductInputs(now, situation, since, self._clearance, confirmed, contradicted))
        cap = self.sensor_sets.config(result.sensor_set) if self.sensor_sets is not None else None
        if cap is not None and cap.speed_limit_mps > 0.0:
            # センサの組の上限と、走り方の上限の小さい方
            limit = cap.speed_limit_mps if conduct.speed_limit_mps <= 0.0 else min(cap.speed_limit_mps,
                                                                                   conduct.speed_limit_mps)
            conduct = dataclasses.replace(conduct, action=ConductAction.SLOW, speed_limit_mps=limit)
        if not self.config.machine.recovery_enabled:
            conduct = dataclasses.replace(conduct, speed_limit_mps=0.0)
        return conduct

    def _gate_distrust(self) -> frozenset:
        """門に使わせない情報源: 監督が切り離している GNSS と、残差の倍率が大きい GNSS。"""
        gnss = self.gate.settings.gnss
        if not gnss:
            return frozenset()
        if self.machine.isolating and self.machine.blame == gnss:
            self._gnss_held = self.gate.settings.gnss_hold_after_distrust
            return frozenset({gnss})
        if self._gnss_held:
            return frozenset({gnss})
        if any(high and self.config.residuals[name].a == gnss for name, high in self._residual_high.items()):
            return frozenset({gnss})
        return frozenset()

    def _release_held_gnss(self, snapshot: Snapshot, gate_step: GateStep, result: StepResult) -> None:
        """切り離しを解いた GNSS を、門が確かめた姿勢と申告の精度の範囲で合ったら、また使わせる。

        疑う相手が GNSS から推定器に移るだけで切り離しが解けることがあり、GNSS が正しいとは確かめていないため。
        """
        s = self.gate.settings
        if not self._gnss_held or (self.machine.isolating and self.machine.blame == s.gnss):
            return
        confirmed = next((e for e in gate_step.events if e.name == 'pose_confirmed'), None)
        sample = snapshot.pose(s.gnss, s.gnss_max_age_sec)
        if confirmed is None or sample is None:
            return
        sigma = position_std(sample.cov)
        distance = math.hypot(sample.pose[0] - confirmed.fields['x'], sample.pose[1] - confirmed.fields['y'])
        if sigma <= s.gnss_max_std_m and distance <= s.gnss_sigma_k * sigma:
            self._gnss_held = False
            result.events.append(Event(f'{self.config.event_prefix}.gnss_released', 'info', {
                'distance_m': round(distance, 3), 'std_m': round(sigma, 3)}))

    def _apply_gate(self, now: float, result: StepResult, gate_step: GateStep) -> None:
        """門の結果を、周期の結果 (イベント・置き直しの指示・一時停止) に入れ、置き直したら累積と判定を捨てる。"""
        for event in gate_step.events:
            result.events.append(Event(f'{self.config.event_prefix}.{event.name}', event.level, event.fields))
            self._last_event = result.last_event = event.name
        reset = gate_step.reset
        if reset is not None:
            result.directives.append(Directive(
                'reinit', self.machine.blame, Candidate(reset.pose, reset.cov, 'gate'),
                'fused' if reset.targets == 'fused' else 'all'))
        decided = reset is not None or any(event.name == 'gate_opened' for event in gate_step.events)
        if decided and self._awaiting_gate and self.machine.state in (State.RECOVERING, State.DEGRADED):
            self._awaiting_gate = False
            self._after_reinit(now)
        else:
            if decided and self._awaiting_gate:
                # 切り離しの間 (再初期化を求める前) に決まった。再初期化を求めたときに、決まったものとして数える
                self._awaiting_gate = False
                self._gate_decided = True
            if reset is not None:
                self.fusion.reset()
                for detector in self.detectors.values():
                    detector.reset()
                self.start_grace(now, self.config.grace_after_reset_sec)
        result.hold_requested = gate_step.hold
        result.hold_reason = gate_step.hold_reason
        result.gate = gate_step

    def _supervise(self, snapshot: Snapshot) -> StepResult:
        now = snapshot.stamp
        if self._grace_until is None:
            self._grace_until = now + self.config.startup_grace_sec
        self._check_maps(snapshot)
        self._update_scans(snapshot)
        in_grace = now < self._grace_until

        self._track_motion(snapshot)

        evidences = {name: detector.evaluate(snapshot) for name, detector in self.detectors.items()}
        self._check_events(evidences)
        fusion = self.fusion.update(evidences, active=not in_grace)
        state_before = self.machine.state
        converged = self._converged(evidences)
        decision = self.machine.step(now, fusion, converged)
        state = decision.state

        if state_before == State.NORMAL and state == State.SUSPECT:
            self._suspect_since = now
        if state == State.NORMAL and state_before != State.NORMAL:
            self._suspect_since = None
            self._since_reinit = None
            self._awaiting_gate = False
            self._gate_decided = False
            self.fusion.reset()
        if Action.REINIT in decision.actions:
            self.fusion.reset()
            self._reinit_pending = True
            self._next_plan = now
            self._plan_attempts = 0
        if Action.RESTORE in decision.actions and self.config.grace_after_recovery_sec > 0.0:
            self.start_grace(now, self.config.grace_after_recovery_sec)
        if self.gate is not None and Action.ISOLATE in decision.actions and self.machine.hold_requested:
            # 切り離した時点で門を閉じさせる (再初期化を求めるまでの間も、疑わしい推定器を融合に入れない)
            self.gate.force_close(now, 'supervisor')
            self._awaiting_gate = True
        if decision.event:
            self._last_event = decision.event
        directives = self._directives(decision)
        plan = self._recover(snapshot, state, directives)
        self._request_observation(now, state_before, state, converged, directives)

        normal = state == State.NORMAL
        for name, detector in self.detectors.items():
            detector.after_step(evidences[name], normal)

        gate_open = self.gate is None or self.gate.state == GateState.OPEN
        trusted = normal and not in_grace and gate_open and self._trusted(evidences, fusion)
        self._update_anchors(snapshot, trusted)
        safety = self._check_safety(snapshot)
        residuals = self._update_residuals(snapshot, trusted)

        self._state_events(state_before, decision, fusion)
        sensor_set = self._update_sensor_set(snapshot)
        events, self._pending = self._pending, []
        return StepResult(sensor_set=sensor_set, 
            stamp=now, state=state, decision=decision, fusion=fusion, evidences=evidences,
            trusted=trusted, in_grace=in_grace, hold_requested=self.machine.hold_requested, safety=safety,
            isolated_sources=self._isolated(), last_event=self._last_event, events=events,
            directives=directives, plan=plan, blame=self.machine.blame, residuals=residuals,
            applied_scales=self._applied_scales(residuals))

    def _applied_scales(self, residuals: Dict[str, ResidualStats]) -> Dict[str, float]:
        """残差ごとに、残差の a の分散にかける倍率。a の情報源を切り離しているあいだは、上限にして、ほぼ使わないようにする。"""
        blame = self.machine.blame
        scales = {}
        for name, stats in residuals.items():
            cfg = self.residuals[name].config
            scale = cfg.max_scale if self.machine.isolating and blame == cfg.a else stats.scale
            scales[name] = min(max(scale, 1.0), cfg.max_scale)
        return scales

    def _update_residuals(self, snapshot: Snapshot, trusted: bool) -> Dict[str, ResidualStats]:
        """信頼できる周期に、残差の 2 つの情報源 a、b の位置の差 (共分散で正規化) を残し、統計を求める。大きさが変わったらイベントを出す。"""
        result: Dict[str, ResidualStats] = {}
        for name, tracker in self.residuals.items():
            cfg = tracker.config
            if trusted:
                a = snapshot.pose_aligned(cfg.a, cfg.max_age_sec)
                b = snapshot.pose_aligned(cfg.b, cfg.max_age_sec)
                if a is not None and b is not None:
                    tracker.add(snapshot.stamp, *_residual_terms(a[0], b[0], cfg.extra_std_m))
            stats = tracker.update(snapshot.stamp)
            result[name] = stats
            high = self._residual_high[name]
            if not high and stats.scale >= cfg.notify_scale:
                self._residual_high[name] = True
            elif high and stats.scale < cfg.notify_scale * 0.8:
                self._residual_high[name] = False
            else:
                continue
            self._emit('residual_changed', 'warn' if self._residual_high[name] else 'info', {
                'residual': name, 'high': self._residual_high[name], 'scale': round(stats.scale, 3),
                'mean_d2': round(stats.mean_d2, 3), 'count': stats.count})
        return result

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
        if self.scans.too_old(snapshot.stamp, scan):
            self.scans.clear()
            return
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
        if self.gate is not None:
            self._recover_by_gate(snapshot)
            return None
        if self.planner is None:
            self._reinit_pending = False
            self._emit('reinit_skipped', 'error', {'reason': 'no_generators'})
            return None
        self._plan_attempts += 1
        self._next_plan = now + self.config.selection.retry_sec
        if self._needs_new_observation():
            distance, rotation = self._since_reinit
            self._emit('reinit_skipped', 'warn', {
                'reason': 'no_new_observation', 'attempt': self._plan_attempts,
                'distance_m': round(distance, 3), 'rotation_rad': round(rotation, 3)})
            return None
        context = RecoveryContext(
            snapshot=snapshot, anchors=self.anchors, suspect_since=self._suspect_since, scans=self.scans,
            safety=self.config.safety)
        plan = self.planner.plan(context, exclude=self.machine.blame)
        self._plan_events(plan)
        if plan.chosen is not None:
            directives.append(Directive('reinit', self.machine.blame, plan.chosen))
            self._after_reinit(now)
        return plan

    def _recover_by_gate(self, snapshot: Snapshot) -> None:
        """門を閉じさせ、候補を作るものの候補を足して、門の照合の規則で決めさせる。決まったら _apply_gate が受ける。"""
        now = snapshot.stamp
        self._reinit_pending = False
        self._plan_attempts += 1
        if self._gate_decided:
            # 切り離しの間に門が決めた (置き直した、または推定器を採った)
            self._gate_decided = False
            self._after_reinit(now)
            self._emit('reinit_requested', 'warn', {
                'via': 'gate', 'blame': self.machine.blame, 'attempt': self._plan_attempts, 'decided': True})
            return
        extra: Dict[str, CandidatePose] = {}
        notes: Dict[str, str] = {}
        if self.planner is not None:
            context = RecoveryContext(
                snapshot=snapshot, anchors=self.anchors, suspect_since=self._suspect_since, scans=self.scans,
                safety=self.config.safety)
            exclude = {self.machine.blame} | set(self._gate_distrust())
            candidates, notes = self.planner.generate(context, exclude)
            for candidate in candidates:
                name = f'recovery_{candidate.origin}'
                if name in extra:
                    name = f'{name}_{len(extra)}'
                extra[name] = _candidate_pose(candidate)
        self.gate.force_close(now, 'supervisor', extra)
        self._awaiting_gate = True
        self._emit('reinit_requested', 'warn', {
            'via': 'gate', 'blame': self.machine.blame, 'attempt': self._plan_attempts,
            'candidates': ','.join(extra), 'generators': ','.join(f'{n}:{s}' for n, s in notes.items())})

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
        self._since_reinit = [0.0, 0.0]
        self._progress_at = now

    def _track_motion(self, snapshot: Snapshot) -> None:
        """recovery.observe の motion で、最後の再初期化からの走行距離と回転を足していく。"""
        cfg = self.config.observe
        if cfg is None:
            return
        motion = snapshot.motion_at(cfg.motion, snapshot.stamp)
        if motion is None:
            return
        last, self._observe_motion = self._observe_motion, motion
        if last is None or self._since_reinit is None:
            return
        self._since_reinit[0] += math.hypot(motion[0] - last[0], motion[1] - last[1])
        self._since_reinit[1] += abs(math.remainder(motion[2] - last[2], 2.0 * math.pi))

    def _needs_new_observation(self) -> bool:
        """再初期化した後、観測が増える (reseed_min_* だけ動く) まで撒き直さないか。"""
        cfg = self.config.observe
        if cfg is None or self._since_reinit is None:
            return False
        if cfg.reseed_min_distance_m <= 0.0 and cfg.reseed_min_rotation_rad <= 0.0:
            return False
        distance, rotation = self._since_reinit
        return not (cfg.reseed_min_distance_m > 0.0 and distance >= cfg.reseed_min_distance_m
                    or cfg.reseed_min_rotation_rad > 0.0 and rotation >= cfg.reseed_min_rotation_rad)

    def _request_observation(
        self, now: float, before: State, state: State, converged: Optional[bool], directives: List[Directive],
    ) -> None:
        """RECOVERING で、再初期化も収束の確認もできない時間が続いたら、観測を増やす動作を求める (指示 observe)。"""
        cfg = self.config.observe
        if self.gate is not None or cfg is None or cfg.request_after_sec <= 0.0 or state != State.RECOVERING:
            return
        if before != State.RECOVERING:
            self._progress_at = now
        if converged is True and self.machine.reinit_done:
            self._progress_at = now
        waited = now - self._progress_at
        if waited < cfg.request_after_sec:
            return
        if self._observe_at is not None and now - self._observe_at < cfg.request_interval_sec:
            return
        self._observe_at = now
        directives.append(Directive('observe', self.machine.blame))
        self._emit('observe_requested', 'warn', {'waited_sec': round(waited, 1), 'blame': self.machine.blame})

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
        """recovery.convergence のインスタンスが、すべて正常なら True、どれかが異常なら False。
        convergence_mode が any なら、どれか 1 つが正常なら True、正常がなくどれかが異常なら False。"""
        statuses = [evidences[name].status for name in self.config.convergence if name in evidences]
        if self.config.convergence_mode == 'any' and any(s == Status.NORMAL for s in statuses):
            return True
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
        if self.machine.isolating and blame and blame != 'unknown':
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
            self._clearance = None
            return None
        result = self.safety.check(occupancy, aligned[0].pose)
        if self.conduct is not None:
            self._clearance = 0.0 if result.in_lethal else self.safety.clearance(occupancy, aligned[0].pose)
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


def _residual_terms(a: PoseSample, b: PoseSample, extra_std_m: float) -> Tuple[float, float, float]:
    """情報源 a、b の位置の差の二乗と、a の分散、b の分散 + 時刻のずれの分散 (x, y の分散の平均)。"""
    delta2 = float((a.pose[0] - b.pose[0]) ** 2 + (a.pose[1] - b.pose[1]) ** 2)
    var_a = float(np.mean(np.diag(np.asarray(a.cov, dtype=float))[:2]))
    var_b = float(np.mean(np.diag(np.asarray(b.cov, dtype=float))[:2]))
    return delta2, var_a, var_b + extra_std_m ** 2


def _candidate_pose(candidate: Candidate, sigma_k: float = 3.0) -> CandidatePose:
    """復旧の候補を、門の照合の候補にする。探す範囲は共分散の標準偏差の sigma_k 倍 (照合が狭い範囲と上限で挟む)。"""
    cov = np.asarray(candidate.cov, dtype=float)
    std_xy = math.sqrt(max(cov[0][0], cov[1][1], 0.0))
    std_yaw = math.sqrt(max(cov[2][2], 0.0))
    return CandidatePose(candidate.pose, sigma_k * std_xy, sigma_k * std_yaw)


def _pose_fields(pose: Pose2) -> Dict[str, float]:
    return {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 3)}
