"""推定器の姿勢を融合に入れる前の門 (ROS に依存しない)。

閉じる理由の部品 (判定) のどれかが異常を出したら門を閉じ、候補を地図と照合して決めてから開ける。
閉じる理由の部品は「何か変だ」を出すだけで、正しい姿勢は決めない。門を開けるのは照合が決めたときだけ。

周期ごとに step(snapshot, result) を呼ぶ。照合が要るときは GateStep.request を返すので、呼び出し側が
MapMatcher.decide で照合して (重いので別のスレッドで)、次の周期の step に result として渡す。
"""
import enum
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

from .geometry import Pose2, compose, inverse, separated, wrap_angle
from .grid import OccupancyMap
from .interfaces import Detector, Evidence, SourceKind, Status
from .map_match import CandidatePose, MapMatchConfig, MatchDecision
from .scan_buffer import ScanBuffer, ScanBufferConfig
from .snapshot import DistanceFieldCache, Snapshot

CONTINUATION = 'continuation'
MANUAL = 'manual'


class GateState(enum.Enum):
    UNCONFIRMED = 'unconfirmed'   # 確かめた姿勢がない (起動の直後、地図の切り替えの後)。門は開ける
    OPEN = 'open'
    CLOSED = 'closed'             # 照合の決着を待つ
    CORRECTING = 'correcting'     # 大きい補正。止まってから置き直す


@dataclass
class GateSettings:
    # 情報源の名前 (門をかける推定器、融合の結果、動き、地図とスキャン、GNSS (空なら使わない))
    estimator: str = ''
    fused: str = ''
    motion: str = ''
    reference: str = ''
    gnss: str = ''
    # 候補に使う標本の古さの上限
    max_age_sec: float = 2.0
    gnss_max_age_sec: float = 2.0
    # GNSS の候補に使う申告の精度の上限と、探す範囲 (申告の標準偏差の倍)
    gnss_max_std_m: float = 1.0
    gnss_sigma_k: float = 3.0
    # 前の続きの候補の探す範囲を、確かめてから走った道のり 1 m あたり広げる量
    continuation_xy_per_m: float = 0.05
    continuation_yaw_per_m: float = 0.01
    # 決まらなかったときに照合し直す間隔
    retry_sec: float = 2.0
    # 門が開いている間に、確かめた姿勢を更新する道のり
    confirm_distance_m: float = 10.0
    # 置き直し・推定器を採った後に、閉じる理由を使わない道のりと時間 (どちらも満たすまで)
    grace_after_reset_m: float = 1.0
    grace_after_reset_sec: float = 12.0
    # その場で置き直す補正の大きさの上限 (融合の結果との差)。超えれば止まってから置き直す
    small_correction_m: float = 0.3
    small_correction_rad: float = 0.1
    # 止まったとみなす速さと、その続く時間。止まるのを待つ時間の上限
    stopped_speed_mps: float = 0.05
    stopped_yaw_rate_rps: float = 0.05
    stopped_sec: float = 1.0
    correction_hold_max_sec: float = 10.0
    # 門を閉じたまま決まらないときの上限 (時間か道のり)。超えたら undecided_hold_sec の間止まって照合し、
    # 決まらなければ解いて undecided_drive_m 走ってから、また止まる
    max_closed_sec: float = 30.0
    max_closed_m: float = 15.0
    undecided_hold_sec: float = 10.0
    undecided_drive_m: float = 5.0
    # 置き直す姿勢の標準偏差
    reset_std_m: float = 0.1
    reset_std_yaw_rad: float = 0.05


@dataclass
class CloserConfig:
    """閉じる理由の部品。異常が close_after_sec 続いたら門を閉じる。"""
    name: str
    detector: Detector
    close_after_sec: float = 0.0
    type: str = ''
    sources: Dict[str, str] = field(default_factory=dict)


@dataclass
class MotionGateConfig:
    settings: GateSettings
    closers: List[CloserConfig]
    map_match: MapMatchConfig
    scan_buffer: ScanBufferConfig
    kinds: Dict[str, SourceKind] = field(default_factory=dict)


@dataclass
class MatchRequest:
    """照合の依頼。candidates の姿勢と points (base の座標の点) は、時刻 stamp のもの。"""
    generation: int
    reason: str
    stamp: float
    motion: Pose2
    odometer: float
    candidates: Dict[str, CandidatePose]
    points: np.ndarray
    occupancy: OccupancyMap
    fields: DistanceFieldCache

    def field_for(self, x: float, y: float, radius: float):
        return self.fields.around(self.occupancy, x, y, radius)


@dataclass
class ResetCommand:
    """置き直し。targets は 'all' (推定器と融合) か 'fused' (融合だけ)。"""
    pose: Pose2
    cov: np.ndarray
    targets: str


@dataclass
class GateEvent:
    name: str
    level: str
    fields: Dict[str, Any]


@dataclass
class GateStep:
    state: GateState
    gate_open: bool
    hold: bool = False
    hold_reason: str = ''
    reset: Optional[ResetCommand] = None
    request: Optional[MatchRequest] = None
    events: List[GateEvent] = field(default_factory=list)
    evidences: Dict[str, Evidence] = field(default_factory=dict)
    closing: List[str] = field(default_factory=list)
    in_grace: bool = False


@dataclass
class _Anchor:
    """確かめた姿勢と、その時刻の動きと道のり。"""
    pose: Pose2
    motion: Pose2
    odometer: float


def _propagate(pose: Pose2, motion_then: Pose2, motion_now: Pose2) -> Pose2:
    """時刻 then の姿勢を、動きの情報源で now へ進める。"""
    return compose(pose, compose(inverse(motion_then), motion_now))


def _pose_fields(pose: Pose2, prefix: str = '') -> Dict[str, float]:
    """イベントのフィールド (x, y, yaw)。イベントの値は 1 つずつの値にする (doc/monitoring.md)。"""
    return {f'{prefix}x': round(float(pose[0]), 3), f'{prefix}y': round(float(pose[1]), 3),
            f'{prefix}yaw': round(float(pose[2]), 3)}


class MotionGate:
    def __init__(self, config: MotionGateConfig) -> None:
        self.config = config
        self.settings = config.settings
        self._buffer: ScanBuffer = config.scan_buffer.make()
        self._fields = DistanceFieldCache()
        self.state = GateState.UNCONFIRMED
        self._anchor: Optional[_Anchor] = None
        self._generation = 0
        self._busy = False
        self._next_match = -math.inf
        self._last_confirm_odometer = 0.0
        self._odometer = 0.0
        self._last_motion: Optional[Tuple[float, Pose2]] = None
        self._still_since: Optional[float] = None
        self._anomaly_since: Dict[str, float] = {}
        self._grace_until: Tuple[float, float] = (-math.inf, -math.inf)
        self._closed_since: Tuple[float, float] = (0.0, 0.0)
        self._undecided_hold_since: Optional[float] = None
        self._next_undecided_hold = -math.inf
        self._correction: Optional[Tuple[Pose2, Pose2, str]] = None
        self._correction_since = 0.0
        self._manual: Optional[Pose2] = None
        self._map_version: Optional[int] = None
        self._last_scan_stamp: Optional[float] = None

    # ------------------------------------------------------------------ 外から

    def manual_pose(self, pose: Pose2) -> None:
        """手動の初期姿勢を受けた。次の周期で、その周りを照合してから置き直す (照合中の結果は捨てる)。"""
        self._manual = pose
        self._generation += 1

    # ------------------------------------------------------------------ 周期

    def step(self, snapshot: Snapshot, result: Optional[Tuple[MatchRequest, MatchDecision]] = None) -> GateStep:
        s = self.settings
        t = snapshot.stamp
        out = GateStep(self.state, self.state != GateState.CLOSED and self.state != GateState.CORRECTING)
        if result is not None:
            self._busy = False
        motion = snapshot.motion_at(s.motion, t)
        if motion is None:
            return out
        self._advance(t, motion)
        self._watch_map(snapshot, out)
        self._buffer_scan(snapshot)
        out.evidences = {c.name: c.detector.evaluate(snapshot) for c in self.config.closers}
        closing = self._closing(t, out.evidences)
        out.closing = closing
        if result is not None and result[0].generation == self._generation:
            self._apply(result[0], result[1], snapshot, motion, out)

        in_grace = t < self._grace_until[0] or self._odometer < self._grace_until[1]
        out.in_grace = in_grace
        if self.state == GateState.OPEN and closing and not in_grace:
            self._close(t, 'closer', out, closers=','.join(closing))
        if self.state == GateState.CORRECTING:
            self._continue_correction(t, motion, snapshot, out)
        if self.state == GateState.CLOSED:
            self._watch_limits(t, out)
        if not self._busy:
            out.request = self._next_request(snapshot, motion)
            if out.request is not None:
                self._busy = True
        out.state = self.state
        out.gate_open = self.state in (GateState.UNCONFIRMED, GateState.OPEN)
        if self.state == GateState.CORRECTING:
            out.hold, out.hold_reason = True, 'correction'
        elif self._undecided_hold_since is not None:
            out.hold, out.hold_reason = True, 'undecided'
        return out

    def _advance(self, t: float, motion: Pose2) -> None:
        s = self.settings
        if self._last_motion is not None:
            t0, m0 = self._last_motion
            step = math.hypot(motion[0] - m0[0], motion[1] - m0[1])
            self._odometer += step
            dt = t - t0
            still = dt > 0 and step / dt <= s.stopped_speed_mps and abs(
                wrap_angle(motion[2] - m0[2])) / dt <= s.stopped_yaw_rate_rps
            if not still:
                self._still_since = None
            elif self._still_since is None:
                self._still_since = t0
        self._last_motion = (t, motion)

    def _watch_map(self, snapshot: Snapshot, out: GateStep) -> None:
        version = snapshot.reference(self.settings.reference).map_version
        if self._map_version is not None and version != self._map_version:
            self.state = GateState.UNCONFIRMED
            self._anchor = None
            self._correction = None
            self._undecided_hold_since = None
            self._buffer.clear()
            self._fields = DistanceFieldCache()
            self._generation += 1
            out.events.append(GateEvent('gate_map_changed', 'info', {'map_version': version}))
        self._map_version = version

    def _buffer_scan(self, snapshot: Snapshot) -> None:
        s = self.settings
        scan = snapshot.reference(s.reference).scan
        if scan is None or scan.stamp == self._last_scan_stamp:
            return
        self._last_scan_stamp = scan.stamp
        motion = snapshot.motion_at(s.motion, scan.stamp)
        if motion is not None:
            self._buffer.add(scan, motion)

    def _closing(self, t: float, evidences: Mapping[str, Evidence]) -> List[str]:
        """異常が close_after_sec 続いた閉じる理由の部品。"""
        closing = []
        for closer in self.config.closers:
            if evidences[closer.name].status != Status.ANOMALY:
                self._anomaly_since.pop(closer.name, None)
                continue
            since = self._anomaly_since.setdefault(closer.name, t)
            if t - since >= closer.close_after_sec:
                closing.append(closer.name)
        return closing

    # ------------------------------------------------------------------ 状態

    def _close(self, t: float, reason: str, out: GateStep, **fields: Any) -> None:
        self.state = GateState.CLOSED
        self._closed_since = (t, self._odometer)
        self._next_match = min(self._next_match, t)
        self._next_undecided_hold = -math.inf
        out.events.append(GateEvent('gate_closed', 'warn', {'reason': reason, **fields}))

    def _open(self, t: float, how: str, out: GateStep, pose: Pose2, motion: Pose2) -> None:
        self.state = GateState.OPEN
        self._anchor = _Anchor(pose, motion, self._odometer)
        self._last_confirm_odometer = self._odometer
        self._grace_until = (t + self.settings.grace_after_reset_sec,
                             self._odometer + self.settings.grace_after_reset_m)
        self._anomaly_since.clear()
        if self._undecided_hold_since is not None:
            out.events.append(GateEvent('hold', 'info', {'active': False, 'reason': 'undecided'}))
        self._undecided_hold_since = None
        out.events.append(GateEvent('gate_opened', 'info', {'how': how, **_pose_fields(pose)}))

    def _reset(self, t: float, pose: Pose2, targets: str, how: str, out: GateStep, motion: Pose2,
               fused: Optional[Pose2]) -> None:
        s = self.settings
        cov = np.diag([s.reset_std_m ** 2, s.reset_std_m ** 2, s.reset_std_yaw_rad ** 2])
        out.reset = ResetCommand(pose, cov, targets)
        fields: Dict[str, Any] = {'how': how, 'targets': targets, **_pose_fields(pose)}
        if fused is not None:
            fields['shift_m'] = round(math.hypot(pose[0] - fused[0], pose[1] - fused[1]), 3)
            fields['turn_rad'] = round(abs(wrap_angle(pose[2] - fused[2])), 3)
        out.events.append(GateEvent('pose_reset', 'info', fields))
        self._open(t, 'reset' if how != MANUAL else MANUAL, out, pose, motion)

    def _correct(self, t: float, pose: Pose2, targets: str, out: GateStep, motion: Pose2,
                 fused: Optional[Pose2]) -> None:
        """決めた姿勢 (今の時刻) へ戻す。融合の結果との差が小さければその場で、大きければ止まってから。"""
        s = self.settings
        if fused is None or not separated(pose, fused, s.small_correction_m, s.small_correction_rad):
            if targets == 'none':
                self._open(t, 'adopted', out, pose, motion)
            else:
                self._reset(t, pose, targets, 'immediate', out, motion, fused)
            return
        self.state = GateState.CORRECTING
        self._correction = (pose, motion, 'fused' if targets == 'none' else targets)
        self._correction_since = t
        out.events.append(GateEvent('hold', 'info', {'active': True, 'reason': 'correction'}))

    def _continue_correction(self, t: float, motion: Pose2, snapshot: Snapshot, out: GateStep) -> None:
        s = self.settings
        stopped = self._still_since is not None and t - self._still_since >= s.stopped_sec
        if not stopped and t - self._correction_since < s.correction_hold_max_sec:
            return
        pose, motion_then, targets = self._correction
        self._correction = None
        out.events.append(GateEvent('hold', 'info', {'active': False, 'reason': 'correction'}))
        self._reset(t, _propagate(pose, motion_then, motion), targets, 'stopped', out, motion,
                    self._pose_now(snapshot, self.settings.fused))

    def _watch_limits(self, t: float, out: GateStep) -> None:
        """閉じたまま決まらないときの上限。超えたら止めて照合し、決まらなければ解いて少し走らせる。"""
        s = self.settings
        if self._undecided_hold_since is not None:
            if t - self._undecided_hold_since >= s.undecided_hold_sec:
                self._undecided_hold_since = None
                self._next_undecided_hold = self._odometer + s.undecided_drive_m
                out.events.append(GateEvent('hold', 'warn', {'active': False, 'reason': 'undecided'}))
            return
        since_t, since_odometer = self._closed_since
        over = t - since_t >= s.max_closed_sec or self._odometer - since_odometer >= s.max_closed_m
        if over and self._odometer >= self._next_undecided_hold:
            self._undecided_hold_since = t
            self._next_match = min(self._next_match, t)
            out.events.append(GateEvent('hold', 'warn', {
                'active': True, 'reason': 'undecided', 'closed_sec': round(t - since_t, 1),
                'closed_m': round(self._odometer - since_odometer, 1)}))

    # ------------------------------------------------------------------ 照合

    def _pose_now(self, snapshot: Snapshot, name: str, max_age: Optional[float] = None) -> Optional[Pose2]:
        if not name:
            return None
        aligned = snapshot.pose_aligned(name, self.settings.max_age_sec if max_age is None else max_age)
        return None if aligned is None else aligned[0].pose

    def _next_request(self, snapshot: Snapshot, motion: Pose2) -> Optional[MatchRequest]:
        s = self.settings
        t = snapshot.stamp
        if self._manual is not None:
            reason = MANUAL
        elif self.state == GateState.CORRECTING:
            return None
        elif self.state in (GateState.UNCONFIRMED, GateState.CLOSED):
            if t < self._next_match:
                return None
            reason = 'startup' if self.state == GateState.UNCONFIRMED else 'closed'
        elif self._odometer - self._last_confirm_odometer >= s.confirm_distance_m:
            reason = 'confirm'
        else:
            return None
        occupancy = snapshot.reference(s.reference).occupancy
        if occupancy is None or not len(self._buffer):
            return None
        candidates = self._candidates(snapshot, motion, reason)
        if not candidates:
            return None
        if reason == MANUAL:
            self._manual = None
        if reason == 'confirm':
            self._last_confirm_odometer = self._odometer
        points = self._buffer.points_in((0.0, 0.0, 0.0), motion)
        return MatchRequest(self._generation, reason, t, motion, self._odometer, candidates, points, occupancy,
                            self._fields)

    def _candidates(self, snapshot: Snapshot, motion: Pose2, reason: str) -> Dict[str, CandidatePose]:
        s = self.settings
        c = self.config.map_match
        narrow = (c.narrow_xy_m, c.narrow_yaw_rad)
        if reason == MANUAL:
            return {MANUAL: CandidatePose(self._manual, *narrow)}
        candidates: Dict[str, CandidatePose] = {}
        for name in (s.estimator, s.fused):
            pose = self._pose_now(snapshot, name)
            if pose is not None:
                candidates[name] = CandidatePose(pose, *narrow)
        if reason == 'startup':
            # 推定器が出す前に決めると、収束する前の推定器を、融合の結果を照合した姿勢へ置き直してしまう
            return candidates if s.estimator in candidates else {}
        if reason == 'confirm':
            candidates.pop(s.fused, None)
        continuation_yaw = None
        if self._anchor is not None:
            a = self._anchor
            travelled = self._odometer - a.odometer
            continuation_yaw = c.narrow_yaw_rad + s.continuation_yaw_per_m * travelled
            candidates[CONTINUATION] = CandidatePose(
                _propagate(a.pose, a.motion, motion), c.narrow_xy_m + s.continuation_xy_per_m * travelled,
                continuation_yaw)
        if reason == 'confirm' or not s.gnss:
            return candidates
        gnss = snapshot.pose_aligned(s.gnss, s.gnss_max_age_sec)
        if gnss is not None:
            sample = gnss[0]
            sigma = math.sqrt(max(float(sample.cov[0][0]), float(sample.cov[1][1]), 0.0))
            yaw_from = candidates.get(CONTINUATION) or candidates.get(s.fused) or candidates.get(s.estimator)
            if sigma <= s.gnss_max_std_m and yaw_from is not None:
                candidates[s.gnss] = CandidatePose(
                    (sample.pose[0], sample.pose[1], yaw_from.pose[2]), s.gnss_sigma_k * sigma,
                    continuation_yaw if continuation_yaw is not None else c.cap_yaw_rad)
        return candidates

    def _apply(self, request: MatchRequest, decision: MatchDecision, snapshot: Snapshot, motion: Pose2,
               out: GateStep) -> None:
        s = self.settings
        t = snapshot.stamp
        level = 'info' if decision.best is not None else 'warn'
        out.events.append(GateEvent('map_match', level, {
            'reason': request.reason, 'result': decision.best, 'why': decision.reason,
            'margin': None if decision.margin is None else round(decision.margin, 3),
            'agree': ','.join(decision.agree), **{f'ratio_{k}': round(v, 3) for k, v in decision.ratios.items()}}))
        fused = self._pose_now(snapshot, s.fused)

        def now(pose: Pose2) -> Pose2:
            return _propagate(pose, request.motion, motion)

        if request.reason == MANUAL:
            given = request.candidates[MANUAL].pose
            pose = decision.pose if decision.best == MANUAL else given
            out.events.append(GateEvent('manual_pose', 'info', {
                **_pose_fields(given, 'given_'), 'corrected': decision.best == MANUAL and separated(
                    pose, given, self.config.map_match.step_xy_m, self.config.map_match.step_yaw_rad)}))
            self._correction = None
            self._reset(t, now(pose), 'all', MANUAL, out, motion, fused)
            return
        if decision.best is None:
            if request.reason != 'confirm':
                self._next_match = t + s.retry_sec
            return
        estimator = s.estimator
        adopted = estimator in decision.agree
        target = now(request.candidates[estimator].pose if adopted else decision.pose)
        if request.reason == 'confirm':
            if self.state != GateState.OPEN:
                return
            if adopted:
                self._anchor = _Anchor(request.candidates[estimator].pose, request.motion, request.odometer)
                out.events.append(GateEvent('pose_confirmed', 'info', _pose_fields(target)))
            elif estimator not in decision.same:
                self._close(t, 'confirm', out, result=decision.best)
                self._correct(t, target, 'all', out, motion, fused)
            return
        if self.state not in (GateState.UNCONFIRMED, GateState.CLOSED):
            return
        if request.reason == 'startup' and estimator in decision.same:
            # 推定器と同じ仮説なら、収束の途中を邪魔しないよう置き直さない
            self._open(t, 'startup', out, target, motion)
            return
        self._correct(t, target, 'none' if adopted else 'all', out, motion, fused)
