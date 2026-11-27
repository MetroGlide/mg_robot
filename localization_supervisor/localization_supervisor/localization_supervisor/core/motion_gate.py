"""推定器の姿勢を融合に入れる前の AMCL 検証ゲート (ROS に依存しない)。

遮断条件 (判定) のどれかが異常を出したら AMCL 検証ゲートを閉じ、候補を地図と照合して決めてから開ける。
遮断条件は「何か変だ」を出すだけで、正しい姿勢は決めない。AMCL 検証ゲートを開けるのは照合が決めたときだけ。

周期ごとに step(snapshot, result) を呼ぶ。照合が要るときは GateStep.request を返すので、呼び出し側が
MapMatcher.decide で照合して (重いので別のスレッドで)、次の周期の step に result として渡す。

自己位置監視の部品として使うときは、自己位置監視が疑った情報源を step の distrust に渡し (GNSS を使う遮断条件と候補を使わない)、
自己位置監視が推定器の異常を確かめたら force_close で閉じさせる (候補を足せる。決まるまで止めて照合する)。
"""
import enum
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Mapping, Optional, Tuple

import numpy as np

from .detectors.common import position_std
from .geometry import Pose2, compose, inverse, separated, wrap_angle
from .grid import OccupancyMap, scan_points_in_map
from .initial_search import InitialSearchConfig, InitialSearchResult, search_initial_pose
from .interfaces import Detector, Evidence, SourceKind, Status
from .map_match import CandidatePose, MapMatchConfig, MapMatcher, MatchDecision
from .scan_buffer import ScanBuffer, ScanBufferConfig
from .snapshot import DistanceFieldCache, Snapshot

CONTINUATION = 'continuation'
MANUAL = 'manual'
# 走行側からの検証要求の照合
VERIFY = 'verify'
# 起動の直後と地図の切り替えの後に、GNSS の周りを向きの全周で照合して見つけた姿勢の候補
INITIAL = 'initial_search'
# 初期全周探索に使う、1 回分のスキャンの点の数の上限
INITIAL_MAX_POINTS = 360


class GateState(enum.Enum):
    UNCONFIRMED = 'unconfirmed'   # 未初期化 (起動の直後、地図の切り替えの後)。AMCL 検証ゲートは開ける
    OPEN = 'open'
    CLOSED = 'closed'             # 照合の決着を待つ
    CORRECTING = 'correcting'     # 大きい補正。止まってから補正する


@dataclass
class GateSettings:
    # 情報源の名前 (AMCL 検証ゲートをかける推定器、融合の結果、動き、地図とスキャン、GNSS (空なら使わない))
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
    # 推測航法姿勢の候補の探す範囲を、確かめてから走った道のり 1 m あたり広げる量
    continuation_xy_per_m: float = 0.05
    continuation_yaw_per_m: float = 0.01
    # 手動の初期姿勢の候補の探す範囲 (人が与える姿勢のおおよその誤差より広く)
    manual_xy_m: float = 1.5
    manual_yaw_rad: float = 0.5
    # 決まらなかったときに照合し直す間隔
    retry_sec: float = 2.0
    # AMCL 検証ゲートが開いている間に、検証済み姿勢を更新する道のり
    confirm_distance_m: float = 10.0
    # 姿勢補正・推定器を採った後に、遮断条件を使わない道のりと時間 (どちらも満たすまで)
    grace_after_reset_m: float = 1.0
    grace_after_reset_sec: float = 12.0
    # その場で補正する補正の大きさの上限 (融合の結果との差)。超えれば止まってから補正する
    small_correction_m: float = 0.3
    small_correction_rad: float = 0.1
    # 止まったとみなす速さと、その続く時間。止まるのを待つ時間の上限
    stopped_speed_mps: float = 0.05
    stopped_yaw_rate_rps: float = 0.05
    stopped_sec: float = 1.0
    correction_hold_max_sec: float = 10.0
    # AMCL 検証ゲートを閉じたまま決まらないときの上限 (時間か道のり)。超えたら undecided_hold_sec の間止まって照合し、
    # 決まらなければ解いて undecided_drive_m 走ってから、また止まる
    max_closed_sec: float = 30.0
    max_closed_m: float = 15.0
    undecided_hold_sec: float = 10.0
    undecided_drive_m: float = 5.0
    # 補正する姿勢の標準偏差
    reset_std_m: float = 0.1
    reset_std_yaw_rad: float = 0.05
    # 初期全周探索 (設定の initial_search があるとき) に使う GNSS の申告の精度の上限と、続けて満たす標本の数
    initial_gnss_max_std_m: float = 5.0
    initial_gnss_consecutive: int = 2
    # GNSS の再採用待機: 申告の精度が、直近 gnss_probation_window_sec の最も悪い値から gnss_probation_ratio 倍以上
    # 良くなり、その最も悪い値が gnss_max_std_m より悪かったら、gnss_probation_sec の間 GNSS を使わない
    # (屋根を抜けた直後などに、位置はずれたまま申告だけが良くなるため)。0 なら使わない
    gnss_probation_sec: float = 0.0
    gnss_probation_ratio: float = 3.0
    gnss_probation_window_sec: float = 10.0
    # 自己位置監視が除外した GNSS を、除外を解いた後も、AMCL 検証ゲートが検証済み姿勢と合うまで AMCL 検証ゲートと復旧に使わせない (自己位置監視が読む。
    # 疑う相手が推定器に移るだけで除外が解けることがあり、GNSS が正しいとは確かめていないため)。検証済み姿勢と
    # GNSS の位置の差が申告の標準偏差の gnss_sigma_k 倍以内で、申告の精度が gnss_max_std_m 以下なら、また使わせる
    gnss_hold_after_distrust: bool = False
    # 検証要求 (走行側から) を受け付ける間隔の下限 [s]
    verify_min_interval_sec: float = 5.0
    # 拡大探索: 決まらずに止めた回数がこれに達した後の停止の間は、推測航法姿勢 (なければ融合の結果) の周りを、
    # 初期全周探索 (設定の initial_search) で探して候補に足す。0 なら使わない
    wide_search_after_holds: int = 0


@dataclass
class CloserConfig:
    """遮断条件。異常が close_after_sec 続いたら AMCL 検証ゲートを閉じる。

    active_in_grace が偽の部品は、姿勢補正の後の猶予の間は AMCL 検証ゲートを閉じない (姿勢補正の前の標本と比べる部品など)。
    """
    name: str
    detector: Detector
    close_after_sec: float = 0.0
    active_in_grace: bool = False
    type: str = ''
    sources: Dict[str, str] = field(default_factory=dict)


@dataclass
class MotionGateConfig:
    settings: GateSettings
    closers: List[CloserConfig]
    map_match: MapMatchConfig
    scan_buffer: ScanBufferConfig
    kinds: Dict[str, SourceKind] = field(default_factory=dict)
    # 未初期化間 (起動の直後、地図の切り替えの後) に、GNSS の周りを向きの全周で照合する (None ならしない)
    initial_search: Optional[InitialSearchConfig] = None
    # 判定と照合だけをし、何もしない (AMCL 検証ゲートを開けたまま、補正せず、止める要求を出さない)。イベントは出す
    observe_only: bool = False


@dataclass
class InitialSearch:
    """初期全周探索の依頼。center (GNSS の位置) と points (1 回分のスキャン、base の座標) は、動きが motion の時刻のもの。"""
    center: Pose2
    points: np.ndarray
    motion: Pose2
    config: InitialSearchConfig


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
    initial: Optional[InitialSearch] = None
    # 初期全周探索の結果 (run_match が入れる)
    initial_result: Optional[InitialSearchResult] = None

    def field_for(self, x: float, y: float, radius: float):
        return self.fields.around(self.occupancy, x, y, radius)


def run_match(matcher: MapMatcher, request: MatchRequest) -> MatchDecision:
    """照合の依頼を処理する (重いので、周期の処理と別のスレッドで呼ぶ)。

    初期全周探索の依頼があれば先に行い、はっきり合った姿勢を候補 INITIAL として足してから、ほかの候補と同じ規則で決める。
    """
    candidates = dict(request.candidates)
    initial = request.initial
    if initial is not None:
        result = search_initial_pose(request.occupancy, initial.points, initial.center, initial.config)
        request.initial_result = result
        if result.reason == 'matched':
            candidates[INITIAL] = CandidatePose(
                _propagate(result.pose, initial.motion, request.motion), result.std_m, result.std_yaw_rad)
    return matcher.decide(request.field_for, request.points, candidates)


@dataclass
class ResetCommand:
    """姿勢補正。targets は 'all' (推定器と融合) か 'fused' (融合だけ)。"""
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
    # この周期に決着した検証要求 (番号, 結果)。結果は VERIFY_RESULTS のどれか
    verified: Optional[Tuple[int, str]] = None
    # 直近に決着した検証要求 (状態に出し続ける)
    last_verify: Optional[Tuple[int, str]] = None


# 検証要求の結果: 推定器のまま確かめた / 別の姿勢へ補正する / 決まらない (AMCL 検証ゲートを閉じ、止めて照合する)
VERIFY_CONFIRMED = 'confirmed'
VERIFY_RELOCATED = 'relocated'
VERIFY_UNDECIDED = 'undecided'
VERIFY_RESULTS = (VERIFY_CONFIRMED, VERIFY_RELOCATED, VERIFY_UNDECIDED)


@dataclass
class _Anchor:
    """検証済み姿勢と、その時刻の動きと道のり。"""
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
        # 検証要求: 番号の続き、待っている番号、受け付けた時刻、直近の決着 (番号, 結果)
        self._verify_seq = 0
        self._verify_pending: Optional[int] = None
        self._verify_at = -math.inf
        self._verify_undecided: Optional[int] = None
        # 確かめるまでに、決まらずに止めた回数
        self._undecided_holds = 0
        self.last_verify: Optional[Tuple[int, str]] = None
        self._map_version: Optional[int] = None
        self._last_scan_stamp: Optional[float] = None
        self._distrust: frozenset = frozenset()
        self._supervisor_distrust: frozenset = frozenset()
        # GNSS の申告の精度の履歴 (時刻, 標準偏差) と、再採用待機の終わり
        self._gnss_std: Deque[Tuple[float, float]] = deque()
        self._probation_until = -math.inf
        # 外から足された候補 (force_close。次に閉じたまま照合するときに一度だけ使う) と、出していないイベント
        self._extra: Dict[str, CandidatePose] = {}
        self._pending: List[GateEvent] = []
        # 外から閉じさせた (融合の結果も汚れているおそれがある) ので、決まるまで止めて照合する
        self._escalated = False
        # 初期全周探索に使える GNSS の標本が続いた数と、最後に数えた標本の時刻。GNSS の位置を広い共分散で出したか
        # (最初に確かめるまでに 1 回だけ。推定器に広く探させる)
        self._gnss_good = 0
        self._gnss_stamp: Optional[float] = None
        self._fallback_done = False
        # スキャンが古い (照合しない) 間か
        self._scan_stale = False

    # ------------------------------------------------------------------ 外から

    def manual_pose(self, pose: Pose2) -> None:
        """手動の初期姿勢を受けた。次の周期で、その周りを照合してから補正する (照合中の結果は捨てる)。"""
        self._manual = pose
        self._generation += 1

    def request_verify(self, t: float) -> Optional[int]:
        """走行側からの検証要求。受け付ければ番号を返す (間隔の下限より早い・待っている要求があれば None)。

        AMCL 検証ゲートが開いていれば、次の周期に、閉じたときと同じ候補で照合する。開いていなければ (未初期化、閉じて照合・
        停止の最中)、次の周期に「決まらない」として返す。
        """
        if self._verify_pending is not None or t - self._verify_at < self.settings.verify_min_interval_sec:
            return None
        self._verify_seq += 1
        self._verify_at = t
        if self.state == GateState.OPEN:
            self._verify_pending = self._verify_seq
        else:
            self._verify_undecided = self._verify_seq
        return self._verify_seq

    def force_close(self, t: float, reason: str, extra: Optional[Mapping[str, CandidatePose]] = None) -> None:
        """外 (自己位置監視) が推定器の異常を確かめた。AMCL 検証ゲートを閉じ、extra を候補に足して照合し直す。

        閉じた理由の部品より遅く気づくので、融合の結果も推定器に引かれているおそれがある。そのため、決まらない間は
        閉じたまま決まらないときの上限を超えたものとして扱い、止めて照合する (止める時間と、解いて走る道のりは同じ上限)。
        確かめる前 (起動の直後など) と、止めてから補正する途中は、そのまま続ける。
        """
        self._extra = dict(extra or {})
        if self.state in (GateState.UNCONFIRMED, GateState.CORRECTING):
            return
        self._escalated = True
        if self.state == GateState.OPEN:
            self.state = GateState.CLOSED
            self._closed_since = (t, self._odometer)
            self._next_undecided_hold = -math.inf
            self._pending.append(GateEvent('gate_closed', 'warn', {'reason': reason}))
        self._next_match = min(self._next_match, t)
        self._generation += 1

    # ------------------------------------------------------------------ 周期

    def since_confirmed_m(self) -> Optional[float]:
        """最後に確かめてから走った道のり [m]。検証済み姿勢がなければ None。"""
        return None if self._anchor is None else self._odometer - self._anchor.odometer

    def closed_for_sec(self, t: float) -> float:
        """閉じてからの時間 [s] (開いていれば 0)。"""
        if self.state not in (GateState.CLOSED, GateState.CORRECTING):
            return 0.0
        return t - self._closed_since[0]

    def step(self, snapshot: Snapshot, result: Optional[Tuple[MatchRequest, MatchDecision]] = None,
             distrust: frozenset = frozenset()) -> GateStep:
        """distrust の情報源を使う遮断条件は閉じず、その情報源の候補は使わない。"""
        out = self._step(snapshot, result, distrust)
        if self._verify_undecided is not None:
            self._finish_verify(self._verify_undecided, VERIFY_UNDECIDED, out)
            self._verify_undecided = None
        out.last_verify = self.last_verify
        if self.config.observe_only:
            # AMCL 検証ゲートの中では補正したものとして続ける (決めた姿勢を検証済み姿勢にする)。外へは何も出さない
            out.reset = None
            out.gate_open = True
            out.hold, out.hold_reason = False, ''
            for event in out.events:
                if event.name == 'pose_reset':
                    event.fields['applied'] = False
        return out

    def _step(self, snapshot: Snapshot, result: Optional[Tuple[MatchRequest, MatchDecision]],
              distrust: frozenset) -> GateStep:
        s = self.settings
        t = snapshot.stamp
        out = GateStep(self.state, self.state != GateState.CLOSED and self.state != GateState.CORRECTING)
        out.events, self._pending = self._pending, []
        # 自己位置監視が疑う GNSS (初期全周探索にも使わない) と、再採用待機 (遮断条件と候補にだけ使わない。初期全周探索は
        # GNSS を探す範囲の中心にだけ使い、地図照合で確かめるので止めない)
        self._supervisor_distrust = frozenset(distrust)
        self._distrust = self._supervisor_distrust | self._probation(snapshot, t, out)
        if result is not None:
            self._busy = False
        motion = snapshot.motion_at(s.motion, t)
        if motion is None:
            return out
        self._advance(t, motion)
        self._watch_map(snapshot, out)
        self._buffer_scan(snapshot)
        stale = self._watch_scan_age(snapshot, out)
        self._count_initial_gnss(t, snapshot)
        out.evidences = {c.name: c.detector.evaluate(snapshot) for c in self.config.closers}
        closing = self._closing(t, out.evidences)
        out.closing = closing
        if stale:
            return self._finish(out)
        if result is not None and result[0].generation == self._generation:
            self._apply(result[0], result[1], snapshot, motion, out)

        in_grace = t < self._grace_until[0] or self._odometer < self._grace_until[1]
        out.in_grace = in_grace
        if in_grace:
            active = {c.name for c in self.config.closers if c.active_in_grace}
            closing = [name for name in closing if name in active]
        if self.state == GateState.OPEN and closing:
            self._close(t, 'closer', out, closers=','.join(closing))
        if self.state == GateState.CORRECTING:
            self._continue_correction(t, motion, snapshot, out)
        if self.state == GateState.CLOSED:
            self._watch_limits(t, out)
        if not self._busy:
            out.request = self._next_request(snapshot, motion)
            if out.request is not None:
                self._busy = True
        return self._finish(out)

    def _finish(self, out: GateStep) -> GateStep:
        """周期の終わりの状態と、停止要求を出す。"""
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

    def _watch_scan_age(self, snapshot: Snapshot, out: GateStep) -> bool:
        """最新のスキャンが古ければ真 (scan_buffer の max_scan_age_sec)。古くなったら、溜めたスキャン・待っている照合・
        停止を捨てる。古い間は照合も停止もしない。"""
        t = snapshot.stamp
        scan = snapshot.reference(self.settings.reference).scan
        stale = self._buffer.too_old(t, scan)
        if stale and not self._scan_stale:
            self._buffer.clear()
            self._generation += 1
            self._undecided_hold_since = None
            if self.state == GateState.CORRECTING:
                self._correction = None
                self.state = GateState.CLOSED
            out.events.append(GateEvent('gate_scan_stale', 'warn', {'age_sec': round(t - scan.stamp, 1)}))
        elif not stale and self._scan_stale:
            self._closed_since = (t, self._odometer)
            out.events.append(GateEvent('gate_scan_fresh', 'info', {}))
        self._scan_stale = stale
        return stale

    def _probation(self, snapshot: Snapshot, t: float, out: GateStep) -> frozenset:
        """GNSS の再採用待機なら {GNSS}。申告の精度が急に良くなったら始める。"""
        s = self.settings
        if not s.gnss or s.gnss_probation_sec <= 0.0:
            return frozenset()
        sample = snapshot.pose(s.gnss, s.gnss_max_age_sec)
        if sample is not None and (not self._gnss_std or sample.stamp > self._gnss_std[-1][0]):
            std = position_std(sample.cov)
            while self._gnss_std and self._gnss_std[0][0] < sample.stamp - s.gnss_probation_window_sec:
                self._gnss_std.popleft()
            worst = max((v for _, v in self._gnss_std), default=0.0)
            self._gnss_std.append((sample.stamp, std))
            if worst > s.gnss_max_std_m and std * s.gnss_probation_ratio <= worst and t >= self._probation_until:
                self._probation_until = t + s.gnss_probation_sec
                out.events.append(GateEvent('gnss_probation', 'info', {
                    'std_m': round(std, 3), 'worst_std_m': round(worst, 3), 'sec': s.gnss_probation_sec}))
        return frozenset({s.gnss}) if t < self._probation_until else frozenset()

    def _count_initial_gnss(self, t: float, snapshot: Snapshot) -> None:
        """初期全周探索に使える (申告の精度が上限以下の) GNSS の標本が、続けていくつ届いたか。

        未初期化間に数が足りたら、やり直しの間隔を待たずに照合する (起動で走りながら待つ距離を減らす)。
        """
        s = self.settings
        if self.config.initial_search is None or not s.gnss:
            return
        sample = snapshot.pose(s.gnss, s.gnss_max_age_sec)
        if sample is None or sample.stamp == self._gnss_stamp:
            return
        self._gnss_stamp = sample.stamp
        sigma = position_std(sample.cov)
        self._gnss_good = self._gnss_good + 1 if sigma <= s.initial_gnss_max_std_m else 0
        if self._gnss_good == s.initial_gnss_consecutive and self.state == GateState.UNCONFIRMED:
            self._next_match = min(self._next_match, t)

    def _initial_search(self, snapshot: Snapshot) -> Optional[InitialSearch]:
        """初期全周探索の依頼 (GNSS の精度が続けて足り、GNSS を疑っておらず、スキャンがあるとき)。"""
        s = self.settings
        cfg = self.config.initial_search
        if cfg is None or s.gnss in self._supervisor_distrust or self._gnss_good < s.initial_gnss_consecutive:
            return None
        gnss = snapshot.pose(s.gnss, s.gnss_max_age_sec)
        scan = snapshot.reference(s.reference).scan
        if gnss is None or scan is None:
            return None
        motion = snapshot.motion_at(s.motion, scan.stamp)
        if motion is None:
            return None
        points = scan_points_in_map(
            np.asarray(scan.ranges, dtype=float), scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, (0.0, 0.0, 0.0), scan.sensor_pose or (0.0, 0.0, 0.0), INITIAL_MAX_POINTS)
        # GNSS の時刻からスキャンの時刻へ、動きで進める (向きは全周を探すので、GNSS の向きのまま使う)
        gnss_motion = snapshot.motion_at(s.motion, gnss.stamp)
        center = gnss.pose if gnss_motion is None else _propagate(gnss.pose, gnss_motion, motion)
        return InitialSearch(center, points, motion, cfg)

    def _wide_search_due(self) -> bool:
        """拡大探索をする停止の間か。"""
        n = self.settings.wide_search_after_holds
        return (self.config.initial_search is not None and n > 0 and self._undecided_hold_since is not None
                and self._undecided_holds >= n)

    def _wide_search(self, snapshot: Snapshot, motion: Pose2) -> Optional[InitialSearch]:
        """拡大探索の依頼。中心は推測航法姿勢 (なければ融合の結果)。向きは全周を探す。"""
        s = self.settings
        scan = snapshot.reference(s.reference).scan
        if scan is None:
            return None
        scan_motion = snapshot.motion_at(s.motion, scan.stamp)
        if scan_motion is None:
            return None
        if self._anchor is not None:
            center = _propagate(self._anchor.pose, self._anchor.motion, scan_motion)
        else:
            fused = self._pose_now(snapshot, s.fused)
            if fused is None:
                return None
            center = _propagate(fused, motion, scan_motion)
        points = scan_points_in_map(
            np.asarray(scan.ranges, dtype=float), scan.angle_min, scan.angle_increment, scan.range_min,
            scan.range_max, (0.0, 0.0, 0.0), scan.sensor_pose or (0.0, 0.0, 0.0), INITIAL_MAX_POINTS)
        return InitialSearch(center, points, scan_motion, self.config.initial_search)

    def _closing(self, t: float, evidences: Mapping[str, Evidence]) -> List[str]:
        """異常が close_after_sec 続いた遮断条件。"""
        closing = []
        for closer in self.config.closers:
            if evidences[closer.name].status != Status.ANOMALY or self._distrust & set(closer.sources.values()):
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
        self._escalated = False
        self._extra = {}
        self._fallback_done = True
        self._undecided_holds = 0
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
        # 判定だけのときは止まるのを待たない (止める要求を出さないので、待つと上限の時間まで決着が遅れる)
        if (self.config.observe_only or fused is None
                or not separated(pose, fused, s.small_correction_m, s.small_correction_rad)):
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
        over = self._escalated or t - since_t >= s.max_closed_sec or self._odometer - since_odometer >= s.max_closed_m
        if over and self._odometer >= self._next_undecided_hold:
            self._undecided_hold_since = t
            self._undecided_holds += 1
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
        elif self._verify_pending is not None:
            reason = VERIFY
        elif self._odometer - self._last_confirm_odometer >= s.confirm_distance_m:
            reason = 'confirm'
        else:
            return None
        occupancy = snapshot.reference(s.reference).occupancy
        ready = occupancy is not None and len(self._buffer) > 0
        candidates = self._candidates(snapshot, motion, reason) if ready else {}
        initial = None
        if ready and reason == 'startup':
            initial = self._initial_search(snapshot)
        elif ready and reason == 'closed' and self._wide_search_due():
            initial = self._wide_search(snapshot, motion)
        if not candidates and initial is None:
            if reason == VERIFY:
                # 照合できない (地図・スキャン・候補がない) 要求は、待たせずに「決まらない」で返す
                self._verify_undecided, self._verify_pending = self._verify_pending, None
            return None
        if reason == MANUAL:
            self._manual = None
        if reason == 'confirm':
            self._last_confirm_odometer = self._odometer
        points = self._buffer.points_in((0.0, 0.0, 0.0), motion)
        return MatchRequest(self._generation, reason, t, motion, self._odometer, candidates, points, occupancy,
                            self._fields, initial)

    def _candidates(self, snapshot: Snapshot, motion: Pose2, reason: str) -> Dict[str, CandidatePose]:
        s = self.settings
        c = self.config.map_match
        narrow = (c.narrow_xy_m, c.narrow_yaw_rad)
        if reason == MANUAL:
            return {MANUAL: CandidatePose(self._manual, s.manual_xy_m, s.manual_yaw_rad)}
        candidates: Dict[str, CandidatePose] = {}
        for name in (s.estimator, s.fused):
            pose = self._pose_now(snapshot, name)
            if pose is not None:
                candidates[name] = CandidatePose(pose, *narrow)
        if reason == 'startup':
            # 推定器が出す前に決めると、収束する前の推定器を、融合の結果を照合した姿勢へ補正してしまう
            # (初期全周探索があるときは、その結果だけで決められる)
            if s.estimator not in candidates:
                return {}
            # 推定器と融合の結果がともにずれた起動 (動きながらの起動など) でも、健全な GNSS の周りで正しい姿勢を探す
            self._add_gnss(snapshot, candidates, candidates.get(s.fused) or candidates[s.estimator],
                           c.cap_yaw_rad)
            return candidates
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
        if reason == 'confirm':
            return candidates
        for name, extra in self._extra.items():
            candidates.setdefault(name, extra)
        yaw_from = candidates.get(CONTINUATION) or candidates.get(s.fused) or candidates.get(s.estimator)
        self._add_gnss(snapshot, candidates, yaw_from,
                       continuation_yaw if continuation_yaw is not None else c.cap_yaw_rad)
        return candidates

    def _add_gnss(self, snapshot: Snapshot, candidates: Dict[str, CandidatePose],
                  yaw_from: Optional[CandidatePose], yaw_range: float) -> None:
        """申告の精度が良い GNSS の位置を候補に足す。向きは yaw_from から借りる。"""
        s = self.settings
        if not s.gnss or yaw_from is None or s.gnss in self._distrust:
            return
        gnss = snapshot.pose_aligned(s.gnss, s.gnss_max_age_sec)
        if gnss is None:
            return
        sample = gnss[0]
        sigma = position_std(sample.cov)
        if sigma <= s.gnss_max_std_m:
            candidates[s.gnss] = CandidatePose(
                (sample.pose[0], sample.pose[1], yaw_from.pose[2]), s.gnss_sigma_k * sigma, yaw_range)

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
        if request.reason == 'startup' and self.config.initial_search is not None:
            self._apply_startup(request, decision, t, motion, fused, out)
            return
        if request.reason == VERIFY:
            self._apply_verify(request, decision, t, motion, fused, out)
            return
        if request.initial_result is not None:
            result = request.initial_result
            out.events.append(GateEvent('wide_search', 'info' if result.reason == 'matched' else 'warn', {
                'result': result.reason, 'ratio': round(result.ratio, 3), 'margin': round(result.margin, 3),
                **_pose_fields(result.pose), **_pose_fields(request.initial.center, 'center_')}))
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
            # 推定器と同じ仮説なら、収束の途中を邪魔しないよう補正しない
            self._open(t, 'startup', out, target, motion)
            return
        self._correct(t, target, 'none' if adopted else 'all', out, motion, fused)

    def _apply_verify(self, request: MatchRequest, decision: MatchDecision, t: float, motion: Pose2,
                      fused: Optional[Pose2], out: GateStep) -> None:
        """検証要求の照合の結果。推定器と同じ仮説なら検証済み姿勢を更新し、別の仮説なら閉じて姿勢補正、
        決まらなければ閉じて止めて照合する (自己位置監視が閉じさせたときと同じ)。"""
        number = self._verify_pending
        self._verify_pending = None
        if number is None or self.state != GateState.OPEN:
            if number is not None:
                self._finish_verify(number, VERIFY_UNDECIDED, out)
            return
        estimator = self.settings.estimator
        if decision.best is None:
            self.force_close(t, VERIFY)
            self._finish_verify(number, VERIFY_UNDECIDED, out)
            return
        adopted = estimator in decision.agree
        target = _propagate(request.candidates[estimator].pose if adopted else decision.pose, request.motion, motion)
        # 確かめたのは、推定器をそのまま採れるときだけ (近くても採れなければ、決めた姿勢へ補正する)
        if adopted:
            self._anchor = _Anchor(request.candidates[estimator].pose, request.motion, request.odometer)
            self._last_confirm_odometer = self._odometer
            out.events.append(GateEvent('pose_confirmed', 'info', _pose_fields(target)))
            self._finish_verify(number, VERIFY_CONFIRMED, out)
            return
        self._close(t, VERIFY, out, result=decision.best)
        self._correct(t, target, 'all', out, motion, fused)
        self._finish_verify(number, VERIFY_RELOCATED, out)

    def _finish_verify(self, number: int, result: str, out: GateStep) -> None:
        self.last_verify = (number, result)
        out.verified = (number, result)
        out.events.append(GateEvent('verify_result', 'info' if result == VERIFY_CONFIRMED else 'warn',
                                    {'id': number, 'result': result}))

    def _apply_startup(self, request: MatchRequest, decision: MatchDecision, t: float, motion: Pose2,
                       fused: Optional[Pose2], out: GateStep) -> None:
        """初期全周探索があるときの、未初期化間の決め方。

        - 推定器と同じ仮説に決まれば、補正せずに開ける (収束の途中を邪魔しない)
        - 別の仮説に補正するのは、初期全周探索の姿勢と同じ仮説に決まったときだけ (狭い候補だけで、低い一致率の
          別の仮説を選ばない)。まだ走行に使う姿勢が定まっていないので、止まるのを待たずにその場で補正する
        - 初期全周探索が合わず、まだ一度も確かめていなければ、GNSS の位置を広い共分散で 1 回だけ出す
          (推定器に広く探させる。確かめていないので、未初期化状態のまま)
        """
        s = self.settings
        result = request.initial_result
        if result is not None:
            out.events.append(GateEvent('initial_search', 'info' if result.reason == 'matched' else 'warn', {
                'result': result.reason, 'ratio': round(result.ratio, 3), 'margin': round(result.margin, 3),
                **_pose_fields(result.pose), **_pose_fields(request.initial.center, 'gnss_')}))
        if self.state != GateState.UNCONFIRMED:
            return

        def now(pose: Pose2) -> Pose2:
            return _propagate(pose, request.motion, motion)

        if decision.best is not None and s.estimator in decision.same:
            given = request.candidates[s.estimator].pose if s.estimator in decision.agree else decision.pose
            self._open(t, 'startup', out, now(given), motion)
            return
        if decision.best is not None and INITIAL in decision.same:
            self._reset(t, now(decision.pose), 'all', 'startup', out, motion, None)
            return
        self._next_match = t + s.retry_sec
        if result is None or result.reason == 'matched' or self._fallback_done:
            return
        self._fallback_done = True
        cfg = self.config.initial_search
        pose = now(_propagate(request.initial.center, request.initial.motion, request.motion))
        out.reset = ResetCommand(pose, np.diag([cfg.fallback_std_m ** 2, cfg.fallback_std_m ** 2,
                                                cfg.fallback_std_yaw_rad ** 2]), 'all')
        out.events.append(GateEvent('pose_reset', 'warn', {'how': 'fallback', 'targets': 'all', **_pose_fields(pose)}))
