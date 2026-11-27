"""走行の「いまの活動」の合成。ROS に依存しない純関数。

シーケンサの状態と Nav2 側・自己位置監視の入力から、表示用の短い要約を 1 つ決める。
判定はここだけで行い、UI は表示するだけにする。code の値は mg_msgs/msg/NavigationActivity.msg の定数と合わせる。

合成は 3 段に分ける。
1. 事実 (derive_facts): 生の入力から、判定に使う事実を 1 回だけ作る。
2. 規則 (RULES): code ごとに、当たる条件と活動の作り方を書く。各規則は自分の条件だけを書く。
3. 順位表 (PRIORITY): 規則を並べた 1 か所だけの表。上から最初に当たった規則を採る。
順位を変えるときは PRIORITY を並べ替えるだけにする。規則を足すときに変える場所は doc/architecture.md に書く。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, FrozenSet, Optional, Tuple

from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState

# code
IDLE = 'idle'
DEPARTURE_WAIT = 'departure_wait'
NAVIGATING = 'navigating'
MANUAL_GOAL = 'manual_goal'
EXTERNAL_GOAL = 'external_goal'
OBSTACLE_SLOWDOWN = 'obstacle_slowdown'
OBSTACLE_STOP = 'obstacle_stop'
SENSOR_TIMEOUT_STOP = 'sensor_timeout_stop'
RECOVERY = 'recovery'
DETOUR_REJECTED = 'detour_rejected'
NO_PROGRESS = 'no_progress'
ACTION = 'action'
TRIGGER_WAIT = 'trigger_wait'
READY_TO_START = 'ready_to_start'
PAUSED = 'paused'
LOCALIZATION_HOLD = 'localization_hold'
WATCHDOG_PAUSE = 'watchdog_pause'
LOCALIZATION_RECOVERING_DRIVE = 'localization_recovering_drive'
LOCALIZATION_SLOWDOWN = 'localization_slowdown'
NO_LIDAR_DRIVE = 'no_lidar_drive'
LOCALIZATION_UNKNOWN = 'localization_unknown'
GOAL_REACHED = 'goal_reached'
ERROR = 'error'

# nav2_msgs/msg/CollisionMonitorState の action_type
COLLISION_STOP = 1
COLLISION_SLOWDOWN = 2
COLLISION_APPROACH = 3
# 速度を落とす動作 (slowdown と approach)。どちらも「障害物で減速中」として出す
COLLISION_SLOWDOWN_TYPES = (COLLISION_SLOWDOWN, COLLISION_APPROACH)

# データ途絶で停止したときの polygon_name の接頭辞 (mg_navigation/doc/costmap_and_collision.md)
SOURCE_TIMEOUT_PREFIX = 'source_timeout:'

# localization_supervisor_msgs/msg/LocalizationStatus の state
LOCALIZATION_STATE_NAMES: Dict[int, str] = {
    0: 'NORMAL', 1: 'SUSPECT', 2: 'ISOLATED', 3: 'RECOVERING', 4: 'DEGRADED'}
# 情報源を除外した・推定器を初期化し直した・復旧できていない (止めずに走っているときに出す)
LOCALIZATION_RECOVERING_STATES = frozenset((2, 3, 4))
# LocalizationStatus.sensor_set の LiDAR なしの組
SENSOR_SET_NO_LIDAR = 'no_lidar'

# 走行に戻らない状態 (この間は Nav2 のゴールがあっても走行として扱わない)
_STOPPED_STATES = frozenset((
    SequencerState.SUSPENDED, SequencerState.ON_STARTING, SequencerState.ON_ARRIVING))

# リカバリーの BT ノード名と文言。先に書いたものを優先する。
# 自作の BT (mg_waypoint_navigation/behavior_trees) の名前を先に、Nav2 既定の BT 用の汎用の名前を後に置く
RECOVERY_TEXTS: Tuple[Tuple[str, str], ...] = (
    ('EscapeKeepout', 'リカバリー: 進入禁止の領域から脱出'),
    ('Replan', 'リカバリー: 経路を引き直し'),
    ('ClearAndReplan', 'リカバリー: 障害物の記録を消して引き直し'),
    ('SpinLeftAndReplan', 'リカバリー: 左へ回転'),
    ('SpinRightAndReplan', 'リカバリー: 右へ回転'),
    ('BackUpAndReplan', 'リカバリー: 後退'),
    ('ClearLocalCostmapAndWait', 'リカバリー: 障害物の記録を消して待機'),
    ('Spin', 'リカバリー: 回転'),
    ('BackUp', 'リカバリー: 後退'),
    ('Wait', 'リカバリー: 待機'),
)
RECOVERY_NODE_NAMES: FrozenSet[str] = frozenset(name for name, _ in RECOVERY_TEXTS)

# 到着後アクションの type と文言
ACTION_TEXTS: Dict[str, str] = {
    'load_map': '地図切り替えを実行中',
    'wait': '待機中',
    'amcl_reset': '自己位置をリセット中',
    'set_navigation_mode': '走行モードを切り替え中',
}


@dataclass(frozen=True)
class CollisionInput:
    action_type: int
    polygon_name: str


@dataclass(frozen=True)
class DetourInput:
    """遠回りが長すぎるため棄却した、直近の経路の候補。長さの単位は [m]。"""
    path_length: float
    limit: float


@dataclass(frozen=True)
class LocalizationInput:
    """自己位置監視の状態 (/localization/status) のうち、活動に使うもの。"""
    state: int
    situation: str = ''
    speed_limit: float = 0.0
    sensor_set: str = ''
    blame: str = ''


@dataclass(frozen=True)
class ActivityInputs:
    state: SequencerState
    current_index: int
    total_waypoints: int
    countdown_ms_remaining: int
    pause_requesters: Tuple[str, ...]
    distance_remaining: float
    current_action: str
    waiting_trigger: bool
    manual_goal_active: bool
    external_goal_active: bool
    collision: Optional[CollisionInput]
    running_recovery_nodes: FrozenSet[str]
    stalled_sec: float
    rejected_detour: Optional[DetourInput] = None
    trigger_acceptable: bool = False
    # 自己位置監視の状態。使わない設定なら localization_expected が False。
    # 使う設定で None なら、届いていない (古い) とみなす
    localization: Optional[LocalizationInput] = None
    localization_expected: bool = False


@dataclass(frozen=True)
class ActivitySettings:
    stall_warn_sec: float
    # 一時停止の要求元のうち、自己位置監視とセンサの見張りのもの (送る側の requester_id と合わせる)
    localization_requesters: FrozenSet[str] = frozenset(('localization',))
    watchdog_requesters: FrozenSet[str] = frozenset(('driver_watchdog',))


@dataclass(frozen=True)
class Activity:
    code: str
    text: str
    detail: str = ''
    index: int = -1
    total: int = -1
    remaining: float = -1.0


@dataclass(frozen=True)
class Facts:
    """判定に使う事実。入力から 1 回だけ作り、規則はこれだけを見る。"""
    inputs: ActivityInputs
    # 一時停止の要求で止まっている (SUSPENDED、または IDLE で要求が出ていて走り出せない)
    suspended: bool
    # シーケンサのゴール (地点・手動) の走行と、シーケンサを通らないゴールの走行
    own_drive: bool
    external_drive: bool
    driving: bool
    recovery_text: Optional[str]
    collision_stop: bool
    sensor_timeout: bool
    collision_slowdown: bool
    stalled: bool
    localization_requested_pause: bool
    watchdog_requested_pause: bool
    # 走行中の地点の番号 (手動・外部のゴールでは -1)
    index: int
    total: int


def _recovery_text(running_nodes: FrozenSet[str]) -> Optional[str]:
    for name, text in RECOVERY_TEXTS:
        if name in running_nodes:
            return text
    return None


def derive_facts(inputs: ActivityInputs, settings: ActivitySettings) -> Facts:
    state = inputs.state
    own_drive = state == SequencerState.NAVIGATING or (
        inputs.manual_goal_active and state not in _STOPPED_STATES)
    external_drive = (inputs.external_goal_active and not own_drive
                      and state not in _STOPPED_STATES and state != SequencerState.ERROR)
    collision = inputs.collision
    collision_stop = collision is not None and collision.action_type == COLLISION_STOP
    requesters = frozenset(inputs.pause_requesters)
    no_waypoint = inputs.manual_goal_active or external_drive
    return Facts(
        inputs=inputs,
        suspended=(state == SequencerState.SUSPENDED or (
            state == SequencerState.IDLE and bool(requesters) and not (own_drive or external_drive))),
        own_drive=own_drive,
        external_drive=external_drive,
        driving=own_drive or external_drive,
        recovery_text=_recovery_text(inputs.running_recovery_nodes),
        collision_stop=collision_stop,
        sensor_timeout=collision_stop and collision.polygon_name.startswith(SOURCE_TIMEOUT_PREFIX),
        collision_slowdown=(collision is not None
                            and collision.action_type in COLLISION_SLOWDOWN_TYPES),
        # 外部のゴールの進み具合は測っていない (ProgressMonitor はシーケンサのゴール専用)
        stalled=own_drive and inputs.stalled_sec >= settings.stall_warn_sec,
        localization_requested_pause=bool(requesters & settings.localization_requesters),
        watchdog_requested_pause=bool(requesters & settings.watchdog_requesters),
        index=-1 if no_waypoint else inputs.current_index,
        total=-1 if no_waypoint else inputs.total_waypoints,
    )


# ---------------------------------------------------------------------------
# 規則。applies は自分の条件だけを書き、build は活動を作る
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Rule:
    applies: Callable[[Facts], bool]
    build: Callable[[Facts], Activity]


def _sequence_position(f: Facts) -> Dict[str, int]:
    return {'index': f.inputs.current_index, 'total': f.inputs.total_waypoints}


def _paused(code: str, text: str) -> Callable[[Facts], Activity]:
    return lambda f: Activity(
        code, text, detail=', '.join(f.inputs.pause_requesters), **_sequence_position(f))


def _driving(code: str, text: str, detail: Callable[[Facts], str] = lambda f: '',
             with_remaining: bool = False) -> Callable[[Facts], Activity]:
    """走行中の活動。with_remaining なら、シーケンサのゴールの残り距離を付ける。"""
    def build(f: Facts) -> Activity:
        remaining = f.inputs.distance_remaining if with_remaining and f.own_drive else -1.0
        return Activity(code, text, detail=detail(f), index=f.index, total=f.total,
                        remaining=remaining)
    return build


def _localization(f: Facts) -> Optional[LocalizationInput]:
    return f.inputs.localization


def _localization_detail(f: Facts) -> str:
    loc = f.inputs.localization
    if loc is None:
        return ''
    parts = [LOCALIZATION_STATE_NAMES.get(loc.state, str(loc.state))]
    if loc.situation:
        parts.append(loc.situation)
    if loc.speed_limit > 0.0:
        parts.append(f'上限 {loc.speed_limit:.2f} m/s')
    if loc.blame:
        parts.append(loc.blame)
    return ', '.join(parts)


def _detour_detail(f: Facts) -> str:
    detour = f.inputs.rejected_detour
    return f'経路 {detour.path_length:.0f} m / 上限 {detour.limit:.0f} m'


def _collision_name(f: Facts) -> str:
    return f.inputs.collision.polygon_name


def _build_recovery(f: Facts) -> Activity:
    return Activity(RECOVERY, f.recovery_text, index=f.index, total=f.total)


def _build_action(f: Facts) -> Activity:
    action = f.inputs.current_action
    return Activity(ACTION, ACTION_TEXTS.get(action, 'アクション実行中'), detail=action,
                    **_sequence_position(f))


def _build_navigating(f: Facts) -> Activity:
    return Activity(NAVIGATING, '地点へ走行中', index=f.index, total=f.total,
                    remaining=f.inputs.distance_remaining)


RULES: Dict[str, Rule] = {
    # 一時停止 (要求元で分ける。detail には全部の要求元を並べる)
    LOCALIZATION_HOLD: Rule(
        lambda f: f.suspended and f.localization_requested_pause,
        _paused(LOCALIZATION_HOLD, '自己位置の確認のため停止中')),
    WATCHDOG_PAUSE: Rule(
        lambda f: f.suspended and f.watchdog_requested_pause,
        _paused(WATCHDOG_PAUSE, 'センサ監視による一時停止中')),
    PAUSED: Rule(lambda f: f.suspended, _paused(PAUSED, '一時停止中')),
    # シーケンサの停止中の状態
    DEPARTURE_WAIT: Rule(
        lambda f: f.inputs.state == SequencerState.ON_STARTING,
        lambda f: Activity(DEPARTURE_WAIT, '出発待ち', remaining=f.inputs.countdown_ms_remaining / 1000.0,
                           **_sequence_position(f))),
    ACTION: Rule(lambda f: f.inputs.state == SequencerState.ON_ARRIVING, _build_action),
    ERROR: Rule(
        lambda f: f.inputs.state == SequencerState.ERROR and not f.driving,
        lambda f: Activity(ERROR, 'エラーで停止中')),
    # 走行中。遠回りの棄却は、リカバリー中だけ、リカバリーの表示に代えて出す
    DETOUR_REJECTED: Rule(
        lambda f: f.driving and f.recovery_text is not None and f.inputs.rejected_detour is not None,
        _driving(DETOUR_REJECTED, '遠回りの経路を棄却して待機中', _detour_detail)),
    RECOVERY: Rule(lambda f: f.driving and f.recovery_text is not None, _build_recovery),
    SENSOR_TIMEOUT_STOP: Rule(
        lambda f: f.driving and f.sensor_timeout,
        _driving(SENSOR_TIMEOUT_STOP, 'センサ途絶で停止中',
                 lambda f: f.inputs.collision.polygon_name[len(SOURCE_TIMEOUT_PREFIX):])),
    OBSTACLE_STOP: Rule(
        lambda f: f.driving and f.collision_stop,
        _driving(OBSTACLE_STOP, '障害物で停止中', _collision_name)),
    NO_PROGRESS: Rule(
        lambda f: f.driving and f.stalled, _driving(NO_PROGRESS, '進めていません')),
    LOCALIZATION_RECOVERING_DRIVE: Rule(
        lambda f: (f.driving and _localization(f) is not None
                   and _localization(f).state in LOCALIZATION_RECOVERING_STATES),
        _driving(LOCALIZATION_RECOVERING_DRIVE, '自己位置を復旧しながら走行中',
                 _localization_detail, with_remaining=True)),
    LOCALIZATION_SLOWDOWN: Rule(
        lambda f: f.driving and _localization(f) is not None and _localization(f).speed_limit > 0.0,
        _driving(LOCALIZATION_SLOWDOWN, '自己位置の不確かさで減速中',
                 _localization_detail, with_remaining=True)),
    NO_LIDAR_DRIVE: Rule(
        lambda f: (f.driving and _localization(f) is not None
                   and _localization(f).sensor_set == SENSOR_SET_NO_LIDAR),
        _driving(NO_LIDAR_DRIVE, 'LiDAR なしで走行中', _localization_detail, with_remaining=True)),
    LOCALIZATION_UNKNOWN: Rule(
        lambda f: f.driving and f.inputs.localization_expected and _localization(f) is None,
        _driving(LOCALIZATION_UNKNOWN, '自己位置の状態が届かないまま走行中', with_remaining=True)),
    OBSTACLE_SLOWDOWN: Rule(
        lambda f: f.driving and f.collision_slowdown,
        _driving(OBSTACLE_SLOWDOWN, '障害物で減速中', _collision_name)),
    EXTERNAL_GOAL: Rule(
        lambda f: f.external_drive, lambda f: Activity(EXTERNAL_GOAL, '外部指定のゴールへ走行中')),
    MANUAL_GOAL: Rule(
        lambda f: f.own_drive and f.inputs.manual_goal_active,
        lambda f: Activity(MANUAL_GOAL, '指定地点へ走行中', remaining=f.inputs.distance_remaining)),
    NAVIGATING: Rule(lambda f: f.own_drive, _build_navigating),
    # 待機
    # 開始待ちは、MMI の LED1 と同じく ~/trigger で始められる (trigger_acceptable) ときだけ
    TRIGGER_WAIT: Rule(
        lambda f: (not f.driving and f.inputs.state == SequencerState.IDLE
                   and f.inputs.waiting_trigger and f.inputs.trigger_acceptable),
        lambda f: Activity(TRIGGER_WAIT, 'トリガー待ち', **_sequence_position(f))),
    READY_TO_START: Rule(
        lambda f: (not f.driving and f.inputs.state == SequencerState.IDLE
                   and f.inputs.trigger_acceptable),
        lambda f: Activity(READY_TO_START, '開始待ち', **_sequence_position(f))),
    GOAL_REACHED: Rule(
        lambda f: not f.driving and f.inputs.state == SequencerState.GOAL_REACHED,
        lambda f: Activity(GOAL_REACHED, '全地点に到着')),
    IDLE: Rule(lambda f: True, lambda f: Activity(IDLE, 'Waypoint 走行なし')),
}

# 優先順位 (上から。最初に当たった規則を採る)。一時停止・停止中・走行中・待機の群は、
# 規則の条件 (driving など) で互いに排他なので、順位は群の中で効く。最後の IDLE は必ず当たる
PRIORITY: Tuple[str, ...] = (
    LOCALIZATION_HOLD,
    WATCHDOG_PAUSE,
    PAUSED,
    DEPARTURE_WAIT,
    ACTION,
    ERROR,
    DETOUR_REJECTED,
    RECOVERY,
    SENSOR_TIMEOUT_STOP,
    OBSTACLE_STOP,
    NO_PROGRESS,
    LOCALIZATION_RECOVERING_DRIVE,
    LOCALIZATION_SLOWDOWN,
    NO_LIDAR_DRIVE,
    LOCALIZATION_UNKNOWN,
    OBSTACLE_SLOWDOWN,
    EXTERNAL_GOAL,
    MANUAL_GOAL,
    NAVIGATING,
    TRIGGER_WAIT,
    READY_TO_START,
    GOAL_REACHED,
    IDLE,
)


def validate_priority() -> None:
    """順位表と規則の code が一対一か、最後が必ず当たる IDLE かを確かめる (読み込み時に落とす)。"""
    if len(set(PRIORITY)) != len(PRIORITY):
        raise RuntimeError(f'PRIORITY に重複がある: {PRIORITY}')
    if set(PRIORITY) != set(RULES):
        raise RuntimeError(
            f'PRIORITY と RULES の code が一致しない: 順位だけ {set(PRIORITY) - set(RULES)}, '
            f'規則だけ {set(RULES) - set(PRIORITY)}')
    if PRIORITY[-1] != IDLE:
        raise RuntimeError('PRIORITY の最後は IDLE にする')


validate_priority()


def compose_activity(inputs: ActivityInputs, settings: ActivitySettings) -> Activity:
    """入力から「いまの活動」を 1 つ決める。"""
    facts = derive_facts(inputs, settings)
    for code in PRIORITY:
        rule = RULES[code]
        if rule.applies(facts):
            return rule.build(facts)
    raise AssertionError('IDLE の規則は必ず当たる')


# ---------------------------------------------------------------------------
# 変化の判定
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ActivityChange:
    """採った活動の code の変化 (イベント sequencer.activity_changed の中身)。"""
    from_code: str
    activity: Activity
    # 前の code が続いた秒数 (最初の活動では 0) と、新しい code がすでに続いている秒数
    from_sec: float
    since_sec: float


# 変化の判定にかけず、すぐに採る活動 (止まっていることを示す活動)。短くても止まった事実は出す。
# 判定にかけるのは走行中の活動の揺れ (障害物・自己位置での細かな切り替わり) だけにする
IMMEDIATE_CODES: FrozenSet[str] = frozenset((
    LOCALIZATION_HOLD, WATCHDOG_PAUSE, PAUSED, DEPARTURE_WAIT, ACTION, ERROR,
    TRIGGER_WAIT, READY_TO_START,
))


class ActivityDebouncer:
    """新しい code が min_duration_sec 続いたときだけ採る。0 なら全ての変化をすぐに採る。
    IMMEDIATE_CODES の活動は、続いた秒数によらずすぐに採る。

    採るまでの間は、前に採った活動をそのまま出す。採った code の開始は、新しい code が
    出始めた時刻にさかのぼる (elapsed_sec と前の code の続いた秒数をそれで数える)。
    同じ code の間は、補足や残り距離などの値を新しいものに入れ替える。
    """

    def __init__(self, min_duration_sec: float) -> None:
        self._min_duration_sec = max(0.0, min_duration_sec)
        self._accepted: Optional[Activity] = None
        self._accepted_since = 0.0
        self._candidate_code: Optional[str] = None
        self._candidate_since = 0.0

    def update(self, activity: Activity, now: float
               ) -> Tuple[Activity, float, Optional[ActivityChange]]:
        """(出す活動, その code が続いている秒数, 採った変化。なければ None) を返す。"""
        accepted = self._accepted
        if accepted is None:
            self._accept(activity, now)
            return activity, 0.0, ActivityChange('', activity, 0.0, 0.0)
        if activity.code == accepted.code:
            self._accepted = activity
            self._candidate_code = None
            return activity, now - self._accepted_since, None
        if activity.code != self._candidate_code:
            self._candidate_code = activity.code
            self._candidate_since = now
        since = now - self._candidate_since
        if since < self._min_duration_sec and activity.code not in IMMEDIATE_CODES:
            return accepted, now - self._accepted_since, None
        change = ActivityChange(
            accepted.code, activity, self._candidate_since - self._accepted_since, since)
        self._accept(activity, self._candidate_since)
        return activity, since, change

    def _accept(self, activity: Activity, since: float) -> None:
        self._accepted = activity
        self._accepted_since = since
        self._candidate_code = None
