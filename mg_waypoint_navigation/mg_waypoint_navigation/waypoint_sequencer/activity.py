"""走行の「いまの活動」の合成。ROS に依存しない純関数。

シーケンサの状態と Nav2 側の入力 (collision_monitor・BT のリカバリー・位置が進まない時間) から、
表示用の短い要約を 1 つ決める。判定はここだけで行い、UI は表示するだけにする。
code・emphasis の値は mg_msgs/msg/NavigationActivity.msg の定数と合わせる。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, Optional, Tuple

from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState

# emphasis (表示の強調。健全性ではない)
NORMAL = 0
NOTICE = 1
ALERT = 2

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
NO_PROGRESS = 'no_progress'
ACTION = 'action'
TRIGGER_WAIT = 'trigger_wait'
PAUSED = 'paused'
GOAL_REACHED = 'goal_reached'
ERROR = 'error'

# nav2_msgs/msg/CollisionMonitorState の action_type
COLLISION_STOP = 1
COLLISION_SLOWDOWN = 2

# データ途絶で停止したときの polygon_name の接頭辞 (mg_navigation/doc/nav2_config.md)
SOURCE_TIMEOUT_PREFIX = 'source_timeout:'

# リカバリーの BT ノード名と文言。先に書いたものを優先する。
# 自作の BT (mg_waypoint_navigation/behavior_trees) の名前を先に、Nav2 既定の BT 用の汎用の名前を後に置く
RECOVERY_TEXTS: Tuple[Tuple[str, str], ...] = (
    ('SpinLeftAndReplan', 'リカバリー: 左へ回転'),
    ('SpinRightAndReplan', 'リカバリー: 右へ回転'),
    ('BackUpAndReplan', 'リカバリー: 後退'),
    ('ClearCostmapsAndWait', 'リカバリー: 障害物の記録を消して待機'),
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


@dataclass(frozen=True)
class Activity:
    code: str
    text: str
    emphasis: int = NORMAL
    detail: str = ''
    index: int = -1
    total: int = -1
    remaining: float = -1.0


def _recovery_text(running_nodes: FrozenSet[str]) -> Optional[str]:
    for name, text in RECOVERY_TEXTS:
        if name in running_nodes:
            return text
    return None


def _compose_driving(
        inputs: ActivityInputs, stall_warn_sec: float, external: bool = False) -> Activity:
    """Nav2 のゴールを走行中の活動。リカバリー・停止・進めない・減速・通常の順に決める。

    external は、シーケンサが送っていないゴール (RViz の 2D Goal Pose など) の走行。
    """
    no_waypoint = inputs.manual_goal_active or external
    index = -1 if no_waypoint else inputs.current_index
    total = -1 if no_waypoint else inputs.total_waypoints
    recovery = _recovery_text(inputs.running_recovery_nodes)
    if recovery is not None:
        return Activity(RECOVERY, recovery, NOTICE, index=index, total=total)

    collision = inputs.collision
    if collision is not None and collision.action_type == COLLISION_STOP:
        if collision.polygon_name.startswith(SOURCE_TIMEOUT_PREFIX):
            return Activity(
                SENSOR_TIMEOUT_STOP, 'センサ途絶で停止中', ALERT,
                detail=collision.polygon_name[len(SOURCE_TIMEOUT_PREFIX):],
                index=index, total=total)
        return Activity(
            OBSTACLE_STOP, '障害物で停止中', NOTICE,
            detail=collision.polygon_name, index=index, total=total)

    if inputs.stalled_sec >= stall_warn_sec:
        return Activity(NO_PROGRESS, '進めていません', ALERT, index=index, total=total)

    if collision is not None and collision.action_type == COLLISION_SLOWDOWN:
        return Activity(
            OBSTACLE_SLOWDOWN, '障害物で減速中',
            detail=collision.polygon_name, index=index, total=total)

    if external:
        return Activity(EXTERNAL_GOAL, '外部指定のゴールへ走行中')

    remaining = inputs.distance_remaining
    if inputs.manual_goal_active:
        return Activity(MANUAL_GOAL, '指定地点へ走行中', remaining=remaining)
    return Activity(
        NAVIGATING, '地点へ走行中', index=index, total=total, remaining=remaining)


def compose_activity(inputs: ActivityInputs, stall_warn_sec: float) -> Activity:
    """入力から「いまの活動」を 1 つ決める。優先順位は上から。"""
    state = inputs.state
    if state == SequencerState.SUSPENDED:
        return Activity(
            PAUSED, '一時停止中', NOTICE,
            detail=', '.join(inputs.pause_requesters),
            index=inputs.current_index, total=inputs.total_waypoints)
    if state == SequencerState.ON_STARTING:
        return Activity(
            DEPARTURE_WAIT, '出発待ち',
            index=inputs.current_index, total=inputs.total_waypoints,
            remaining=inputs.countdown_ms_remaining / 1000.0)
    if state == SequencerState.ON_ARRIVING:
        action = inputs.current_action
        text = ACTION_TEXTS.get(action, 'アクション実行中')
        return Activity(
            ACTION, text, detail=action,
            index=inputs.current_index, total=inputs.total_waypoints)
    if state == SequencerState.NAVIGATING or inputs.manual_goal_active:
        return _compose_driving(inputs, stall_warn_sec)
    if state == SequencerState.ERROR:
        return Activity(ERROR, 'エラーで停止中', ALERT)
    if inputs.external_goal_active:
        return _compose_driving(inputs, stall_warn_sec, external=True)
    if state == SequencerState.IDLE and inputs.waiting_trigger:
        return Activity(
            TRIGGER_WAIT, 'トリガー待ち',
            index=inputs.current_index, total=inputs.total_waypoints)
    if state == SequencerState.GOAL_REACHED:
        return Activity(GOAL_REACHED, '全地点に到着')
    return Activity(IDLE, 'Waypoint 走行なし')


class ActivityTracker:
    """code が続いている秒数を数える。code が変わったときだけ 0 に戻る。"""

    def __init__(self) -> None:
        self._code: Optional[str] = None
        self._since: float = 0.0

    def elapsed_sec(self, code: str, now: float) -> float:
        if code != self._code:
            self._code = code
            self._since = now
        return now - self._since
