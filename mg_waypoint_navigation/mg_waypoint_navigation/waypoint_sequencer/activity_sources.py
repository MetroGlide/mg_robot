"""「いまの活動」の合成に使う Nav2 側の入力 (collision_monitor の状態・BT のリカバリー) を保持する。

購読して最新の値を保存するだけで、判定は activity.py が行う。
"""
from __future__ import annotations

import threading
from typing import FrozenSet, Optional, Set

import rclpy.node
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy)

from mg_waypoint_navigation.waypoint_sequencer.activity import (
    CollisionInput,
    RECOVERY_NODE_NAMES,
)

# 走行に戻ったことを示す BT ノード。これが動き出したら、リカバリーは終わっている
_FOLLOW_PATH_NODE = 'FollowPath'
_RUNNING = 'RUNNING'

# Nav2 のゴールが生きている状態
_LIVE_GOAL_STATUSES = frozenset(
    (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING))

# シーケンサのゴールの結果と、アクションの状態トピックの到着順の差で、外部のゴールと
# 取り違えないための持続時間 [s]
EXTERNAL_GOAL_HOLD_SEC = 1.0


class ActivitySources:
    """collision_monitor の最新の状態と、実行中のリカバリーの BT ノード名を保持する。

    collision_monitor の状態は変化したときしか届かない。まだ届いていなければ「障害物なし」として扱う。
    BT のログは終了の通知を取りこぼしても表示が残り続けないよう、走行に戻ったとき
    (FollowPath が動き出したとき) と、走行中でなくなったときに記録を消す。

    Nav2 のアクションの状態は、シーケンサが送っていないゴール (RViz の 2D Goal Pose など)
    の有無を知るために使う。状態は変化したときしか届かないので、発行側が居なくなったときは
    最後の値を捨てる。
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        callback_group: CallbackGroup,
        collision_state_topic: str,
        bt_log_topic: str,
        nav_status_topic: str,
    ):
        self._node = node
        self._nav_status_topic = nav_status_topic
        self._lock = threading.Lock()
        self._collision: Optional[CollisionInput] = None
        self._recovery_nodes: Set[str] = set()
        self._nav_goal_live = False
        self._external_since: Optional[float] = None

        if collision_state_topic:
            node.create_subscription(
                CollisionMonitorState, collision_state_topic,
                self.on_collision_state, 1, callback_group=callback_group)
        # 発行側の QoS (reliable) と、どちらの設定でも繋がるよう best effort で受ける
        if bt_log_topic:
            node.create_subscription(
                BehaviorTreeLog, bt_log_topic, self.on_bt_log,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=callback_group)
        # Nav2 のアクションの状態と同じ QoS。再起動直後も現在の状態が届く
        if nav_status_topic:
            node.create_subscription(
                GoalStatusArray, nav_status_topic, self.on_nav_status,
                QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=callback_group)

    def on_nav_status(self, msg: GoalStatusArray) -> None:
        live = any(s.status in _LIVE_GOAL_STATUSES for s in msg.status_list)
        with self._lock:
            self._nav_goal_live = live

    def external_goal_active(self, own_goal_live: bool, now: float) -> bool:
        """シーケンサが送っていない Nav2 ゴールが、EXTERNAL_GOAL_HOLD_SEC 以上生きているか。

        own_goal_live は、シーケンサ自身のゴールが結果の返るまでの間にあるか。
        """
        if not self._nav_status_topic:
            return False
        publishing = self._node.count_publishers(self._nav_status_topic) > 0
        with self._lock:
            if not (self._nav_goal_live and publishing) or own_goal_live:
                self._external_since = None
                return False
            if self._external_since is None:
                self._external_since = now
            return now - self._external_since >= EXTERNAL_GOAL_HOLD_SEC

    def on_collision_state(self, msg: CollisionMonitorState) -> None:
        with self._lock:
            self._collision = CollisionInput(msg.action_type, msg.polygon_name)

    def on_bt_log(self, msg: BehaviorTreeLog) -> None:
        with self._lock:
            for event in msg.event_log:
                if event.node_name == _FOLLOW_PATH_NODE:
                    if event.current_status == _RUNNING:
                        self._recovery_nodes.clear()
                elif event.node_name in RECOVERY_NODE_NAMES:
                    if event.current_status == _RUNNING:
                        self._recovery_nodes.add(event.node_name)
                    else:
                        self._recovery_nodes.discard(event.node_name)

    def clear_recovery(self) -> None:
        """新しいゴールを送ったときや、走行中でなくなったときに、リカバリーの記録を消す。"""
        with self._lock:
            self._recovery_nodes.clear()

    @property
    def collision(self) -> Optional[CollisionInput]:
        with self._lock:
            return self._collision

    @property
    def recovery_nodes(self) -> FrozenSet[str]:
        with self._lock:
            return frozenset(self._recovery_nodes)
