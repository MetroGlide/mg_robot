"""「いまの活動」の合成に使う Nav2 側の入力 (collision_monitor の状態・BT のリカバリー) を保持する。

購読して最新の値を保存するだけで、判定は activity.py が行う。
"""
from __future__ import annotations

import threading
from typing import FrozenSet, Optional, Set

import rclpy.node
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mg_waypoint_navigation.waypoint_sequencer.activity import (
    CollisionInput,
    RECOVERY_NODE_NAMES,
)

# 走行に戻ったことを示す BT ノード。これが動き出したら、リカバリーは終わっている
_FOLLOW_PATH_NODE = 'FollowPath'
_RUNNING = 'RUNNING'


class ActivitySources:
    """collision_monitor の最新の状態と、実行中のリカバリーの BT ノード名を保持する。

    collision_monitor の状態は変化したときしか届かない。まだ届いていなければ「障害物なし」として扱う。
    BT のログは終了の通知を取りこぼしても表示が残り続けないよう、走行に戻ったとき
    (FollowPath が動き出したとき) と、走行中でなくなったときに記録を消す。
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        callback_group: CallbackGroup,
        collision_state_topic: str,
        bt_log_topic: str,
    ):
        self._lock = threading.Lock()
        self._collision: Optional[CollisionInput] = None
        self._recovery_nodes: Set[str] = set()

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
