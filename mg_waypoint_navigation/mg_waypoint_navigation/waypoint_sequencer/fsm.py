"""ウェイポイントシーケンサーの有限状態機械"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Callable, List, Optional, Set

import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mg_waypoint_navigation.waypoint import (
    NAVIGATION_MODES,
    NavigationConfig,
    Waypoint,
    WaypointList,
)
from mg_waypoint_navigation.waypoint_sequencer.action_executor import ActionExecutor
from mg_waypoint_navigation.waypoint_sequencer.navigator import (
    DEFAULT_BT_MODE,
    NavigationResult,
    WaypointNavigator,
)
from mg_waypoint_navigation.waypoint_sequencer.states import (
    ALLOWED_TRANSITIONS,
    CommandResult,
    SequencerState,
)


# 手動ゴールの BT。ウェイポイントの navigation_mode に、Nav2 既定の BT を加えたもの
MANUAL_GOAL_MODES = NAVIGATION_MODES + (DEFAULT_BT_MODE,)

# 手動ゴールを受け付けるシーケンスの状態 (走行中・停止待ちなどは拒否する)
MANUAL_GOAL_STATES = frozenset({
    SequencerState.IDLE,
    SequencerState.GOAL_REACHED,
    SequencerState.ERROR,
})


class CountdownTimer:
    """指定時間後にコールバックを呼ぶ一発タイマー。

    コールバックには開始時の世代番号を渡す。start()/cancel() のたびに世代が進むので、
    受け側で generation と比較すれば、発火済みで処理待ちだった古いタイマーを無視できる。
    """

    def __init__(self, on_done: Callable[[int], None]):
        self._on_done = on_done
        self._timer: Optional[threading.Timer] = None
        self._duration_ms: int = 0
        self._start: float = 0.0
        self._generation: int = 0

    @property
    def generation(self) -> int:
        return self._generation

    def start(self, duration_ms: int) -> None:
        self.cancel()
        self._duration_ms = max(0, duration_ms)
        self._start = time.monotonic()
        self._timer = threading.Timer(
            self._duration_ms / 1000.0, self._on_done, args=(self._generation,))
        self._timer.daemon = True
        self._timer.start()

    def cancel(self) -> None:
        self._generation += 1
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    @property
    def remaining_ms(self) -> int:
        if self._timer is None:
            return 0
        elapsed_ms = (time.monotonic() - self._start) * 1000
        return max(0, int(self._duration_ms - elapsed_ms))


class PauseSlotManager:
    """Named Pause Slot を管理する。"""

    def __init__(self):
        self._slots: Set[str] = set()

    @property
    def is_active(self) -> bool:
        return bool(self._slots)

    @property
    def requesters(self) -> List[str]:
        return sorted(self._slots)

    def add(self, requester_id: str) -> None:
        self._slots.add(requester_id)

    def remove(self, requester_id: str) -> None:
        self._slots.discard(requester_id)

    def clear_all(self) -> None:
        self._slots.clear()


class WaypointSequencerFSM:
    """
    SequencerState の遷移と副作用を管理するFSM。

    - 各遷移中状態(ON_STARTING/ON_ARRIVING)への入場は専用エントリー関数経由のみ。
    - 処理完了後は内部コールバックで次の安定状態へ移行する。
    - 外部から _transition() を呼ばない。
    - IDLE は「未開始」と「途中ウェイポイントのトリガー待ち」を兼ねる（_current_index で区別）。
    - 通過点を通過 (PASSED) した後は Nav2 のゴールが走り続けている。すぐ次のウェイポイントへ
      向かう場合はそのゴールを上書きし、それ以外 (アクション実行・終了・トリガー待ち・一時停止)
      では navigator.cancel() で止める。
    - pause は一時停止であり、全スロット解除で中断した箇所から自動再開する。
      ON_ARRIVING 中の pause はアクション完了を待ち、次ウェイポイントへ進む手前で SUSPENDED になる。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._navigator = WaypointNavigator(node)
        self._executor = ActionExecutor(node)

        self._state = SequencerState.IDLE
        self._lock = threading.RLock()

        self._waypoints: WaypointList = WaypointList()
        self._current_index: int = 0

        self._stop_pending: bool = False
        self._pre_suspend_state: SequencerState = SequencerState.IDLE
        self._saved_countdown_ms: int = 0
        self._navigation_mode: str = "normal"
        self._navigation_mode_pub = node.create_publisher(
            String, "~/navigation_mode",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._publish_navigation_mode()
        # UI などから送った手動ゴールが実行中か (ウェイポイントの走行とは独立に扱う)
        self._manual_goal_active: bool = False

        self._countdown_timer = CountdownTimer(self._on_starting_done)
        self._pause_manager = PauseSlotManager()

    # ------------------------------------------------------------------
    # 状態参照
    # ------------------------------------------------------------------

    @property
    def state(self) -> SequencerState:
        return self._state

    @property
    def current_index(self) -> int:
        return self._current_index

    @property
    def total_waypoints(self) -> int:
        return self._waypoints.get_size()

    @property
    def countdown_ms_remaining(self) -> int:
        if self._state != SequencerState.ON_STARTING:
            return 0
        return self._countdown_timer.remaining_ms

    @property
    def pause_requesters(self) -> List[str]:
        return self._pause_manager.requesters

    @property
    def distance_remaining(self) -> float:
        return self._navigator.distance_remaining

    @property
    def map_loader(self):
        return self._executor.map_loader

    def _publish_navigation_mode(self) -> None:
        """次のゴールで使うナビゲーションモードと BT のファイル名を JSON で配信する。"""
        msg = String()
        msg.data = json.dumps({
            "mode": self._navigation_mode,
            "behavior_tree": os.path.basename(
                self._navigator.behavior_tree_for(self._navigation_mode)),
        })
        self._navigation_mode_pub.publish(msg)

    # ------------------------------------------------------------------
    # ウェイポイント設定
    # ------------------------------------------------------------------

    def load_waypoints(self, waypoints: WaypointList) -> None:
        with self._lock:
            self._waypoints = waypoints
            self._current_index = 0

    def set_next_index(self, index: int) -> bool:
        """IDLE / SUSPENDED 時のみインデックスを変更できる"""
        with self._lock:
            if self._state not in (SequencerState.IDLE, SequencerState.SUSPENDED):
                return False
            if index < 0 or index >= self._waypoints.get_size():
                return False
            self._current_index = index
            return True

    def reload_waypoints(self, waypoints: WaypointList) -> CommandResult:
        """IDLE / GOAL_REACHED / ERROR 時のみウェイポイントリストを差し替える"""
        with self._lock:
            allowed = {SequencerState.IDLE,
                       SequencerState.GOAL_REACHED, SequencerState.ERROR}
            if self._state not in allowed:
                return CommandResult(False, f"Cannot reload in state {self._state.value}")
            self._waypoints = waypoints
            size = waypoints.get_size()
            if size == 0:
                self._current_index = 0
            elif self._current_index >= size:
                self._current_index = size - 1
            return CommandResult(True, f"Reloaded {size} waypoints")

    # ------------------------------------------------------------------
    # 外部コマンド
    # ------------------------------------------------------------------

    def navigate_to_pose(self, pose, navigation_mode: str) -> CommandResult:
        """BT を指定して、シーケンスとは独立に 1 つのゴールを Nav2 に送る (手動ゴール)。

        シーケンスが走行中でないとき (IDLE / GOAL_REACHED / ERROR) で、pause 中でないときだけ受け付ける。
        手動ゴールの実行中は start() を拒否し、stop() でキャンセルする。

        normal / queue_wait は、set_navigation_mode アクションと同じく BT と global_costmap の
        センサ障害物層を切り替える (queue_wait で無効、normal で有効)。default は BT も層も変えない。
        戻すのは normal で送るか Nav2 の再起動で、ゴールの終了時には戻さない。
        """
        with self._lock:
            rejection = self._manual_goal_rejection(navigation_mode)
        if rejection is not None:
            return CommandResult(False, rejection)

        if navigation_mode != DEFAULT_BT_MODE:
            # サービスの応答待ち (最長数秒) の間も stop などを受けられるよう、ロックの外で呼ぶ
            ok, message = self._executor.costmap_switcher.set_global_obstacle_layers(
                navigation_mode != "queue_wait")
            if not ok:
                return CommandResult(False, message)

        with self._lock:
            # 層の切り替えの待ちの間に状態が変わっていたら、ゴールは送らない
            rejection = self._manual_goal_rejection(navigation_mode)
            if rejection is not None:
                return CommandResult(False, rejection)
            waypoint = Waypoint(
                index=-1,
                pose=pose,
                navigation=NavigationConfig(is_through_point=False),
            )
            self._manual_goal_active = True
            self._navigator.send_goal(
                waypoint,
                self._on_manual_goal_result,
                navigation_mode=navigation_mode,
            )
            return CommandResult(True, "OK")

    def _manual_goal_rejection(self, navigation_mode: str) -> Optional[str]:
        """ロック保持中に呼ぶ。手動ゴールを受け付けられない理由。受け付けられるなら None。"""
        if navigation_mode not in MANUAL_GOAL_MODES:
            return (f"Unknown navigation_mode {navigation_mode!r}; "
                    f"expected one of {MANUAL_GOAL_MODES}")
        if self._state not in MANUAL_GOAL_STATES:
            return (f"Cannot send a manual goal in state {self._state.value}; "
                    "stop the sequence first")
        if self._pause_manager.is_active:
            return f"Paused by {', '.join(self._pause_manager.requesters)}"
        return None

    def start(self, countdown_ms: int) -> CommandResult:
        with self._lock:
            if self._manual_goal_active:
                return CommandResult(
                    False, "A manual goal is running; stop it first")

            if self._pause_manager.is_active and self._state in (
                SequencerState.IDLE, SequencerState.GOAL_REACHED
            ):
                return CommandResult(
                    False,
                    f"Paused by {', '.join(self._pause_manager.requesters)}",
                )

            if self._state == SequencerState.IDLE:
                if self._waypoints.get_size() == 0:
                    return CommandResult(False, "No waypoints loaded")
                self._enter_on_starting(countdown_ms)
                return CommandResult(True, "OK")

            if self._state == SequencerState.GOAL_REACHED:
                self._current_index = 0
                self._enter_on_starting(countdown_ms)
                return CommandResult(True, "OK")

            return CommandResult(False, f"Cannot start from state {self._state.value}")

    def stop(self) -> CommandResult:
        with self._lock:
            if self._manual_goal_active:
                self._navigator.cancel()
                self._manual_goal_active = False
                if self._state == SequencerState.IDLE:
                    return CommandResult(True, "Manual goal canceled")

            if self._state == SequencerState.IDLE:
                return CommandResult(True, "Already idle")

            if self._state == SequencerState.ON_STARTING:
                self._countdown_timer.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            if self._state == SequencerState.NAVIGATING:
                self._navigator.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            if self._state == SequencerState.ON_ARRIVING:
                self._stop_pending = True
                return CommandResult(True, "Stop deferred until actions complete")

            if self._state in (
                SequencerState.SUSPENDED,
                SequencerState.GOAL_REACHED,
                SequencerState.ERROR,
            ):
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            return CommandResult(False, f"Unhandled state {self._state.value}")

    def pause_request(self, requester_id: str, active: bool) -> None:
        with self._lock:
            if active:
                self._pause_manager.add(requester_id)
                self._apply_pause()
            else:
                self._pause_manager.remove(requester_id)
                self._try_resume()

    # ------------------------------------------------------------------
    # エントリー関数（各状態への唯一の入口）
    # ------------------------------------------------------------------

    def _enter_on_starting(self, countdown_ms: int) -> None:
        self._transition(SequencerState.ON_STARTING)
        self._countdown_timer.start(countdown_ms)

    def _enter_navigating(self) -> None:
        waypoint = self._waypoints.get(self._current_index)
        self._transition(SequencerState.NAVIGATING)
        self._navigator.send_goal(
            waypoint,
            self._on_navigation_result,
            navigation_mode=self._navigation_mode
        )

    def _enter_on_arriving(self, actions) -> None:
        self._navigator.cancel()
        for action in actions:
            if action.type == "set_navigation_mode":
                self._navigation_mode = getattr(action, "mode", "normal")
                self._publish_navigation_mode()
        self._transition(SequencerState.ON_ARRIVING)
        self._executor.execute(actions, self._on_arriving_done)

    # ------------------------------------------------------------------
    # 内部完了コールバック
    # ------------------------------------------------------------------

    def _on_starting_done(self, generation: int) -> None:
        with self._lock:
            if (self._state != SequencerState.ON_STARTING
                    or generation != self._countdown_timer.generation):
                return
            self._enter_navigating()

    def _on_manual_goal_result(self, result: NavigationResult) -> None:
        with self._lock:
            self._manual_goal_active = False
        self._node.get_logger().info(f"Manual goal finished: {result.value}")

    def _on_navigation_result(self, result: NavigationResult) -> None:
        with self._lock:
            if self._state != SequencerState.NAVIGATING:
                return

            if result == NavigationResult.CANCELED:
                return

            if result not in (NavigationResult.SUCCEEDED, NavigationResult.PASSED):
                self._node.get_logger().error(
                    f"Navigation to waypoint {self._current_index} failed"
                )
                self._transition(SequencerState.ERROR)
                return

            waypoint = self._waypoints.get(self._current_index)
            if waypoint.on_reached_actions:
                self._enter_on_arriving(waypoint.on_reached_actions)
            else:
                self._advance_to_next()

    def _on_arriving_done(self) -> None:
        with self._lock:
            if self._state != SequencerState.ON_ARRIVING:
                return
            self._advance_to_next()

    # ------------------------------------------------------------------
    # ウェイポイント進行ロジック
    # ------------------------------------------------------------------

    def _advance_to_next(self) -> None:
        if self._stop_pending:
            self._stop_pending = False
            self._pause_manager.clear_all()
            self._navigator.cancel()
            self._transition(SequencerState.IDLE)
            return

        waypoint = self._waypoints.get(self._current_index)
        self._current_index += 1

        if self._current_index >= self._waypoints.get_size():
            self._navigator.cancel()
            self._transition(SequencerState.GOAL_REACHED)
            return

        if any(a.type == "wait_trigger" for a in waypoint.on_reached_actions):
            self._navigator.cancel()
            self._transition(SequencerState.IDLE)
            return

        if self._pause_manager.is_active:
            self._navigator.cancel()
            self._pre_suspend_state = SequencerState.NAVIGATING
            self._transition(SequencerState.SUSPENDED)
            return

        self._enter_navigating()

    # ------------------------------------------------------------------
    # Named Pause Slot 管理
    # ------------------------------------------------------------------

    def _apply_pause(self) -> None:
        if self._state == SequencerState.ON_STARTING:
            self._saved_countdown_ms = self._countdown_timer.remaining_ms
            self._countdown_timer.cancel()
            self._pre_suspend_state = SequencerState.ON_STARTING
            self._transition(SequencerState.SUSPENDED)
        elif self._state == SequencerState.NAVIGATING:
            self._navigator.cancel()
            self._pre_suspend_state = SequencerState.NAVIGATING
            self._transition(SequencerState.SUSPENDED)

    def _try_resume(self) -> None:
        if self._pause_manager.is_active or self._state != SequencerState.SUSPENDED:
            return
        if self._pre_suspend_state == SequencerState.ON_STARTING:
            self._enter_on_starting(self._saved_countdown_ms)
        else:
            self._enter_navigating()

    # ------------------------------------------------------------------
    # 状態遷移
    # ------------------------------------------------------------------

    def _transition(self, new_state: SequencerState) -> None:
        allowed = ALLOWED_TRANSITIONS.get(self._state, frozenset())
        if new_state not in allowed:
            self._node.get_logger().error(
                f"FSM: invalid transition {self._state.value} -> {new_state.value}"
            )
            return
        old = self._state
        self._state = new_state
        self._node.get_logger().info(f"FSM: {old.value} -> {new_state.value}")
