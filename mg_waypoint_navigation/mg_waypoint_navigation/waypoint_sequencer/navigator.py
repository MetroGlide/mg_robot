"""Nav2 NavigateToPose アクションクライアントのラッパー"""
from __future__ import annotations

import enum
import threading
from math import hypot
from typing import Callable, List, Optional

import rclpy.node
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from mg_waypoint_navigation.waypoint import Waypoint

# 経路上の最近傍点を探す範囲 [m]。前回の最近傍点から経路に沿ってこの長さだけ先まで探す。
_PLAN_SEARCH_WINDOW_M = 2.0


class NavigationResult(enum.Enum):
    SUCCEEDED = "SUCCEEDED"
    PASSED = "PASSED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class _LiveState(enum.Enum):
    NONE = "NONE"
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"


class WaypointNavigator:
    """Nav2 NavigateToPose を呼び出す非同期ラッパー。

    2 種類のゴールを区別して管理する。
    - 論理ゴール: FSM から見た現在のゴール。send_goal() ごとに世代 ID を振り、
      cancel() 後や次の send_goal() 後に届いた古い応答・結果は無視する。
    - Nav2 ゴール: Nav2 上で実際に走っているゴール。通過点の通過 (PASSED) 後も走り続け、
      次の send_goal() で上書き (preemption) されるか、cancel() で止められる。
      BT が異なる場合やキャンセル中は、終了を待ってから次のゴールを送る。

    通過点の判定は、送信後に受信した /plan のうち終点がゴールと一致するものだけを使い、
    その経路上の残り距離とゴールまでの直線距離が through_tolerance 以下になったら通過とする。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            node, NavigateToPose, "navigate_to_pose",
            callback_group=self._callback_group)

        self._lock = threading.Lock()

        # 論理ゴール
        self._goal_id: int = 0
        self._active: bool = False
        self._result_callback: Optional[Callable[[
            NavigationResult], None]] = None
        self._waypoint: Optional[Waypoint] = None
        self._through_tolerance: Optional[float] = None
        self._send_time_ns: int = 0
        self._distance_remaining: float = 0.0
        self._plan_points: List[tuple] = []
        self._plan_suffix: List[float] = []
        self._plan_index: int = 0

        # Nav2 ゴール
        self._live_seq: int = 0
        self._live_state = _LiveState.NONE
        self._live_handle: Optional[ClientGoalHandle] = None
        self._live_bt: str = ""
        self._live_canceling: bool = False
        self._on_live_idle: Optional[Callable[[], None]] = None

        share_dir = get_package_share_directory("mg_waypoint_navigation")
        self._bt_xml_normal = node.declare_parameter(
            "bt_xml_normal",
            share_dir + "/behavior_trees/mg_navigate_to_pose.xml"
        ).value
        self._bt_xml_queue_wait = node.declare_parameter(
            "bt_xml_queue_wait",
            share_dir + "/behavior_trees/mg_navigate_to_pose_queue_wait.xml"
        ).value
        plan_topic = node.declare_parameter("plan_topic", "/plan").value
        self._plan_goal_match_tolerance = node.declare_parameter(
            "plan_goal_match_tolerance", 0.6).value

        self._plan_sub = node.create_subscription(
            Path, plan_topic, self._plan_callback, 1,
            callback_group=self._callback_group)

    @property
    def distance_remaining(self) -> float:
        return self._distance_remaining

    def send_goal(
        self,
        waypoint: Waypoint,
        result_callback: Callable[[NavigationResult], None],
        navigation_mode: str = "normal",
    ) -> None:
        """ゴールを送る。Nav2 上で同じ BT のゴールが走っていれば上書きする。"""
        with self._lock:
            self._goal_id += 1
            goal_id = self._goal_id
            self._active = True
            self._result_callback = result_callback
            self._waypoint = waypoint
            self._through_tolerance = (
                waypoint.navigation.through_tolerance
                if waypoint.navigation.is_through_point
                else None
            )
            self._send_time_ns = self._node.get_clock().now().nanoseconds
            self._plan_points = []
            self._plan_suffix = []
            self._plan_index = 0

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                "navigate_to_pose action server not available")
            self._finish(goal_id, NavigationResult.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = waypoint.pose
        goal.behavior_tree = (
            self._bt_xml_queue_wait
            if navigation_mode == "queue_wait"
            else self._bt_xml_normal
        )

        self._dispatch(goal_id, goal)

    def cancel(self) -> None:
        """論理ゴールを終了し、Nav2 上のゴールを止める。受理前なら受理直後にキャンセルする。"""
        with self._lock:
            self._active = False
            self._on_live_idle = None
            handle = self._begin_cancel_live()
        if handle is not None:
            handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # 論理ゴール
    # ------------------------------------------------------------------

    def _is_current(self, goal_id: int) -> bool:
        return self._active and goal_id == self._goal_id

    def _finish(self, goal_id: int, result: NavigationResult) -> None:
        with self._lock:
            if not self._is_current(goal_id):
                return
            self._active = False
            callback = self._result_callback
        if callback:
            callback(result)

    # ------------------------------------------------------------------
    # Nav2 ゴール
    # ------------------------------------------------------------------

    def _dispatch(self, goal_id: int, goal) -> None:
        """Nav2 にゴールを送る。上書きできない状態なら、今のゴールの終了後に送る。"""
        cancel_handle = None
        with self._lock:
            if not self._is_current(goal_id):
                return
            can_send = (
                self._live_state == _LiveState.NONE
                or (not self._live_canceling
                    and self._live_bt == goal.behavior_tree)
            )
            if can_send:
                self._live_seq += 1
                seq = self._live_seq
                self._live_state = _LiveState.PENDING
                self._live_handle = None
                self._live_bt = goal.behavior_tree
                self._live_canceling = False
            else:
                self._on_live_idle = lambda: self._dispatch(goal_id, goal)
                cancel_handle = self._begin_cancel_live()

        if not can_send:
            if cancel_handle is not None:
                cancel_handle.cancel_goal_async()
            return

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self._feedback_callback(
                goal_id, msg),
        )
        future.add_done_callback(
            lambda f: self._goal_response_callback(goal_id, seq, f))

    def _begin_cancel_live(self) -> Optional[ClientGoalHandle]:
        """ロック保持中に呼ぶ。キャンセル要求を送るべきハンドルを返す。"""
        if self._live_state == _LiveState.NONE or self._live_canceling:
            return None
        self._live_canceling = True
        if self._live_state == _LiveState.ACTIVE:
            return self._live_handle
        return None

    def _mark_live_done(self, seq: int) -> Optional[Callable[[], None]]:
        """ロック保持中に呼ぶ。最新の Nav2 ゴールが終了したら、待機中の送信処理を返す。"""
        if seq != self._live_seq:
            return None
        self._live_state = _LiveState.NONE
        self._live_handle = None
        self._live_canceling = False
        on_idle = self._on_live_idle
        self._on_live_idle = None
        return on_idle

    def _goal_response_callback(self, goal_id: int, seq: int, future) -> None:
        handle = future.result()
        if not handle.accepted:
            with self._lock:
                on_idle = self._mark_live_done(seq)
            self._node.get_logger().warn("NavigateToPose goal rejected")
            self._finish(goal_id, NavigationResult.FAILED)
            if on_idle:
                on_idle()
            return

        with self._lock:
            if seq != self._live_seq:
                # 受理前に次のゴールで上書き済み。Nav2 側で ABORTED になる。
                return
            self._live_state = _LiveState.ACTIVE
            self._live_handle = handle
            cancel_now = self._live_canceling
        if cancel_now:
            self._node.get_logger().info(
                "Canceling NavigateToPose goal accepted after cancel request")
            handle.cancel_goal_async()

        handle.get_result_async().add_done_callback(
            lambda f: self._result_done_callback(goal_id, seq, f))

    def _result_done_callback(self, goal_id: int, seq: int, future) -> None:
        status = future.result().status
        with self._lock:
            on_idle = self._mark_live_done(seq)
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = NavigationResult.SUCCEEDED
        elif status == GoalStatus.STATUS_CANCELED:
            result = NavigationResult.CANCELED
        else:
            result = NavigationResult.FAILED
        self._finish(goal_id, result)
        if on_idle:
            on_idle()

    # ------------------------------------------------------------------
    # 通過点判定
    # ------------------------------------------------------------------

    def _plan_callback(self, msg: Path) -> None:
        with self._lock:
            if (not self._active or self._through_tolerance is None
                    or self._waypoint is None or not msg.poses):
                return
            stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                        + msg.header.stamp.nanosec)
            if stamp_ns != 0 and stamp_ns < self._send_time_ns:
                return
            goal = self._waypoint.pose.pose.position
            end = msg.poses[-1].pose.position
            if hypot(end.x - goal.x, end.y - goal.y) > self._plan_goal_match_tolerance:
                return

            points = [(p.pose.position.x, p.pose.position.y)
                      for p in msg.poses]
            suffix = [0.0] * len(points)
            for i in range(len(points) - 2, -1, -1):
                suffix[i] = suffix[i + 1] + hypot(
                    points[i + 1][0] - points[i][0],
                    points[i + 1][1] - points[i][1])
            self._plan_points = points
            self._plan_suffix = suffix
            self._plan_index = 0

    def _remaining_plan_length(self, x: float, y: float) -> float:
        """ロック保持中に呼ぶ。採用済み経路に沿ったゴールまでの残り距離。"""
        start = self._plan_index
        best = start
        best_dist = float("inf")
        for i in range(start, len(self._plan_points)):
            if self._plan_suffix[start] - self._plan_suffix[i] > _PLAN_SEARCH_WINDOW_M:
                break
            px, py = self._plan_points[i]
            d = hypot(px - x, py - y)
            if d < best_dist:
                best, best_dist = i, d
        self._plan_index = best
        return best_dist + self._plan_suffix[best]

    def _feedback_callback(self, goal_id: int, feedback_msg) -> None:
        with self._lock:
            if not self._is_current(goal_id):
                return
            self._distance_remaining = feedback_msg.feedback.distance_remaining

            tolerance = self._through_tolerance
            if tolerance is None or not self._plan_points:
                return
            current = feedback_msg.feedback.current_pose.pose.position
            remaining = self._remaining_plan_length(current.x, current.y)
            goal = self._waypoint.pose.pose.position
            if (remaining > tolerance
                    or hypot(goal.x - current.x, goal.y - current.y) > tolerance):
                return
            index = self._waypoint.index

        self._node.get_logger().info(
            f"Passed through waypoint {index} (tolerance {tolerance} m)")
        self._finish(goal_id, NavigationResult.PASSED)
