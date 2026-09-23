"""Nav2 NavigateToPose アクションクライアントのラッパー"""
from __future__ import annotations

import enum
import threading
from math import sqrt
from typing import Callable, Optional

import rclpy.node
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from mg_waypoint_navigation.waypoint import Waypoint


class NavigationResult(enum.Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class WaypointNavigator:
    """Nav2 NavigateToPose を呼び出す非同期ラッパー。

    ゴールごとに世代 ID を振り、cancel() 後や次ゴール送信後に届いた
    古いゴールの応答・フィードバック・結果は無視する。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            node, NavigateToPose, "navigate_to_pose",
            callback_group=self._callback_group)

        self._lock = threading.Lock()
        self._goal_id: int = 0
        self._active: bool = False
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._result_callback: Optional[Callable[[
            NavigationResult], None]] = None
        self._distance_remaining: float = 0.0
        self._through_tolerance: Optional[float] = None
        self._through_cancel: bool = False
        self._path_computed: bool = False
        self._waypoint: Optional[Waypoint] = None

        share_dir = get_package_share_directory("mg_waypoint_navigation")
        self._bt_xml_normal = node.declare_parameter(
            "bt_xml_normal",
            share_dir + "/behavior_trees/mg_navigate_to_pose.xml"
        ).value
        self._bt_xml_queue_wait = node.declare_parameter(
            "bt_xml_queue_wait",
            share_dir + "/behavior_trees/mg_navigate_to_pose_queue_wait.xml"
        ).value
        goal_checker_service = node.declare_parameter(
            "goal_checker_set_parameters_service",
            "/controller_server/set_parameters"
        ).value
        self._goal_checker_xy_param = node.declare_parameter(
            "goal_checker_xy_tolerance_param",
            "general_goal_checker.xy_goal_tolerance"
        ).value
        self._set_params_client = node.create_client(
            SetParameters, goal_checker_service,
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
        with self._lock:
            self._goal_id += 1
            goal_id = self._goal_id
            self._active = True
            self._goal_handle = None
            self._result_callback = result_callback
            self._through_tolerance = (
                waypoint.navigation.through_tolerance
                if waypoint.navigation.is_through_point
                else None
            )
            self._through_cancel = False
            self._path_computed = False
            self._waypoint = waypoint

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

        self._apply_reach_tolerance(
            waypoint.navigation.reach_tolerance,
            lambda: self._send_goal_async(goal_id, goal),
        )

    def cancel(self) -> None:
        """現在のゴールをキャンセルする。受理前なら受理直後にキャンセルする。"""
        with self._lock:
            self._active = False
            handle = self._goal_handle
            self._goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # 内部処理
    # ------------------------------------------------------------------

    def _is_current(self, goal_id: int) -> bool:
        return self._active and goal_id == self._goal_id

    def _finish(self, goal_id: int, result: NavigationResult) -> None:
        with self._lock:
            if not self._is_current(goal_id):
                return
            self._active = False
            self._goal_handle = None
            callback = self._result_callback
        if callback:
            callback(result)

    def _apply_reach_tolerance(
        self, tolerance: float, on_done: Callable[[], None]
    ) -> None:
        """goal_checker の xy_goal_tolerance を設定してから on_done を呼ぶ。失敗しても続行する。"""
        if not self._set_params_client.service_is_ready():
            self._node.get_logger().warn(
                "goal_checker parameter service not available. "
                f"reach_tolerance={tolerance} is not applied."
            )
            on_done()
            return

        param = Parameter()
        param.name = self._goal_checker_xy_param
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(tolerance)
        request = SetParameters.Request()
        request.parameters = [param]

        def _done(future) -> None:
            response = future.result()
            if response is None or not all(
                    r.successful for r in response.results):
                self._node.get_logger().warn(
                    f"Failed to set {self._goal_checker_xy_param}={tolerance}"
                )
            on_done()

        self._set_params_client.call_async(request).add_done_callback(_done)

    def _send_goal_async(self, goal_id: int, goal) -> None:
        with self._lock:
            if not self._is_current(goal_id):
                return
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self._feedback_callback(
                goal_id, msg),
        )
        future.add_done_callback(
            lambda f: self._goal_response_callback(goal_id, f))

    def _goal_response_callback(self, goal_id: int, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn("NavigateToPose goal rejected")
            self._finish(goal_id, NavigationResult.FAILED)
            return

        with self._lock:
            current = self._is_current(goal_id)
            if current:
                self._goal_handle = handle
        if not current:
            self._node.get_logger().info(
                "Canceling stale NavigateToPose goal accepted after cancel")
            handle.cancel_goal_async()
            return

        handle.get_result_async().add_done_callback(
            lambda f: self._result_done_callback(goal_id, f))

    def _result_done_callback(self, goal_id: int, future) -> None:
        status = future.result().status
        with self._lock:
            through_cancel = self._through_cancel
        if through_cancel:
            result = NavigationResult.SUCCEEDED
        elif status == GoalStatus.STATUS_SUCCEEDED:
            result = NavigationResult.SUCCEEDED
        elif status == GoalStatus.STATUS_CANCELED:
            result = NavigationResult.CANCELED
        else:
            result = NavigationResult.FAILED
        self._finish(goal_id, result)

    def _check_actual_arrival_through_tolerance(self, current_pose) -> bool:
        if self._waypoint is None or self._through_tolerance is None:
            return False
        dx = self._waypoint.pose.pose.position.x - current_pose.position.x
        dy = self._waypoint.pose.pose.position.y - current_pose.position.y
        return sqrt(dx * dx + dy * dy) <= self._through_tolerance

    def _feedback_callback(self, goal_id: int, feedback_msg) -> None:
        with self._lock:
            if not self._is_current(goal_id):
                return
            old_distance_remaining = self._distance_remaining
            self._distance_remaining = feedback_msg.feedback.distance_remaining

            if self._through_tolerance is None or self._through_cancel:
                return

            if not self._path_computed:
                if (self._distance_remaining > 0.0
                        and self._distance_remaining != old_distance_remaining):
                    self._node.get_logger().info(
                        "Path computed. Distance to goal: "
                        f"{self._distance_remaining:.2f} m"
                    )
                    self._path_computed = True
                return

            if not (self._distance_remaining <= self._through_tolerance
                    and self._check_actual_arrival_through_tolerance(
                        feedback_msg.feedback.current_pose.pose)):
                return

            self._node.get_logger().info(
                f"Within through tolerance ({self._through_tolerance} m). "
                "Canceling goal to proceed to next waypoint."
            )
            self._through_cancel = True
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()
