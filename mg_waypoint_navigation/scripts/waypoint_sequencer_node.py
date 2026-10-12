#!/usr/bin/env python3
import copy

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater
from std_msgs.msg import ColorRGBA, Int16
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mg_msgs.msg import NavigationActivity, PauseRequest, SequencerStatus
from mg_msgs.msg import WaypointInfo
from mg_msgs.msg import WaypointList as WaypointListMsg
from mg_msgs.srv import LoadMaps, SendGoal, StartSequence

from mg_utils.event_log import EventLog
from mg_waypoint_navigation.waypoint import WaypointList, WaypointsLoader
from mg_waypoint_navigation.waypoint_sequencer.activity import (
    ActivityInputs,
    ActivityTracker,
    compose_activity,
)
from mg_waypoint_navigation.waypoint_sequencer.activity_sources import ActivitySources
from mg_waypoint_navigation.waypoint_sequencer.fsm import WaypointSequencerFSM
from mg_waypoint_navigation.waypoint_sequencer.health import progress_health
from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState

_LEVELS = {'ok': DiagnosticStatus.OK, 'warn': DiagnosticStatus.WARN}


class ProgressTask(DiagnosticTask):
    """診断 `progress`: 走行中に位置が進んでいるか。"""

    def __init__(self, node: 'WaypointSequencerNode') -> None:
        super().__init__('progress')
        self._node = node

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        level, message, values = progress_health(
            self._node.fsm.navigation_progress(), self._node.stall_warn_sec)
        stat.summary(_LEVELS[level], message)
        for key, value in values.items():
            stat.add(key, value)
        return stat


class WaypointSequencerNode(Node):
    def __init__(self):
        super().__init__("waypoint_sequencer_node")

        self._declare_parameters()

        self._fsm = WaypointSequencerFSM(self)

        self._init_ros_communications()
        self._load_waypoints()

    # ------------------------------------------------------------------
    # パラメータ
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter("load_path", "")
        self.declare_parameter("publish_waypoint_status", True)
        self.declare_parameter("waypoint_status_freq_hz", 10.0)
        self.declare_parameter("publish_waypoints_list", True)

        self._load_path: str = self.get_parameter("load_path").value
        self._publish_status: bool = self.get_parameter(
            "publish_waypoint_status").value
        self._status_freq: float = self.get_parameter(
            "waypoint_status_freq_hz").value
        if self._status_freq <= 0.0:
            self.get_logger().warn(
                f"waypoint_status_freq_hz={self._status_freq} is invalid. Using default 10.0 Hz."
            )
            self._status_freq = 10.0
        self._publish_list: bool = self.get_parameter(
            "publish_waypoints_list").value
        # 走行中に位置が進まない時間がこれを超えたら、/diagnostics に WARN を出す [s]
        self._stall_warn_sec: float = self.declare_parameter(
            "stall_warn_sec", 30.0).value
        # 走行の「いまの活動」(~/activity) を配信するか。false なら購読もしない
        self._publish_activity: bool = self.declare_parameter(
            "publish_activity", True).value
        self._activity_freq: float = self.declare_parameter(
            "activity_freq_hz", 2.0).value
        if self._activity_freq <= 0.0:
            self.get_logger().warn(
                f"activity_freq_hz={self._activity_freq} is invalid. Using default 2.0 Hz."
            )
            self._activity_freq = 2.0
        # 空にすると、その入力は使わない
        self._activity_collision_topic: str = self.declare_parameter(
            "activity_collision_state_topic", "/collision_monitor_state").value
        self._activity_bt_log_topic: str = self.declare_parameter(
            "activity_bt_log_topic", "/behavior_tree_log").value
        self._activity_nav_status_topic: str = self.declare_parameter(
            "activity_nav_status_topic", "/navigate_to_pose/_action/status").value

    # ------------------------------------------------------------------
    # ROS 通信初期化
    # ------------------------------------------------------------------

    def _init_ros_communications(self):
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._start_srv = self.create_service(
            StartSequence, "~/start", self._on_start_srv
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._on_stop_srv)
        self._reload_srv = self.create_service(
            Trigger, "~/reload_waypoints", self._on_reload_waypoints_srv
        )
        self._navigate_to_pose_srv = self.create_service(
            SendGoal, "~/navigate_to_pose", self._on_navigate_to_pose_srv
        )
        # 地図の読み込みは最長 20 秒ほど待つので、他のサービスを止めないよう別のグループで受ける
        self._load_map_srv = self.create_service(
            LoadMaps, "~/load_map", self._on_load_map_srv,
            callback_group=ReentrantCallbackGroup()
        )

        self._set_index_sub = self.create_subscription(
            Int16, "~/set_next_waypoint_index", self._on_set_index, 1
        )
        self._pause_request_sub = self.create_subscription(
            PauseRequest, "~/pause_request", self._on_pause_request, 10
        )

        if self._publish_status:
            self._status_pub = self.create_publisher(
                SequencerStatus, "~/status", 10
            )
            self._status_timer = self.create_timer(
                1.0 / self._status_freq, self._publish_status_cb
            )

        if self._publish_activity:
            # 表示用の要約が他のコールバックを止めないよう、専用のグループで動かす
            activity_group = MutuallyExclusiveCallbackGroup()
            self._activity_sources = ActivitySources(
                self, activity_group,
                self._activity_collision_topic, self._activity_bt_log_topic,
                self._activity_nav_status_topic)
            self._activity_tracker = ActivityTracker()
            self._activity_pub = self.create_publisher(
                NavigationActivity, "~/activity", latched_qos)
            self._activity_timer = self.create_timer(
                1.0 / self._activity_freq, self._publish_activity_cb,
                callback_group=activity_group)

        if self._publish_list:
            self._waypoints_list_pub = self.create_publisher(
                WaypointListMsg, "~/waypoints", latched_qos
            )

        self._markers_pub = self.create_publisher(
            MarkerArray, "~/waypoints_markers", latched_qos
        )

        self._events = EventLog(self.get_logger())
        self._updater = Updater(self)
        self._updater.setHardwareID("waypoint_sequencer")
        self._updater.add(ProgressTask(self))
        self._events.info("node.started", name="waypoint_sequencer_node")

    @property
    def fsm(self) -> WaypointSequencerFSM:
        return self._fsm

    @property
    def stall_warn_sec(self) -> float:
        return self._stall_warn_sec

    # ------------------------------------------------------------------
    # ウェイポイントロード
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        if not self._load_path:
            self.get_logger().warn("load_path not set. No waypoints loaded.")
            return

        try:
            loader = WaypointsLoader(self._load_path)
            waypoints = loader.load()
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return

        self._fsm.load_waypoints(waypoints)
        self.get_logger().info(
            f"Loaded {waypoints.get_size()} waypoints from {self._load_path}"
        )
        self._log_waypoint_warnings(waypoints)

        if self._publish_list:
            self._publish_waypoints_list(waypoints)
        self._publish_markers(waypoints)

    def _log_waypoint_warnings(self, waypoints: WaypointList):
        for warning in waypoints.warnings:
            self.get_logger().warn(warning)

    # ------------------------------------------------------------------
    # サービスコールバック
    # ------------------------------------------------------------------

    def _on_start_srv(
        self, request: StartSequence.Request, response: StartSequence.Response
    ):
        result = self._fsm.start(request.countdown_ms)
        response.success = result.success
        response.message = result.message
        return response

    def _on_stop_srv(self, request: Trigger.Request, response: Trigger.Response):
        result = self._fsm.stop()
        response.success = result.success
        response.message = result.message
        return response

    def _on_reload_waypoints_srv(self, request: Trigger.Request, response: Trigger.Response):
        if not self._load_path:
            response.success = False
            response.message = "load_path is not set"
            return response
        try:
            loader = WaypointsLoader(self._load_path)
            waypoints = loader.load()
        except Exception as e:
            response.success = False
            response.message = f"Failed to load waypoints: {e}"
            return response
        result = self._fsm.reload_waypoints(waypoints)
        response.success = result.success
        response.message = result.message
        if result.success:
            self._log_waypoint_warnings(waypoints)
            if self._publish_list:
                self._publish_waypoints_list(waypoints)
            self._publish_markers(waypoints)
        return response

    # ------------------------------------------------------------------
    # トピックコールバック
    # ------------------------------------------------------------------

    def _on_navigate_to_pose_srv(self, request: SendGoal.Request, response: SendGoal.Response):
        result = self._fsm.navigate_to_pose(request.pose, request.navigation_mode)
        response.success = result.success
        response.message = result.message
        return response

    def _on_load_map_srv(self, request: LoadMaps.Request, response: LoadMaps.Response):
        response.success, response.message = self._fsm.map_loader.load(
            request.localization, request.planning)
        return response

    def _on_set_index(self, msg: Int16):
        ok = self._fsm.set_next_index(int(msg.data))
        if not ok:
            self.get_logger().warn(
                f"Cannot set index to {msg.data} in state {self._fsm.state.value}"
            )

    def _on_pause_request(self, msg: PauseRequest):
        self._fsm.pause_request(
            requester_id=msg.requester_id,
            active=msg.active,
        )

    # ------------------------------------------------------------------
    # パブリッシュ
    # ------------------------------------------------------------------

    def _publish_status_cb(self):
        msg = SequencerStatus()
        now = self.get_clock().now().to_msg()
        msg.stamp = now
        msg.state = self._fsm.state.value
        msg.current_index = self._fsm.current_index
        msg.total_waypoints = self._fsm.total_waypoints
        msg.countdown_ms_remaining = self._fsm.countdown_ms_remaining
        msg.is_paused = self._fsm.state == SequencerState.SUSPENDED
        msg.pause_requesters = self._fsm.pause_requesters
        msg.distance_remaining = self._fsm.distance_remaining
        msg.current_action = self._fsm.current_action
        msg.waiting_trigger = self._fsm.waiting_trigger
        self._status_pub.publish(msg)

    def _publish_activity_cb(self):
        fsm = self._fsm
        state = fsm.state
        external_goal_active = self._activity_sources.external_goal_active(
            fsm.has_live_nav_goal, self.get_clock().now().nanoseconds / 1e9)
        driving = (state == SequencerState.NAVIGATING or fsm.manual_goal_active
                   or external_goal_active)
        if not driving:
            self._activity_sources.clear_recovery()
        progress = fsm.navigation_progress()
        inputs = ActivityInputs(
            state=state,
            current_index=fsm.current_index,
            total_waypoints=fsm.total_waypoints,
            countdown_ms_remaining=fsm.countdown_ms_remaining,
            pause_requesters=tuple(fsm.pause_requesters),
            distance_remaining=fsm.distance_remaining,
            current_action=fsm.current_action,
            waiting_trigger=fsm.waiting_trigger,
            manual_goal_active=fsm.manual_goal_active,
            external_goal_active=external_goal_active,
            collision=self._activity_sources.collision,
            running_recovery_nodes=self._activity_sources.recovery_nodes,
            stalled_sec=0.0 if progress is None else progress.stalled_sec,
        )
        activity = compose_activity(inputs, self._stall_warn_sec)
        now = self.get_clock().now()

        msg = NavigationActivity()
        msg.stamp = now.to_msg()
        msg.code = activity.code
        msg.text = activity.text
        msg.detail = activity.detail
        msg.emphasis = activity.emphasis
        msg.elapsed_sec = self._activity_tracker.elapsed_sec(
            activity.code, now.nanoseconds / 1e9)
        msg.index = activity.index
        msg.total = activity.total
        msg.remaining = activity.remaining
        self._activity_pub.publish(msg)

    def _publish_waypoints_list(self, waypoints: WaypointList):
        msg = WaypointListMsg()
        msg.loaded_time = self.get_clock().now().to_msg()
        for wp in waypoints.get_all():
            info = WaypointInfo()
            info.index = wp.index
            info.pose = wp.pose
            info.reach_tolerance = wp.navigation.reach_tolerance
            info.is_through_point = wp.navigation.is_through_point
            info.on_reached_action_types = [
                a.type for a in wp.on_reached_actions
            ]
            msg.waypoints.append(info)
        self._waypoints_list_pub.publish(msg)

    def _publish_markers(self, waypoints: WaypointList):
        marker_array = MarkerArray()
        for wp in waypoints.get_all():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = wp.index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(wp.pose.pose)
            marker.pose.position.z = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            if wp.navigation.is_through_point:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoints_text"
            text_marker.id = wp.index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = copy.deepcopy(wp.pose.pose)
            text_marker.pose.position.z = 1.5
            text_marker.scale.z = 0.4
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = str(wp.index)
            marker_array.markers.append(text_marker)

        self._markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencerNode()
    # アクションスレッドのサービス応答や Nav2 の応答を、他のコールバック実行中にも処理できるようにする
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
