from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Callable, List, Optional

from mg_msgs.msg import SequencerStatus
from mg_msgs.msg import WaypointList as WaypointListMsg
from mg_msgs.srv import StartSequence
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.nav2 import call_service
from std_msgs.msg import Int16
from std_srvs.srv import Trigger

from mg_scenario_test.sequencer_tracker import Progress, SequencerProgressTracker

if TYPE_CHECKING:
    import rclpy.node

_POLL_SEC = 0.05


class SequencerClient:
    """waypoint_sequencer_node への操作と進捗監視をまとめたクライアント。"""

    def __init__(
        self,
        node: "rclpy.node.Node",
        namespace: str,
        idle_stall_sec: float = 10.0,
    ):
        self._node = node
        self._ns = namespace
        self._lock = threading.Lock()
        self._tracker = SequencerProgressTracker(idle_stall_sec)
        self._start_client = node.create_client(
            StartSequence, f"/{namespace}/start")
        self._stop_client = node.create_client(Trigger, f"/{namespace}/stop")
        self._index_pub = node.create_publisher(
            Int16, f"/{namespace}/set_next_waypoint_index", 1)
        self._status_sub = node.create_subscription(
            SequencerStatus, f"/{namespace}/status", self._on_status, 10)

    @property
    def namespace(self) -> str:
        return self._ns

    @property
    def state(self) -> Optional[str]:
        with self._lock:
            return self._tracker.state

    @property
    def index(self) -> Optional[int]:
        with self._lock:
            return self._tracker.index

    def _on_status(self, msg: SequencerStatus) -> None:
        with self._lock:
            self._tracker.update(msg.state, msg.current_index, time.monotonic())

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._start_client.wait_for_service(timeout_sec=timeout_sec)

    def begin_run(self) -> None:
        with self._lock:
            self._tracker.reset_run()

    def start(self, countdown_ms: int, timeout_sec: float = 10.0) -> None:
        req = StartSequence.Request()
        req.countdown_ms = countdown_ms
        res = call_service(self._node, self._start_client, req, timeout_sec)
        if not res.success:
            raise ScenarioError(f"/{self._ns}/start rejected: {res.message}")

    def stop(self, timeout_sec: float = 10.0) -> None:
        res = call_service(self._node, self._stop_client, Trigger.Request(), timeout_sec)
        if not res.success:
            raise ScenarioError(f"/{self._ns}/stop rejected: {res.message}")

    def set_next_index(self, index: int, timeout_sec: float = 5.0) -> None:
        """次の waypoint index を設定し、status に反映されたことを確認する。"""
        deadline = time.monotonic() + timeout_sec
        while self._index_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise ScenarioError(
                    f"/{self._ns}/set_next_waypoint_index has no subscriber")
            time.sleep(_POLL_SEC)

        msg = Int16()
        msg.data = index
        self._index_pub.publish(msg)

        while time.monotonic() <= deadline:
            with self._lock:
                if self._tracker.index == index:
                    return
            time.sleep(_POLL_SEC)
        with self._lock:
            state = self._tracker.state
        raise ScenarioError(
            f"set_next_waypoint_index({index}) was not applied "
            f"(sequencer state={state}; only IDLE/SUSPENDED accept it)")

    def wait_passed(
        self,
        expected_index: int,
        timeout_sec: float,
        abort_event: threading.Event,
        clock_now: Callable[[], float],
    ) -> Progress:
        """current_index が expected_index を超えるまで待つ。

        timeout_sec は clock_now (sim 時間) で測る。IDLE 停滞の判定は壁時計で行う。
        タイムアウトまたは abort_event による中断時は Progress.WAITING を返す。
        """
        started = time.monotonic()
        deadline = clock_now() + timeout_sec
        while True:
            now = time.monotonic()
            with self._lock:
                progress = self._tracker.evaluate(expected_index, started, now)
            if progress != Progress.WAITING:
                return progress
            if clock_now() > deadline or abort_event.is_set():
                return Progress.WAITING
            time.sleep(_POLL_SEC)

    def fetch_waypoints(self, timeout_sec: float = 5.0) -> Optional[List]:
        """latched な ~/waypoints トピックから WaypointInfo のリストを取得する。"""
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        received: List = [None]
        done = threading.Event()

        def cb(msg):
            received[0] = msg.waypoints
            done.set()

        sub = self._node.create_subscription(
            WaypointListMsg, f"/{self._ns}/waypoints", cb, latched_qos)
        try:
            done.wait(timeout=timeout_sec)
        finally:
            self._node.destroy_subscription(sub)
        if received[0] is None:
            return None
        return list(received[0])
