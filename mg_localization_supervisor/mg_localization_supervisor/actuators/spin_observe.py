"""観測を増やす指示 (observe) が出たら、Nav2 の behavior_server の Spin で、その場で首を振る。

止まったままでは、AMCL も GNSS も更新されず、スキャンも同じなので、復旧の候補を選べない・確かめられない。
回転だけで並進しないので、進入禁止の領域にかかりにくい。走行を止めてほしい状態 (hold_requested) の間だけ動かす
(シーケンサは一時停止でゴールを取り消しているので、BT の Spin とは競合しない)。
衝突の確認は behavior_server の局所コストマップの予測 (simulate_ahead_time) で、collision_monitor は通らない。

1 回の指示で、angle_rad だけ回り、逆向きに 2 倍回り、元の向きに戻る (例: ±90°)。途中の Spin が失敗
(衝突の予測で中止など) しても、次の向きを試す。止めてほしい状態でなくなったら、取り消す。
"""
import math
from typing import Any, List, Mapping, Optional

from builtin_interfaces.msg import Duration
from localization_supervisor.core.interfaces import ParamSpec
from localization_supervisor.core.supervisor import StepResult
from localization_supervisor.ros.actuators import Actuator
from nav2_msgs.action import Spin
from rclpy.action import ActionClient


class SpinObserve(Actuator):
    PARAMS = {
        'action': ParamSpec(str, '/spin'),
        'angle_rad': ParamSpec(float, math.pi / 2.0),
        'time_allowance_sec': ParamSpec(float, 15.0, '1 回の Spin の時間の上限'),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any) -> None:
        super().__init__(name, params, node)
        self._client = ActionClient(node, Spin, self.params['action'])
        self._queue: List[float] = []
        self._goal: Optional[Any] = None
        self._sending = False

    def apply(self, result: StepResult) -> None:
        if not result.hold_requested:
            self._stop()
            return
        if any(d.kind == 'observe' for d in result.directives) and not self._busy():
            angle = self.params['angle_rad']
            self._queue = [angle, -2.0 * angle, angle]
            self._send_next()

    def _busy(self) -> bool:
        return self._sending or self._goal is not None or bool(self._queue)

    def _send_next(self) -> None:
        if not self._queue:
            return
        if not self._client.server_is_ready():
            self.node.get_logger().warning(f"action {self.params['action']} is not available")
            self._queue = []
            return
        goal = Spin.Goal()
        goal.target_yaw = float(self._queue.pop(0))
        allowance = self.params['time_allowance_sec']
        goal.time_allowance = Duration(sec=int(allowance), nanosec=int((allowance % 1.0) * 1e9))
        self._sending = True
        self._client.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future) -> None:
        self._sending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._send_next()
            return
        self._goal = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future) -> None:
        self._goal = None
        self._send_next()

    def _stop(self) -> None:
        self._queue = []
        if self._goal is not None:
            self._goal.cancel_goal_async()
            self._goal = None
