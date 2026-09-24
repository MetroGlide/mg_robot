"""走行中に常時評価する monitor。

monitor は run 開始前に start() され、run 終了後に stop() → result() で判定される。
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Literal, Optional

from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from sim_scenario_test.engine.result import CheckResult, ResultStatus
from sim_scenario_test.registry import register_monitor

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext

# diagnostic_msgs/DiagnosticStatus.ERROR
_DIAGNOSTIC_ERROR_LEVEL = 2


class TopicMonitor:
    """トピックを購読して判定する monitor の基底クラス。サブクラスで MSG_TYPE と _on_msg を定義する。

    QoS は best effort (reliable の配信元からも受信できる)。
    """
    MSG_TYPE = None

    def __init__(self, ctx: "ScenarioContext", topic: str):
        self._ctx = ctx
        self._topic = topic
        self._lock = threading.Lock()
        self._sub = None

    def start(self) -> None:
        self._sub = self._ctx.node.create_subscription(
            self.MSG_TYPE, self._topic, self._callback, qos_profile_sensor_data)

    def stop(self) -> None:
        if self._sub is not None:
            self._ctx.node.destroy_subscription(self._sub)
            self._sub = None

    def _callback(self, msg) -> None:
        with self._lock:
            self._on_msg(msg)

    def _on_msg(self, msg) -> None:
        raise NotImplementedError

    def result(self) -> CheckResult:
        with self._lock:
            return self._evaluate()

    def _evaluate(self) -> CheckResult:
        raise NotImplementedError


def occurrence_result(count: int, expect: str, what: str) -> CheckResult:
    """expect (occurs / never) と発生回数から判定する。"""
    if expect == "occurs":
        if count > 0:
            return CheckResult("", ResultStatus.PASSED, f"{what} occurred {count} time(s)")
        return CheckResult("", ResultStatus.FAILED, f"{what} never occurred")
    if count == 0:
        return CheckResult("", ResultStatus.PASSED, f"{what} never occurred")
    return CheckResult("", ResultStatus.FAILED, f"{what} occurred {count} time(s)")


@dataclass
class BtNodeSpec:
    node: str
    status: Literal["IDLE", "RUNNING", "SUCCESS", "FAILURE"] = "RUNNING"
    expect: Literal["occurs", "never"] = "occurs"
    topic: str = "/behavior_tree_log"


@register_monitor("bt_node", BtNodeSpec)
class BtNodeMonitor(TopicMonitor):
    """BT のノード node が status になった回数を数え、発生する／しないことを判定する。"""
    MSG_TYPE = BehaviorTreeLog

    def __init__(self, ctx: "ScenarioContext", spec: BtNodeSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._count = 0

    def _on_msg(self, msg) -> None:
        self._count += sum(
            1 for e in msg.event_log
            if e.node_name == self._spec.node and e.current_status == self._spec.status)

    def _evaluate(self) -> CheckResult:
        what = f"BT node '{self._spec.node}' -> {self._spec.status}"
        return occurrence_result(self._count, self._spec.expect, what)


@dataclass
class MinScanRangeSpec:
    # この距離 [m] 未満の測距があれば接触とみなして FAILED にする
    min_range: float
    topic: str = "/scan"


@register_monitor("min_scan_range", MinScanRangeSpec)
class MinScanRangeMonitor(TopicMonitor):
    """LiDAR の最小測距値が min_range 以上であることを監視する (衝突・急接近の近似検出)。"""
    MSG_TYPE = LaserScan

    def __init__(self, ctx: "ScenarioContext", spec: MinScanRangeSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._min: Optional[float] = None
        self._received = 0

    def _on_msg(self, msg) -> None:
        self._received += 1
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max]
        if valid:
            current = min(valid)
            self._min = current if self._min is None else min(self._min, current)

    def _evaluate(self) -> CheckResult:
        if self._received == 0:
            return CheckResult(
                "", ResultStatus.ERROR, f"no scan received on {self._spec.topic}")
        observed = "no valid range" if self._min is None else f"{self._min:.2f} m"
        status = (ResultStatus.FAILED
                  if self._min is not None and self._min < self._spec.min_range
                  else ResultStatus.PASSED)
        return CheckResult("", status, f"minimum range {observed} (limit {self._spec.min_range} m)")


@dataclass
class NoDiagnosticErrorsSpec:
    # 空の場合はすべての診断が対象
    names: List[str] = field(default_factory=list)
    topic: str = "/diagnostics"


@register_monitor("no_diagnostic_errors", NoDiagnosticErrorsSpec)
class NoDiagnosticErrorsMonitor(TopicMonitor):
    """/diagnostics に ERROR 以上のレベルが出ないことを監視する。"""
    MSG_TYPE = DiagnosticArray

    def __init__(self, ctx: "ScenarioContext", spec: NoDiagnosticErrorsSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._errors: dict = {}

    def _on_msg(self, msg) -> None:
        for status in msg.status:
            if status.level < _DIAGNOSTIC_ERROR_LEVEL:
                continue
            if self._spec.names and status.name not in self._spec.names:
                continue
            self._errors[status.name] = status.message

    def _evaluate(self) -> CheckResult:
        if not self._errors:
            return CheckResult("", ResultStatus.PASSED, "no diagnostic errors")
        detail = ", ".join(f"{n}: {m}" for n, m in sorted(self._errors.items()))
        return CheckResult("", ResultStatus.FAILED, f"diagnostic errors: {detail}")


_TOPIC_TYPES = {"String": String, "Bool": Bool, "OccupancyGrid": OccupancyGrid}


@dataclass
class TopicReceivedSpec:
    topic: str
    type: Literal["String", "Bool", "OccupancyGrid"] = "String"
    # String のとき、この文字列と一致する data を持つメッセージだけを数える (空なら全部)
    data: str = ""
    expect: Literal["occurs", "never"] = "occurs"
    min_count: int = 1


@register_monitor("topic_received", TopicReceivedSpec)
class TopicReceivedMonitor(TopicMonitor):
    """トピックのメッセージ (条件に合うもの) を min_count 回以上受信する／しないことを判定する。

    action の効果の確認 (publish を実行した、地図を再読み込みした等) に使う。
    """

    def __init__(self, ctx: "ScenarioContext", spec: TopicReceivedSpec):
        self.MSG_TYPE = _TOPIC_TYPES[spec.type]
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._count = 0

    def _on_msg(self, msg) -> None:
        if self._spec.data and getattr(msg, "data", None) != self._spec.data:
            return
        self._count += 1

    def _evaluate(self) -> CheckResult:
        what = f"message on {self._spec.topic}" + (
            f" (data '{self._spec.data}')" if self._spec.data else "")
        if self._spec.expect == "occurs" and self._count < self._spec.min_count:
            return CheckResult(
                "", ResultStatus.FAILED,
                f"{what}: received {self._count}, expected at least {self._spec.min_count}")
        return occurrence_result(self._count, self._spec.expect, what)


@dataclass
class MaxSpeedSpec:
    # 並進速度の上限 [m/s]
    limit: float
    topic: str = "/odom"


@register_monitor("max_speed", MaxSpeedSpec)
class MaxSpeedMonitor(TopicMonitor):
    """オドメトリの並進速度が limit を超えないことを監視する。"""
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: MaxSpeedSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._max = None

    def _on_msg(self, msg) -> None:
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        self._max = speed if self._max is None else max(self._max, speed)

    def _evaluate(self) -> CheckResult:
        if self._max is None:
            return CheckResult("", ResultStatus.ERROR, f"no odometry on {self._spec.topic}")
        status = ResultStatus.FAILED if self._max > self._spec.limit else ResultStatus.PASSED
        return CheckResult(
            "", status, f"max speed {self._max:.2f} m/s (limit {self._spec.limit} m/s)")


@dataclass
class MaxStopDurationSpec:
    # 区間内でこの秒数 (sim 時間) を超えて速度が speed_threshold 未満のままなら FAILED
    # (省略時は上限なし)
    max_stop_sec: Optional[float] = None
    # 区間内で、少なくともこの秒数の連続した停止があること (一時停止の確認用。0 なら確認しない)
    min_stop_sec: float = 0.0
    speed_threshold: float = 0.05
    # 監視区間: from_goal_started 番目のゴールの開始から、until_goal_reached 番目のゴールの到達まで
    from_goal_started: int = 0
    until_goal_reached: int = 0
    topic: str = "/odom"


@register_monitor("max_stop_duration", MaxStopDurationSpec)
class MaxStopDurationMonitor(TopicMonitor):
    """指定した区間の停止時間を監視する (通過点で止まらないこと、一時停止で止まること)。"""
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: MaxStopDurationSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._stop_since: Optional[float] = None
        self._longest = 0.0
        self._observed = False

    def _in_window(self) -> bool:
        events = self._ctx.events
        return (events.find("goal_started", index=self._spec.from_goal_started) is not None
                and events.find("goal_reached", index=self._spec.until_goal_reached) is None)

    def _on_msg(self, msg) -> None:
        if not self._in_window():
            self._stop_since = None
            return
        self._observed = True
        v = msg.twist.twist.linear
        now = self._ctx.clock.now()
        if math.hypot(v.x, v.y) < self._spec.speed_threshold:
            if self._stop_since is None:
                self._stop_since = now
            self._longest = max(self._longest, now - self._stop_since)
        else:
            self._stop_since = None

    def _evaluate(self) -> CheckResult:
        if not self._observed:
            return CheckResult("", ResultStatus.ERROR, "the monitored window was never entered")
        too_long = (self._spec.max_stop_sec is not None
                    and self._longest > self._spec.max_stop_sec)
        too_short = self._longest < self._spec.min_stop_sec
        status = ResultStatus.FAILED if too_long or too_short else ResultStatus.PASSED
        return CheckResult(
            "", status,
            f"longest stop {self._longest:.1f} s "
            f"(max {self._spec.max_stop_sec}, min {self._spec.min_stop_sec})")
