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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

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
    topic: str = "/scan_front_lidar"


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
