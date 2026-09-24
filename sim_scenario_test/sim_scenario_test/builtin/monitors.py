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
from sim_scenario_test.errors import ScenarioError, ScenarioValidationError
from sim_scenario_test.geometry import (
    Point,
    Pose,
    PoseSpec,
    polygon_circle_distance,
    polygon_polygon_distance,
    rectangle_vertices,
    transform_polygon,
)
from sim_scenario_test.registry import register_monitor

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext

# diagnostic_msgs/DiagnosticStatus.ERROR
_DIAGNOSTIC_ERROR_LEVEL = 2
# ロボットの動きで距離が縮まったとみなす最小の変化量 [m] (姿勢の測定ノイズを除く)
_CLOSING_EPSILON = 1e-4


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
class ObstacleClearanceSpec:
    # 監視する障害物 (scenario.obstacles のキー)
    obstacles: List[str]
    # ロボットの動きで、フットプリントと障害物の表面の距離がこの値以下になったら FAILED にする [m]。
    # 0 なら接触 (重なり) だけを検出する
    min_clearance: float = 0.0
    # ロボットが動いているとみなす並進速度 [m/s] と角速度 [rad/s] (どちらかを超えたときだけ評価する)。
    # 停止直前の低速や、停止中の自己位置推定の揺れ (数 cm) を、ロボットが原因の接近から除く
    speed_threshold: float = 0.1
    angular_threshold: float = 0.2
    # fuel / local モデルなど形状を持たない障害物を、この半径の円として扱う [m]
    radius: Optional[float] = None
    # 姿勢を評価するタイミングの元になるトピック (受信するたびに評価する)
    topic: str = "/odom"

    def validate(self, where: str) -> None:
        if not self.obstacles:
            raise ScenarioValidationError(f"{where}: 'obstacles' must not be empty")


def _obstacle_shape(model, radius: Optional[float], name: str):
    """障害物の形状を (半径, None) の円または (None, 矩形の寸法) として返す。"""
    if model.type == "primitive" and model.shape in ("cylinder", "sphere"):
        return model.size.get("radius", 0.5), None
    if model.type == "primitive" and model.shape == "box":
        return None, (model.size.get("x", 1.0), model.size.get("y", 1.0))
    if radius is None:
        raise ScenarioValidationError(
            f"obstacle_clearance: obstacle '{name}' has no known shape; set 'radius'")
    return radius, None


@register_monitor("obstacle_clearance", ObstacleClearanceSpec)
class ObstacleClearanceMonitor(TopicMonitor):
    """ロボットの動きが原因で、フットプリントが障害物へ近づきすぎないことを監視する。

    set_pose で動かす障害物 (歩行者など) は物理的に押し返されず、ロボットを通り抜けられる。
    障害物が勝手に近づいた場合を除くため、前回の評価から今回までの動きを 2 つに分け、
    ロボットだけが動いた場合の距離 (前回の障害物の位置 × 今回のロボットの姿勢) が、前回より
    縮まって min_clearance を下回ったときだけを、ロボットが原因の接近として数える。
    ロボットが走行・旋回 (speed_threshold / angular_threshold 超) で障害物へ寄った場合が対象で、
    停止中のロボットへ障害物が寄る場合や、ロボットが離れていく場合は対象外。
    全期間の最小距離は参考値として出す。
    ロボットの姿勢は TF (推定値) なので、数 cm の誤差を含む。
    """
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: ObstacleClearanceSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        footprint = ctx.profile.robot.footprint
        if not footprint:
            raise ScenarioValidationError(
                "obstacle_clearance: profile.robot.footprint is not defined")
        self._footprint: List[Point] = [(x, y) for x, y in footprint]
        self._shapes = {}
        for name in spec.obstacles:
            if name not in ctx.scenario.obstacles:
                raise ScenarioValidationError(
                    f"obstacle_clearance: obstacle '{name}' is not defined")
            self._shapes[name] = _obstacle_shape(
                ctx.scenario.obstacles[name].model, spec.radius, name)
        self._received = 0
        self._observed = False
        self._min_all: Optional[float] = None
        # ロボットの動きが原因の最小距離 (障害物名、走行開始からの経過時間 [s]、
        # ロボットの並進速度 [m/s]、ロボット座標での障害物の位置 (x, y) [m]: 原因の切り分け用)
        self._min_caused: Optional[float] = None
        self._min_caused_info = ("", 0.0, 0.0, (0.0, 0.0))
        self._previous = {}

    def _obstacle_pose(self, name: str) -> Optional[Pose]:
        world_pose = self._ctx.entity_poses.get(name)
        if world_pose is None:
            return None
        return self._ctx.poses.to_map(PoseSpec(
            "world", world_pose.x, world_pose.y, world_pose.z, world_pose.yaw))

    def _distance(self, name: str, obstacle: Pose, robot: Pose) -> float:
        robot_polygon = transform_polygon(self._footprint, robot)
        radius, box = self._shapes[name]
        if box is not None:
            return polygon_polygon_distance(
                robot_polygon, transform_polygon(rectangle_vertices(*box), obstacle))
        return polygon_circle_distance(robot_polygon, (obstacle.x, obstacle.y), radius)

    def _on_msg(self, msg) -> None:
        self._received += 1
        try:
            robot = self._ctx.poses.robot.get()
        except ScenarioError:
            return
        twist = msg.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        moving = (speed > self._spec.speed_threshold
                  or abs(twist.angular.z) > self._spec.angular_threshold)
        for name in self._spec.obstacles:
            obstacle = self._obstacle_pose(name)
            if obstacle is None:
                self._previous.pop(name, None)
                continue
            self._observed = True
            distance = self._distance(name, obstacle, robot)
            self._min_all = distance if self._min_all is None else min(self._min_all, distance)
            previous = self._previous.get(name)
            self._previous[name] = (robot, obstacle, distance)
            if previous is None:
                continue
            previous_robot, previous_obstacle, previous_distance = previous
            caused = self._distance(name, previous_obstacle, robot)
            if not moving or caused >= previous_distance - _CLOSING_EPSILON:
                continue
            if self._min_caused is None or caused < self._min_caused:
                relative = robot.inverse().compose(previous_obstacle)
                self._min_caused = caused
                self._min_caused_info = (
                    name, self._ctx.elapsed(), speed, (relative.x, relative.y))

    def _evaluate(self) -> CheckResult:
        if self._received == 0:
            return CheckResult(
                "", ResultStatus.ERROR, f"no odometry on {self._spec.topic}")
        if not self._observed:
            return CheckResult(
                "", ResultStatus.ERROR,
                f"obstacle(s) {self._spec.obstacles} were never observed")
        reference = f"overall min {self._min_all:.2f} m"
        if self._min_caused is None:
            return CheckResult(
                "", ResultStatus.PASSED, f"robot never approached obstacles ({reference})")
        name, at, speed, (x, y) = self._min_caused_info
        status = (ResultStatus.FAILED
                  if self._min_caused <= self._spec.min_clearance else ResultStatus.PASSED)
        return CheckResult(
            "", status,
            f"min clearance by robot motion {self._min_caused:.2f} m "
            f"(limit {self._spec.min_clearance} m, '{name}' at t={at:.1f}s, "
            f"robot speed {speed:.2f} m/s, position in robot frame ({x:.2f}, {y:.2f}); "
            f"{reference})")


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
