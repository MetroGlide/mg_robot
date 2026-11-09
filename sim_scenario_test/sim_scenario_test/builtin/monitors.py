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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from sim_scenario_test.engine.result import CheckResult, ResultStatus
from sim_scenario_test.errors import ScenarioError, ScenarioValidationError
from sim_scenario_test.geometry import (
    Point,
    Pose,
    PoseSpec,
    normalize_angle,
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
    # 全期間の最小距離 (障害物が動いた分を含む) がこの値 [m] 以内まで近づいたことを要求する。
    # 障害物に出会わないまま PASS する (タイミングのずれ、速度の変更など) のを検出する。省略時は要求しない
    require_within: Optional[float] = None
    # ロボットの姿勢の取り方。tf: 自己位置推定 (map -> base)。sim: シミュレータ上の真の姿勢
    # (地図にない障害物で自己位置推定がずれる配置でも、実際に当たったかを測る)
    pose_source: str = "tf"

    def validate(self, where: str) -> None:
        if not self.obstacles:
            raise ScenarioValidationError(f"{where}: 'obstacles' must not be empty")
        if self.pose_source not in ("tf", "sim"):
            raise ScenarioValidationError(f"{where}: 'pose_source' must be 'tf' or 'sim'")
        if self.require_within is not None and self.require_within <= 0.0:
            raise ScenarioValidationError(f"{where}: 'require_within' must be > 0")


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
    ロボットの姿勢は、pose_source が tf なら TF (推定値) で、数 cm の誤差を含む。地図にない
    障害物に囲まれるなどで自己位置推定が大きくずれる配置では、sim (シミュレータ上の真の姿勢) にする。
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
        if spec.pose_source == "sim":
            # 真の姿勢を出せないバックエンドなら、ここで ScenarioError にする
            ctx.backend.entity_pose(ctx.profile.sim.robot_entity)

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

    def _robot_pose(self) -> Optional[Pose]:
        if self._spec.pose_source == "sim":
            world_pose = self._ctx.backend.entity_pose(self._ctx.profile.sim.robot_entity)
            if world_pose is None:
                return None
            return self._ctx.poses.to_map(PoseSpec(
                "world", world_pose.x, world_pose.y, world_pose.z, world_pose.yaw))
        try:
            return self._ctx.poses.robot.get()
        except ScenarioError:
            return None

    def _on_msg(self, msg) -> None:
        self._received += 1
        robot = self._robot_pose()
        if robot is None:
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
        required = self._spec.require_within
        if required is not None and self._min_all > required:
            return CheckResult(
                "", ResultStatus.FAILED,
                f"obstacles never came within {required} m of the robot "
                f"({reference}): the scenario did not meet the obstacle")
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
class PoseErrorSpec:
    # 推定 (TF の map -> base) と真の姿勢の差が、位置でこれ [m] か、向きでこれ [rad] を超えている間を
    # 「ずれている」とみなす
    deviation_m: float = 1.0
    deviation_yaw: float = 0.35
    # 位置の差の最大 [m] の上限 (省略時は判定しない)
    max_error_m: Optional[float] = None
    # ずれている間に、ロボットが実際に走った距離 [m] の上限 (省略時は判定しない)。
    # ずれたまま走ることを測る
    max_deviated_distance_m: Optional[float] = None
    # ずれていた時間の合計 [s] の上限 (sim 時間。省略時は判定しない)。止まったままずれていることを測る
    max_deviated_sec: Optional[float] = None
    # 姿勢を比べる間隔 [s] (sim 時間)
    sample_sec: float = 0.2
    # 比べるタイミングの元になるトピック
    topic: str = "/odom"

    def validate(self, where: str) -> None:
        if self.deviation_m <= 0.0 or self.deviation_yaw <= 0.0 or self.sample_sec <= 0.0:
            raise ScenarioValidationError(
                f"{where}: 'deviation_m', 'deviation_yaw' and 'sample_sec' must be > 0")


@register_monitor("pose_error", PoseErrorSpec)
class PoseErrorMonitor(TopicMonitor):
    """自己位置推定とシミュレータ上の真の姿勢の差を監視する。

    差の最大、ずれていた時間、ずれている間に実際に走った距離を出す。上限を指定すると判定する。
    """
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: PoseErrorSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        # 真の姿勢を出せないバックエンドなら、ここで ScenarioError にする
        ctx.backend.entity_pose(ctx.profile.sim.robot_entity)
        self._samples = 0
        self._last_time: Optional[float] = None
        self._last_truth: Optional[Pose] = None
        self._max_error = 0.0
        self._max_yaw_error = 0.0
        self._deviated_sec = 0.0
        self._deviated_distance = 0.0
        self._last_error = (0.0, 0.0)

    def _truth(self) -> Optional[Pose]:
        world_pose = self._ctx.backend.entity_pose(self._ctx.profile.sim.robot_entity)
        if world_pose is None:
            return None
        return self._ctx.poses.to_map(PoseSpec(
            "world", world_pose.x, world_pose.y, world_pose.z, world_pose.yaw))

    def _on_msg(self, msg) -> None:
        now = self._ctx.clock.now()
        if self._last_time is not None and now - self._last_time < self._spec.sample_sec:
            return
        try:
            estimate = self._ctx.poses.robot.get()
        except ScenarioError:
            return
        truth = self._truth()
        if truth is None:
            return
        error = estimate.distance_xy(truth)
        yaw_error = abs(normalize_angle(estimate.yaw - truth.yaw))
        deviated = error > self._spec.deviation_m or yaw_error > self._spec.deviation_yaw
        if deviated and self._last_time is not None:
            self._deviated_sec += now - self._last_time
            self._deviated_distance += truth.distance_xy(self._last_truth)
        self._samples += 1
        self._max_error = max(self._max_error, error)
        self._max_yaw_error = max(self._max_yaw_error, yaw_error)
        self._last_error = (error, yaw_error)
        self._last_time = now
        self._last_truth = truth

    def _evaluate(self) -> CheckResult:
        if self._samples == 0:
            return CheckResult("", ResultStatus.ERROR, "no pose was compared")
        failed = (
            (self._spec.max_error_m is not None and self._max_error > self._spec.max_error_m)
            or (self._spec.max_deviated_distance_m is not None
                and self._deviated_distance > self._spec.max_deviated_distance_m)
            or (self._spec.max_deviated_sec is not None
                and self._deviated_sec > self._spec.max_deviated_sec))
        return CheckResult(
            "", ResultStatus.FAILED if failed else ResultStatus.PASSED,
            f"max error {self._max_error:.2f} m / {math.degrees(self._max_yaw_error):.0f} deg "
            f"(limit {self._spec.max_error_m}), deviated {self._deviated_sec:.1f} s "
            f"(limit {self._spec.max_deviated_sec}), "
            f"travelled {self._deviated_distance:.2f} m while deviated "
            f"(limit {self._spec.max_deviated_distance_m}), "
            f"last error {self._last_error[0]:.2f} m / {math.degrees(self._last_error[1]):.0f} deg")


@dataclass
class MapOverlapSpec:
    # ロボットの footprint (base 座標系の多角形 [[x, y], ...])
    footprint: List[List[float]]
    # 事前地図 (nav_msgs/OccupancyGrid)。推定の footprint がこの占有セルにかかるかを見る
    map_topic: str = "/planning_map"
    # 占有とみなす OccupancyGrid の値の下限
    occupied_threshold: int = 65
    # 見かけの食い込みが起きることを必須にする (起きなければ、試験になっていないので ERROR)
    require_overlap: bool = False
    # 1 回の見かけの食い込みが続く時間の上限 [s] (sim 時間。省略時は判定しない)
    max_overlap_sec: Optional[float] = None
    # true なら、走行の終わりに見かけの食い込みが解けていること
    ends_clear: bool = True
    sample_sec: float = 0.2
    topic: str = "/odom"

    def validate(self, where: str) -> None:
        if len(self.footprint) < 3:
            raise ScenarioValidationError(f"{where}: 'footprint' needs at least 3 points")
        if self.sample_sec <= 0.0:
            raise ScenarioValidationError(f"{where}: 'sample_sec' must be > 0")


def _point_in_polygon(x: float, y: float, polygon: List[List[float]]) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


@register_monitor("map_overlap", MapOverlapSpec)
class MapOverlapMonitor(TopicMonitor):
    """推定の footprint が事前地図の障害物にかかり、真の footprint はかかっていない状態を測る。

    自己位置がずれて、実際には接触していないのに経路を作れなくなる状態 (見かけの食い込み) の回数と長さを出す。
    """
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: MapOverlapSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        ctx.backend.entity_pose(ctx.profile.sim.robot_entity)
        self._map_sub = None
        self._map: Optional[OccupancyGrid] = None
        self._points: List[Point] = []
        self._last_time: Optional[float] = None
        self._samples = 0
        self._count = 0
        self._since: Optional[float] = None
        self._longest = 0.0

    def start(self) -> None:
        super().start()
        self._map_sub = self._ctx.node.create_subscription(
            OccupancyGrid, self._spec.map_topic, self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def stop(self) -> None:
        super().stop()
        if self._map_sub is not None:
            self._ctx.node.destroy_subscription(self._map_sub)
            self._map_sub = None

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map = msg
            step = msg.info.resolution / 2.0
            xs = [p[0] for p in self._spec.footprint]
            ys = [p[1] for p in self._spec.footprint]
            self._points = [
                (min(xs) + i * step, min(ys) + j * step)
                for i in range(int((max(xs) - min(xs)) / step) + 1)
                for j in range(int((max(ys) - min(ys)) / step) + 1)
                if _point_in_polygon(min(xs) + i * step, min(ys) + j * step, self._spec.footprint)]

    def _overlaps(self, pose: Pose) -> bool:
        info = self._map.info
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        for px, py in self._points:
            col = int(math.floor(
                (pose.x + c * px - s * py - info.origin.position.x) / info.resolution))
            row = int(math.floor(
                (pose.y + s * px + c * py - info.origin.position.y) / info.resolution))
            if (0 <= col < info.width and 0 <= row < info.height
                    and self._map.data[row * info.width + col] >= self._spec.occupied_threshold):
                return True
        return False

    def _truth(self) -> Optional[Pose]:
        world_pose = self._ctx.backend.entity_pose(self._ctx.profile.sim.robot_entity)
        if world_pose is None:
            return None
        return self._ctx.poses.to_map(PoseSpec(
            "world", world_pose.x, world_pose.y, world_pose.z, world_pose.yaw))

    def _on_msg(self, msg) -> None:
        now = self._ctx.clock.now()
        if self._map is None or (self._last_time is not None
                                 and now - self._last_time < self._spec.sample_sec):
            return
        try:
            estimate = self._ctx.poses.robot.get()
        except ScenarioError:
            return
        truth = self._truth()
        if truth is None:
            return
        self._last_time = now
        self._samples += 1
        apparent = self._overlaps(estimate) and not self._overlaps(truth)
        if apparent and self._since is None:
            self._since = now
            self._count += 1
        elif not apparent and self._since is not None:
            self._longest = max(self._longest, now - self._since)
            self._since = None

    def _evaluate(self) -> CheckResult:
        if self._samples == 0:
            return CheckResult("", ResultStatus.ERROR, f"no pose was compared with {self._spec.map_topic}")
        if self._spec.require_overlap and self._count == 0:
            return CheckResult("", ResultStatus.ERROR, "apparent overlap never occurred (the test did not exercise it)")
        ongoing = self._since is not None
        longest = max(self._longest, self._ctx.clock.now() - self._since) if ongoing else self._longest
        failed = ((self._spec.max_overlap_sec is not None and longest > self._spec.max_overlap_sec)
                  or (self._spec.ends_clear and ongoing))
        return CheckResult(
            "", ResultStatus.FAILED if failed else ResultStatus.PASSED,
            f"apparent overlap {self._count} time(s), longest {longest:.1f} s "
            f"(limit {self._spec.max_overlap_sec})" + (", still overlapping at the end" if ongoing else ""))


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
    # true なら、その場の回転 (角速度が turn_threshold 以上) も動いているとみなし、停止に数えない
    # (resume_within の include_turning と同じ考え方)
    include_turning: bool = False
    turn_threshold: float = 0.1
    # 監視区間: from_goal_started 番目のゴールの開始から、until_goal_reached 番目のゴールの到達まで
    from_goal_started: int = 0
    until_goal_reached: int = 0
    # 指定すると、このタイムライン項目 (timeline[].name) が発火してから区間を始める
    # (障害物を出してからの停止を測る)
    after_timeline: Optional[str] = None
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
        if (self._spec.after_timeline is not None
                and events.find("timeline_fired", entry=self._spec.after_timeline) is None):
            return False
        return (events.find("goal_started", index=self._spec.from_goal_started) is not None
                and events.find("goal_reached", index=self._spec.until_goal_reached) is None)

    def _on_msg(self, msg) -> None:
        if not self._in_window():
            self._stop_since = None
            return
        self._observed = True
        v = msg.twist.twist.linear
        now = self._ctx.clock.now()
        turning = (self._spec.include_turning
                   and abs(msg.twist.twist.angular.z) >= self._spec.turn_threshold)
        if math.hypot(v.x, v.y) < self._spec.speed_threshold and not turning:
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


@dataclass
class ResumeWithinSpec:
    # このタイムライン項目 (timeline[].name) が発火してから、走り出すまでの時間を測る
    after_timeline: str
    # これ (sim 時間 [s]) を超えても走り出さなければ FAILED
    max_sec: float
    speed_threshold: float = 0.05
    # この時間 (sim 時間 [s]) 続けて speed_threshold 以上なら走り出したとみなす
    # (障害物の除去や首振りの直後の一瞬の速度で誤判定しない)。走り出した時刻は、続き始めた時刻
    min_moving_sec: float = 0.5
    # true なら、その場の回転 (角速度が turn_threshold 以上) も走り出しとみなす
    # (向きを直してから進むコントローラーで、再開を回転の開始から数える)
    include_turning: bool = False
    turn_threshold: float = 0.1
    topic: str = "/odom"


@register_monitor("resume_within", ResumeWithinSpec)
class ResumeWithinMonitor(TopicMonitor):
    """障害物の除去などのタイムライン項目から、ロボットが走り出すまでの時間を判定する。"""
    MSG_TYPE = Odometry

    def __init__(self, ctx: "ScenarioContext", spec: ResumeWithinSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._delay: Optional[float] = None
        self._moving_since: Optional[float] = None
        # include_turning が false のときの参考の値: 回転を始めた時刻
        # (その場の回転で走り出しが遅れて見える場合の切り分け用)
        self._turn_delay: Optional[float] = None

    def _on_msg(self, msg) -> None:
        if self._delay is not None:
            return
        fired = self._ctx.events.find("timeline_fired", entry=self._spec.after_timeline)
        if fired is None:
            return
        now = self._ctx.clock.now()
        turning = abs(msg.twist.twist.angular.z) >= self._spec.turn_threshold
        if self._turn_delay is None and turning:
            self._turn_delay = now - fired.time
        v = msg.twist.twist.linear
        moving = math.hypot(v.x, v.y) >= self._spec.speed_threshold or (
            self._spec.include_turning and turning)
        if not moving:
            self._moving_since = None
            return
        if self._moving_since is None:
            self._moving_since = now
        if now - self._moving_since >= self._spec.min_moving_sec:
            self._delay = self._moving_since - fired.time

    def _evaluate(self) -> CheckResult:
        if self._ctx.events.find("timeline_fired", entry=self._spec.after_timeline) is None:
            return CheckResult(
                "", ResultStatus.ERROR,
                f"timeline entry '{self._spec.after_timeline}' never fired")
        if self._delay is None:
            return CheckResult(
                "", ResultStatus.FAILED,
                f"did not move after '{self._spec.after_timeline}'")
        status = (ResultStatus.PASSED if self._delay <= self._spec.max_sec
                  else ResultStatus.FAILED)
        turn = ("" if self._turn_delay is None or self._spec.include_turning
                else f", started turning {self._turn_delay:.1f} s after")
        return CheckResult(
            "", status,
            f"moved {self._delay:.1f} s after '{self._spec.after_timeline}' "
            f"(max {self._spec.max_sec} s){turn}")
