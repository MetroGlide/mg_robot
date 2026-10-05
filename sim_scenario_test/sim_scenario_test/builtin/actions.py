from __future__ import annotations

import dataclasses
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from std_srvs.srv import SetBool

from sim_scenario_test.errors import ScenarioError, ScenarioValidationError
from sim_scenario_test.geometry import Pose, PoseSpec
from sim_scenario_test.nav2 import call_service
from sim_scenario_test.registry import register_action

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext


@dataclass
class RespawnSpec:
    pose: PoseSpec
    # 初期位置設定後、自己位置推定が落ち着くまで待つ sim 時間 (秒)
    settle_sec: float = 2.0
    set_initial_pose: bool = True
    clear_costmaps: bool = True
    # 自己位置推定が指定位置からこの距離 [m] 以内に収束するまで待つ (0 以下で確認しない)
    converge_tolerance: float = 0.5
    converge_timeout_sec: float = 10.0


@register_action("respawn", RespawnSpec)
def respawn(ctx: "ScenarioContext", spec: RespawnSpec, stop: threading.Event) -> None:
    """ロボットを移動し、自己位置の初期化とコストマップのクリアまで行う。"""
    pose_map = ctx.poses.to_map(spec.pose)
    world = ctx.poses.to_world(spec.pose)
    ctx.backend.set_entity_pose(
        ctx.profile.sim.robot_entity,
        Pose(world.x, world.y, world.z + ctx.profile.sim.robot_spawn_z, world.yaw))
    if spec.set_initial_pose:
        ctx.nav2.publish_initial_pose(pose_map)
    ctx.clock.sleep(spec.settle_sec, stop)
    if spec.set_initial_pose and spec.converge_tolerance > 0.0:
        _wait_converged(ctx, pose_map, spec, stop)
    if spec.clear_costmaps:
        ctx.nav2.clear_costmaps()


def _wait_converged(
    ctx: "ScenarioContext", target: Pose, spec: RespawnSpec, stop: threading.Event
) -> None:
    deadline = ctx.clock.now() + spec.converge_timeout_sec
    while True:
        error = ctx.poses.robot.get().distance_xy(target)
        if error <= spec.converge_tolerance:
            return
        if stop.is_set():
            return
        if ctx.clock.now() > deadline:
            raise ScenarioError(
                f"localization did not converge within {spec.converge_timeout_sec:.0f}s "
                f"(error {error:.2f} m, tolerance {spec.converge_tolerance} m)")
        ctx.clock.sleep(0.2, stop)


@dataclass
class TeleportSpec:
    pose: PoseSpec
    # 省略時はロボット
    entity: str = ""


@register_action("teleport", TeleportSpec)
def teleport(ctx: "ScenarioContext", spec: TeleportSpec, stop: threading.Event) -> None:
    """エンティティをシミュレータ上で移動する (自己位置推定には通知しない)。"""
    entity = spec.entity or ctx.profile.sim.robot_entity
    world = ctx.poses.to_world(spec.pose)
    if entity == ctx.profile.sim.robot_entity:
        world = Pose(world.x, world.y, world.z + ctx.profile.sim.robot_spawn_z, world.yaw)
    ctx.backend.set_entity_pose(entity, world)
    if entity != ctx.profile.sim.robot_entity:
        ctx.entity_poses[entity] = world


@dataclass
class SetInitialPoseSpec:
    pose: PoseSpec


@register_action("set_initial_pose", SetInitialPoseSpec)
def set_initial_pose(
    ctx: "ScenarioContext", spec: SetInitialPoseSpec, stop: threading.Event
) -> None:
    """自己位置推定に初期位置 (/initialpose) を与える。"""
    ctx.nav2.publish_initial_pose(ctx.poses.to_map(spec.pose))


@register_action("clear_costmaps")
def clear_costmaps(ctx: "ScenarioContext", spec: None, stop: threading.Event) -> None:
    """Nav2 のグローバル・ローカルコストマップをクリアする。"""
    ctx.nav2.clear_costmaps()


@dataclass
class SpawnSpec:
    obstacle: str
    pose: PoseSpec
    # x, y に ±jitter [m] の一様乱数を加える (シナリオの seed で再現できる)
    jitter: float = 0.0


@register_action("spawn", SpawnSpec)
def spawn(ctx: "ScenarioContext", spec: SpawnSpec, stop: threading.Event) -> None:
    """obstacles に定義した障害物を配置する。"""
    model = ctx.scenario.obstacles[spec.obstacle].model
    if model.path:
        model = dataclasses.replace(
            model, path=ctx.expand(model.path, f"obstacles.{spec.obstacle}.model.path"))
    world = ctx.poses.to_world(spec.pose)
    if spec.jitter > 0.0:
        world = Pose(
            world.x + ctx.rng.uniform(-spec.jitter, spec.jitter),
            world.y + ctx.rng.uniform(-spec.jitter, spec.jitter),
            world.z, world.yaw)
    ctx.logger.info(
        f"[spawn] '{spec.obstacle}' at world ({world.x:.2f}, {world.y:.2f})")
    ctx.backend.spawn_entity(spec.obstacle, model, world)
    ctx.track_spawned(spec.obstacle)
    ctx.entity_poses[spec.obstacle] = world
    _wait_entity(ctx, spec.obstacle)


def _wait_entity(ctx: "ScenarioContext", name: str, timeout_sec: float = 15.0) -> None:
    """spawn の受理後に、エンティティが実際に生成されたことを確認する。"""
    deadline = time.monotonic() + timeout_sec
    while not ctx.backend.entity_exists(name):
        if time.monotonic() > deadline:
            raise ScenarioError(
                f"'{name}' was accepted by the simulator but does not exist after "
                f"{timeout_sec:.0f}s (model download failure?)")
        time.sleep(0.2)


@dataclass
class DespawnSpec:
    obstacle: str


@register_action("despawn", DespawnSpec)
def despawn(ctx: "ScenarioContext", spec: DespawnSpec, stop: threading.Event) -> None:
    """配置済みの障害物を削除する。"""
    ctx.backend.remove_entity(spec.obstacle)
    ctx.untrack_spawned(spec.obstacle)
    ctx.entity_poses.pop(spec.obstacle, None)


@dataclass
class MoveObstacleSpec:
    obstacle: str
    to: PoseSpec
    # 移動速度 [m/s] と姿勢の更新周期 [Hz]。更新は gz service 経由なので 5 Hz 程度が上限
    speed: float = 1.0
    rate_hz: float = 5.0

    def validate(self, where: str) -> None:
        if self.speed <= 0.0 or self.rate_hz <= 0.0:
            raise ScenarioValidationError(f"{where}: 'speed' and 'rate_hz' must be > 0")


@register_action("move_obstacle", MoveObstacleSpec)
def move_obstacle(
    ctx: "ScenarioContext", spec: MoveObstacleSpec, stop: threading.Event
) -> None:
    """配置済みの障害物を、現在位置から to まで等速で直線移動させる (移動する歩行者など)。"""
    start = ctx.entity_poses.get(spec.obstacle)
    if start is None:
        raise ScenarioError(f"'{spec.obstacle}' has not been spawned")
    goal = ctx.poses.to_world(spec.to)
    distance = start.distance_xy(goal)
    t0 = ctx.clock.now()
    while True:
        travelled = min(spec.speed * (ctx.clock.now() - t0), distance)
        ratio = 1.0 if distance == 0.0 else travelled / distance
        pose = Pose(
            start.x + (goal.x - start.x) * ratio,
            start.y + (goal.y - start.y) * ratio,
            goal.z, goal.yaw)
        ctx.backend.set_entity_pose(spec.obstacle, pose)
        ctx.entity_poses[spec.obstacle] = pose
        if ratio >= 1.0 or stop.is_set():
            return
        ctx.clock.sleep(1.0 / spec.rate_hz, stop)


@dataclass
class CallSetBoolSpec:
    service: str
    value: bool


@register_action("call_set_bool", CallSetBoolSpec)
def call_set_bool(
    ctx: "ScenarioContext", spec: CallSetBoolSpec, stop: threading.Event
) -> None:
    """std_srvs/SetBool サービスを呼ぶ (センサの配信停止など故障の注入に使う)。"""
    key = f"set_bool:{spec.service}"
    if key not in ctx.extensions:
        ctx.extensions[key] = ctx.node.create_client(SetBool, spec.service)
    request = SetBool.Request()
    request.data = spec.value
    response = call_service(ctx.node, ctx.extensions[key], request, 5.0)
    if not response.success:
        raise ScenarioError(f"{spec.service} rejected: {response.message}")


@dataclass
class DelaySpec:
    sec: float

    def validate(self, where: str) -> None:
        if self.sec < 0.0:
            raise ScenarioValidationError(f"{where}: 'sec' must be >= 0")


@register_action("delay", DelaySpec)
def delay(ctx: "ScenarioContext", spec: DelaySpec, stop: threading.Event) -> None:
    """sim 時間で sec 秒待つ。"""
    ctx.clock.sleep(spec.sec, stop)


@dataclass
class LogSpec:
    message: str


@register_action("log", LogSpec)
def log(ctx: "ScenarioContext", spec: LogSpec, stop: threading.Event) -> None:
    """ログにメッセージを出力する。"""
    ctx.logger.info(f"[log] {spec.message}")
    ctx.events.emit("log", message=spec.message)
