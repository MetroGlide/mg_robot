from __future__ import annotations

import dataclasses
import threading
from dataclasses import dataclass
from typing import TYPE_CHECKING

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.geometry import Pose, PoseSpec
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
    if spec.clear_costmaps:
        ctx.nav2.clear_costmaps()


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


@register_action("spawn", SpawnSpec)
def spawn(ctx: "ScenarioContext", spec: SpawnSpec, stop: threading.Event) -> None:
    """obstacles に定義した障害物を配置する。"""
    model = ctx.scenario.obstacles[spec.obstacle].model
    if model.path:
        model = dataclasses.replace(
            model, path=ctx.expand(model.path, f"obstacles.{spec.obstacle}.model.path"))
    world = ctx.poses.to_world(spec.pose)
    ctx.logger.info(
        f"[spawn] '{spec.obstacle}' at world ({world.x:.2f}, {world.y:.2f})")
    ctx.backend.spawn_entity(spec.obstacle, model, world)
    ctx.track_spawned(spec.obstacle)


@dataclass
class DespawnSpec:
    obstacle: str


@register_action("despawn", DespawnSpec)
def despawn(ctx: "ScenarioContext", spec: DespawnSpec, stop: threading.Event) -> None:
    """配置済みの障害物を削除する。"""
    ctx.backend.remove_entity(spec.obstacle)
    ctx.untrack_spawned(spec.obstacle)


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
