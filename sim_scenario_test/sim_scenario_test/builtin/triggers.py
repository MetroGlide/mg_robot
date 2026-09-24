from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from sim_scenario_test.geometry import PoseSpec
from sim_scenario_test.registry import register_trigger

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext


@dataclass
class AtTimeSpec:
    sec: float


@register_trigger("at_time", AtTimeSpec)
class AtTime:
    """走行開始から sec 秒 (sim 時間) 経過したら成立する。"""

    def __init__(self, ctx: "ScenarioContext", spec: AtTimeSpec):
        self._ctx = ctx
        self._spec = spec

    def poll(self) -> bool:
        return self._ctx.elapsed() >= self._spec.sec


@dataclass
class GoalIndexSpec:
    index: int


class _GoalEvent:
    EVENT = ""

    def __init__(self, ctx: "ScenarioContext", spec: GoalIndexSpec):
        self._ctx = ctx
        self._spec = spec

    def poll(self) -> bool:
        return self._ctx.events.find(self.EVENT, index=self._spec.index) is not None


@register_trigger("goal_started", GoalIndexSpec)
class GoalStarted(_GoalEvent):
    """index 番目のゴールへの走行を開始したら成立する (before フック実行後)。"""
    EVENT = "goal_started"


@register_trigger("goal_reached", GoalIndexSpec)
class GoalReached(_GoalEvent):
    """index 番目のゴールに到達したら成立する。"""
    EVENT = "goal_reached"


@register_trigger("goal_failed", GoalIndexSpec)
class GoalFailed(_GoalEvent):
    """index 番目のゴールへの走行が失敗したら成立する。"""
    EVENT = "goal_failed"


@dataclass
class RobotTravelledSpec:
    distance: float


@register_trigger("robot_travelled", RobotTravelledSpec)
class RobotTravelled:
    """走行開始からのロボットの移動距離 (経路長) が distance [m] に達したら成立する。"""

    def __init__(self, ctx: "ScenarioContext", spec: RobotTravelledSpec):
        self._ctx = ctx
        self._spec = spec
        self._last = None
        self._total = 0.0

    def poll(self) -> bool:
        pose = self._ctx.poses.robot.get()
        if self._last is not None:
            self._total += pose.distance_xy(self._last)
        self._last = pose
        return self._total >= self._spec.distance


@dataclass
class RobotNearSpec:
    x: float
    y: float
    radius: float
    frame: Literal["map", "world"] = "map"


@register_trigger("robot_near", RobotNearSpec)
class RobotNear:
    """ロボットが指定点から radius [m] 以内に入ったら成立する。"""

    def __init__(self, ctx: "ScenarioContext", spec: RobotNearSpec):
        self._ctx = ctx
        self._spec = spec
        self._target = ctx.poses.to_map(PoseSpec(frame=spec.frame, x=spec.x, y=spec.y))

    def poll(self) -> bool:
        robot = self._ctx.poses.robot.get()
        return robot.distance_xy(self._target) <= self._spec.radius
