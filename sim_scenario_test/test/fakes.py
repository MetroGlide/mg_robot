"""エンジンをシミュレータ・ROS なしで動かすためのフェイク群。"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List
from unittest.mock import MagicMock

from sim_scenario_test.context import Clock, ScenarioContext
from sim_scenario_test.drivers.base import RunDriver
from sim_scenario_test.engine.result import Outcome
from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose
from sim_scenario_test.poses import PoseResolver, RobotPoseSource
from sim_scenario_test.profile import Profile
from sim_scenario_test.registry import DEFAULT_REGISTRY, register_driver
from sim_scenario_test.sim.base import SimulationBackend


class FakeClock(Clock):
    """実時間の 10 倍速で進む時計。"""

    def __init__(self):
        self._t0 = time.monotonic()

    def now(self) -> float:
        return (time.monotonic() - self._t0) * 10.0


class FakeBackend(SimulationBackend):
    def __init__(self, fail_spawn: bool = False):
        self.calls: List[tuple] = []
        self.fail_spawn = fail_spawn

    def is_ready(self) -> bool:
        return True

    def set_entity_pose(self, name, pose):
        self.calls.append(("set_pose", name, pose))

    def spawn_entity(self, name, model, pose):
        if self.fail_spawn:
            raise ScenarioError("spawn failed")
        self.calls.append(("spawn", name, pose))

    def remove_entity(self, name):
        self.calls.append(("remove", name))


class FakeRobot(RobotPoseSource):
    def __init__(self, pose: Pose = Pose()):
        self.pose = pose

    def wait_available(self, timeout_sec: float) -> bool:
        return True

    def get(self) -> Pose:
        return self.pose


@dataclass
class FakeGoalsSpec:
    goals: int = 2
    # 失敗させるゴール index (-1 で失敗なし)
    fail_at: int = -1
    # 各ゴールの所要時間 (FakeClock の sim 秒)
    goal_sec: float = 2.0
    # true の場合 abort されるまで走行を終えない
    hang: bool = False


@register_driver("fake_goals", FakeGoalsSpec)
class FakeGoalsDriver(RunDriver):
    """ゴール到達を模擬するドライバ。"""

    def run(self) -> Outcome:
        ctx = self.ctx
        spec = self.spec
        reached = 0
        for i in range(spec.goals):
            ctx.events.emit("goal_started", index=i)
            end = ctx.clock.now() + spec.goal_sec
            while spec.hang or ctx.clock.now() < end:
                if ctx.abort_event.is_set():
                    return Outcome(spec.goals, reached, i, ctx.abort_reason)
                time.sleep(0.005)
            if i == spec.fail_at:
                ctx.events.emit("goal_failed", index=i)
                return Outcome(spec.goals, reached, i, "fake failure")
            ctx.events.emit("goal_reached", index=i)
            reached += 1
        return Outcome(spec.goals, reached)


def make_context(scenario, backend=None, robot=None) -> ScenarioContext:
    node = MagicMock()
    profile = Profile(name="test", worlds={"w": {"sim_world": "w"}})
    return ScenarioContext(
        node=node,
        registry=DEFAULT_REGISTRY,
        scenario=scenario,
        profile=profile,
        clock=FakeClock(),
        backend=backend or FakeBackend(),
        poses=PoseResolver(Pose(), robot or FakeRobot()),
        nav2=MagicMock(),
    )


def wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False
