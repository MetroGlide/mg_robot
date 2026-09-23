from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, List, Optional

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from sim_scenario_test.drivers.base import RunDriver
from sim_scenario_test.engine.result import Outcome
from sim_scenario_test.errors import ScenarioError, ScenarioValidationError
from sim_scenario_test.geometry import Pose, PoseSpec, quaternion_from_yaw
from sim_scenario_test.registry import register_driver
from sim_scenario_test.spec import ActionCall

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext


@dataclass
class GoalHooks:
    """ゴールごとに走行開始前・到達後に同期実行する action。"""
    before: List[ActionCall] = field(default_factory=list)
    after: List[ActionCall] = field(default_factory=list)


@dataclass
class NavGoal:
    pose: PoseSpec
    before: List[ActionCall] = field(default_factory=list)
    after: List[ActionCall] = field(default_factory=list)


@dataclass
class Nav2GoalsSpec:
    # goals と waypoints_file はどちらか一方を指定する
    goals: List[NavGoal] = field(default_factory=list)
    waypoints_file: str = ""
    waypoints_format: str = "poses"
    # waypoints_file 使用時の waypoint index ごとのフック
    hooks: Dict[int, GoalHooks] = field(default_factory=dict)
    # ゴール 1 つあたりの制限 (sim 時間の秒)
    goal_timeout_sec: float = 300.0
    behavior_tree: str = ""

    def validate(self, where: str) -> None:
        if bool(self.goals) == bool(self.waypoints_file):
            raise ScenarioValidationError(
                f"{where}: specify exactly one of 'goals' or 'waypoints_file'")
        if self.hooks and not self.waypoints_file:
            raise ScenarioValidationError(
                f"{where}: 'hooks' is only valid with 'waypoints_file' "
                "(use before/after in each goal)")


@dataclass
class _Plan:
    label: str
    pose: PoseSpec
    hooks: GoalHooks


@register_driver("nav2_goals", Nav2GoalsSpec)
class Nav2GoalsDriver(RunDriver):
    """Nav2 の NavigateToPose でゴールを順に走行する (nav2_simple_commander を利用)。"""

    def __init__(self, ctx: "ScenarioContext", spec: Nav2GoalsSpec):
        super().__init__(ctx, spec)
        self._plans = self._build_plans()
        self._navigator = BasicNavigator(node_name="scenario_navigator")

    def _build_plans(self) -> List[_Plan]:
        spec = self.spec
        if spec.goals:
            return [
                _Plan(f"goal[{i}]", g.pose, GoalHooks(g.before, g.after))
                for i, g in enumerate(spec.goals)
            ]
        where = "run.nav2_goals.waypoints_file"
        path = self.ctx.expand(spec.waypoints_file, where)
        loader = self.ctx.registry.get("waypoint_format", spec.waypoints_format, where).impl
        waypoints = loader(path)
        known = {wp.index for wp in waypoints}
        unknown = sorted(set(spec.hooks) - known)
        if unknown:
            raise ScenarioValidationError(
                f"run.nav2_goals.hooks: unknown waypoint index {unknown} in {path}")
        return [
            _Plan(f"waypoint {wp.index}", wp.pose, spec.hooks.get(wp.index, GoalHooks()))
            for wp in waypoints
        ]

    def wait_ready(self, timeout_sec: float) -> None:
        if not self._navigator.nav_to_pose_client.wait_for_server(timeout_sec=timeout_sec):
            raise ScenarioError("navigate_to_pose action server not available")

    def run(self) -> Outcome:
        ctx = self.ctx
        total = len(self._plans)
        reached = 0
        for i, plan in enumerate(self._plans):
            if ctx.abort_event.is_set():
                return Outcome(total, reached, i, ctx.abort_reason)
            ctx.logger.info(f"[nav2_goals] {i + 1}/{total} {plan.label}")
            ctx.run_actions(plan.hooks.before, ctx.abort_event)
            if ctx.abort_event.is_set():
                return Outcome(total, reached, i, ctx.abort_reason)
            target = ctx.poses.to_map(plan.pose)
            ctx.events.emit("goal_started", index=i)
            failure = self._navigate(target)
            if failure is not None:
                ctx.events.emit("goal_failed", index=i, reason=failure)
                return Outcome(total, reached, i, failure)
            ctx.events.emit("goal_reached", index=i)
            reached += 1
            ctx.run_actions(plan.hooks.after, ctx.abort_event)
        return Outcome(total, reached)

    def _navigate(self, target: Pose) -> Optional[str]:
        nav = self._navigator
        goal = PoseStamped()
        goal.header.frame_id = self.ctx.profile.frames.map
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = target.x
        goal.pose.position.y = target.y
        qx, qy, qz, qw = quaternion_from_yaw(target.yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        if not nav.goToPose(goal, self.spec.behavior_tree):
            return "goal rejected"
        deadline = self.ctx.clock.now() + self.spec.goal_timeout_sec
        while not nav.isTaskComplete():
            if self.ctx.abort_event.is_set():
                nav.cancelTask()
                return self.ctx.abort_reason
            if self.ctx.clock.now() > deadline:
                nav.cancelTask()
                return f"not finished within {self.spec.goal_timeout_sec:.0f}s"
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            return None
        return f"navigation finished with {result.name}"

    def close(self) -> None:
        self._navigator.destroy_node()
