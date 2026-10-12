"""MG-01 固有の monitor (カスタム nav2_msgs/CollisionMonitorState と mg_msgs/NavigationActivity を使うもの)。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from mg_msgs.msg import NavigationActivity
from nav2_msgs.msg import CollisionMonitorState
from sim_scenario_test.builtin.monitors import TopicMonitor, occurrence_result
from sim_scenario_test.engine.result import CheckResult
from sim_scenario_test.registry import register_monitor

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext

# nav2_msgs/CollisionMonitorState の action_type
_ACTION_TYPES = {"STOP": 1, "SLOWDOWN": 2, "APPROACH": 3}


@dataclass
class CollisionMonitorActionSpec:
    action: Literal["STOP", "SLOWDOWN", "APPROACH"]
    expect: Literal["occurs", "never"] = "occurs"
    topic: str = "/collision_monitor_state"


@register_monitor("collision_monitor_action", CollisionMonitorActionSpec)
class CollisionMonitorActionMonitor(TopicMonitor):
    """collision_monitor が action を発動した回数を数え、発動する／しないことを判定する。"""
    MSG_TYPE = CollisionMonitorState

    def __init__(self, ctx: "ScenarioContext", spec: CollisionMonitorActionSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._count = 0

    def _on_msg(self, msg) -> None:
        if msg.action_type == _ACTION_TYPES[self._spec.action]:
            self._count += 1

    def _evaluate(self) -> CheckResult:
        what = f"collision_monitor {self._spec.action}"
        return occurrence_result(self._count, self._spec.expect, what)


@dataclass
class NavigationActivitySpec:
    # NavigationActivity.code (mg_msgs/msg/NavigationActivity.msg の定数)
    code: str
    expect: Literal["occurs", "never"] = "occurs"
    topic: str = "/waypoint_sequencer_node/activity"


@register_monitor("navigation_activity", NavigationActivitySpec)
class NavigationActivityMonitor(TopicMonitor):
    """シーケンサの「いまの活動」が code になった回数を数え、発生する／しないことを判定する。"""
    MSG_TYPE = NavigationActivity

    def __init__(self, ctx: "ScenarioContext", spec: NavigationActivitySpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._count = 0

    def _on_msg(self, msg) -> None:
        if msg.code == self._spec.code:
            self._count += 1

    def _evaluate(self) -> CheckResult:
        what = f"navigation activity '{self._spec.code}'"
        return occurrence_result(self._count, self._spec.expect, what)
