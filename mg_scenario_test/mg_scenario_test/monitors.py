"""MG-01 固有の monitor (カスタム nav2_msgs/CollisionMonitorState、mg_msgs/NavigationActivity、
自己位置の監督の状態 localization_supervisor_msgs/LocalizationStatus を使うもの)。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Optional

from localization_supervisor_msgs.msg import LocalizationStatus
from mg_msgs.msg import NavigationActivity
from nav2_msgs.msg import CollisionMonitorState
from sim_scenario_test.builtin.monitors import TopicMonitor, occurrence_result
from sim_scenario_test.engine.result import CheckResult, ResultStatus
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


# localization_supervisor_msgs/LocalizationStatus の state
_LOCALIZATION_STATES = {
    "NORMAL": 0, "SUSPECT": 1, "ISOLATED": 2, "RECOVERING": 3, "DEGRADED": 4}


@dataclass
class LocalizationStateSpec:
    # LocalizationStatus.state の名前
    state: Literal["NORMAL", "SUSPECT", "ISOLATED", "RECOVERING", "DEGRADED"]
    expect: Literal["occurs", "never"] = "occurs"
    # 走行後の最後の状態 (省略時は見ない)
    ends_in: Optional[Literal["NORMAL", "SUSPECT", "ISOLATED", "RECOVERING", "DEGRADED"]] = None
    topic: str = "/localization/status"


@register_monitor("localization_state", LocalizationStateSpec)
class LocalizationStateMonitor(TopicMonitor):
    """自己位置の監督の状態が state になった配信の回数を数え、その状態になる／ならないことを判定する。
    ends_in を指定すると、最後に受信した状態がそれであること (復旧して戻ったこと) も判定する。"""
    MSG_TYPE = LocalizationStatus

    def __init__(self, ctx: "ScenarioContext", spec: LocalizationStateSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._count = 0
        self._last: Optional[int] = None

    def _on_msg(self, msg) -> None:
        if msg.state == _LOCALIZATION_STATES[self._spec.state]:
            self._count += 1
        self._last = msg.state

    def _evaluate(self) -> CheckResult:
        result = occurrence_result(self._count, self._spec.expect, f"localization state {self._spec.state}")
        if self._spec.ends_in is None or result.status != ResultStatus.PASSED:
            return result
        names = {number: name for name, number in _LOCALIZATION_STATES.items()}
        last = "(no message)" if self._last is None else names.get(self._last, str(self._last))
        if last != self._spec.ends_in:
            return CheckResult(
                "", ResultStatus.FAILED, f"{result.message}, but ended in {last} (expected {self._spec.ends_in})")
        return CheckResult("", ResultStatus.PASSED, f"{result.message}, ended in {last}")
