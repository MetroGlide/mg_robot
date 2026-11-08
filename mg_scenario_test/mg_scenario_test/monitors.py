"""MG-01 固有の monitor と trigger (カスタム nav2_msgs/CollisionMonitorState、mg_msgs/NavigationActivity、
自己位置の監督の状態 localization_supervisor_msgs/LocalizationStatus を使うもの)。"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Literal, Optional

from localization_supervisor_msgs.msg import LocalizationStatus
from mg_msgs.msg import NavigationActivity
from nav2_msgs.msg import CollisionMonitorState
from sim_scenario_test.builtin.monitors import TopicMonitor, occurrence_result
from sim_scenario_test.engine.result import CheckResult, ResultStatus
from rclpy.qos import qos_profile_sensor_data
from sim_scenario_test.registry import register_monitor, register_trigger

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


# localization_supervisor_msgs/LocalizationCheck の status
_CHECK_UNKNOWN = 2


@dataclass
class LocalizationStateSpec:
    # LocalizationStatus.state の名前
    state: Literal["NORMAL", "SUSPECT", "ISOLATED", "RECOVERING", "DEGRADED"]
    expect: Literal["occurs", "never"] = "occurs"
    # 走行後の最後の状態 (省略時は見ない)
    ends_in: Optional[Literal["NORMAL", "SUSPECT", "ISOLATED", "RECOVERING", "DEGRADED"]] = None
    # 指定すると、疑わしい情報源 (blame) がこれである配信だけを数える (例: amcl。正しい原因で復旧したか)
    blame: Optional[str] = None
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
        if (msg.state == _LOCALIZATION_STATES[self._spec.state]
                and (self._spec.blame is None or msg.blame == self._spec.blame)):
            self._count += 1
        self._last = msg.state

    def _evaluate(self) -> CheckResult:
        what = f"localization state {self._spec.state}"
        if self._spec.blame is not None:
            what += f" with blame {self._spec.blame}"
        result = occurrence_result(self._count, self._spec.expect, what)
        if self._spec.ends_in is None or result.status != ResultStatus.PASSED:
            return result
        names = {number: name for name, number in _LOCALIZATION_STATES.items()}
        last = "(no message)" if self._last is None else names.get(self._last, str(self._last))
        if last != self._spec.ends_in:
            return CheckResult(
                "", ResultStatus.FAILED, f"{result.message}, but ended in {last} (expected {self._spec.ends_in})")
        return CheckResult("", ResultStatus.PASSED, f"{result.message}, ended in {last}")


@dataclass
class LocalizationHoldSpec:
    # 監督が走行を止める要求 (hold_requested) を出していた時間の合計の上限 [s] (sim 時間。省略時は判定しない)
    max_total_sec: Optional[float] = None
    # true なら、走行の終わりに要求が解けていること (止まったままでないこと) も判定する
    ends_released: bool = True
    topic: str = "/localization/status"


@register_monitor("localization_hold", LocalizationHoldSpec)
class LocalizationHoldMonitor(TopicMonitor):
    """自己位置の監督が走行を止める要求を出していた回数と時間の合計を測る。"""
    MSG_TYPE = LocalizationStatus

    def __init__(self, ctx: "ScenarioContext", spec: LocalizationHoldSpec):
        super().__init__(ctx, spec.topic)
        self._spec = spec
        self._received = 0
        self._count = 0
        self._total = 0.0
        self._since: Optional[float] = None

    def _on_msg(self, msg) -> None:
        self._received += 1
        now = self._ctx.clock.now()
        if msg.hold_requested and self._since is None:
            self._since = now
            self._count += 1
        elif not msg.hold_requested and self._since is not None:
            self._total += now - self._since
            self._since = None

    def _evaluate(self) -> CheckResult:
        if self._received == 0:
            return CheckResult("", ResultStatus.ERROR, f"no message on {self._spec.topic}")
        total = self._total + (self._ctx.clock.now() - self._since if self._since is not None else 0.0)
        holding = self._since is not None
        failed = ((self._spec.max_total_sec is not None and total > self._spec.max_total_sec)
                  or (self._spec.ends_released and holding))
        return CheckResult(
            "", ResultStatus.FAILED if failed else ResultStatus.PASSED,
            f"hold requested {self._count} time(s), {total:.1f} s in total "
            f"(limit {self._spec.max_total_sec})" + (", still holding at the end" if holding else ""))


@dataclass
class LocalizationReadySpec:
    # この秒数 (sim 時間)、監督が NORMAL で、起動・再初期化の後の猶予の外である状態が続いたら成立する
    for_sec: float = 5.0
    # 加えて、これらの判定のインスタンス (例: amcl_gnss) が一度は UNKNOWN 以外になっていること
    # (GNSS は走り出すまで出ないので、GNSS による初期化が済んだことの目安になる)
    checks: List[str] = field(default_factory=list)
    topic: str = "/localization/status"


@register_trigger("localization_ready", LocalizationReadySpec)
class LocalizationReady:
    """自己位置の監督が、故障を注入してよい落ち着いた状態になったら成立する。

    故障の注入 (/initialpose) と同じ経路を使う初期化を、準備の条件にしない。
    """

    def __init__(self, ctx: "ScenarioContext", spec: LocalizationReadySpec):
        self._ctx = ctx
        self._spec = spec
        self._lock = threading.Lock()
        self._seen = set()
        self._settled_since: Optional[float] = None
        self._sub = ctx.node.create_subscription(
            LocalizationStatus, spec.topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg) -> None:
        with self._lock:
            self._on_msg(msg)

    def _on_msg(self, msg) -> None:
        now = self._ctx.clock.now()
        self._seen.update(c.name for c in msg.checks if c.status != _CHECK_UNKNOWN)
        if msg.state == _LOCALIZATION_STATES["NORMAL"] and not msg.in_grace:
            if self._settled_since is None:
                self._settled_since = now
        else:
            self._settled_since = None

    def poll(self) -> bool:
        with self._lock:
            if self._settled_since is None or not self._seen.issuperset(self._spec.checks):
                return False
            return self._ctx.clock.now() - self._settled_since >= self._spec.for_sec
