from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from sim_scenario_test.engine.result import CheckResult, Outcome, ResultStatus
from sim_scenario_test.registry import register_expectation

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext


@register_expectation("reached_all")
def reached_all(ctx: "ScenarioContext", spec: None, outcome: Outcome) -> CheckResult:
    """すべてのゴールに到達したこと (expect 省略時の既定)。"""
    if outcome.failure is None and outcome.reached == outcome.total:
        return CheckResult("", ResultStatus.PASSED, f"{outcome.reached}/{outcome.total} reached")
    return CheckResult(
        "", ResultStatus.FAILED,
        f"{outcome.reached}/{outcome.total} reached; goal {outcome.failed_index}: "
        f"{outcome.failure}")


@dataclass
class TimeLimitSpec:
    sec: float


@register_expectation("time_limit", TimeLimitSpec)
def time_limit(ctx: "ScenarioContext", spec: TimeLimitSpec, outcome: Outcome) -> CheckResult:
    """走行が sec 秒 (sim 時間) 以内に終わったこと。"""
    finished = ctx.events.find("run_finished")
    if finished is None or ctx.run_start_time is None:
        return CheckResult("", ResultStatus.ERROR, "run did not finish")
    elapsed = finished.time - ctx.run_start_time
    status = ResultStatus.PASSED if elapsed <= spec.sec else ResultStatus.FAILED
    return CheckResult("", status, f"{elapsed:.1f}s (limit {spec.sec:.1f}s)")


@dataclass
class NavigationFailsSpec:
    # 省略時はどのゴールで失敗してもよい
    index: Optional[int] = None


@register_expectation("navigation_fails", NavigationFailsSpec)
def navigation_fails(
    ctx: "ScenarioContext", spec: NavigationFailsSpec, outcome: Outcome
) -> CheckResult:
    """走行が失敗すること (到達不能ゴール等の異常系テスト用)。"""
    if outcome.failure is None:
        return CheckResult("", ResultStatus.FAILED, "navigation unexpectedly succeeded")
    if spec.index is not None and outcome.failed_index != spec.index:
        return CheckResult(
            "", ResultStatus.FAILED,
            f"failed at goal {outcome.failed_index}, expected {spec.index}")
    return CheckResult(
        "", ResultStatus.PASSED,
        f"failed at goal {outcome.failed_index} as expected: {outcome.failure}")
