from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, List

from sim_scenario_test.engine.readiness import wait_until_ready
from sim_scenario_test.engine.result import (
    CheckResult,
    Outcome,
    ResultStatus,
    ScenarioResult,
    aggregate,
)
from sim_scenario_test.engine.timeline import Timeline
from sim_scenario_test.errors import ScenarioError, ScenarioValidationError

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext
    from sim_scenario_test.drivers.base import RunDriver

_SEP = "=" * 60


class ScenarioEngine:
    """シナリオの実行順序を司る。

    readiness -> setup -> [driver.run と timeline・monitor を並行実行]
    -> teardown (+ 生成した障害物の自動削除) -> expectation 評価
    """

    def __init__(self, ctx: "ScenarioContext"):
        self._ctx = ctx

    def execute(self) -> ScenarioResult:
        ctx = self._ctx
        scenario = ctx.scenario
        log = ctx.logger
        checks: List[CheckResult] = []
        outcome = Outcome()
        wall_start = time.monotonic()
        driver: "RunDriver | None" = None
        timeline = None
        monitors: List[Any] = []
        executed = False

        self._log_header()
        try:
            driver = scenario.run.entry.impl(ctx, scenario.run.spec)
            wait_until_ready(ctx, driver)
            log.info("[setup] start")
            ctx.run_actions(scenario.setup)
            ctx.run_start_time = ctx.clock.now()
            ctx.events.emit("run_started")
            timeline = Timeline(ctx, scenario.timeline)
            for call in scenario.monitors:
                monitor = call.entry.impl(ctx, call.spec)
                monitor.start()
                monitors.append((call.name, monitor))
            timeline.start()
            log.info("[run] start")
            outcome = driver.run()
            ctx.events.emit("run_finished")
            if ctx.abort_is_error:
                raise ScenarioError(ctx.abort_reason)
            executed = True
        except (ScenarioError, ScenarioValidationError) as e:
            log.error(f"[scenario] ERROR: {e}")
            checks.append(CheckResult("execution", ResultStatus.ERROR, str(e)))
        finally:
            if timeline is not None:
                timeline.stop()
                checks.extend(timeline.checks())
            for name, monitor in monitors:
                monitor.stop()
                result = monitor.result()
                result.name = f"monitor:{name}"
                checks.append(result)
            if driver is not None:
                driver.close()
            checks.extend(self._teardown())

        if executed:
            checks.extend(self._evaluate_expectations(outcome))

        result = ScenarioResult(
            name=scenario.name,
            status=aggregate(checks),
            checks=checks,
            outcome=outcome,
            elapsed_wall_sec=time.monotonic() - wall_start,
            elapsed_sim_sec=ctx.elapsed(),
            events=ctx.events.to_list(ctx.run_start_time or 0.0),
        )
        self._log_result(result)
        return result

    def _evaluate_expectations(self, outcome: Outcome) -> List[CheckResult]:
        ctx = self._ctx
        results = []
        for call in ctx.scenario.expect:
            try:
                check = call.entry.impl(ctx, call.spec, outcome)
            except ScenarioError as e:
                check = CheckResult(call.name, ResultStatus.ERROR, str(e))
            check.name = f"expect:{call.name}"
            results.append(check)
        return results

    def _teardown(self) -> List[CheckResult]:
        ctx = self._ctx
        results = []
        if ctx.scenario.teardown:
            try:
                ctx.run_actions(ctx.scenario.teardown)
            except ScenarioError as e:
                results.append(CheckResult("teardown", ResultStatus.ERROR, str(e)))
        leftovers = ctx.spawned()
        if leftovers:
            ctx.logger.info(f"[teardown] removing spawned entities {leftovers}")
        for name in leftovers:
            try:
                ctx.backend.remove_entity(name)
                ctx.untrack_spawned(name)
            except ScenarioError as e:
                results.append(CheckResult("teardown", ResultStatus.ERROR, str(e)))
        return results

    def _log_header(self) -> None:
        ctx = self._ctx
        log = ctx.logger
        log.info(_SEP)
        log.info(f"  Scenario : {ctx.scenario.name}")
        log.info(f"  Profile  : {ctx.profile.name}")
        log.info(f"  World    : {ctx.scenario.world}")
        log.info(f"  Run      : {ctx.scenario.run.name}")
        log.info(_SEP)

    def _log_result(self, result: ScenarioResult) -> None:
        log = self._ctx.logger
        log.info(_SEP)
        line = f"  Result : {result.status.value}"
        if result.success:
            log.info(line)
        else:
            log.error(line)
        log.info(f"  Goals  : {result.outcome.reached} / {result.outcome.total} reached")
        log.info(
            f"  Time   : {result.elapsed_sim_sec:.1f} s (sim), "
            f"{result.elapsed_wall_sec:.1f} s (wall)")
        for check in result.checks:
            message = f"    [{check.status.value}] {check.name}"
            if check.message:
                message += f": {check.message}"
            if check.status == ResultStatus.PASSED:
                log.info(message)
            else:
                log.error(message)
        log.info(_SEP)
