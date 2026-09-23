from __future__ import annotations

import time
from typing import TYPE_CHECKING

from sim_scenario_test.errors import ScenarioError

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext
    from sim_scenario_test.drivers.base import RunDriver


def wait_until_ready(ctx: "ScenarioContext", driver: "RunDriver") -> None:
    """シミュレータ・スタック・ドライバが操作可能になるまで待つ。"""
    spec = ctx.profile.readiness
    deadline = time.monotonic() + spec.timeout_sec
    log = ctx.logger

    def remaining(what: str) -> float:
        left = deadline - time.monotonic()
        if left <= 0.0:
            raise ScenarioError(
                f"readiness timeout ({spec.timeout_sec:.0f}s) while waiting for {what}")
        return left

    log.info("[ready] waiting for clock ...")
    if ctx.node.get_parameter("use_sim_time").value:
        while ctx.node.get_clock().now().nanoseconds == 0:
            remaining("/clock")
            time.sleep(0.1)

    sim_world = ctx.world_vars["sim_world"]
    log.info(f"[ready] waiting for simulator world '{sim_world}' ...")
    while not ctx.backend.is_ready():
        remaining(f"simulator world '{sim_world}' (is the world name correct?)")
        time.sleep(1.0)

    frames = ctx.profile.frames
    log.info(f"[ready] waiting for TF {frames.map} -> {frames.base} ...")
    if not ctx.poses.robot.wait_available(remaining(f"TF {frames.map} -> {frames.base}")):
        raise ScenarioError(f"TF {frames.map} -> {frames.base} not available")

    for manager in spec.lifecycle_managers:
        log.info(f"[ready] waiting for lifecycle manager '{manager}' to be active ...")
        while not ctx.nav2.lifecycle_active(manager, min(2.0, remaining(manager))):
            time.sleep(1.0)

    for service in spec.services:
        log.info(f"[ready] waiting for service {service} ...")
        while service not in {n for n, _ in ctx.node.get_service_names_and_types()}:
            remaining(f"service {service}")
            time.sleep(0.5)

    log.info("[ready] waiting for run driver ...")
    driver.wait_ready(remaining("run driver"))
    log.info("[ready] all dependencies are up")
