from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Optional

from sim_scenario_test.engine.result import CheckResult, Outcome, ResultStatus
from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import PoseSpec
from sim_scenario_test.registry import register_expectation

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext


@dataclass
class ReachedAllSpec:
    # 走行の終わりの、推定と真の姿勢の位置の差の上限 [m]。推定の上でゴールに着いても、実際のロボットが着いていない
    # (障害物に当たって車輪だけ回るなど) 回を落とす。真の姿勢を出せないバックエンドでは見ない。null なら見ない
    max_end_error_m: Optional[float] = 3.0


@register_expectation("reached_all", ReachedAllSpec)
def reached_all(ctx: "ScenarioContext", spec: ReachedAllSpec, outcome: Outcome) -> CheckResult:
    """すべてのゴールに、実際のロボットも着いたこと (expect 省略時の既定)。"""
    if outcome.failure is not None or outcome.reached != outcome.total:
        return CheckResult(
            "", ResultStatus.FAILED,
            f"{outcome.reached}/{outcome.total} reached; goal {outcome.failed_index}: "
            f"{outcome.failure}")
    message = f"{outcome.reached}/{outcome.total} reached"
    error = None if spec.max_end_error_m is None else _end_error(ctx)
    if error is not None and error > spec.max_end_error_m:
        return CheckResult("", ResultStatus.FAILED,
                           f"{message} by the estimate, but the robot is {error:.2f} m away from it "
                           f"(limit {spec.max_end_error_m})")
    return CheckResult("", ResultStatus.PASSED, message)


def _end_error(ctx: "ScenarioContext") -> Optional[float]:
    """走行の終わりの推定と真の姿勢の位置の差 [m]。真の姿勢か推定を取れなければ None。"""
    try:
        world_pose = ctx.backend.entity_pose(ctx.profile.sim.robot_entity)
        estimate = ctx.poses.robot.get()
    except ScenarioError:
        return None
    if world_pose is None:
        return None
    truth = ctx.poses.to_map(PoseSpec("world", world_pose.x, world_pose.y, world_pose.z, world_pose.yaw))
    return estimate.distance_xy(truth)


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
class FinalPoseErrorSpec:
    x: float
    y: float
    tolerance: float = 0.5
    frame: Literal["map", "world"] = "map"


@register_expectation("final_pose_error", FinalPoseErrorSpec)
def final_pose_error(
    ctx: "ScenarioContext", spec: FinalPoseErrorSpec, outcome: Outcome
) -> CheckResult:
    """走行後のロボット位置が、指定した点から tolerance [m] 以内にあること (停止精度)。"""
    target = ctx.poses.to_map(PoseSpec(frame=spec.frame, x=spec.x, y=spec.y))
    error = ctx.poses.robot.get().distance_xy(target)
    status = ResultStatus.PASSED if error <= spec.tolerance else ResultStatus.FAILED
    return CheckResult("", status, f"error {error:.2f} m (tolerance {spec.tolerance} m)")


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
