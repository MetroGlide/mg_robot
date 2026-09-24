"""monitor・spawn 確認・respawn 収束確認の単体テスト。"""
from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from fakes import FakeBackend, FakeRobot, make_context
from sim_scenario_test.builtin.actions import RespawnSpec, SpawnSpec, respawn, spawn
from sim_scenario_test.builtin.monitors import (
    BtNodeMonitor,
    BtNodeSpec,
    MinScanRangeMonitor,
    MinScanRangeSpec,
    NoDiagnosticErrorsMonitor,
    NoDiagnosticErrorsSpec,
)
from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose, PoseSpec


def _bt_msg(*events):
    return SimpleNamespace(event_log=[
        SimpleNamespace(node_name=n, current_status=s) for n, s in events])


@pytest.mark.parametrize("expect, events, status", [
    ("occurs", [("BackUp", "RUNNING")], ResultStatus.PASSED),
    ("occurs", [("BackUp", "SUCCESS")], ResultStatus.FAILED),
    ("never", [], ResultStatus.PASSED),
    ("never", [("Wait", "RUNNING"), ("BackUp", "RUNNING")], ResultStatus.FAILED),
])
def test_bt_node_monitor(expect, events, status):
    monitor = BtNodeMonitor(MagicMock(), BtNodeSpec(node="BackUp", expect=expect))
    monitor._callback(_bt_msg(*events))
    assert monitor.result().status == status


def _scan(*ranges):
    return SimpleNamespace(ranges=list(ranges), range_min=0.05, range_max=10.0)


def test_min_scan_range_monitor():
    monitor = MinScanRangeMonitor(MagicMock(), MinScanRangeSpec(min_range=0.2))
    assert monitor.result().status == ResultStatus.ERROR
    monitor._callback(_scan(float("inf"), 1.0, 0.01))
    monitor._callback(_scan(0.5))
    assert monitor.result().status == ResultStatus.PASSED
    monitor._callback(_scan(0.1))
    result = monitor.result()
    assert result.status == ResultStatus.FAILED and "0.10" in result.message


def _diag(*items):
    return SimpleNamespace(status=[
        SimpleNamespace(name=n, level=lv, message="bad") for n, lv in items])


def test_diagnostics_monitor():
    monitor = NoDiagnosticErrorsMonitor(MagicMock(), NoDiagnosticErrorsSpec(names=["lidar"]))
    monitor._callback(_diag(("gps", 2), ("lidar", 1)))
    assert monitor.result().status == ResultStatus.PASSED
    monitor._callback(_diag(("lidar", 2)))
    assert monitor.result().status == ResultStatus.FAILED


def test_monitor_stop_destroys_subscription():
    ctx = MagicMock()
    monitor = BtNodeMonitor(ctx, BtNodeSpec(node="x"))
    monitor.start()
    monitor.stop()
    ctx.node.destroy_subscription.assert_called_once()


def _scenario_ctx(backend=None, robot=None):
    from sim_scenario_test.loader import parse_scenario
    from sim_scenario_test.registry import DEFAULT_REGISTRY
    scenario = parse_scenario({
        "version": "2.0", "name": "t", "world": "w",
        "obstacles": {"box": {"model": {"type": "primitive", "shape": "box"}}},
        "run": {"fake_goals": {}},
    }, "t", DEFAULT_REGISTRY)
    return make_context(scenario, backend=backend, robot=robot)


def test_spawn_verifies_entity_exists(monkeypatch):
    class GhostBackend(FakeBackend):
        def entity_exists(self, name):
            return False

    ctx = _scenario_ctx(GhostBackend())
    monkeypatch.setattr("sim_scenario_test.builtin.actions.time.sleep", lambda s: None)
    times = iter(range(0, 1000, 3))
    monkeypatch.setattr("sim_scenario_test.builtin.actions.time.monotonic", lambda: next(times))
    with pytest.raises(ScenarioError, match="does not exist"):
        spawn(ctx, SpawnSpec("box", PoseSpec()), threading.Event())


def test_respawn_waits_for_convergence():
    robot = FakeRobot(Pose(5.0, 5.0))
    ctx = _scenario_ctx(robot=robot)
    spec = RespawnSpec(pose=PoseSpec(x=0.0), settle_sec=0.0, converge_timeout_sec=1.0)
    with pytest.raises(ScenarioError, match="did not converge"):
        respawn(ctx, spec, threading.Event())
    robot.pose = Pose(0.2, 0.0)
    respawn(ctx, spec, threading.Event())
    ctx.nav2.clear_costmaps.assert_called()


# ---------------------------------------------------------------------------
# P5: move_obstacle / jitter / robot_travelled / call_set_bool
# ---------------------------------------------------------------------------

def test_move_obstacle_interpolates_and_ends_at_target():
    from sim_scenario_test.builtin.actions import MoveObstacleSpec, move_obstacle
    backend = FakeBackend()
    ctx = _scenario_ctx(backend)
    spawn(ctx, SpawnSpec("box", PoseSpec(x=0.0, y=0.0)), threading.Event())
    # FakeClock は実時間の 10 倍速: 2 m を 5 m/s (実 0.04 s) で移動
    move_obstacle(ctx, MoveObstacleSpec("box", PoseSpec(x=2.0, y=0.0), speed=5.0, rate_hz=200.0),
                  threading.Event())
    poses = [c[2] for c in backend.calls if c[0] == "set_pose"]
    assert poses[-1].x == pytest.approx(2.0)
    assert all(a.x <= b.x for a, b in zip(poses, poses[1:]))
    assert ctx.entity_poses["box"].x == pytest.approx(2.0)


def test_move_obstacle_requires_spawned_entity():
    from sim_scenario_test.builtin.actions import MoveObstacleSpec, move_obstacle
    with pytest.raises(ScenarioError, match="not been spawned"):
        move_obstacle(_scenario_ctx(), MoveObstacleSpec("box", PoseSpec()), threading.Event())


def test_move_obstacle_can_be_interrupted():
    from sim_scenario_test.builtin.actions import MoveObstacleSpec, move_obstacle
    backend = FakeBackend()
    ctx = _scenario_ctx(backend)
    spawn(ctx, SpawnSpec("box", PoseSpec()), threading.Event())
    stop = threading.Event()
    stop.set()
    move_obstacle(ctx, MoveObstacleSpec("box", PoseSpec(x=100.0), speed=0.1), stop)
    assert len([c for c in backend.calls if c[0] == "set_pose"]) == 1


def test_spawn_jitter_is_reproducible():
    def spawn_x(seed):
        backend = FakeBackend()
        ctx = _scenario_ctx(backend)
        ctx.rng.seed(seed)
        spawn(ctx, SpawnSpec("box", PoseSpec(x=1.0), jitter=0.5), threading.Event())
        return [c[2] for c in backend.calls if c[0] == "spawn"][0].x

    assert spawn_x(1) == spawn_x(1)
    assert spawn_x(1) != spawn_x(2)
    assert 0.5 <= spawn_x(1) <= 1.5


def test_robot_travelled_accumulates_path_length():
    from sim_scenario_test.builtin.triggers import RobotTravelled, RobotTravelledSpec
    robot = FakeRobot(Pose(0.0, 0.0))
    trigger = RobotTravelled(_scenario_ctx(robot=robot), RobotTravelledSpec(distance=2.0))
    assert not trigger.poll()
    robot.pose = Pose(1.0, 0.0)
    assert not trigger.poll()
    robot.pose = Pose(1.0, 1.5)
    assert trigger.poll()


def test_call_set_bool(monkeypatch):
    from sim_scenario_test.builtin import actions
    ctx = _scenario_ctx()
    calls = []

    def fake_call(node, client, request, timeout):
        calls.append(request.data)
        return SimpleNamespace(success=True, message="")

    monkeypatch.setattr(actions, "call_service", fake_call)
    monkeypatch.setattr(actions, "SetBool", MagicMock())
    actions.call_set_bool(ctx, actions.CallSetBoolSpec("/x", False), threading.Event())
    ctx.node.create_client.assert_called_once()
    assert calls == [False]
