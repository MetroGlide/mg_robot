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
