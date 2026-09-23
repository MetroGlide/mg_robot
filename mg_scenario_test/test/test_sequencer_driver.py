"""MgSequencerDriver の進行制御の単体テスト (SequencerClient をフェイクに置換)。"""
from __future__ import annotations

from types import SimpleNamespace
from typing import List
from unittest.mock import MagicMock

import pytest

from mg_scenario_test.plugins import MgSequencerDriver, MgSequencerSpec, WaypointHooks
from mg_scenario_test.sequencer_tracker import Progress
from sim_scenario_test.errors import ScenarioError


class FakeClient:
    def __init__(self, progresses: List[Progress], states: List[str]):
        self._progresses = list(progresses)
        self._states = list(states)
        self.state = "IDLE"
        self.index = 0
        self.calls: List[tuple] = []

    def fetch_waypoints(self):
        pose = SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        return [SimpleNamespace(index=i, pose=pose) for i in range(3)]

    def stop(self):
        self.calls.append(("stop",))

    def set_next_index(self, index):
        self.calls.append(("set_index", index))

    def begin_run(self):
        self.calls.append(("begin",))

    def start(self, countdown_ms):
        self.calls.append(("start", countdown_ms))

    def wait_passed(self, expected, timeout, abort_event, clock_now):
        self.calls.append(("wait", expected))
        progress = self._progresses.pop(0)
        if self._states:
            self.state = self._states.pop(0)
        return progress


def _driver(client, **spec_kwargs):
    ctx = MagicMock()
    ctx.abort_event.is_set.return_value = False
    spec = MgSequencerSpec(**spec_kwargs)
    ctx.extensions = {f"mg_sequencer:{spec.namespace}": client}
    ctx.run_actions.return_value = True
    return MgSequencerDriver(ctx, spec), ctx


def test_all_reached_starts_only_when_idle():
    client = FakeClient([Progress.REACHED] * 3, states=["IDLE", "NAVIGATING", "GOAL_REACHED"])
    driver, _ = _driver(client, countdown_ms=10, hooks={1: WaypointHooks(countdown_ms=3000)})
    outcome = driver.run()
    assert (outcome.total, outcome.reached, outcome.failure) == (3, 3, None)
    assert client.calls[:3] == [("stop",), ("set_index", 0), ("begin",)]
    starts = [c for c in client.calls if c[0] == "start"]
    # 開始時 (countdown 既定値) と、waypoint 0 通過後に IDLE だった waypoint 1 (個別 countdown)
    assert starts == [("start", 10), ("start", 3000)]


def test_start_index_skips_earlier_waypoints():
    client = FakeClient([Progress.REACHED] * 2, states=["NAVIGATING", "GOAL_REACHED"])
    driver, _ = _driver(client, start_index=1)
    outcome = driver.run()
    assert outcome.reached == 2
    assert ("set_index", 1) in client.calls
    assert [c for c in client.calls if c[0] == "wait"] == [("wait", 1), ("wait", 2)]


def test_sequencer_error_is_navigation_failure():
    client = FakeClient([Progress.REACHED, Progress.ERROR], states=["NAVIGATING", "ERROR"])
    driver, ctx = _driver(client)
    outcome = driver.run()
    assert outcome.failed_index == 1
    assert "ERROR" in outcome.failure


def test_stall_is_scenario_error():
    client = FakeClient([Progress.STALLED], states=["IDLE"])
    driver, _ = _driver(client)
    with pytest.raises(ScenarioError, match="IDLE"):
        driver.run()


def test_timeout_stops_sequencer():
    client = FakeClient([Progress.WAITING], states=["NAVIGATING"])
    driver, _ = _driver(client)
    outcome = driver.run()
    assert "not passed" in outcome.failure
    assert client.calls[-1] == ("stop",)


def test_unknown_hook_index_is_error():
    client = FakeClient([], states=[])
    driver, _ = _driver(client, hooks={7: WaypointHooks()})
    with pytest.raises(ScenarioError, match="unknown waypoint index"):
        driver.run()


def test_manual_start_mode_does_not_call_start():
    client = FakeClient([Progress.REACHED] * 3, states=["IDLE", "IDLE", "GOAL_REACHED"])
    driver, _ = _driver(client, auto_start=False)
    driver.run()
    assert not any(c[0] in ("start", "stop", "set_index") for c in client.calls)
