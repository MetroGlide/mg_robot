"""ScenarioEngine の結合テスト (フェイクのバックエンド・ドライバ・時計を使用)。"""
from __future__ import annotations

import pytest

from fakes import FakeBackend, make_context
from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.engine.runner import ScenarioEngine
from sim_scenario_test.loader import parse_scenario
from sim_scenario_test.registry import DEFAULT_REGISTRY


def _scenario(run=None, **extra):
    raw = {
        "version": "2.0",
        "name": "engine_test",
        "world": "w",
        "obstacles": {"box": {"model": {"type": "primitive", "shape": "box"}}},
        "run": {"fake_goals": run or {}},
    }
    raw.update(extra)
    return parse_scenario(raw, "scenario", DEFAULT_REGISTRY)


def _execute(scenario, backend=None):
    backend = backend or FakeBackend()
    result = ScenarioEngine(make_context(scenario, backend=backend)).execute()
    return result, backend


def _statuses(result):
    return {c.name: c.status for c in result.checks}


def test_pass_with_timeline_and_auto_cleanup():
    scenario = _scenario(timeline=[{
        "name": "spawn_box",
        "when": {"goal_started": {"index": 1}},
        "do": [{"spawn": {"obstacle": "box", "pose": {"frame": "robot", "x": 1.0}}}],
    }])
    result, backend = _execute(scenario)
    assert result.status == ResultStatus.PASSED, result.checks
    assert result.outcome.reached == 2
    assert [c[0] for c in backend.calls] == ["spawn", "remove"]
    assert _statuses(result)["spawn_box"] == ResultStatus.PASSED


def test_navigation_failure_is_failed():
    result, _ = _execute(_scenario(run={"fail_at": 1}))
    assert result.status == ResultStatus.FAILED
    assert _statuses(result)["expect:reached_all"] == ResultStatus.FAILED
    assert result.outcome.failed_index == 1


def test_expected_failure_passes():
    scenario = _scenario(run={"fail_at": 0}, expect=[{"navigation_fails": {"index": 0}}])
    result, _ = _execute(scenario)
    assert result.status == ResultStatus.PASSED


def test_required_timeline_entry_never_fired_is_error():
    scenario = _scenario(timeline=[{
        "name": "never",
        "when": {"goal_started": {"index": 5}},
        "do": [{"delay": {"sec": 0.1}}],
    }])
    result, _ = _execute(scenario)
    assert result.status == ResultStatus.ERROR
    assert _statuses(result)["never"] == ResultStatus.ERROR


def test_optional_timeline_entry_may_not_fire():
    scenario = _scenario(timeline=[{
        "when": {"goal_started": {"index": 5}},
        "required": False,
        "do": [{"delay": {"sec": 0.1}}],
    }])
    result, _ = _execute(scenario)
    assert result.status == ResultStatus.PASSED


def test_setup_failure_is_error_and_skips_expectations():
    scenario = _scenario(setup=[{"spawn": {"obstacle": "box", "pose": {}}}])
    result, _ = _execute(scenario, FakeBackend(fail_spawn=True))
    assert result.status == ResultStatus.ERROR
    assert "expect:reached_all" not in _statuses(result)
    assert "spawn failed" in _statuses_message(result, "execution")


def test_timeline_action_failure_aborts_run_as_error():
    scenario = _scenario(
        run={"hang": True},
        timeline=[{"when": {"at_time": {"sec": 0.5}},
                   "do": [{"spawn": {"obstacle": "box", "pose": {}}}]}])
    result, _ = _execute(scenario, FakeBackend(fail_spawn=True))
    assert result.status == ResultStatus.ERROR
    assert _statuses(result)["timeline"] == ResultStatus.ERROR


def test_scenario_timeout_is_failed():
    result, _ = _execute(_scenario(run={"hang": True}, timeout=1.0))
    assert result.status == ResultStatus.FAILED
    assert "timeout" in result.outcome.failure


def test_until_interrupts_running_entry():
    scenario = _scenario(
        run={"goals": 1, "goal_sec": 1.0},
        timeline=[{
            "name": "long",
            "when": {"goal_started": {"index": 0}},
            "until": {"goal_reached": {"index": 0}},
            "do": [{"delay": {"sec": 100.0}},
                   {"spawn": {"obstacle": "box", "pose": {}}}],
        }])
    result, backend = _execute(scenario)
    assert result.status == ResultStatus.PASSED
    assert backend.calls == []
    assert "interrupted" in _statuses_message(result, "long")


def test_time_limit():
    ok, _ = _execute(
        _scenario(run={"goals": 1}, expect=["reached_all", {"time_limit": {"sec": 60}}]))
    ng, _ = _execute(_scenario(run={"goals": 1}, expect=[{"time_limit": {"sec": 0.5}}]))
    assert ok.status == ResultStatus.PASSED
    assert ng.status == ResultStatus.FAILED


def test_result_to_dict():
    result, _ = _execute(_scenario(run={"goals": 1}))
    data = result.to_dict()
    assert data["status"] == "PASSED"
    names = [e["name"] for e in data["events"]]
    assert names[:3] == ["run_started", "goal_started", "goal_reached"]


def test_stalled_clock_is_error(monkeypatch):
    scenario = _scenario(run={"hang": True})
    ctx = make_context(scenario)
    frozen = ctx.clock.now()
    monkeypatch.setattr(ctx.clock, "now", lambda: frozen)
    ctx.profile.sim.clock_stall_sec = 0.2
    result = ScenarioEngine(ctx).execute()
    assert result.status == ResultStatus.ERROR
    assert "stalled" in _statuses_message(result, "execution")


def _statuses_message(result, name):
    return next(c.message for c in result.checks if c.name == name)


@pytest.fixture(autouse=True)
def _fast_poll(monkeypatch):
    monkeypatch.setattr("sim_scenario_test.engine.timeline._LOOP_SEC", 0.005)
