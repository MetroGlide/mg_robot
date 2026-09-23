"""シナリオ YAML の読み込み・検証の単体テスト。"""
from __future__ import annotations

import copy

import pytest

import fakes  # noqa: F401  fake_goals ドライバを登録する
from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.loader import parse_scenario
from sim_scenario_test.registry import DEFAULT_REGISTRY

BASE = {
    "version": "2.0",
    "name": "t",
    "world": "w",
    "obstacles": {"box": {"model": {"type": "primitive", "shape": "box"}}},
    "setup": [{"respawn": {"pose": {"x": 1.0}}}],
    "run": {"nav2_goals": {"goals": [{"pose": {"x": 1.0}}]}},
    "timeline": [{
        "when": {"goal_started": {"index": 0}},
        "do": [{"delay": {"sec": 1.0}}, {"spawn": {"obstacle": "box", "pose": {"frame": "robot"}}}],
    }],
}


def _parse(**changes):
    raw = copy.deepcopy(BASE)
    raw.update(changes)
    return parse_scenario(raw, "scenario", DEFAULT_REGISTRY)


def test_valid_scenario_defaults_to_reached_all():
    s = _parse()
    assert s.run.name == "nav2_goals"
    assert [c.name for c in s.expect] == ["reached_all"]
    assert s.timeline[0].do[1].spec.pose.frame == "robot"


def test_version_is_checked():
    with pytest.raises(ScenarioValidationError, match="version"):
        _parse(version="1.0")


def test_unknown_action_lists_available():
    with pytest.raises(ScenarioValidationError, match="available: .*spawn"):
        _parse(setup=[{"reset_pose": {}}])


def test_undefined_obstacle():
    with pytest.raises(ScenarioValidationError, match="ghost"):
        _parse(setup=[{"despawn": {"obstacle": "ghost"}}])


def test_undefined_obstacle_in_driver_hooks():
    run = {"nav2_goals": {"goals": [
        {"pose": {}, "before": [{"despawn": {"obstacle": "ghost"}}]}]}}
    with pytest.raises(ScenarioValidationError, match="ghost"):
        _parse(run=run)


def test_nav2_goals_requires_exactly_one_source():
    with pytest.raises(ScenarioValidationError, match="exactly one"):
        _parse(run={"nav2_goals": {}})
    with pytest.raises(ScenarioValidationError, match="exactly one"):
        _parse(run={"nav2_goals": {"goals": [{"pose": {}}], "waypoints_file": "a.yaml"}})


def test_robot_near_rejects_robot_frame():
    timeline = [{"when": {"robot_near": {"x": 0, "y": 0, "radius": 1, "frame": "robot"}}}]
    with pytest.raises(ScenarioValidationError, match="frame"):
        _parse(timeline=timeline)


def test_negative_delay():
    with pytest.raises(ScenarioValidationError, match="sec"):
        _parse(setup=[{"delay": {"sec": -1}}])
