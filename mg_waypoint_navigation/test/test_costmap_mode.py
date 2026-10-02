"""CostmapModeSwitcher の単体テスト。ROS2 不要 (conftest が依存をモックする)。"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from mg_waypoint_navigation.waypoint_sequencer import costmap_mode
from mg_waypoint_navigation.waypoint_sequencer.costmap_mode import (
    CostmapModeSwitcher,
)


@pytest.fixture
def env(monkeypatch):
    # Parameter はモックでは属性を共有してしまうので、値を持つ単純なオブジェクトに置き換える
    monkeypatch.setattr(
        costmap_mode, "Parameter",
        lambda: SimpleNamespace(name="", value=SimpleNamespace(
            type=None, bool_value=None)))
    monkeypatch.setattr(
        costmap_mode.SetParametersAtomically, "Request",
        lambda: SimpleNamespace(parameters=[]), raising=False)
    endpoints = MagicMock()
    switcher = CostmapModeSwitcher(MagicMock(), endpoints)
    return SimpleNamespace(switcher=switcher, endpoints=endpoints)


def _respond(env, response):
    env.endpoints.call_service.return_value = response


def test_sets_both_sensor_layers_and_reports_success(env):
    _respond(env, SimpleNamespace(result=SimpleNamespace(successful=True)))

    ok, message = env.switcher.set_global_obstacle_layers(False)

    assert ok
    assert "False" in message
    _, service, request = env.endpoints.call_service.call_args.args
    assert service == "/global_costmap/global_costmap/set_parameters_atomically"
    assert [(p.name, p.value.bool_value) for p in request.parameters] == [
        ("top_obstacle_layer.enabled", False),
        ("obstacle_stvl_layer.enabled", False),
    ]


def test_enabling_sets_true(env):
    _respond(env, SimpleNamespace(result=SimpleNamespace(successful=True)))
    env.switcher.set_global_obstacle_layers(True)
    request = env.endpoints.call_service.call_args.args[2]
    assert all(p.value.bool_value is True for p in request.parameters)


def test_rejected_by_the_costmap_is_a_failure(env):
    _respond(env, SimpleNamespace(result=SimpleNamespace(successful=False)))
    ok, message = env.switcher.set_global_obstacle_layers(False)
    assert not ok
    assert "failed" in message


def test_no_response_is_a_failure(env):
    _respond(env, None)
    ok, _ = env.switcher.set_global_obstacle_layers(True)
    assert not ok
