"""座標変換・姿勢解決の単体テスト。"""
from __future__ import annotations

import math

import pytest

from fakes import FakeRobot
from sim_scenario_test.geometry import Pose, PoseSpec
from sim_scenario_test.poses import PoseResolver


def _approx(pose: Pose, x, y, yaw):
    assert pose.x == pytest.approx(x, abs=1e-9)
    assert pose.y == pytest.approx(y, abs=1e-9)
    assert pose.yaw == pytest.approx(yaw, abs=1e-9)


def test_compose_and_inverse():
    a = Pose(1.0, 2.0, 0.0, math.pi / 2)
    b = Pose(2.0, 1.0, 0.0, 0.1)
    _approx(a.compose(b), 0.0, 4.0, math.pi / 2 + 0.1)
    _approx(a.compose(a.inverse()), 0.0, 0.0, 0.0)


def test_resolver_frames():
    map_in_world = Pose(10.0, 0.0, 0.0, math.pi / 2)
    robot = FakeRobot(Pose(1.0, 0.0, 0.0, math.pi))
    r = PoseResolver(map_in_world, robot)
    _approx(r.to_map(PoseSpec("map", 1.0, 2.0)), 1.0, 2.0, 0.0)
    _approx(r.to_world(PoseSpec("map", 1.0, 0.0)), 10.0, 1.0, math.pi / 2)
    _approx(r.to_map(PoseSpec("world", 10.0, 1.0)), 1.0, 0.0, -math.pi / 2)
    _approx(r.to_map(PoseSpec("robot", 0.5)), 0.5, 0.0, math.pi)
    _approx(r.to_world(PoseSpec("robot", 0.5)), 10.0, 0.5, -math.pi / 2)
