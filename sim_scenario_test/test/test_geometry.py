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


# ---------------------------------------------------------------------------
# 多角形・円の距離
# ---------------------------------------------------------------------------

from sim_scenario_test.geometry import (  # noqa: E402
    point_in_polygon,
    polygon_circle_distance,
    polygon_polygon_distance,
    rectangle_vertices,
    transform_polygon,
)

_FOOTPRINT = [(0.4, 0.3), (0.4, -0.3), (-0.2, -0.3), (-0.2, 0.3)]


def test_transform_polygon_rotates_and_translates():
    moved = transform_polygon([(1.0, 0.0), (0.0, 1.0)], Pose(1.0, 2.0, 0.0, math.pi / 2))
    assert moved[0] == pytest.approx((1.0, 3.0))
    assert moved[1] == pytest.approx((0.0, 2.0))


def test_point_in_polygon():
    assert point_in_polygon((0.0, 0.0), _FOOTPRINT)
    assert not point_in_polygon((0.5, 0.0), _FOOTPRINT)


@pytest.mark.parametrize("center, expected", [
    ((0.0, 0.0), 0.0),        # 内部
    ((0.4 + 0.25, 0.0), 0.0),  # 円が前端に接する
    ((1.0, 0.0), 0.35),       # 前方: 1.0 - 0.4 - 0.25
    ((0.0, 1.0), 0.45),       # 側方: 1.0 - 0.3 - 0.25
    ((0.4 + 3.0, 0.3 + 4.0), 5.0 - 0.25),  # 前方の角
])
def test_polygon_circle_distance(center, expected):
    assert polygon_circle_distance(_FOOTPRINT, center, 0.25) == pytest.approx(expected)


def test_polygon_polygon_distance():
    box = rectangle_vertices(1.0, 1.0)
    assert polygon_polygon_distance(
        _FOOTPRINT, transform_polygon(box, Pose(1.4, 0.0))) == pytest.approx(0.5 - 0.5 + 0.5)
    # 内包・重なり
    assert polygon_polygon_distance(_FOOTPRINT, transform_polygon(box, Pose(0.0, 0.0))) == 0.0
    # 頂点がどちらの内側にもない十字の交差
    cross_a = rectangle_vertices(2.0, 0.2)
    cross_b = rectangle_vertices(0.2, 2.0)
    assert polygon_polygon_distance(cross_a, cross_b) == 0.0
