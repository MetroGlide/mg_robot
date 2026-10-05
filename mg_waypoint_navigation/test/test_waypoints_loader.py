"""
WaypointsLoader の検証ロジックの単体テスト。
ROS2 不要: conftest.py が ROS2 依存を sys.modules でモックする。
"""
from __future__ import annotations

import pytest
import yaml

from mg_waypoint_navigation.waypoint import WaypointsLoader


def _pose(x=0.0, y=0.0):
    return {
        "position": {"x": x, "y": y, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def _load(tmp_path, waypoints):
    path = tmp_path / "waypoints.yaml"
    path.write_text(yaml.safe_dump({"version": "2.0", "waypoints": waypoints}))
    return WaypointsLoader(str(path)).load()


def test_valid_file(tmp_path):
    wl = _load(tmp_path, [
        {"index": 1, "pose": _pose(), "on_reached_actions": [
            {"type": "service", "service": "/foo",
             "srv_module": "std_srvs.srv", "srv_class": "SetBool"},
            {"type": "set_navigation_mode", "mode": "queue_wait"},
        ]},
        {"index": 0, "pose": _pose()},
    ])
    assert [w.index for w in wl.get_all()] == [0, 1]


@pytest.mark.parametrize("action", [
    {"type": "unknown"},
    {"type": "service", "service": "/foo", "srv_module": "std_srvs.srv"},
    {"type": "service", "service": "/foo",
     "srv_module": "no_such_package.srv", "srv_class": "Foo"},
    {"type": "publish", "msg_module": "std_msgs.msg", "msg_class": "String"},
    {"type": "load_map"},
    {"type": "wait", "countdown_ms": -1},
    {"type": "set_navigation_mode", "mode": "fast"},
])
def test_invalid_action_is_rejected(tmp_path, action):
    with pytest.raises(ValueError, match="waypoint index 0"):
        _load(tmp_path, [
            {"index": 0, "pose": _pose(), "on_reached_actions": [action]},
        ])


@pytest.mark.parametrize("indices", [[0, 2], [0, 0], [1, 2]])
def test_non_contiguous_indices_are_rejected(tmp_path, indices):
    with pytest.raises(ValueError, match="indices"):
        _load(tmp_path, [{"index": i, "pose": _pose()} for i in indices])


def test_no_warnings_for_well_spaced_waypoints(tmp_path):
    wl = _load(tmp_path, [
        {"index": 0, "pose": _pose(0.0)},
        {"index": 1, "pose": _pose(10.0),
         "navigation": {"is_through_point": False}},
    ])
    assert wl.warnings == []


def test_warns_close_through_point(tmp_path):
    wl = _load(tmp_path, [
        {"index": 0, "pose": _pose(0.0)},
        {"index": 1, "pose": _pose(2.0)},
        {"index": 2, "pose": _pose(20.0),
         "navigation": {"is_through_point": False}},
    ])
    assert len(wl.warnings) == 1
    assert "waypoint 1" in wl.warnings[0]


def test_warns_through_point_with_actions_and_last(tmp_path):
    wl = _load(tmp_path, [
        {"index": 0, "pose": _pose(0.0),
         "on_reached_actions": [{"type": "amcl_reset"}]},
        {"index": 1, "pose": _pose(10.0)},
    ])
    assert any("on_reached_actions" in w for w in wl.warnings)
    assert any("last waypoint 1" in w for w in wl.warnings)
