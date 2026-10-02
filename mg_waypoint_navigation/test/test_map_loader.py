"""MapLoader の単体テスト。ROS2 不要 (conftest が依存をモックする)。"""
from __future__ import annotations

from unittest.mock import MagicMock

from mg_waypoint_navigation.waypoint_sequencer import map_loader
from mg_waypoint_navigation.waypoint_sequencer.map_loader import MapLoader

SUCCESS = 0


def _loader(initial_localization="", initial_planning="", results=None):
    """results: サービス名 -> 応答の result (None は応答なし)"""
    node = MagicMock()
    params = {
        "initial_localization_map": initial_localization,
        "initial_planning_map": initial_planning,
    }
    node.declare_parameter.side_effect = lambda name, default: MagicMock(
        value=params[name])
    endpoints = MagicMock()

    def call_service(srv_type, name, request, timeout_sec):
        result = (results or {}).get(name, SUCCESS)
        if result is None:
            return None
        return MagicMock(result=result)

    endpoints.call_service.side_effect = call_service
    map_loader.LoadMap.Response.RESULT_SUCCESS = SUCCESS
    return MapLoader(node, endpoints), node, endpoints


def _published(node):
    publisher = node.create_publisher.return_value
    return [call.args[0] for call in publisher.publish.call_args_list]


def test_initial_maps_are_recorded_and_published():
    loader, node, _ = _loader("/m/loc.yaml", "/m/plan.yaml")
    assert loader.loaded == ("/m/loc.yaml", "/m/plan.yaml")
    msg = _published(node)[-1]
    assert (msg.localization, msg.planning) == ("/m/loc.yaml", "/m/plan.yaml")


def test_successful_load_updates_only_the_requested_side():
    loader, node, endpoints = _loader("/m/loc.yaml", "/m/plan.yaml")
    ok, _ = loader.load(localization="/m/loc2.yaml")
    assert ok
    assert loader.loaded == ("/m/loc2.yaml", "/m/plan.yaml")
    assert endpoints.call_service.call_count == 1
    assert endpoints.call_service.call_args.args[1] == map_loader.LOCALIZATION_MAP_SERVER


def test_failed_load_keeps_the_previous_map():
    loader, _, _ = _loader(
        "/m/loc.yaml", "/m/plan.yaml",
        results={map_loader.PLANNING_MAP_SERVER: 1})
    ok, message = loader.load(localization="/m/loc2.yaml", planning="/m/plan2.yaml")
    assert not ok
    assert "planning" in message
    assert loader.loaded == ("/m/loc2.yaml", "/m/plan.yaml")


def test_no_response_is_a_failure_and_keeps_the_previous_map():
    loader, _, _ = _loader(
        "/m/loc.yaml", results={map_loader.LOCALIZATION_MAP_SERVER: None})
    ok, _ = loader.load(localization="/m/loc2.yaml")
    assert not ok
    assert loader.loaded == ("/m/loc.yaml", "")


def test_empty_request_does_nothing():
    loader, node, endpoints = _loader("/m/loc.yaml")
    publishes_before = len(_published(node))
    ok, _ = loader.load()
    assert not ok
    endpoints.call_service.assert_not_called()
    assert len(_published(node)) == publishes_before
