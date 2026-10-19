"""汎用の service アクションの単体テスト。ROS2 不要 (conftest が依存をモックする)。"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from mg_waypoint_navigation.waypoint import ActionConfig
from mg_waypoint_navigation.waypoint_sequencer.actions.base import ActionFailed
from mg_waypoint_navigation.waypoint_sequencer.actions.generic import GenericServiceAction


def _action(response):
    endpoints = MagicMock()
    endpoints.call_service.return_value = response
    config = ActionConfig(
        type="service", service="/localization_supervisor_node/set_profile",
        srv_module="std_srvs.srv", srv_class="SetBool", request={"data": True})
    return GenericServiceAction(config, MagicMock(), endpoints), endpoints


def test_a_successful_response_is_not_a_failure():
    action, endpoints = _action(SimpleNamespace(success=True, message="ok"))
    action.execute()
    endpoints.call_service.assert_called_once()


def test_a_response_without_success_is_not_a_failure():
    # Empty のように、success を持たないサービス
    action, _ = _action(SimpleNamespace())
    action.execute()


def test_a_rejected_request_is_a_failure_with_the_reason():
    action, _ = _action(SimpleNamespace(success=False, message="unknown profile 'x'"))
    with pytest.raises(ActionFailed, match="unknown profile 'x'"):
        action.execute()


def test_no_response_is_still_a_failure():
    action, _ = _action(None)
    with pytest.raises(ActionFailed, match="did not respond"):
        action.execute()
