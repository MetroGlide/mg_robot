"""
WaypointNavigator のゴール世代管理・キャンセルの単体テスト。
ROS2 不要: conftest.py が ROS2 依存を sys.modules でモックする。
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from mg_waypoint_navigation.waypoint import NavigationConfig, Waypoint
from mg_waypoint_navigation.waypoint_sequencer import navigator as navigator_module
from mg_waypoint_navigation.waypoint_sequencer.navigator import (
    NavigationResult,
    WaypointNavigator,
)

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


def _waypoint(index=0, is_through_point=False, reach_tolerance=0.5):
    pose = MagicMock()
    pose.pose.position.x = 10.0
    pose.pose.position.y = 0.0
    return Waypoint(
        index=index,
        pose=pose,
        navigation=NavigationConfig(
            reach_tolerance=reach_tolerance,
            through_tolerance=3.0,
            is_through_point=is_through_point,
        ),
    )


class _Future:
    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def complete(self, result):
        self._result = result
        for cb in self.callbacks:
            cb(self)

    def result(self):
        return self._result


@pytest.fixture
def env():
    node = MagicMock()
    params_client = MagicMock()
    params_client.service_is_ready.return_value = False
    node.create_client.return_value = params_client

    with (
        patch.object(navigator_module, "ActionClient") as MockClient,
        patch.object(navigator_module, "GoalStatus", SimpleNamespace(
            STATUS_SUCCEEDED=STATUS_SUCCEEDED,
            STATUS_CANCELED=STATUS_CANCELED,
        )),
    ):
        action_client = MagicMock()
        action_client.wait_for_server.return_value = True
        MockClient.return_value = action_client

        send_futures: list = []
        feedback_cbs: list = []

        def _send_goal_async(goal, feedback_callback):
            future = _Future()
            send_futures.append(future)
            feedback_cbs.append(feedback_callback)
            return future

        action_client.send_goal_async.side_effect = _send_goal_async

        nav = WaypointNavigator(node)
        results: list = []

        def accept(i=-1):
            handle = MagicMock()
            handle.accepted = True
            result_future = _Future()
            handle.get_result_async.return_value = result_future
            send_futures[i].complete(handle)
            return handle, result_future

        yield SimpleNamespace(
            nav=nav,
            params_client=params_client,
            send_futures=send_futures,
            feedback_cbs=feedback_cbs,
            results=results,
            on_result=results.append,
            accept=accept,
        )


def _finish(result_future, status):
    result_future.complete(SimpleNamespace(status=status))


def test_success(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    _, result_future = env.accept()
    _finish(result_future, STATUS_SUCCEEDED)
    assert env.results == [NavigationResult.SUCCEEDED]


def test_abort_is_failed(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    _, result_future = env.accept()
    _finish(result_future, STATUS_ABORTED)
    assert env.results == [NavigationResult.FAILED]


def test_cancel_before_acceptance_cancels_accepted_goal(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    env.nav.cancel()
    handle, _ = env.accept()
    handle.cancel_goal_async.assert_called_once()
    handle.get_result_async.assert_not_called()
    assert env.results == []


def test_canceled_goal_result_is_not_reported(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    handle, result_future = env.accept()
    env.nav.cancel()
    handle.cancel_goal_async.assert_called_once()
    _finish(result_future, STATUS_CANCELED)
    assert env.results == []


def test_stale_result_does_not_clear_new_goal_handle(env):
    env.nav.send_goal(_waypoint(0), env.on_result)
    _, old_result = env.accept(0)
    env.nav.cancel()
    env.nav.send_goal(_waypoint(1), env.on_result)
    new_handle, _ = env.accept(1)

    _finish(old_result, STATUS_ABORTED)
    assert env.results == []

    env.nav.cancel()
    new_handle.cancel_goal_async.assert_called_once()


def test_through_point_cancel_is_success(env):
    env.nav.send_goal(_waypoint(is_through_point=True), env.on_result)
    handle, result_future = env.accept()

    def feedback(distance, x):
        msg = MagicMock()
        msg.feedback.distance_remaining = distance
        msg.feedback.current_pose.pose.position.x = x
        msg.feedback.current_pose.pose.position.y = 0.0
        env.feedback_cbs[-1](msg)

    feedback(10.0, 0.0)  # 経路計算完了の検出
    feedback(2.0, 8.0)
    handle.cancel_goal_async.assert_called_once()
    _finish(result_future, STATUS_CANCELED)
    assert env.results == [NavigationResult.SUCCEEDED]


def test_server_unavailable_is_failed(env):
    env.nav._action_client.wait_for_server.return_value = False
    env.nav.send_goal(_waypoint(), env.on_result)
    assert env.results == [NavigationResult.FAILED]
