"""
WaypointNavigator のゴール管理・通過点判定・reach_tolerance 反映の単体テスト。
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

BT_NORMAL = "normal.xml"
BT_QUEUE_WAIT = "queue_wait.xml"


def _point(x, y):
    return SimpleNamespace(x=x, y=y)


def _waypoint(index=0, x=10.0, y=0.0, is_through_point=False,
              reach_tolerance=0.5, through_tolerance=3.0):
    pose = SimpleNamespace(pose=SimpleNamespace(position=_point(x, y)))
    return Waypoint(
        index=index,
        pose=pose,
        navigation=NavigationConfig(
            reach_tolerance=reach_tolerance,
            through_tolerance=through_tolerance,
            is_through_point=is_through_point,
        ),
    )


def _path(points, stamp_sec=1):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=0)),
        poses=[SimpleNamespace(pose=SimpleNamespace(position=_point(x, y)))
               for x, y in points],
    )


def _straight(x0, x1, step=0.5):
    n = int(round((x1 - x0) / step))
    return [(x0 + step * i, 0.0) for i in range(n + 1)]


class _Future:
    def __init__(self):
        self._result = None
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def complete(self, result):
        self._result = result
        for cb in self.callbacks:
            cb(self)

    def result(self):
        return self._result


class _Goal:
    pass


@pytest.fixture
def env():
    node = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 0
    params = {"bt_xml_normal": BT_NORMAL, "bt_xml_queue_wait": BT_QUEUE_WAIT,
              "plan_goal_match_tolerance": 0.6}
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=params.get(name, default))
    params_client = MagicMock()
    params_client.service_is_ready.return_value = False
    node.create_client.return_value = params_client

    with (
        patch.object(navigator_module, "ActionClient") as MockClient,
        patch.object(navigator_module, "GoalStatus", SimpleNamespace(
            STATUS_SUCCEEDED=STATUS_SUCCEEDED,
            STATUS_CANCELED=STATUS_CANCELED,
        )),
        patch.object(navigator_module.NavigateToPose, "Goal", _Goal),
    ):
        action_client = MagicMock()
        action_client.wait_for_server.return_value = True
        MockClient.return_value = action_client

        sent: list = []

        def _send_goal_async(goal, feedback_callback):
            future = _Future()
            sent.append(SimpleNamespace(
                goal=goal, future=future, feedback=feedback_callback))
            return future

        action_client.send_goal_async.side_effect = _send_goal_async

        nav = WaypointNavigator(node)
        plan_cb = node.create_subscription.call_args[0][2]
        results: list = []

        def accept(i=-1):
            handle = MagicMock()
            handle.accepted = True
            result_future = _Future()
            handle.get_result_async.return_value = result_future
            sent[i].future.complete(handle)
            return handle, result_future

        def feedback(x, y, i=-1, distance=5.0):
            msg = MagicMock()
            msg.feedback.distance_remaining = distance
            msg.feedback.current_pose.pose.position = _point(x, y)
            sent[i].feedback(msg)

        yield SimpleNamespace(
            nav=nav, node=node, params_client=params_client, sent=sent,
            results=results, on_result=results.append, accept=accept,
            feedback=feedback, plan=plan_cb,
        )


def _finish(result_future, status):
    result_future.complete(SimpleNamespace(status=status))


# ---------------------------------------------------------------------------
# ゴールの基本動作
# ---------------------------------------------------------------------------

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


def test_server_unavailable_is_failed(env):
    env.nav._action_client.wait_for_server.return_value = False
    env.nav.send_goal(_waypoint(), env.on_result)
    assert env.results == [NavigationResult.FAILED]


def test_cancel_before_acceptance_cancels_accepted_goal(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    env.nav.cancel()
    handle, _ = env.accept()
    handle.cancel_goal_async.assert_called_once()
    assert env.results == []


def test_canceled_goal_result_is_not_reported(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    handle, result_future = env.accept()
    env.nav.cancel()
    handle.cancel_goal_async.assert_called_once()
    _finish(result_future, STATUS_CANCELED)
    assert env.results == []


def test_send_after_cancel_waits_for_termination(env):
    env.nav.send_goal(_waypoint(0), env.on_result)
    _, old_result = env.accept(0)
    env.nav.cancel()
    env.nav.send_goal(_waypoint(1), env.on_result)
    assert len(env.sent) == 1

    _finish(old_result, STATUS_CANCELED)
    assert len(env.sent) == 2
    new_handle, _ = env.accept(1)
    env.nav.cancel()
    new_handle.cancel_goal_async.assert_called_once()


# ---------------------------------------------------------------------------
# preemption
# ---------------------------------------------------------------------------

def test_same_bt_goal_is_preempted_without_cancel(env):
    env.nav.send_goal(_waypoint(0), env.on_result)
    handle, old_result = env.accept(0)
    env.nav.send_goal(_waypoint(1, x=20.0), env.on_result)
    assert len(env.sent) == 2
    handle.cancel_goal_async.assert_not_called()

    # 上書きされた古いゴールは Nav2 で ABORTED になるが無視する
    _finish(old_result, STATUS_ABORTED)
    assert env.results == []

    new_handle, new_result = env.accept(1)
    _finish(new_result, STATUS_SUCCEEDED)
    assert env.results == [NavigationResult.SUCCEEDED]


def test_different_bt_cancels_then_sends(env):
    env.nav.send_goal(_waypoint(0), env.on_result)
    handle, old_result = env.accept(0)
    env.nav.send_goal(_waypoint(1), env.on_result, navigation_mode="queue_wait")
    handle.cancel_goal_async.assert_called_once()
    assert len(env.sent) == 1

    _finish(old_result, STATUS_CANCELED)
    assert len(env.sent) == 2
    assert env.sent[1].goal.behavior_tree == BT_QUEUE_WAIT
    assert env.results == []


# ---------------------------------------------------------------------------
# 通過点判定
# ---------------------------------------------------------------------------

def _start_through(env, **kw):
    env.nav.send_goal(_waypoint(is_through_point=True, **kw), env.on_result)
    return env.accept()


def test_through_point_passes_on_valid_plan_without_cancel(env):
    handle, _ = _start_through(env)
    env.plan(_path(_straight(0.0, 10.0)))
    for x in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]:
        env.feedback(x, 0.0)
    assert env.results == []
    env.feedback(7.5, 0.0)
    assert env.results == [NavigationResult.PASSED]
    handle.cancel_goal_async.assert_not_called()


def test_no_pass_without_plan(env):
    _start_through(env)
    env.feedback(9.0, 0.0, distance=1.0)
    assert env.results == []


def test_stale_plan_to_previous_goal_is_ignored(env):
    _start_through(env)
    env.plan(_path(_straight(7.0, 8.0)))  # 前のゴール (x=8) への経路
    env.feedback(7.5, 0.0, distance=0.5)
    assert env.results == []


def test_plan_stamped_before_send_is_ignored(env):
    env.node.get_clock.return_value.now.return_value.nanoseconds = 5_000_000_000
    _start_through(env)
    env.plan(_path(_straight(0.0, 10.0), stamp_sec=4))
    env.feedback(8.0, 0.0)
    assert env.results == []


def test_waypoint_behind_fence_is_not_passed(env):
    # ロボット (0,0) からゴール (0,2) は直線 2m だが、経路は x=5 まで回り込む
    _start_through(env, x=0.0, y=2.0)
    detour = ([(x * 0.5, 0.0) for x in range(11)]
              + [(5.0, y * 0.5) for y in range(1, 5)]
              + [(5.0 - x * 0.5, 2.0) for x in range(1, 11)])
    env.plan(_path(detour))
    env.feedback(0.0, 0.0)
    assert env.results == []


def test_path_looping_back_near_goal_is_not_passed(env):
    # 往路でゴール (0,1) の近くを通る。経路全体から最近傍点を探すと復路側の点が選ばれて
    # 残り距離が小さく出るが、前方探索なら往路上の点が選ばれる。
    _start_through(env, x=0.0, y=1.0, through_tolerance=1.5)
    loop = ([(-5.0 + x * 0.5, 0.0) for x in range(21)]
            + [(5.0, y * 0.5) for y in range(1, 3)]
            + [(5.0 - x * 0.5, 1.0) for x in range(1, 11)])
    env.plan(_path(loop))
    for x in [-5.0, -4.0, -3.0, -2.0, -1.0]:
        env.feedback(x, 0.0)
    env.feedback(0.0, 0.6)
    assert env.results == []


def test_stop_point_does_not_use_through_judgement(env):
    env.nav.send_goal(_waypoint(), env.on_result)
    env.accept()
    env.plan(_path(_straight(0.0, 10.0)))
    env.feedback(9.9, 0.0)
    assert env.results == []


# ---------------------------------------------------------------------------
# reach_tolerance
# ---------------------------------------------------------------------------

def test_reach_tolerance_is_applied_before_sending_goal(env):
    env.params_client.service_is_ready.return_value = True
    params_future = _Future()
    env.params_client.call_async.return_value = params_future

    env.nav.send_goal(_waypoint(reach_tolerance=0.8), env.on_result)
    assert env.sent == []

    request = env.params_client.call_async.call_args[0][0]
    assert request.parameters[0].value.double_value == 0.8

    params_future.complete(SimpleNamespace(
        results=[SimpleNamespace(successful=True)]))
    assert len(env.sent) == 1


def test_goal_not_sent_if_canceled_while_setting_tolerance(env):
    env.params_client.service_is_ready.return_value = True
    params_future = _Future()
    env.params_client.call_async.return_value = params_future

    env.nav.send_goal(_waypoint(), env.on_result)
    env.nav.cancel()
    params_future.complete(SimpleNamespace(
        results=[SimpleNamespace(successful=True)]))
    assert env.sent == []
