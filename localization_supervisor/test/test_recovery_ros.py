"""再初期化の送り先・対処・自分のメッセージの目印のテスト (rclpy は偽物)。"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from localization_supervisor.core.config import ConfigError
from localization_supervisor.core.interfaces import Candidate
from localization_supervisor.core.supervisor import Directive
from localization_supervisor.ros.actuators import build_actuators
from localization_supervisor.ros.actuators.standard import Gate, ServiceCall
from localization_supervisor.ros.actuators.variance_scale import delivered_scales
from localization_supervisor.ros.own_messages import OwnMessages
from localization_supervisor.ros.resetters import build_resetters
from localization_supervisor.ros.resetters.standard import pose_message


class FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def finish(self, success=True):
        self._response = SimpleNamespace(success=success)
        for callback in self.callbacks:
            callback(self)

    def result(self):
        return self._response


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append((request, future))
        return future


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.clients = []
        self.timers = []
        self.destroyed = []
        self.logger = MagicMock()
        self.sec = 100

    def create_publisher(self, msg_type, topic, qos):
        pub = SimpleNamespace(topic=topic, published=[])
        pub.publish = pub.published.append
        self.publishers.append(pub)
        return pub

    def create_client(self, service_type, name):
        client = FakeClient()
        client.name = name
        self.clients.append(client)
        return client

    def destroy_timer(self, timer):
        self.timers.remove(timer)
        self.destroyed.append(timer)

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        node = self
        stamp = SimpleNamespace(sec=node.sec, nanosec=5)
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))

    def get_logger(self):
        return self.logger


def _candidate():
    cov = np.array([[0.04, 0.001, 0.0], [0.001, 0.09, 0.0], [0.0, 0.0, 0.01]])
    return Candidate((1.0, 2.0, 0.5), cov, 'gnss')


def _result(hold=False, blame='', directives=()):
    return SimpleNamespace(hold_requested=hold, blame=blame, directives=list(directives))


# ---------------------------------------------------------------- 自分のメッセージの目印

def test_own_messages_are_consumed_once():
    own = OwnMessages()
    own.add('/initialpose', 10, 5)
    assert not own.consume('/set_pose', 10, 5)
    assert own.consume('/initialpose', 10, 5)
    assert not own.consume('/initialpose', 10, 5)


def test_own_messages_forget_the_oldest():
    own = OwnMessages(capacity=2)
    for sec in range(3):
        own.add('/t', sec, 0)
    assert not own.consume('/t', 0, 0) and own.consume('/t', 2, 0)


# ---------------------------------------------------------------- 再初期化の送り先

def test_pose_message_maps_the_covariance_into_the_6x6():
    msg = pose_message(_candidate(), 'map', SimpleNamespace(sec=1, nanosec=2), 2.0)
    assert msg.pose.pose.position.x == 1.0 and msg.pose.pose.position.y == 2.0
    assert msg.pose.pose.orientation.z == pytest.approx(np.sin(0.25))
    assert msg.pose.pose.orientation.w == pytest.approx(np.cos(0.25))
    cov = msg.pose.covariance
    assert cov[0] == pytest.approx(0.08) and cov[7] == pytest.approx(0.18) and cov[35] == pytest.approx(0.02)
    assert cov[1] == pytest.approx(0.002) and cov[6] == pytest.approx(0.002)


def test_topic_resetter_publishes_and_marks_the_message_as_its_own():
    node, own = FakeNode(), OwnMessages()
    resetters = build_resetters([{'type': 'robot_localization'}], node, own)
    resetters[0].reset(_candidate())
    assert node.publishers[0].topic == '/set_pose' and len(node.publishers[0].published) == 1
    assert own.consume('/set_pose', 100, 5)


def test_amcl_resetter_requests_a_nomotion_update_after_a_delay():
    node, own = FakeNode(), OwnMessages()
    (resetter,) = build_resetters([{'type': 'amcl', 'params': {'nomotion_delay_sec': 0.5}}], node, own)
    resetter.reset(_candidate())
    assert node.publishers[0].topic == '/initialpose'
    assert node.clients[0].name == '/request_nomotion_update' and not node.clients[0].calls
    timer = node.timers[0]
    timer.callback()
    assert timer.cancelled and node.destroyed == [timer] and not node.timers
    assert len(node.clients[0].calls) == 1
    # 続けて初期化し直したら、前のタイマーが残っていれば破棄する
    resetter.reset(_candidate())
    first = node.timers[0]
    resetter.reset(_candidate())
    assert first.cancelled and first in node.destroyed and len(node.timers) == 1


def test_delivered_scales_only_count_an_enabled_variance_scale_actuator():
    node = FakeNode()
    actuators = build_actuators([{'type': 'variance_scale', 'params': {'topic': '/s', 'residual': 'r'}}], node)
    assert delivered_scales(actuators, set(), {'r': 7.0, 'other': 3.0}) == {'r': 7.0, 'other': 1.0}
    assert delivered_scales(actuators, {'variance_scale'}, {'r': 7.0}) == {'r': 1.0}
    assert delivered_scales([], set(), {'r': 7.0}) == {'r': 1.0}


def test_resetter_config_errors():
    with pytest.raises(ConfigError, match='unknown type'):
        build_resetters([{'type': 'nope'}], FakeNode(), OwnMessages())
    with pytest.raises(ConfigError, match='unknown params'):
        build_resetters([{'type': 'amcl', 'params': {'nope': 1}}], FakeNode(), OwnMessages())


# ---------------------------------------------------------------- 対処

def _gate(**params):
    node = FakeNode()
    (gate,) = build_actuators([{'type': 'gate', 'params': {'service': '/arbiter/supervisor/change', **params}}], node)
    return gate, node.clients[0]


def test_gate_closes_while_holding_for_a_listed_blame_and_reopens():
    gate, client = _gate()
    gate.apply(_result())
    assert client.calls[0][0].data is True
    client.calls[0][1].finish()
    gate.apply(_result())
    assert len(client.calls) == 1           # 変わらなければ呼ばない
    gate.apply(_result(hold=True, blame='amcl'))
    assert client.calls[1][0].data is False
    client.calls[1][1].finish()
    gate.apply(_result(hold=True, blame='amcl'))
    assert len(client.calls) == 2
    gate.apply(_result())
    assert client.calls[2][0].data is True


def test_gate_ignores_other_blames_and_non_hold_states():
    gate, client = _gate(blames=['amcl'])
    gate.apply(_result())
    client.calls[0][1].finish()
    gate.apply(_result(hold=True, blame='gnss'))
    gate.apply(_result(hold=False, blame='amcl'))
    assert len(client.calls) == 1


def test_gate_retries_after_a_failure_and_waits_for_the_service():
    gate, client = _gate()
    client.ready = False
    gate.apply(_result())
    assert not client.calls
    client.ready = True
    gate.apply(_result())
    client.calls[0][1].finish(success=False)
    gate.apply(_result())
    assert len(client.calls) == 2


def test_service_call_on_reinit_directive_with_delay():
    node = FakeNode()
    (call,) = build_actuators([{'type': 'service_call', 'params': {
        'service': '/clear', 'on': ['reinit'], 'delay_sec': 1.0}}], node)
    call.apply(_result())
    call.apply(_result(directives=[Directive('isolate', 'amcl')]))
    assert not node.timers
    call.apply(_result(directives=[Directive('reinit', 'amcl', _candidate())]))
    assert not node.clients[0].calls
    (timer,) = node.timers
    timer.callback()
    assert len(node.clients[0].calls) == 1 and timer.cancelled
    # 一度呼んだタイマーは破棄する (再初期化のたびにノードにたまらない)
    assert node.destroyed == [timer] and not node.timers


def test_service_call_without_delay_and_unknown_type():
    node = FakeNode()
    (call,) = build_actuators([{'type': 'service_call', 'params': {'service': '/clear'}}], node)
    call.apply(_result(directives=[Directive('reinit')]))
    assert len(node.clients[0].calls) == 1
    with pytest.raises(ConfigError, match='expected one of'):
        build_actuators([{'type': 'service_call', 'params': {'service': '/x', 'type': 'nope'}}], FakeNode())


def test_actuator_registry_has_standard_types():
    assert Gate and ServiceCall
    with pytest.raises(ConfigError, match='unknown type'):
        build_actuators([{'type': 'nope'}], FakeNode())
