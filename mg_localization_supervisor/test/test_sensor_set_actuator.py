"""センサの組が変わったら、ほかのノードのパラメータを切り替える対処 (actuators/sensor_set.py) のテスト。"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from mg_localization_supervisor.actuators import sensor_set
from mg_localization_supervisor.actuators.sensor_set import SensorSetParameters

SETS = {
    'no_lidar': {'/collision_monitor': {'scan_top.stop_on_timeout': False}},
    'all': {'/collision_monitor': {'scan_top.stop_on_timeout': True}},
}


class _Parameter:
    """rclpy.parameter.Parameter の代わり (送った名前と値を読む)。"""

    def __init__(self, name, value):
        self.name = name
        self.value = value

    def to_parameter_msg(self):
        return self


@pytest.fixture(autouse=True)
def _parameters(monkeypatch):
    monkeypatch.setattr(sensor_set, 'Parameter', _Parameter)


class _Future:
    def __init__(self, ok=True, done=True):
        self._ok = ok
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return SimpleNamespace(results=[SimpleNamespace(successful=self._ok)])


def _actuator(ready=True, future=None):
    node = MagicMock()
    clock = {'t': 0.0}
    node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(nanoseconds=int(clock['t'] * 1e9))
    client = node.create_client.return_value
    client.service_is_ready.return_value = ready
    client.call_async.return_value = future or _Future()
    return SensorSetParameters('sensor_set', {'sets': SETS, 'retry_sec': 2.0}, node), client, clock


def _sent(client):
    return [(p.name, p.value) for call in client.call_async.call_args_list for p in call.args[0].parameters]


def test_nothing_is_sent_at_the_start():
    actuator, client, _ = _actuator()
    actuator.apply(SimpleNamespace(sensor_set=''))
    assert not client.call_async.called


def test_entering_and_leaving_a_set_sends_its_values_once():
    actuator, client, clock = _actuator()
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    clock['t'] = 5.0
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    assert _sent(client) == [('scan_top.stop_on_timeout', False)]
    actuator.apply(SimpleNamespace(sensor_set=''))
    assert _sent(client)[-1] == ('scan_top.stop_on_timeout', True)


def test_an_unreachable_node_is_retried_after_retry_sec():
    actuator, client, clock = _actuator(ready=False)
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    client.service_is_ready.return_value = True
    clock['t'] = 1.0
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    assert not client.call_async.called
    clock['t'] = 2.5
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    assert _sent(client) == [('scan_top.stop_on_timeout', False)]


def test_a_failed_set_is_retried():
    actuator, client, clock = _actuator(future=_Future(ok=False))
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    clock['t'] = 0.5
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    clock['t'] = 3.0
    actuator.apply(SimpleNamespace(sensor_set='no_lidar'))
    assert client.call_async.call_count == 2
