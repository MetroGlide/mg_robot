import pytest

from mg_utils.monitoring.topic_source_config import parse_source_config


def _get(values):
    def get(key, default):
        return values.get(key, default)
    return get


BASE = {'topic': 'scan_top_lidar', 'type': 'sensor_msgs/msg/LaserScan'}


def test_defaults():
    c = parse_source_config('top_lidar', _get(BASE), 1.0)
    assert c.id == 'top_lidar'
    assert c.topic == 'scan_top_lidar'
    assert c.msg_type == 'sensor_msgs/msg/LaserScan'
    assert c.enabled
    assert c.expected_hz == 0.0
    assert c.stale_timeout_sec == 1.0
    assert not c.armed_on_first_message
    assert c.qos == 'reliable'
    assert c.fault_level == 'error'
    assert c.raw


def test_ok_field_disables_raw_subscription():
    c = parse_source_config('motor', _get({**BASE, 'ok_field': 'data'}), 1.0)
    assert c.ok_field == 'data'
    assert not c.raw


def test_source_values_override_defaults():
    values = {**BASE, 'stale_timeout_sec': 3.0, 'expected_hz': 1.0, 'qos': 'best_effort',
              'armed_on_first_message': True, 'fault_level': 'warn', 'enabled': False}
    c = parse_source_config('odom', _get(values), 1.0)
    assert c.stale_timeout_sec == 3.0
    assert c.expected_hz == 1.0
    assert c.qos == 'best_effort'
    assert c.armed_on_first_message
    assert c.fault_level == 'warn'
    assert not c.enabled


@pytest.mark.parametrize('missing', ['topic', 'type'])
def test_required_keys(missing):
    values = {k: v for k, v in BASE.items() if k != missing}
    with pytest.raises(ValueError, match=missing):
        parse_source_config('x', _get(values), 1.0)


@pytest.mark.parametrize('key, value', [('qos', 'volatile'), ('fault_level', 'fatal')])
def test_invalid_choice(key, value):
    with pytest.raises(ValueError, match=key):
        parse_source_config('x', _get({**BASE, key: value}), 1.0)


def test_fault_level_ok_is_accepted():
    assert parse_source_config('x', _get({**BASE, 'fault_level': 'ok'}), 1.0).fault_level == 'ok'


def test_stale_timeout_must_exceed_period():
    with pytest.raises(ValueError, match='stale_timeout_sec'):
        parse_source_config('amcl_pose', _get({**BASE, 'expected_hz': 1.0}), 1.0)
    parse_source_config('amcl_pose', _get({**BASE, 'expected_hz': 1.0, 'stale_timeout_sec': 2.5}), 1.0)
