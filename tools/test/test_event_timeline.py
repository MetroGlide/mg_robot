from datetime import datetime, timedelta, timezone

import pytest

from tools.common import event_timeline as et

UTC = timezone.utc
# 2026-10-10T16:42:25.774Z
EPOCH = datetime(2026, 10, 10, 16, 42, 25, 774000, tzinfo=UTC).timestamp()


def _line(prefix, epoch, node, body, level='INFO'):
    return f'{prefix}[{level}] [{epoch}] [{node}]: {body}'


def test_parse_collector_line():
    line = _line('2026-10-11T01:42:25.800+09:00 ', EPOCH, 'motor_driver_node',
                 'evt=driver.disconnected device=motor_driver reason="read timeout"', 'ERROR')
    r = et.parse_log_line(line, 'navigation')
    assert r.name == 'driver.disconnected'
    assert r.level == 'ERROR'
    assert r.node == 'motor_driver_node'
    assert r.source == 'navigation'
    assert r.fields == {'device': 'motor_driver', 'reason': 'read timeout'}
    # 時刻は本文の epoch (ノードが記録した時刻) を使う
    assert r.time == datetime.fromtimestamp(EPOCH, tz=UTC)
    assert r.domain == 'driver'


def test_parse_logs_export_line_with_container_prefix_and_nanoseconds():
    line = 'mg_robot-navigation-1  | 2026-10-10T16:42:25.774123456Z ' + _line(
        '', EPOCH, 'n', 'evt=gate.changed attached=false holders=waypoint', 'WARN')
    r = et.parse_log_line(line)
    assert r.name == 'gate.changed'
    assert r.level == 'WARN'


def test_non_events_are_ignored():
    assert et.parse_log_line('2026-10-11T01:42:25+09:00 [INFO] [1.0] [n]: hello') is None
    assert et.parse_log_line('plain text') is None
    assert et.parse_log_line('') is None


def test_event_without_ros_prefix_uses_iso_time():
    r = et.parse_log_line('2026-10-10T16:42:25Z evt=node.started name=x')
    assert r.time == datetime(2026, 10, 10, 16, 42, 25, tzinfo=UTC)
    assert r.node == ''


def test_line_without_any_time_is_ignored():
    assert et.parse_log_line('evt=node.started') is None


def test_parse_ros_log_levels():
    r = et.parse_ros_log(EPOCH, 40, 'n', 'evt=watchdog.stale source=top_lidar')
    assert r.level == 'ERROR' and r.fields == {'source': 'top_lidar'}
    assert et.parse_ros_log(EPOCH, 20, 'n', 'hello') is None


def _rec(offset, name='a.b', level='INFO', node='n', source='s', **fields):
    return et.EventRecord(datetime.fromtimestamp(EPOCH + offset, tz=UTC), level, node, name, fields, source)


def test_merge_sorts_and_deduplicates():
    a, b = _rec(1.0), _rec(0.5, name='c.d')
    merged = et.merge([a, b, a])
    assert [r.name for r in merged] == ['c.d', 'a.b']


def test_merge_keeps_events_that_differ_in_fields():
    merged = et.merge([_rec(1.0, x='1'), _rec(1.0, x='2')])
    assert len(merged) == 2


def test_filters():
    records = [
        _rec(0, 'driver.connected'),
        _rec(10, 'watchdog.stale', 'ERROR', source='navigation'),
        _rec(20, 'gate.changed', 'WARN'),
    ]
    since = datetime.fromtimestamp(EPOCH + 5, tz=UTC)
    until = datetime.fromtimestamp(EPOCH + 15, tz=UTC)
    assert [r.name for r in et.filter_records(records, since=since)] == ['watchdog.stale', 'gate.changed']
    assert [r.name for r in et.filter_records(records, until=until)] == ['driver.connected', 'watchdog.stale']
    assert [r.name for r in et.filter_records(records, min_level='WARN')] == ['watchdog.stale', 'gate.changed']
    assert [r.name for r in et.filter_records(records, domains=['gate'])] == ['gate.changed']
    assert [r.name for r in et.filter_records(records, sources=['navigation'])] == ['watchdog.stale']


@pytest.mark.parametrize('text, offset', [('UTC', 0), ('+09:00', 9), ('-0530', -5.5), ('Asia/Tokyo', 9)])
def test_resolve_tz(text, offset):
    tz = et.resolve_tz(text)
    assert tz.utcoffset(datetime(2026, 1, 1)) == timedelta(hours=offset)


def test_resolve_tz_local_is_none():
    assert et.resolve_tz(None) is None
    assert et.resolve_tz('local') is None


def test_markdown_and_csv():
    records = [_rec(0, 'driver.disconnected', 'ERROR', 'motor_driver_node', 'navigation', reason='a|b')]
    tz = et.resolve_tz('+09:00')
    md = et.to_markdown(records, tz)
    assert '| 2026-10-11T01:42:25.774+09:00 | ERROR | navigation | motor_driver_node | driver.disconnected | reason=a\\|b |' in md
    csv_text = et.to_csv(records, tz)
    assert csv_text.splitlines()[0] == 'time,level,source,node,event,fields'
    assert '2026-10-11T01:42:25.774+09:00,ERROR,navigation,motor_driver_node,driver.disconnected,reason=a|b' in csv_text


def test_parse_iso_handles_z_and_naive():
    assert et.parse_iso('2026-10-10T16:42:25Z') == datetime(2026, 10, 10, 16, 42, 25, tzinfo=UTC)
    assert et.parse_iso('2026-10-10T16:42:25') == datetime(2026, 10, 10, 16, 42, 25, tzinfo=UTC)
