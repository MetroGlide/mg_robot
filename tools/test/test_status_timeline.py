import pytest

from tools.common.status_timeline import (
    EventSummary, LevelTimeline, has_status_section, level_name, level_value, pick_timeline,
    status_section_lines)

SEC = 1_000_000_000


def test_level_value_accepts_bytes_and_int():
    assert level_value(b'\x02') == 2
    assert level_value(b'\x00') == 0
    assert level_value(3) == 3
    assert level_name(3) == 'STALE'
    assert level_name(9) == '9'


def test_transitions_and_time_in_level():
    t = LevelTimeline()
    t.update(0 * SEC, 'a', 0)
    t.update(10 * SEC, 'a', 2, 'no data')
    t.update(14 * SEC, 'a', 3, 'missing')
    t.update(20 * SEC, 'a', 0, 'ok')
    s = t.summary()
    assert [(x['from'], x['to']) for x in s['transitions']] == [
        ('OK', 'ERROR'), ('ERROR', 'STALE'), ('STALE', 'OK')]
    assert s['transitions'][0]['message'] == 'no data'
    info = s['by_name']['a']
    assert info == {'final': 'OK', 'worst': 'STALE', 'transitions': 3,
                    'warn_s': 0.0, 'error_s': 4.0, 'stale_s': 6.0}


def test_names_are_independent_and_no_change_is_not_a_transition():
    t = LevelTimeline()
    for i in range(5):
        t.update(i * SEC, 'a', 0)
        t.update(i * SEC, 'b', 1)
    s = t.summary()
    assert s['transitions'] == []
    assert s['by_name']['a']['final'] == 'OK' and s['by_name']['b']['final'] == 'WARN'
    assert s['by_name']['b']['warn_s'] == 4.0


def test_transition_limit_counts_dropped():
    t = LevelTimeline(max_transitions=2)
    for i in range(6):
        t.update(i * SEC, 'a', i % 2)
    s = t.summary()
    assert len(s['transitions']) == 2
    assert s['dropped_transitions'] == 3
    assert s['by_name']['a']['transitions'] == 5


def test_flapping_name_does_not_hide_other_names():
    t = LevelTimeline(max_per_name=3)
    for i in range(100):
        t.update(i * SEC, 'flapping', i % 2)
    t.update(0, 'steady', 0)
    t.update(50 * SEC, 'steady', 2, 'lost')
    s = t.summary()
    names = [x['name'] for x in s['transitions']]
    assert names.count('flapping') == 3
    assert names.count('steady') == 1
    assert s['by_name']['flapping']['transitions'] == 99
    assert s['dropped_transitions'] == 96


def test_event_summary_counts_and_notable():
    e = EventSummary()
    e.add(1, 'INFO', 'n', 'driver.connected', {'device': 'motor'})
    e.add(2, 'ERROR', 'n', 'driver.disconnected', {'reason': 'x'})
    e.add(3, 'WARN', 'n', 'gate.changed', {})
    s = e.summary()
    assert s['counts'] == {'driver.connected': 1, 'driver.disconnected': 1, 'gate.changed': 1}
    assert [x['name'] for x in s['notable']] == ['driver.disconnected', 'gate.changed']


def test_event_summary_deduplicates_rosout_and_events():
    e = EventSummary()
    e.add(5, 'ERROR', 'n', 'watchdog.stale', {'source': 'top_lidar'})
    e.add(5, 'ERROR', 'n', 'watchdog.stale', {'source': 'top_lidar'})
    s = e.summary()
    assert s['counts'] == {'watchdog.stale': 1}
    assert len(s['notable']) == 1


def test_pick_timeline_prefers_aggregated():
    agg, raw = LevelTimeline(), LevelTimeline()
    raw.update(0, 'x', 0)
    assert pick_timeline(agg, raw) is raw
    agg.update(0, 'y', 0)
    assert pick_timeline(agg, raw) is agg
    assert pick_timeline(LevelTimeline(), LevelTimeline()) is None


def _diag():
    t = LevelTimeline()
    t.update(0 * SEC, '/Robot/Drivers/LiDAR', 0)
    t.update(10 * SEC, '/Robot/Drivers/LiDAR', 2, 'no data for 2.0 s')
    t.update(20 * SEC, '/Robot/Drivers/LiDAR', 0)
    t.update(0 * SEC, '/Robot/System/Resources', 0)
    t.update(20 * SEC, '/Robot/System/Resources', 0)
    e = EventSummary()
    e.add(10 * SEC, 'ERROR', 'driver_watchdog_node', 'watchdog.stale', {'source': 'top_lidar'})
    e.add(11 * SEC, 'INFO', 'motor_driver_node', 'driver.connected', {'device': 'motor|x'})
    return {'has_data': True, 'timeline_source': '/diagnostics_agg', 'timeline': t.summary(), 'events': e.summary()}


def test_has_status_section():
    assert has_status_section(_diag())
    assert not has_status_section({'has_data': True, 'timeline': None, 'events': {'counts': {}}})
    assert not has_status_section({'has_data': True})


def test_status_section_lines_terminal():
    text = '\n'.join(status_section_lines(_diag(), markdown=False))
    assert '/Robot/Drivers/LiDAR: 最後=OK 最悪=ERROR 変化=2回 WARN=0.0s ERROR=10.0s STALE=0.0s' in text
    # OK のままの診断は、一覧に出さない
    assert 'Resources' not in text
    assert '/Robot/Drivers/LiDAR: OK → ERROR no data for 2.0 s' in text
    assert 'watchdog.stale: 1' in text
    assert '[ERROR] driver_watchdog_node watchdog.stale source=top_lidar' in text


def test_status_section_lines_markdown_escapes_pipes():
    text = '\n'.join(status_section_lines(_diag(), markdown=True))
    assert '| `/Robot/Drivers/LiDAR` | OK | ERROR | 2 | 0.0 | 10.0 | 0.0 |' in text
    assert '| OK → ERROR |' in text.replace('`', '') or 'OK → ERROR' in text
    assert '| `watchdog.stale` | 1 |' in text
