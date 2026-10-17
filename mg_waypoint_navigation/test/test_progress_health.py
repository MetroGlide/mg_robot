from mg_waypoint_navigation.waypoint_sequencer.health import progress_health
from mg_waypoint_navigation.waypoint_sequencer.progress_monitor import ProgressStatus


def test_not_navigating_is_ok():
    assert progress_health(None, 30.0) == ('ok', 'not navigating', {})


def test_navigating_is_ok_with_values():
    level, message, values = progress_health(ProgressStatus(5.0, 1), 30.0)
    assert (level, message) == ('ok', 'navigating')
    assert values == {'stalled_sec': '5.0', 'number_of_recoveries': '1'}


def test_stalled_is_warn():
    level, message, values = progress_health(ProgressStatus(30.0, 4), 30.0)
    assert (level, message) == ('warn', 'no progress for 30 s')
    assert values['number_of_recoveries'] == '4'
