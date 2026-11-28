import pytest

from tools.common.nav_timing import NavTimingSummary, has_nav_timing_section, nav_timing_section_lines

SEC = 1_000_000_000


def test_tf_gap_is_odom_base_stamp_minus_latest_map_odom():
    s = NavTimingSummary()
    s.process_tf('odom', 'base_footprint', 0.9)  # map→odom がまだないので数えない
    s.process_tf('map', 'odom', 1.0)
    s.process_tf('odom', 'base_footprint', 1.1)
    s.process_tf('odom', 'base_footprint', 0.95)
    s.process_tf('base_footprint', 'base_link', 5.0)
    gap = s.summary()['tf_gap_s']
    assert gap['count'] == 2
    assert gap['max'] == pytest.approx(0.1)
    assert gap['positive_ratio'] == pytest.approx(0.5)


def test_warnings_are_counted_by_kind_and_service():
    s = NavTimingSummary()
    s.process_log('Control loop missed its desired rate of 10.0000Hz')
    s.process_log('Control loop missed its desired rate of 10.0000Hz')
    s.process_log('Exception in transformPose: Lookup would require extrapolation into the future.')
    s.process_log('Node timed out while executing service call to global_costmap/clear_entirely_global_costmap.')
    s.process_log('Failed to get result for wait in node halt!')
    s.process_log('Collision Ahead - Exiting Spin')
    s.process_log('Received a goal, begin computing control effort.')
    assert s.summary()['warnings'] == {
        'control_loop_missed': 2,
        'controller_tf_exception': 1,
        'service_timeout:global_costmap/clear_entirely_global_costmap': 1,
        'halt_no_result:wait': 1,
        'behavior_collision:Spin': 1,
    }


def test_activity_durations_ignore_duplicate_events():
    s = NavTimingSummary()
    s.process_event(0, 'sequencer.activity_changed', {'from': 'idle', 'to': 'navigating'})
    s.process_event(0, 'sequencer.activity_changed', {'from': 'idle', 'to': 'navigating'})  # /events の写し
    s.process_event(10 * SEC, 'localization.hold', {'active': 'true'})
    s.process_event(10 * SEC, 'sequencer.activity_changed', {'from': 'navigating', 'to': 'recovery'})
    s.process_event(13 * SEC, 'sequencer.activity_changed', {'from': 'recovery', 'to': 'navigating'})
    activity = {a['name']: a for a in s.summary(end_ns=20 * SEC)['activity']}
    assert activity['navigating']['seconds'] == pytest.approx(17.0)
    assert activity['navigating']['entries'] == 2
    assert activity['recovery']['seconds'] == pytest.approx(3.0)


def test_section_is_empty_without_data():
    nav = NavTimingSummary().summary()
    assert not has_nav_timing_section(nav)
    assert nav_timing_section_lines(nav, 60.0, markdown=True) == []


def test_section_lines_show_rate_per_minute():
    s = NavTimingSummary()
    for _ in range(30):
        s.process_log('Control loop missed its desired rate of 10.0000Hz')
    lines = nav_timing_section_lines(s.summary(), 60.0, markdown=True)
    assert '| 制御の周期の遅れ (controller_server) | 30 | 30.0 |' in lines
