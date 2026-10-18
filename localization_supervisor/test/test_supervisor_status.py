"""状態のメッセージ・診断の中身 (core/status.py)、イベントの出力、ノードが動かせる設定かの検証のテスト。"""
from unittest.mock import MagicMock

import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.geometry import compose
from localization_supervisor.core.interfaces import Status
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.status import CheckView, StatusView, localization_health, status_view
from localization_supervisor.core.supervisor import Supervisor, make_store
from localization_supervisor.ros.event_sinks import LogfmtEventSink, build_event_sink
from localization_supervisor.ros.supervisor_node import check_node_supports

from supervisor_test_helpers import Drive, pose_sample


@pytest.mark.parametrize('state, level', [
    (State.NORMAL, 'ok'),
    (State.SUSPECT, 'warn'),
    (State.ISOLATED, 'warn'),
    (State.RECOVERING, 'warn'),
    (State.DEGRADED, 'error'),
])
def test_level_by_state(state, level):
    got_level, message, _ = localization_health(StatusView(state))
    assert got_level == level
    assert message == state.name


def test_health_values_show_each_check():
    view = StatusView(State.DEGRADED, blame='amcl', contaminated=['ekf'], fusion_level=4.5, checks=[
        CheckView('amcl_gnss', 'pose_consistency', ['amcl', 'gnss'], Status.ANOMALY, 3.25),
        CheckView('scan_ekf', 'scan_map', ['ekf', 'map_scan'], Status.UNKNOWN)])
    level, message, values = localization_health(view)
    assert (level, message) == ('error', 'DEGRADED (blame: amcl)')
    assert values['check.amcl_gnss'] == 'anomaly 3.250'
    assert values['check.scan_ekf'] == 'unknown'
    assert values['contaminated'] == 'ekf' and values['fusion_level'] == '4.500'


def _config(**overrides):
    raw = {
        'version': 0,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'}},
        'timeline': {'motion_priority': ['odom'], 'alignment_std_per_m': 0.0, 'alignment_std_per_rad': 0.0},
        'detectors': {
            'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 6.0}},
            'amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'}},
        },
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 5.0}},
    }
    raw.update(overrides)
    return parse_config(raw)


def test_status_view_from_step():
    config = _config()
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    store.add_motion('odom', drive.motion(0.0))
    store.add_pose('amcl', pose_sample(0.0, compose(drive.map(0.0), (3.0, 0.0, 0.0))))
    store.add_pose('gnss', pose_sample(0.0, drive.map(0.0)))
    view = status_view(supervisor.step(store.snapshot(0.0)), supervisor.config, 'night')
    # AMCL と GNSS の食い違いだけでは、どちらとも決められない
    assert view.state == State.SUSPECT and view.blame == 'unknown' and view.profile == 'night'
    assert view.lethal_depth_m == -1.0 and not view.footprint_in_lethal
    checks = {c.name: c for c in view.checks}
    assert checks['amcl_gnss'].detector == 'pose_consistency'
    assert checks['amcl_gnss'].sources == ['amcl', 'gnss']
    assert checks['amcl_gnss'].status == Status.ANOMALY and checks['amcl_gnss'].accumulated > 0.0
    assert checks['amcl_jump'].status == Status.UNKNOWN and checks['amcl_jump'].score == 0.0
    assert view.last_event == 'suspect: amcl_gnss'


def test_event_sink_levels_and_plugin_resolution():
    logger = MagicMock()
    sink = build_event_sink({'type': 'logfmt'}, logger)
    assert isinstance(sink, LogfmtEventSink)
    sink.emit('localization.state_changed', 'warn', {'from': 'NORMAL', 'to': 'SUSPECT'})
    logger.warning.assert_called_once_with('evt=localization.state_changed from=NORMAL to=SUSPECT')
    sink.error('localization.lethal', in_lethal=True)
    logger.error.assert_called_once_with('evt=localization.lethal in_lethal=true')
    with pytest.raises(ConfigError):
        build_event_sink({'type': 'nothing'}, logger)
    with pytest.raises(ConfigError):
        build_event_sink({'type': 'logfmt', 'unknown': 1}, logger)


GNSS_RECOVERY = {'generators': {'gnss': {'type': 'pose_source', 'sources': {'pose': 'gnss'}}}}


def test_node_rejects_recovery_without_resetters_and_source_changes_in_profiles():
    check_node_supports(_config())
    with pytest.raises(ConfigError, match='recovery.resetters'):
        check_node_supports(_config(state_machine={'recovery_enabled': True}, recovery=GNSS_RECOVERY))
    check_node_supports(_config(state_machine={'recovery_enabled': True},
                                recovery={**GNSS_RECOVERY, 'resetters': [{'type': 'amcl'}]}))
    config = _config(recovery=GNSS_RECOVERY)
    config.profiles['fix'] = {'state_machine': {'recovery_enabled': True}}
    with pytest.raises(ConfigError, match='profiles.fix'):
        check_node_supports(config)
    config = _config(recovery={**GNSS_RECOVERY, 'resetters': [{'type': 'amcl'}]})
    config.profiles['other_resetter'] = {'recovery': {'resetters': [{'type': 'robot_localization'}]}}
    with pytest.raises(ConfigError, match='cannot change recovery'):
        check_node_supports(config)
    config = _config()
    config.profiles['other_topic'] = {'sources': {'amcl': {'input': {'type': 'pose_with_covariance',
                                                                     'topic': '/other'}}}}
    with pytest.raises(ConfigError, match='sources'):
        check_node_supports(config)
