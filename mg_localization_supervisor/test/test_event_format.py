"""フレームワークが出すイベントが、このプロジェクトのイベントの規約 (doc/monitoring.md) で読めることを確かめる。"""
import pytest

from localization_supervisor.ros.event_sinks.logfmt import format_event as framework_format_event
from mg_utils.event_log import format_event, parse_event

SAMPLES = [
    ('localization.state_changed', dict(from_='NORMAL', to='SUSPECT', reason='isolated: amcl_gnss')),
    ('localization.reinit_requested', dict(source='ekf', x=1.25, y=-3.5, yaw=0.785, scan_match='n/a')),
    ('gate.changed', dict(requester='supervisor', attached=False, holders='supervisor,waypoint')),
    ('node.started', dict(name='localization_supervisor_node', recovery_enabled=True, empty='', none=None)),
    ('localization.state_changed', dict(reason='quote " and = and \\ and\nnewline')),
]


@pytest.mark.parametrize('name, fields', SAMPLES)
def test_framework_event_matches_project_format(name, fields):
    assert framework_format_event(name, **fields) == format_event(name, **fields)


@pytest.mark.parametrize('name, fields', SAMPLES)
def test_framework_event_is_parsed_by_project_parser(name, fields):
    parsed = parse_event(framework_format_event(name, **fields))
    assert parsed is not None
    assert parsed[0] == name
