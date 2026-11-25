"""センサの組 (core/sensor_sets.py: 情報源の欠けから今の組を決める) と、その走り方への効き目のテスト。"""
import copy

import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.interfaces import SourceKind
from localization_supervisor.core.sensor_sets import ALL, SensorSetConfig, SensorSetTracker
from localization_supervisor.core.snapshot import SampleStore

from supervisor_test_helpers import room_scan
from test_supervisor_gate import RAW, Sim

NO_LIDAR = SensorSetConfig('no_lidar', ['map_scan'], missing_after_sec=5.0, present_after_sec=2.0,
                           speed_limit_mps=0.2)


def _tracker():
    store = SampleStore({'map_scan': SourceKind.REFERENCE})
    return store, SensorSetTracker([NO_LIDAR], {'map_scan': SourceKind.REFERENCE})


def _run(store, tracker, start, end, scans):
    changes = []
    t = start
    while t < end:
        if scans:
            store.set_scan('map_scan', room_scan(t, (0.0, 0.0, 0.0)))
        current, changed = tracker.update(store.snapshot(t))
        if changed:
            changes.append(changed)
        t += 0.5
    return current, changes


def test_a_source_missing_long_enough_switches_the_set_and_comes_back_after_it_returns():
    store, tracker = _tracker()
    assert _run(store, tracker, 0.0, 10.0, scans=True) == (ALL, [])
    # 欠けて missing_after_sec を過ぎたら組を変える (起動し直しで戻る短い欠けでは変えない)
    current, changes = _run(store, tracker, 10.0, 14.0, scans=False)
    assert current == ALL and not changes
    current, changes = _run(store, tracker, 14.0, 17.0, scans=False)
    assert current == 'no_lidar' and changes == [(ALL, 'no_lidar')]
    # 戻っても present_after_sec 届き続けるまでは組のまま
    current, changes = _run(store, tracker, 17.0, 18.5, scans=True)
    assert current == 'no_lidar' and not changes
    current, changes = _run(store, tracker, 18.5, 21.0, scans=True)
    assert current == ALL and changes == [('no_lidar', ALL)]


def test_a_source_that_never_arrives_switches_after_the_start():
    store, tracker = _tracker()
    current, changes = _run(store, tracker, 0.0, 6.0, scans=False)
    assert current == 'no_lidar'


def _raw_with_sets(sets, conduct=True):
    raw = copy.deepcopy(RAW)
    raw['sensor_sets'] = sets
    if conduct:
        raw['conduct'] = {}
    return raw


def test_config_reads_sensor_sets():
    config = parse_config(_raw_with_sets([{'name': 'no_lidar', 'missing': ['map_scan'], 'speed_limit_mps': 0.2}]))
    assert [(s.name, s.missing, s.speed_limit_mps) for s in config.sensor_sets] == [('no_lidar', ['map_scan'], 0.2)]


@pytest.mark.parametrize('sets', [
    {'name': 'x'},
    [{'missing': ['map_scan']}],
    [{'name': 'x', 'missing': []}],
    [{'name': 'x', 'missing': ['odom']}],
    [{'name': 'x', 'missing': ['nothing']}],
    [{'name': 'x', 'missing': ['map_scan']}, {'name': 'x', 'missing': ['gnss']}],
])
def test_config_rejects_bad_sensor_sets(sets):
    with pytest.raises(ConfigError):
        parse_config(_raw_with_sets(sets))


def test_the_set_caps_the_speed_while_the_lidar_is_missing():
    sim = Sim(_raw_with_sets([{'name': 'no_lidar', 'missing': ['map_scan'], 'missing_after_sec': 3.0,
                               'speed_limit_mps': 0.2}]))
    sim.run(18.0)
    assert sim.results[-1].sensor_set == '' and sim.results[-1].conduct.speed_limit_mps == 0.0
    sim.scans = False
    results = sim.run(6.0)
    assert results[-1].sensor_set == 'no_lidar'
    assert results[-1].conduct.speed_limit_mps == pytest.approx(0.2)
    assert any(e.name == 'localization.sensor_set_changed' for r in results for e in r.events)
