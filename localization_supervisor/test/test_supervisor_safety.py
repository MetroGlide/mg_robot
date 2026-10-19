"""安全の信号 (footprint が計画用地図の致死セルにかかっているか) と、Supervisor の想定漏れへの対処のテスト。"""
import numpy as np
import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.geometry import compose
from localization_supervisor.core.grid import OccupancyMap
from localization_supervisor.core.interfaces import CONTRACT_VERSION
from localization_supervisor.core.safety import FootprintChecker, SafetyConfig, footprint_samples, inside_polygon
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.supervisor import Supervisor, make_store

from supervisor_test_helpers import Drive, pose_sample, wall_map

FOOTPRINT = [(0.4, 0.25), (0.4, -0.25), (-0.2, -0.3), (-0.2, 0.3)]


def _block_map():
    """10 m 四方 (0.05 m 解像度、原点 (0, 0)) の、x 5〜7 m・y 4〜6 m が致死の地図。"""
    data = np.zeros((200, 200), dtype=np.int8)
    data[80:120, 100:140] = 100
    return OccupancyMap(data, 0.05, 0.0, 0.0)


def test_inside_polygon_and_samples():
    points = np.array([[0.0, 0.0], [0.39, 0.0], [0.5, 0.0], [-0.19, 0.29], [0.3, 0.3]])
    assert inside_polygon(points, FOOTPRINT).tolist() == [True, True, False, True, False]
    samples = footprint_samples(FOOTPRINT, 0.05)
    assert inside_polygon(samples * 0.999, FOOTPRINT).all()
    assert samples[:, 0].min() == pytest.approx(-0.2) and samples[:, 0].max() == pytest.approx(0.4)


def test_footprint_checker_fraction_and_depth():
    checker = FootprintChecker(SafetyConfig('ekf', 'planning', FOOTPRINT, depth_range_m=2.0))
    occupancy = _block_map()
    free = checker.check(occupancy, (2.0, 5.0, 0.0))
    assert not free.in_lethal and free.fraction == 0.0 and free.depth_m == 0.0
    # 前端だけが 0.1 m 食い込む
    edge = checker.check(occupancy, (4.7, 5.0, 0.0))
    assert edge.in_lethal and 0.0 < edge.fraction < 0.3
    assert edge.depth_m == pytest.approx(0.1, abs=0.06)
    # 中ほどまで入ると深い
    deep = checker.check(occupancy, (6.0, 5.0, 0.0))
    assert deep.fraction == pytest.approx(1.0)
    assert deep.depth_m == pytest.approx(1.0, abs=0.1)


def _raw(**overrides):
    raw = {
        'version': CONTRACT_VERSION,
        'sources': {
            'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'},
            'map_scan': {'kind': 'reference'}, 'planning': {'kind': 'reference'}},
        'timeline': {'motion_priority': ['odom'], 'alignment_std_per_m': 0.0, 'alignment_std_per_rad': 0.0},
        'detectors': {
            'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 6.0}},
            'scan_amcl': {'type': 'scan_map', 'sources': {'pose': 'amcl', 'reference': 'map_scan'}},
        },
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 5.0}},
        'state_machine': {'clear_ticks': 3},
        'safety': {'pose': 'gnss', 'map': 'planning', 'footprint': [list(p) for p in FOOTPRINT]},
        'grace': {'after_reset_sec': 3.0},
    }
    raw.update(overrides)
    return raw


def test_safety_config_is_validated():
    assert parse_config(_raw()).safety.footprint[0] == (0.4, 0.25)
    with pytest.raises(ConfigError, match='safety.map'):
        parse_config(_raw(safety={'pose': 'gnss', 'map': 'odom', 'footprint': [[0, 0], [1, 0], [0, 1]]}))
    with pytest.raises(ConfigError, match='safety.footprint'):
        parse_config(_raw(safety={'pose': 'gnss', 'map': 'planning', 'footprint': [[0, 0], [1, 0]]}))


def _feed(supervisor, store, drive, t, amcl_offset=(0.0, 0.0, 0.0)):
    store.add_motion('odom', drive.motion(t))
    store.add_pose('amcl', pose_sample(t, compose(drive.map(t), amcl_offset)))
    store.add_pose('gnss', pose_sample(t, drive.map(t)))
    return supervisor.step(store.snapshot(t))


def test_supervisor_reports_lethal_and_events_on_change():
    config = parse_config(_raw())
    supervisor = Supervisor(config)
    store = make_store(config)
    store.set_map('planning', _block_map())
    # x 方向に 0.5 m/s で、致死の領域 (x 5〜7 m) を通り抜ける
    drive = Drive(map_odom=(3.0, 5.0, 0.0), speed=0.5, yaw_rate=0.0)
    results = [_feed(supervisor, store, drive, float(t)) for t in range(12)]
    lethal = [r.safety.in_lethal for r in results]
    assert lethal[0] is False and any(lethal)
    names = [e.name for r in results for e in r.events]
    assert names.count('localization.lethal') == 2
    entered = next(e for r in results for e in r.events if e.name == 'localization.lethal')
    assert entered.level == 'warn' and entered.fields['in_lethal'] is True


def test_supervisor_external_reinit_resets_to_normal_with_grace():
    config = parse_config(_raw())
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    results = [_feed(supervisor, store, drive, float(t), (3.0, 0.0, 0.0)) for t in range(3)]
    assert results[-1].state == State.DEGRADED
    supervisor.external_reinit(3.0, '/initialpose')
    result = _feed(supervisor, store, drive, 3.0, (3.0, 0.0, 0.0))
    assert result.state == State.NORMAL and result.in_grace
    assert result.fusion.level == 0.0
    assert [e.name for e in result.events] == ['localization.external_reinit']
    assert result.events[0].fields == {'topic': '/initialpose', 'state': 'DEGRADED'}
    assert result.last_event == 'external_reinit'
    assert not _feed(supervisor, store, drive, 6.5).in_grace


def test_supervisor_map_change_resets_detectors():
    config = parse_config(_raw())
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    store.set_map('map_scan', wall_map())
    first = _feed(supervisor, store, drive, 0.0)
    assert not first.events and not first.in_grace
    # 計画用地図 (判定が使わない地図) が替わっても捨てない
    store.set_map('planning', _block_map())
    store.set_map('planning', _block_map())
    assert not _feed(supervisor, store, drive, 1.0).events
    store.set_map('map_scan', wall_map())
    result = _feed(supervisor, store, drive, 2.0)
    assert [e.name for e in result.events] == ['localization.map_changed']
    assert result.in_grace


def test_supervisor_disable_and_weight_overrides_survive_profile_switch():
    config = parse_config(_raw(), profiles={'strict': {'fusion': {'params': {'confirm_level': 2.0}}}})
    supervisor = Supervisor(config)
    supervisor.set_disabled(['scan_amcl'])
    assert list(supervisor.detectors) == ['amcl_gnss']
    supervisor.set_weight('amcl_gnss', 0.5)
    supervisor.set_profile('strict')
    assert list(supervisor.detectors) == ['amcl_gnss']
    assert supervisor.fusion.instances['amcl_gnss'].weight == 0.5
    assert supervisor.fusion.params['confirm_level'] == 2.0
    supervisor.set_weight('amcl_gnss', None)
    assert supervisor.fusion.instances['amcl_gnss'].weight == 1.0
    with pytest.raises(ValueError):
        supervisor.set_profile('missing')
    with pytest.raises(ValueError):
        supervisor.set_disabled(['missing'])
    store = make_store(supervisor.config)
    names = [e.name for e in supervisor.step(store.snapshot(0.0)).events]
    assert names == ['localization.detectors_changed', 'localization.profile_changed']
