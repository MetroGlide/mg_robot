"""オフライン評価の通しのテスト: 人工のメッセージの列と地図を作り、最小の設定例 (config/examples) で回す。"""
import math
import pathlib
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from PIL import Image

from localization_supervisor.core.config import load_config, parse_config
from localization_supervisor.core.faults import parse_faults
from localization_supervisor.core.state_machine import State
from localization_supervisor.ros.adapters import AdapterContext
from localization_supervisor.ros.map_io import load_map_yaml
from localization_supervisor.ros.offline_runner import OfflineOptions, OfflineSession, write_outputs

from supervisor_test_helpers import Drive, room_scan, wall_map

EXAMPLE = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'examples' / 'amcl_odom.yaml'
T_START = 1000.0
DURATION = 60.0


class FixedSensor(AdapterContext):
    """センサは base と同じ位置 (TF の代わり)。"""

    def lookup(self, parent, child, stamp=None):
        return (0.0, 0.0, 0.0)


def _header(t, frame):
    sec = int(math.floor(t))
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9))), frame_id=frame)


def _pose(x, y, yaw):
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)))


def _covariance(var_xy, var_yaw):
    cov = [0.0] * 36
    cov[0] = cov[7] = var_xy
    cov[35] = var_yaw
    return cov


def _messages():
    """(記録時刻 [ns], トピック, メッセージ) の列。オドメトリ 10 Hz、AMCL 2 Hz、スキャン 5 Hz。"""
    drive = Drive(map_odom=(-3.0, -3.0, 0.0), speed=0.2, yaw_rate=0.02)
    for i in range(int(DURATION * 10)):
        t = i * 0.1
        stamp = T_START + t
        t_ns = int(round(stamp * 1e9))
        odom = SimpleNamespace(header=_header(stamp, 'odom'), child_frame_id='base_link',
                               pose=SimpleNamespace(pose=_pose(*drive.odom(t)), covariance=_covariance(0.0, 0.0)))
        yield t_ns, '/odom', odom
        truth = drive.map(t)
        if i % 5 == 0:
            amcl = SimpleNamespace(header=_header(stamp, 'map'),
                                   pose=SimpleNamespace(pose=_pose(*truth), covariance=_covariance(0.01, 0.005)))
            yield t_ns, '/amcl_pose', amcl
        if i % 2 == 0:
            sample = room_scan(stamp, truth, n=360)
            yield t_ns, '/scan', SimpleNamespace(
                header=_header(stamp, 'laser'), ranges=list(sample.ranges), angle_min=sample.angle_min,
                angle_increment=sample.angle_increment, range_min=sample.range_min, range_max=sample.range_max)


@pytest.fixture(scope='module')
def map_yaml(tmp_path_factory):
    directory = tmp_path_factory.mktemp('offline')
    pixels = np.where(np.flipud(wall_map().data) >= 65, 0, 254).astype(np.uint8)
    Image.fromarray(pixels).save(directory / 'map.pgm')
    (directory / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [-10.0, -10.0, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return str(directory / 'map.yaml')


def _run(map_yaml, faults=()):
    options = OfflineOptions(maps={'map_scan': map_yaml}, faults=list(faults))
    session = OfflineSession(load_config(str(EXAMPLE)), options, FixedSensor(), T_START)
    return session.run(_messages())


def test_map_yaml_matches_occupancy(map_yaml):
    loaded = load_map_yaml(map_yaml)
    expected = wall_map()
    assert loaded.origin_x == expected.origin_x and loaded.resolution == expected.resolution
    assert np.array_equal(loaded.data >= 65, expected.data >= 65)


def test_example_config_runs_offline_without_false_alarm(map_yaml, tmp_path):
    result = _run(map_yaml)
    assert len(result.records) >= DURATION - 1
    assert all(r.state == State.NORMAL for r in result.records)
    after_grace = result.steps[15:]
    assert all(s['checks']['scan_amcl']['status'] == 'normal' for s in after_grace)
    assert all(s['trusted'] for s in after_grace)
    write_outputs(result, str(tmp_path / 'out'), 'example')
    assert (tmp_path / 'out' / 'report.md').read_text(encoding='utf-8').startswith('# オフライン評価')


def test_injected_offset_is_detected_and_blamed(map_yaml):
    faults = parse_faults({'faults': [
        {'type': 'pose_offset', 'source': 'amcl', 'at': 30.0, 'offset': {'x': 1.5, 'y': 0.0, 'yaw': 0.2}}]})
    result = _run(map_yaml, faults)
    outcome = result.report.faults[0]
    assert outcome.suspect_after == 1.0
    assert outcome.confirm_after is not None and outcome.confirm_after <= 5.0
    assert outcome.blame == 'amcl'
    assert result.report.false_alarms == 0
    assert [e['to'] for e in result.events][:2] == ['SUSPECT', 'DEGRADED']


def _recovery_config():
    with open(EXAMPLE, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    raw['state_machine'] = {'recovery_enabled': True, 'isolate_hold_sec': 2.0, 'recover_ok_ticks': 3}
    raw['recovery'] = {
        'convergence': ['scan_amcl'],
        'scan_buffer': {'reference': 'map_scan', 'motion': 'odom'},
        'generators': {
            'dead_reckoning': {'type': 'anchor_motion', 'sources': {'motion': 'odom'}},
            'submap': {'type': 'submap_match', 'sources': {'motion': 'odom', 'reference': 'map_scan'},
                       'params': {'peaks': 2, 'min_range_m': 0.6}}},
        'scorers': {'scan': {'type': 'scan_match', 'sources': {'reference': 'map_scan', 'motion': 'odom'}}},
        'selection': {'min_score': 0.5}}
    return parse_config(raw)


def test_recovery_is_evaluated_against_the_recorded_pose(map_yaml):
    faults = parse_faults({'faults': [
        {'type': 'pose_offset', 'source': 'amcl', 'at': 30.0, 'offset': {'x': 1.5, 'y': 0.0, 'yaw': 0.2}}]})
    options = OfflineOptions(maps={'map_scan': map_yaml}, faults=faults, apply_reset=['amcl'], truth='amcl')
    session = OfflineSession(_recovery_config(), options, FixedSensor(), T_START)
    result = session.run(_messages())
    report = result.report
    assert report.reinits and report.wrong_reinits == 0 and report.spurious_reinits == 0
    assert report.reinits[0].error_m < 0.5
    outcome = report.faults[0]
    assert outcome.reinit_after is not None and outcome.recovered_after is not None
    assert result.records[-1].state == State.NORMAL
    assert any('reinit' in step for step in result.steps)


def test_without_apply_reset_the_directive_is_only_recorded(map_yaml):
    faults = parse_faults({'faults': [
        {'type': 'pose_offset', 'source': 'amcl', 'at': 30.0, 'offset': {'x': 1.5, 'y': 0.0, 'yaw': 0.2}}]})
    options = OfflineOptions(maps={'map_scan': map_yaml}, faults=faults, truth='amcl')
    result = OfflineSession(_recovery_config(), options, FixedSensor(), T_START).run(_messages())
    assert result.report.reinits and result.report.reinits[0].error_m < 0.5
