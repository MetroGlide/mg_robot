"""Snapshot の段階での故障注入と、評価の集計のテスト。"""
import numpy as np
import pytest

from localization_supervisor.core.evaluation import TickRecord, evaluate, format_report
from localization_supervisor.core.faults import (
    FaultInjectingStore, FaultInjector, parse_faults, validate_sources)
from localization_supervisor.core.interfaces import MotionSample, SourceKind, Status
from localization_supervisor.core.state_machine import State

from supervisor_test_helpers import pose_sample, room_scan

KINDS = {'amcl': SourceKind.POSE, 'gnss': SourceKind.POSE, 'ekf': SourceKind.POSE,
         'odom': SourceKind.MOTION, 'map_scan': SourceKind.REFERENCE}


def _store(faults, t0=100.0):
    return FaultInjectingStore(FaultInjector(parse_faults({'faults': faults}), t0), KINDS, ('odom',),
                               horizon_sec=1000.0)


def test_parse_rejects_invalid():
    with pytest.raises(ValueError, match='unknown type'):
        parse_faults({'faults': [{'type': 'nope', 'source': 'amcl', 'at': 1.0}]})
    with pytest.raises(ValueError, match='end is required'):
        parse_faults({'faults': [{'type': 'source_drop', 'source': 'gnss', 'start': 1.0}]})
    with pytest.raises(ValueError, match="'rate' is required"):
        parse_faults({'faults': [{'type': 'pose_drift', 'source': 'amcl', 'start': 1.0, 'end': 2.0}]})
    with pytest.raises(ValueError, match='must be pose sources'):
        validate_sources(parse_faults({'faults': [
            {'type': 'pose_offset', 'source': 'odom', 'at': 1.0, 'offset': {'x': 1.0}}]}), KINDS)


def test_pose_offset_persists_and_followers_share():
    store = _store([{'type': 'pose_offset', 'source': 'amcl', 'at': 10.0, 'offset': {'x': 3.0, 'yaw': 0.1},
                     'followers': {'ekf': 0.5}}])
    store.add_pose('amcl', pose_sample(105.0, (1.0, 2.0, 0.0)))
    store.add_pose('amcl', pose_sample(115.0, (1.0, 2.0, 0.0)))
    store.add_pose('amcl', pose_sample(500.0, (1.0, 2.0, 0.0)))
    poses = [s.pose for s in store.poses['amcl'].samples()]
    assert poses[0] == pytest.approx((1.0, 2.0, 0.0))
    assert poses[1] == pytest.approx((4.0, 2.0, 0.1))
    assert poses[2] == pytest.approx((4.0, 2.0, 0.1))
    store.add_pose('ekf', pose_sample(115.0, (0.0, 0.0, 0.0)))
    assert store.poses['ekf'].latest().pose == pytest.approx((1.5, 0.0, 0.05))
    store.add_pose('gnss', pose_sample(115.0, (0.0, 0.0, 0.0)))
    assert store.poses['gnss'].latest().pose == (0.0, 0.0, 0.0)


def test_pose_drift_grows_then_holds():
    store = _store([{'type': 'pose_drift', 'source': 'amcl', 'start': 10.0, 'end': 20.0, 'rate': {'x': 0.1}}])
    for t in (5.0, 15.0, 20.0, 50.0):
        store.add_pose('amcl', pose_sample(100.0 + t, (0.0, 0.0, 0.0)))
    assert [s.pose[0] for s in store.poses['amcl'].samples()] == pytest.approx([0.0, 0.5, 1.0, 1.0])


def test_source_drop_and_scan_faults():
    store = _store([
        {'type': 'source_drop', 'source': 'gnss', 'start': 0.0, 'end': 10.0},
        {'type': 'scan_sector_drop', 'source': 'map_scan', 'start': 0.0, 'end': 10.0, 'sector_deg': [-90, 90]},
        {'type': 'scan_thin', 'source': 'map_scan', 'start': 20.0, 'end': 30.0, 'keep_ratio': 0.25}])
    store.add_pose('gnss', pose_sample(105.0, (0.0, 0.0, 0.0)))
    store.add_pose('gnss', pose_sample(111.0, (0.0, 0.0, 0.0)))
    assert len(store.poses['gnss']) == 1
    store.set_scan('map_scan', room_scan(105.0, (0.0, 0.0, 0.0)))
    scan = store.references['map_scan'].scan
    angles = np.degrees(scan.angle_min + scan.angle_increment * np.arange(scan.ranges.size))
    assert np.all(np.isinf(scan.ranges[np.abs(angles) <= 90.0]))
    assert np.all(np.isfinite(scan.ranges[np.abs(angles) > 91.0]))
    store.set_scan('map_scan', room_scan(125.0, (0.0, 0.0, 0.0)))
    assert np.isfinite(store.references['map_scan'].scan.ranges).sum() == 90
    store.add_motion('odom', MotionSample(105.0, (0.0, 0.0, 0.0)))
    assert len(store.motions['odom']) == 1


def _records(states, anomalies=()):
    return [TickRecord(float(t), s, 0.0, 'amcl' if s != State.NORMAL else '',
                       {'check': Status.ANOMALY if t in anomalies else Status.NORMAL})
            for t, s in enumerate(states)]


def test_evaluation_detection_and_false_alarms():
    faults = parse_faults({'faults': [
        {'type': 'pose_offset', 'source': 'amcl', 'start': 10.0, 'end': 20.0, 'offset': {'x': 3.0}},
        {'type': 'source_drop', 'source': 'gnss', 'start': 40.0, 'end': 50.0}]})
    states = [State.NORMAL] * 60
    states[12] = State.SUSPECT
    states[13:16] = [State.DEGRADED] * 3
    states[45:47] = [State.DEGRADED] * 2   # 標本が欠けるだけの故障の間の確認は誤検知
    report = evaluate(_records(states, anomalies={3, 12, 13}), faults, settle_sec=5.0)
    offset = report.faults[0]
    assert offset.expected and offset.suspect_after == 2.0 and offset.confirm_after == 3.0
    assert offset.blame == 'amcl' and offset.instance_after['check'] == 2.0
    assert not report.faults[1].expected
    assert report.detected == 1 and report.false_alarms == 1
    stats = report.instances['check']
    assert stats.detected == 1 and stats.false_positive_ticks == 1
    assert stats.clean_known_ticks == 60 - 16
    text = format_report(report, 'x')
    assert 'pose_offset' in text and 'check' in text


def test_evaluation_without_records():
    assert evaluate([], []).detected == 0
