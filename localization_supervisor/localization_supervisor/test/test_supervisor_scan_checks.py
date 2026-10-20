"""スキャンを地図に照合する判定 (submap_map) と、スキャンのバッファのテスト。"""
import math

import pytest

from localization_supervisor.core.detectors.submap_map import SubmapMap
from localization_supervisor.core.geometry import compose
from localization_supervisor.core.interfaces import MotionSample, SourceKind, Status
from localization_supervisor.core.plugins import resolve_params
from localization_supervisor.core.scan_buffer import ScanBuffer
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.core.timeline import AlignmentConfig

from supervisor_test_helpers import Drive, pose_sample, room_scan, wall_map

KINDS = {'amcl': SourceKind.POSE, 'odom': SourceKind.MOTION, 'map_scan': SourceKind.REFERENCE}


def _store():
    store = SampleStore(KINDS, ('odom',), horizon_sec=60.0, max_extrapolation_sec=0.5,
                        alignment=AlignmentConfig(0.0, 0.0))
    store.set_map('map_scan', wall_map())
    return store


def _detector(cls, sources, **params):
    return cls('check', resolve_params(cls, params, 'check'), sources)


def _still(store, t, true_pose, amcl_pose):
    """止まっているロボット。スキャンは true_pose で、AMCL は amcl_pose。"""
    store.set_scan('map_scan', room_scan(t, true_pose))
    store.add_motion('odom', MotionSample(t - 1.0, (0.0, 0.0, 0.0)))
    store.add_motion('odom', MotionSample(t, (0.0, 0.0, 0.0)))
    store.add_pose('amcl', pose_sample(t, amcl_pose))
    return store.snapshot(t)


def test_scan_buffer_keeps_keyframes_within_distance():
    buffer = ScanBuffer(keyframe_distance_m=1.0, keyframe_angle_rad=1.0, window_distance_m=3.0,
                        max_points_per_scan=10, max_range_m=30.0)
    scan = room_scan(0.0, (0.0, 0.0, 0.0))
    assert buffer.add(scan, (0.0, 0.0, 0.0))
    assert not buffer.add(room_scan(0.1, (0.0, 0.0, 0.0)), (0.5, 0.0, 0.0))
    for i in range(1, 6):
        assert buffer.add(room_scan(float(i), (0.0, 0.0, 0.0)), (float(i), 0.0, 0.0))
    assert len(buffer) == 4 and buffer.travelled() == pytest.approx(3.0)
    # 時刻が戻ったら捨てる
    buffer.add(room_scan(0.5, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    assert len(buffer) == 1


def test_scan_buffer_points_follow_motion():
    buffer = ScanBuffer(0.1, 0.1, 100.0, 50, 30.0)
    drive = Drive(map_odom=(0.0, 0.0, 0.0), speed=0.5, yaw_rate=0.2)
    for t in (0.0, 2.0, 4.0):
        buffer.add(room_scan(t, drive.map(t)), drive.odom(t))
    # 正しい今の姿勢でまとめた点は、どれも壁の上にある
    points = buffer.points_in(drive.map(4.0), drive.odom(4.0))
    assert points.shape == (150, 2)
    assert (abs(abs(points).max(axis=1) - 9.95) < 0.05).all()


def _drive_submap(detector, offset=(0.0, 0.0, 0.0), duration=40):
    store = _store()
    drive = Drive(map_odom=(1.0, -2.0, 0.3), speed=0.5, yaw_rate=0.2)
    results = []
    for t in range(duration):
        t = float(t)
        store.add_motion('odom', drive.motion(t))
        store.set_scan('map_scan', room_scan(t, drive.map(t)))
        store.add_pose('amcl', pose_sample(t, compose(drive.map(t), offset)))
        results.append(detector.evaluate(store.snapshot(t)))
    return results


def test_submap_map_matches_at_intervals_after_enough_travel():
    detector = _detector(SubmapMap, {'pose': 'amcl', 'motion': 'odom', 'reference': 'map_scan'},
                         interval_sec=5.0, min_distance_m=5.0, gain_threshold=0.2)
    results = _drive_submap(detector)
    known = [i for i, r in enumerate(results) if r.status != Status.UNKNOWN]
    # 5 m 走るまで (10 s) は照合しない。その後は 5 s ごと
    assert known[0] >= 10
    assert all(b - a == 5 for a, b in zip(known, known[1:]))
    assert all(results[i].status == Status.NORMAL for i in known)
    assert results[known[-1]].values['ratio'] > 0.9


def test_submap_map_detects_offset_by_gain():
    detector = _detector(SubmapMap, {'pose': 'amcl', 'motion': 'odom', 'reference': 'map_scan'},
                         interval_sec=5.0, min_distance_m=5.0, gain_threshold=0.2)
    results = _drive_submap(detector, offset=(0.5, 0.0, 0.0))
    known = [r for r in results if r.status != Status.UNKNOWN]
    assert known and all(r.status == Status.ANOMALY for r in known)
    assert math.hypot(known[-1].values['dx'], known[-1].values['dy']) == pytest.approx(0.5, abs=0.15)


def test_submap_map_reset_clears_buffer():
    detector = _detector(SubmapMap, {'pose': 'amcl', 'motion': 'odom', 'reference': 'map_scan'})
    _drive_submap(detector, duration=15)
    assert len(detector.buffer) > 0
    detector.reset()
    assert len(detector.buffer) == 0
