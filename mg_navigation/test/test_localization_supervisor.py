import math

import numpy as np
import pytest

from mg_navigation.localization_supervisor import checks as ck
from mg_navigation.localization_supervisor import scan_check as sc
from mg_navigation.localization_supervisor.state_machine import (
    Action, Checks, MachineConfig, State, SupervisorMachine)


# ---------------------------------------------------------------- checks

def test_compose_and_inverse_roundtrip():
    a = (1.0, 2.0, 0.7)
    identity = ck.compose(a, ck.inverse(a))
    assert identity == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_implied_map_odom_is_constant_when_odom_is_consistent():
    map_odom = (10.0, -5.0, 0.4)
    poses = []
    for base in [(0.0, 0.0, 0.0), (3.0, 1.0, 0.5), (6.0, 2.0, -0.2)]:
        map_base = ck.compose(map_odom, base)
        poses.append(ck.implied_map_odom(map_base, base))
    for p in poses:
        assert p == pytest.approx(map_odom, abs=1e-12)
    assert ck.pose_jump(poses[0], poses[2], (6.0, 2.0, -0.2)) == pytest.approx((0.0, 0.0), abs=1e-12)


def test_pose_jump_detects_amcl_step():
    before = (10.0, -5.0, 0.4)
    after = (12.0, -5.0, 0.4 + 0.5)
    dpos, dyaw = ck.pose_jump(before, after, (0.0, 0.0, 0.0))
    assert dpos == pytest.approx(2.0)
    assert dyaw == pytest.approx(0.5)


def test_amcl_gnss_d2():
    # 差 1 m、AMCL の分散 0.25、GNSS の分散 0.0025 -> 1 / 0.2525
    d2 = ck.amcl_gnss_d2((1.0, 0.0), (0.25, 0.25), (0.0, 0.0), 0.0025)
    assert d2 == pytest.approx(1.0 / 0.2525)
    # 追加の分散を入れると小さくなる
    assert ck.amcl_gnss_d2((1.0, 0.0), (0.25, 0.25), (0.0, 0.0), 0.0025, 1.0) < d2


# ------------------------------------------------------------ scan_check

def _wall_map():
    """20 m 四方の、周囲が壁の地図 (0.1 m 解像度、原点 (-10, -10))。"""
    size = 200
    data = np.zeros((size, size), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    return sc.OccupancyMap(data, 0.1, -10.0, -10.0)


def _room_scan(pose, n=360, max_range=30.0):
    """四角い部屋 (壁は ±9.95 m) の中の pose から見たスキャンの距離を作る。"""
    angles = -math.pi + 2.0 * math.pi * np.arange(n) / n
    ranges = np.empty(n)
    for k, a in enumerate(angles):
        d = pose[2] + a
        dx, dy = math.cos(d), math.sin(d)
        ts = []
        for bound, origin, direction in ((9.95, pose[0], dx), (-9.95, pose[0], dx),
                                          (9.95, pose[1], dy), (-9.95, pose[1], dy)):
            if abs(direction) > 1e-9:
                t = (bound - origin) / direction
                if t > 0:
                    ts.append(t)
        ranges[k] = min(ts) if ts else max_range
    return ranges, -math.pi, 2.0 * math.pi / n


def test_match_ratio_high_at_true_pose_low_when_shifted():
    occupancy = _wall_map()
    truth = (2.0, -1.0, 0.3)
    ranges, angle_min, angle_inc = _room_scan(truth)
    field = sc.build_local_distance_field(occupancy, 0.0, 0.0, 15.0)
    assert field is not None
    lidar = (0.0, 0.0, 0.0)
    good = sc.scan_points_in_map(ranges, angle_min, angle_inc, 0.05, 30.0, truth, lidar)
    ratio_good, _ = sc.match_ratio(field, good, tolerance=0.3)
    shifted = (truth[0] + 1.5, truth[1], truth[2])
    bad = sc.scan_points_in_map(ranges, angle_min, angle_inc, 0.05, 30.0, shifted, lidar)
    ratio_bad, _ = sc.match_ratio(field, bad, tolerance=0.3)
    assert ratio_good > 0.95
    assert ratio_bad < ratio_good - 0.2


def test_match_gain_is_small_at_true_pose_and_large_when_shifted():
    occupancy = _wall_map()
    truth = (2.0, -1.0, 0.3)
    ranges, angle_min, angle_inc = _room_scan(truth)
    field = sc.build_local_distance_field(occupancy, 0.0, 0.0, 15.0)
    lidar = (0.0, 0.0, 0.0)
    offsets = sc.search_offsets(1.0, 0.25)
    assert offsets.shape == (81, 2)
    good = sc.scan_points_in_map(ranges, angle_min, angle_inc, 0.05, 30.0, truth, lidar)
    ratio_good, gain_good = sc.match_gain(field, good, 0.3, offsets)
    shifted = (truth[0] + 0.75, truth[1] - 0.5, truth[2])
    bad = sc.scan_points_in_map(ranges, angle_min, angle_inc, 0.05, 30.0, shifted, lidar)
    ratio_bad, gain_bad = sc.match_gain(field, bad, 0.3, offsets)
    assert gain_good == pytest.approx(0.0, abs=0.02)
    assert ratio_good > 0.95
    assert gain_bad > 0.3


def test_match_gain_none_without_enough_points():
    field = sc.LocalDistanceField(np.zeros((10, 10)), 0.0, 0.0, 0.1)
    assert sc.match_gain(field, np.zeros((5, 2)), 0.3, sc.search_offsets(0.5, 0.25)) is None


def test_local_distance_field_none_without_structure_and_lookup_outside():
    empty = sc.OccupancyMap(np.zeros((100, 100), dtype=np.int8), 0.1, 0.0, 0.0)
    assert sc.build_local_distance_field(empty, 5.0, 5.0, 4.0) is None
    field = sc.build_local_distance_field(_wall_map(), 0.0, 0.0, 5.0, min_occupied_cells=1)
    assert field is None or np.isnan(field.lookup(np.array([[500.0, 500.0]]))[0])


def test_match_ratio_needs_enough_points():
    field = sc.LocalDistanceField(np.zeros((10, 10)), 0.0, 0.0, 0.1)
    assert sc.match_ratio(field, np.zeros((5, 2)), 0.3, min_points=30) is None


def test_scan_points_are_thinned_and_filtered():
    ranges = np.full(720, 5.0)
    ranges[:100] = np.inf
    points = sc.scan_points_in_map(ranges, 0.0, 0.01, 0.05, 30.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                   max_points=100)
    assert points.shape == (100, 2)
    assert np.allclose(np.hypot(points[:, 0], points[:, 1]), 5.0)


# --------------------------------------------------------- state machine

def _run(machine, sequence, start=0.0):
    """(Checks, ...) の列を 1 秒ごとに与え、各回の Decision を返す。"""
    return [machine.step(start + i, c) for i, c in enumerate(sequence)]


def _normal(n):
    return [Checks(gnss=False, jump=False, scan=False, diff=False) for _ in range(n)]


def test_normal_operation_never_triggers():
    machine = SupervisorMachine()
    decisions = _run(machine, _normal(600))
    assert all(d.state == State.NORMAL and not d.actions for d in decisions)


def test_unavailable_checks_do_not_trigger():
    machine = SupervisorMachine()
    decisions = _run(machine, [Checks() for _ in range(100)])
    assert all(d.state == State.NORMAL for d in decisions)


def test_single_jump_only_suspects_then_clears():
    machine = SupervisorMachine()
    seq = _normal(3) + [Checks(jump=True, gnss=False, scan=False, diff=False)] + _normal(8)
    decisions = _run(machine, seq)
    states = [d.state for d in decisions]
    assert State.SUSPECT in states
    assert State.ISOLATED not in states
    assert states[-1] == State.NORMAL


def test_jump_with_corroboration_isolates():
    machine = SupervisorMachine()
    seq = _normal(2) + [Checks(jump=True, scan=True)] + [Checks(scan=True), Checks(scan=True)]
    decisions = _run(machine, seq)
    isolated = [d for d in decisions if d.state == State.ISOLATED]
    assert isolated and Action.ISOLATE in isolated[0].actions
    assert not machine.attached


def test_persistent_gnss_disagreement_isolates():
    machine = SupervisorMachine()
    decisions = _run(machine, [Checks(gnss=True) for _ in range(8)])
    assert decisions[2].state == State.SUSPECT
    assert decisions[5].state == State.ISOLATED and decisions[5].actions == [Action.ISOLATE]


def test_recovery_path_and_attach():
    cfg = MachineConfig(isolate_hold_sec=3.0, recover_ok_ticks=3)
    machine = SupervisorMachine(cfg)
    decisions = _run(machine, [Checks(gnss=True) for _ in range(6)])
    assert decisions[-1].state == State.ISOLATED
    t = 6.0
    # 待機中は何もしない。待った後に初期化し直す
    d = machine.step(t + 1.0, Checks())
    assert d.state == State.ISOLATED and not d.actions
    d = machine.step(t + 3.0, Checks())
    assert d.state == State.RECOVERING and d.actions == [Action.REINIT]
    # 初期化後の飛びは無視し、正常が続けば戻す
    for i in range(2):
        d = machine.step(t + 4.0 + i, Checks(jump=True, gnss=False, scan=True, converged=True))
        assert d.state == State.RECOVERING
    d = machine.step(t + 6.0, Checks(gnss=False, scan=True, converged=True))
    assert d.state == State.NORMAL and d.actions == [Action.ATTACH]
    assert machine.attached


def test_recovery_needs_amcl_to_converge_near_ekf():
    cfg = MachineConfig(isolate_hold_sec=1.0, recover_ok_ticks=3, recover_timeout_sec=100.0)
    machine = SupervisorMachine(cfg)
    _run(machine, [Checks(gnss=True) for _ in range(6)])
    t = 6.0
    assert machine.step(t + 1.0, Checks()).state == State.RECOVERING
    # AMCL がまだ離れている、または収束を判定できない間は戻さない
    for i in range(5):
        assert machine.step(t + 2.0 + i, Checks(gnss=False, converged=False)).state == State.RECOVERING
        assert machine.step(t + 2.5 + i, Checks(gnss=False, converged=None)).state == State.RECOVERING
    # 途中で離れたら数え直す
    machine.step(t + 10.0, Checks(converged=True))
    machine.step(t + 11.0, Checks(converged=True))
    machine.step(t + 12.0, Checks(converged=False))
    assert machine.step(t + 13.0, Checks(converged=True)).state == State.RECOVERING
    machine.step(t + 14.0, Checks(converged=True))
    assert machine.step(t + 15.0, Checks(converged=True)).state == State.NORMAL


def test_recovery_timeout_retries_then_degrades_then_retries():
    cfg = MachineConfig(isolate_hold_sec=1.0, recover_timeout_sec=10.0, max_reinit_attempts=2,
                        degraded_retry_sec=20.0)
    machine = SupervisorMachine(cfg)
    _run(machine, [Checks(gnss=True) for _ in range(6)])
    t = 6.0
    assert machine.step(t + 1.0, Checks()).state == State.RECOVERING
    # 異常が続いて時間切れ -> 再試行 -> 時間切れ -> DEGRADED
    d = machine.step(t + 11.0, Checks(gnss=True))
    assert d.actions == [Action.REINIT] and d.event.startswith('reinit retry')
    d = machine.step(t + 21.0, Checks(gnss=True))
    assert d.state == State.DEGRADED and d.actions == [Action.NOTIFY_DEGRADED]
    assert not machine.attached
    assert machine.step(t + 30.0, Checks()).state == State.DEGRADED
    d = machine.step(t + 41.0, Checks())
    assert d.state == State.RECOVERING and d.actions == [Action.REINIT]


def test_pose_jump_is_not_amplified_by_distance_from_odom_origin():
    # odom の原点から 500 m 離れていると、AMCL の yaw が 0.01 rad ぶれるだけで、implied な map->odom の
    # 並進は 5 m 変わる。ロボットの姿勢としては同じなので、飛びは 0 (並進どうしを比べると誤検知になる)
    base_before = (500.0, 0.0, 0.0)
    base_after = (500.5, 0.0, 0.0)
    a = ck.implied_map_odom((10.0, 20.0, 0.30), base_before)
    b = ck.implied_map_odom((10.5, 20.0, 0.31), base_after)
    assert math.hypot(b[0] - a[0], b[1] - a[1]) > 4.0
    dpos, dyaw = ck.pose_jump(a, b, base_after)
    assert dpos < 0.2
    assert dyaw == pytest.approx(0.01)
