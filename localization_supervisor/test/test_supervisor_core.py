"""中核 (時刻のそろえ方、判定、総合、状態遷移、アンカー、Supervisor) のテスト。"""
import math

import numpy as np
import pytest

from localization_supervisor.core.anchors import Anchor, AnchorConfig, AnchorStore
from localization_supervisor.core.config import parse_config
from localization_supervisor.core.detectors.covariance import Covariance
from localization_supervisor.core.detectors.pose_consistency import PoseConsistency
from localization_supervisor.core.detectors.pose_jump import PoseJump
from localization_supervisor.core.detectors.scan_map import ScanMap
from localization_supervisor.core.fusion import ConsecutiveFusion, CusumFusion, attribute
from localization_supervisor.core.geometry import compose
from localization_supervisor.core.interfaces import (
    CONTRACT_VERSION, Evidence, FusionResult, InstanceInfo, MotionSample, SourceKind, Status)
from localization_supervisor.core.plugins import resolve_params
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.core.state_machine import Action, MachineConfig, State, StateMachine
from localization_supervisor.core.supervisor import Supervisor, make_store
from localization_supervisor.core.timeline import AlignmentConfig

from supervisor_test_helpers import Drive, pose_sample, room_scan, wall_map

KINDS = {'amcl': SourceKind.POSE, 'gnss': SourceKind.POSE, 'odom': SourceKind.MOTION,
         'map_scan': SourceKind.REFERENCE}


def _store(alignment=AlignmentConfig(0.0, 0.0)):
    return SampleStore(KINDS, ('odom',), horizon_sec=60.0, max_extrapolation_sec=0.5, alignment=alignment)


def _detector(cls, name, sources, **params):
    return cls(name, resolve_params(cls, params, name), sources)


# ---------------------------------------------------------------- timeline

def test_align_moves_pose_by_motion_between_stamps():
    drive = Drive()
    store = _store()
    for i in range(101):
        store.add_motion('odom', drive.motion(i * 0.1))
    store.add_pose('amcl', pose_sample(3.0, drive.map(3.0)))
    snap = store.snapshot(10.0)
    aligned, age = snap.pose_aligned('amcl')
    assert age == pytest.approx(7.0)
    assert aligned.pose == pytest.approx(drive.map(10.0), abs=1e-3)


def test_align_adds_uncertainty_proportional_to_motion():
    drive = Drive(speed=1.0, yaw_rate=0.0)
    store = _store(AlignmentConfig(std_per_m=0.1, std_per_rad=0.0))
    for i in range(11):
        store.add_motion('odom', drive.motion(float(i)))
    store.add_pose('amcl', pose_sample(0.0, drive.map(0.0), std_xy=0.0))
    aligned, _ = store.snapshot(10.0).pose_aligned('amcl')
    assert aligned.cov[0, 0] == pytest.approx(1.0)


def test_align_outside_motion_history_is_unknown():
    drive = Drive()
    store = _store()
    for i in range(10, 21):
        store.add_motion('odom', drive.motion(float(i)))
    store.add_pose('amcl', pose_sample(2.0, drive.map(2.0)))
    assert store.snapshot(20.0).pose_aligned('amcl') is None


def test_motion_priority_falls_back_to_next_source():
    kinds = {'amcl': SourceKind.POSE, 'lidar_odom': SourceKind.MOTION, 'wheel': SourceKind.MOTION}
    store = SampleStore(kinds, ('lidar_odom', 'wheel'), alignment=AlignmentConfig(0.0, 0.0))
    drive = Drive()
    for i in range(11):
        store.add_motion('wheel', drive.motion(float(i)))
    store.add_motion('lidar_odom', drive.motion(10.0))
    motion = store.timeline.motion(2.0, 10.0)
    assert motion.source == 'wheel'


# ---------------------------------------------------------------- detectors

def test_pose_consistency_is_not_fooled_by_stamp_gap_while_moving():
    # 2 m/s で動いていて、GNSS は 1 秒古い。そろえなければ 2 m の食い違いに見える
    drive = Drive(speed=2.0, yaw_rate=0.0)
    store = _store()
    for i in range(51):
        store.add_motion('odom', drive.motion(i * 0.1))
    store.add_pose('amcl', pose_sample(5.0, drive.map(5.0)))
    store.add_pose('gnss', pose_sample(4.0, drive.map(4.0)))
    detector = _detector(PoseConsistency, 'amcl_gnss', {'a': 'amcl', 'b': 'gnss'}, threshold=6.0)
    evidence = detector.evaluate(store.snapshot(5.0))
    assert evidence.status == Status.NORMAL
    assert evidence.values['distance'] < 0.05


def test_pose_consistency_detects_offset_and_respects_std_limit():
    drive = Drive()
    store = _store()
    for i in range(51):
        store.add_motion('odom', drive.motion(i * 0.1))
    shifted = compose(drive.map(5.0), (3.0, 0.0, 0.0))
    store.add_pose('amcl', pose_sample(5.0, shifted))
    store.add_pose('gnss', pose_sample(5.0, drive.map(5.0), std_xy=0.5))
    detector = _detector(PoseConsistency, 'amcl_gnss', {'a': 'amcl', 'b': 'gnss'}, threshold=6.0)
    evidence = detector.evaluate(store.snapshot(5.0))
    assert evidence.status == Status.ANOMALY and evidence.score > 1.0
    limited = _detector(PoseConsistency, 'amcl_gnss', {'a': 'amcl', 'b': 'gnss'}, threshold=6.0, b_max_std_m=0.3)
    assert limited.evaluate(store.snapshot(5.0)).status == Status.UNKNOWN
    distance = _detector(PoseConsistency, 'diff', {'a': 'amcl', 'b': 'gnss'}, metric='distance', threshold=1.0)
    assert distance.evaluate(store.snapshot(5.0)).values['value'] == pytest.approx(3.0)


def test_pose_jump_detects_only_new_jumps():
    drive = Drive()
    store = _store()
    detector = _detector(PoseJump, 'jump', {'pose': 'amcl', 'motion': 'odom'})
    results = []
    for i in range(30):
        t = float(i)
        store.add_motion('odom', drive.motion(t))
        pose = drive.map(t) if i < 20 else compose(drive.map(t), (2.0, 0.0, 0.0))
        store.add_pose('amcl', pose_sample(t, pose))
        results.append(detector.evaluate(store.snapshot(t)))
    assert all(r.status != Status.ANOMALY for r in results[:20])
    assert results[20].status == Status.ANOMALY
    assert results[20].values['jump_m'] == pytest.approx(2.0, abs=1e-6)
    # 飛んだ後の位置で落ち着けば、正常に戻る
    assert all(r.status == Status.NORMAL for r in results[21:])
    # 新しい標本がなければ判定しない
    assert detector.evaluate(store.snapshot(30.0)).status == Status.UNKNOWN


def test_covariance_metrics():
    store = _store()
    store.add_pose('amcl', pose_sample(1.0, (0.0, 0.0, 0.0), std_xy=1.0))
    snap = store.snapshot(1.0)
    trace = _detector(Covariance, 'cov', {'pose': 'amcl'}, threshold=1.0)
    assert trace.evaluate(snap).values['value'] == pytest.approx(2.0)
    det = _detector(Covariance, 'cov', {'pose': 'amcl'}, metric='determinant_xy', threshold=2.0)
    assert det.evaluate(snap).status == Status.NORMAL
    with pytest.raises(ValueError):
        _detector(Covariance, 'cov', {'pose': 'amcl'}, metric='bad', threshold=1.0)


def _scan_store(true_pose, amcl_pose, t=1.0):
    store = _store()
    store.set_map('map_scan', wall_map())
    store.set_scan('map_scan', room_scan(t, true_pose))
    store.add_motion('odom', MotionSample(t - 1.0, (0.0, 0.0, 0.0)))
    store.add_motion('odom', MotionSample(t, (0.0, 0.0, 0.0)))
    store.add_pose('amcl', pose_sample(t, amcl_pose))
    return store


def test_scan_map_normal_and_shifted():
    detector = _detector(ScanMap, 'scan', {'pose': 'amcl', 'reference': 'map_scan'}, search_range_m=0.5)
    good = detector.evaluate(_scan_store((1.0, 2.0, 0.3), (1.0, 2.0, 0.3)).snapshot(1.0))
    assert good.status == Status.NORMAL and good.values['ratio'] > 0.9
    bad = detector.evaluate(_scan_store((1.0, 2.0, 0.3), (1.0, 2.0, 0.8)).snapshot(1.0))
    assert bad.values['ratio'] < 0.5
    # 正常なときの一致率を基準にすると、異常になる
    for _ in range(20):
        detector.after_step(good, normal=True)
    bad = detector.evaluate(_scan_store((1.0, 2.0, 0.3), (1.0, 2.0, 0.8)).snapshot(1.0))
    assert bad.status == Status.ANOMALY


def test_scan_map_learns_baseline_only_when_normal():
    detector = _detector(ScanMap, 'scan', {'pose': 'amcl', 'reference': 'map_scan'},
                         baseline_min_samples=3, ratio_factor=0.9)
    snap = _scan_store((1.0, 2.0, 0.3), (1.0, 2.0, 0.3)).snapshot(1.0)
    evidence = detector.evaluate(snap)
    for _ in range(3):
        detector.after_step(evidence, normal=False)
    assert detector.ratio_threshold() == pytest.approx(0.1)
    for _ in range(3):
        detector.after_step(evidence, normal=True)
    assert detector.ratio_threshold() == pytest.approx(0.9 * evidence.values['ratio'])


def test_scan_map_shares_points_and_field_within_a_tick():
    store = _scan_store((1.0, 2.0, 0.3), (1.0, 2.0, 0.3))
    a = _detector(ScanMap, 'a', {'pose': 'amcl', 'reference': 'map_scan'})
    b = _detector(ScanMap, 'b', {'pose': 'amcl', 'reference': 'map_scan'}, tolerance_m=0.2)
    snap = store.snapshot(1.0)
    a.evaluate(snap)
    b.evaluate(snap)
    assert store.references['map_scan'].fields.builds == 1
    assert len([k for k in snap._cache if k[0] == 'scan_points']) == 1


# ---------------------------------------------------------------- fusion

def _infos(*specs):
    return [InstanceInfo(name=n, subjects=list(s), sources=list(s), **kw) for n, s, kw in specs]


def _ev(status, score=0.0):
    return Evidence(Status(status), score)


def test_cusum_single_big_anomaly_confirms_quickly():
    fusion = CusumFusion('cusum', {'suspect_level': 1.0, 'confirm_level': 5.0,
                                   'blame_margin': 1.0},
                         _infos(('jump', ['amcl'], {'max_score': 10.0})), ['amcl'], {})
    result = fusion.update({'jump': _ev('anomaly', 20.0)}, active=True)
    assert result.confirmed and result.level == pytest.approx(9.0)


def test_cusum_accumulates_small_anomalies_and_decays():
    fusion = CusumFusion('cusum', {'suspect_level': 1.0, 'confirm_level': 3.0,
                                   'blame_margin': 1.0},
                         _infos(('drift', ['amcl'], {})), ['amcl'], {})
    levels = [fusion.update({'drift': _ev('anomaly', 1.3)}, True).level for _ in range(10)]
    assert levels[0] == pytest.approx(0.3)
    assert levels[-1] == pytest.approx(3.0)
    # 判定できない周期は変えない、正常な周期は減る
    assert fusion.update({'drift': _ev('unknown')}, True).level == pytest.approx(3.0)
    assert fusion.update({'drift': _ev('normal', 0.2)}, True).level == pytest.approx(2.2)
    # 猶予の間は進めない
    assert fusion.update({'drift': _ev('anomaly', 5.0)}, False).level == pytest.approx(2.2)


def test_cusum_ignores_zero_weight_instances():
    fusion = CusumFusion('cusum', {'suspect_level': 1.0, 'confirm_level': 3.0, 'blame_margin': 1.0},
                         _infos(('gnss', ['amcl', 'gnss'], {}), ('observed', ['amcl'], {'weight': 0.0})),
                         ['amcl', 'gnss'], {})
    result = fusion.update({'gnss': _ev('normal', 0.1), 'observed': _ev('anomaly', 9.0)}, True)
    # 重み 0 の異常は、累積にも「異常がなくなった」の判定にも効かない
    assert result.level == 0.0 and not result.anomaly and result.cleared


def test_attribute_blame_uses_independent_sources_only():
    instances = {i.name: i for i in _infos(
        ('amcl_gnss', ['amcl', 'gnss'], {}), ('amcl_jump', ['amcl'], {}), ('scan_ekf', ['ekf'], {}),
        ('amcl_ekf', ['amcl', 'ekf'], {}))}
    independent = ['amcl', 'gnss', 'odom']
    derived = {'ekf': ['amcl', 'gnss', 'odom']}

    def run(values, peaks=None):
        values = {'amcl_gnss': 0.0, 'amcl_jump': 0.0, 'scan_ekf': 0.0, 'amcl_ekf': 0.0, **values}
        peaks = {**values, **(peaks or {})}
        return attribute(peaks, values, instances, independent, derived, 1.0)

    # 2 つの情報源の食い違いだけでは決めきれない
    assert run({'amcl_gnss': 9.0}) == ('unknown', [])
    # AMCL だけを疑う判定が加われば AMCL
    assert run({'amcl_gnss': 9.0, 'amcl_jump': 3.0}) == ('amcl', [])
    # 飛びの累積が減っても、疑いが始まってからの最大値で数える
    assert run({'amcl_gnss': 20.0}, {'amcl_jump': 3.0}) == ('amcl', [])
    # 融合の結果が関わる判定は blame に数えず、汚れているとする
    assert run({'scan_ekf': 2.0}) == ('unknown', ['ekf'])
    assert run({}) == ('', [])
    # 融合の結果との食い違いは、相手が blame なら融合の結果を疑わない
    assert run({'amcl_gnss': 9.0, 'amcl_jump': 3.0, 'amcl_ekf': 2.0}) == ('amcl', [])
    assert run({'amcl_gnss': 9.0, 'amcl_jump': 3.0, 'amcl_ekf': 2.0, 'scan_ekf': 1.0}) == ('amcl', ['ekf'])


def test_consecutive_jump_needs_corroboration():
    infos = _infos(('jump', ['amcl'], {'options': {'suspect_after': 1, 'confirm_after': 0,
                                                    'corroborate_after': 2}}),
                   ('scan', ['amcl'], {'options': {'suspect_after': 5, 'confirm_after': 10}}))
    fusion = ConsecutiveFusion('consecutive', {'blame_margin': 1.0}, infos, ['amcl'], {})
    r = fusion.update({'jump': _ev('anomaly', 3.0), 'scan': _ev('normal')}, True)
    assert r.suspect and not r.confirmed
    r = fusion.update({'jump': _ev('unknown'), 'scan': _ev('anomaly', 2.0)}, True)
    assert r.suspect and not r.confirmed
    r = fusion.update({'jump': _ev('unknown'), 'scan': _ev('anomaly', 2.0)}, True)
    assert r.confirmed and r.blame == 'amcl'


# ---------------------------------------------------------------- state machine

def _fr(suspect=False, confirmed=False, cleared=False, anomaly=False, blame='', reason='x'):
    return FusionResult(level=0.0, suspect=suspect, confirmed=confirmed, cleared=cleared, anomaly=anomaly,
                        blame=blame, reason=reason)


def test_notify_only_path():
    machine = StateMachine(MachineConfig(recovery_enabled=False, clear_ticks=2))
    assert machine.step(0.0, _fr(suspect=True, anomaly=True)).state == State.SUSPECT
    d = machine.step(1.0, _fr(suspect=True, confirmed=True, anomaly=True, blame='amcl'))
    assert d.state == State.DEGRADED and d.actions == [Action.NOTIFY_DEGRADED] and d.blame == 'amcl'
    assert not machine.hold_requested
    machine.step(2.0, _fr(cleared=True))
    machine.step(3.0, _fr(anomaly=True))
    machine.step(4.0, _fr(cleared=True))
    d = machine.step(5.0, _fr(cleared=True))
    assert d.state == State.NORMAL and d.actions == [Action.RESTORE]


def test_recovery_path_isolates_reinits_and_restores():
    machine = StateMachine(MachineConfig(recovery_enabled=True, isolate_hold_sec=2.0, recover_ok_ticks=2))
    machine.step(0.0, _fr(suspect=True))
    d = machine.step(1.0, _fr(confirmed=True, suspect=True, blame='amcl'))
    assert d.state == State.ISOLATED and d.actions == [Action.ISOLATE] and machine.hold_requested
    assert machine.step(2.0, _fr()).state == State.ISOLATED
    d = machine.step(3.0, _fr())
    assert d.state == State.RECOVERING and d.actions == [Action.REINIT]
    machine.step(4.0, _fr(), converged=True)
    machine.step(5.0, _fr(), converged=False)
    machine.step(6.0, _fr(), converged=True)
    d = machine.step(7.0, _fr(), converged=True)
    assert d.state == State.NORMAL and d.actions == [Action.RESTORE] and not machine.hold_requested


def _recovering(clear_ticks=2):
    machine = StateMachine(MachineConfig(
        recovery_enabled=True, isolate_hold_sec=1.0, recover_ok_ticks=2, recover_timeout_sec=10.0,
        max_reinit_attempts=1, degraded_retry_sec=100.0, clear_ticks=clear_ticks))
    machine.step(0.0, _fr(suspect=True, anomaly=True))
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert machine.step(2.0, _fr(anomaly=True)).state == State.RECOVERING
    return machine


def test_recovering_without_reinit_returns_to_normal_when_the_anomaly_clears():
    # 一時的な外れ値で確認に進んでも、初期化し直す前に異常がなくなれば、初期化せずに戻す(停止し続けない)
    machine = _recovering()
    assert machine.step(3.0, _fr(cleared=True)).state == State.RECOVERING
    d = machine.step(4.0, _fr(cleared=True))
    assert d.state == State.NORMAL and d.actions == [Action.RESTORE] and d.event == 'cleared'
    assert not machine.hold_requested


def test_recovering_after_reinit_waits_for_convergence_not_for_the_anomaly_to_clear():
    machine = _recovering()
    machine.note_reinit()
    for t in (3.0, 4.0, 5.0):
        assert machine.step(t, _fr(cleared=True), converged=None).state == State.RECOVERING


def test_degraded_with_recovery_returns_to_normal_when_the_anomaly_clears():
    machine = _recovering()
    machine.note_reinit()
    d = machine.step(12.0, _fr(anomaly=True), converged=False)
    assert d.state == State.DEGRADED and machine.hold_requested
    machine.step(13.0, _fr(cleared=True))
    d = machine.step(14.0, _fr(cleared=True))
    assert d.state == State.NORMAL and d.actions == [Action.RESTORE] and not machine.hold_requested


# ---------------------------------------------------------------- anchors

def test_anchor_store_limits_and_lookup():
    store = AnchorStore(AnchorConfig('ekf', 'odom', min_interval_sec=1.0, max_age_sec=10.0, max_distance_m=5.0))
    for i in range(20):
        store.add(Anchor(i * 0.5, pose_sample(i * 0.5, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0), odometer=i * 0.5))
    assert len(store) == 10
    store.prune(now=9.5, odometer=9.5)
    assert store.latest_before(100.0).stamp == 9.0
    assert store.latest_before(5.2).stamp == 5.0
    assert store.latest_before(4.5) is None  # 5 m より前は捨てた


# ---------------------------------------------------------------- supervisor

def _config(**overrides):
    raw = {
        'version': CONTRACT_VERSION,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'}},
        'timeline': {'motion_priority': ['odom'], 'alignment_std_per_m': 0.0, 'alignment_std_per_rad': 0.0},
        'detectors': {
            'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 6.0}},
            'amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'}, 'max_score': 10.0},
        },
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 5.0}},
        'state_machine': {'clear_ticks': 3},
        'anchors': {'pose': 'gnss', 'motion': 'odom', 'min_interval_sec': 1.0},
    }
    raw.update(overrides)
    return parse_config(raw)


def _feed(supervisor, store, drive, t, amcl_offset=(0.0, 0.0, 0.0)):
    store.add_motion('odom', drive.motion(t))
    store.add_pose('amcl', pose_sample(t, compose(drive.map(t), amcl_offset)))
    store.add_pose('gnss', pose_sample(t, drive.map(t)))
    return supervisor.step(store.snapshot(t))


def test_supervisor_detects_kidnap_and_blames_amcl():
    config = _config()
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    results = [_feed(supervisor, store, drive, float(t)) for t in range(10)]
    assert all(r.state == State.NORMAL for r in results)
    assert all(r.trusted for r in results)
    assert len(supervisor.anchors) == 10
    results = [_feed(supervisor, store, drive, float(t), (3.0, 0.0, 0.0)) for t in range(10, 13)]
    assert results[0].state == State.SUSPECT
    assert results[1].state == State.DEGRADED
    assert results[1].fusion.blame == 'amcl'
    assert [e.name for e in results[0].events if e.name.endswith('.state_changed')] == ['localization.state_changed']
    checks = [e for e in results[0].events if e.name == 'localization.check_changed']
    assert checks and all(e.fields['to'] == 'anomaly' for e in checks)
    assert supervisor.suspect_since == 10.0
    assert supervisor.anchors.latest_before(10.0).stamp == 9.0
    # 戻れば NORMAL に戻る
    results = [_feed(supervisor, store, drive, float(t)) for t in range(13, 25)]
    assert results[-1].state == State.NORMAL


def test_supervisor_grace_suppresses_detection():
    config = _config(grace={'startup_sec': 5.0})
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    results = [_feed(supervisor, store, drive, float(t), (3.0, 0.0, 0.0)) for t in range(5)]
    assert all(r.state == State.NORMAL and r.in_grace for r in results)
    assert results[0].evidences['amcl_gnss'].status == Status.ANOMALY


def test_supervisor_profile_switch_and_weight():
    config = _config()
    config.profiles['only_gnss'] = {'enabled_detectors': ['amcl_gnss'], 'fusion': {'type': 'consecutive'}}
    supervisor = Supervisor(config)
    supervisor.set_profile('only_gnss')
    assert list(supervisor.detectors) == ['amcl_gnss']
    assert type(supervisor.fusion).__name__ == 'ConsecutiveFusion'
    supervisor.set_profile('')
    assert sorted(supervisor.detectors) == ['amcl_gnss', 'amcl_jump']
    supervisor.set_weight('amcl_gnss', 0.0)
    store = make_store(supervisor.config)
    drive = Drive()
    results = [_feed(supervisor, store, drive, float(t), (3.0, 0.0, 0.0)) for t in range(5)]
    assert all(r.fusion.accumulated['amcl_gnss'] == 0.0 for r in results)


def test_supervisor_ignores_unknown_evidence():
    config = _config()
    supervisor = Supervisor(config)
    store = make_store(config)
    results = [supervisor.step(store.snapshot(float(t))) for t in range(30)]
    assert all(r.state == State.NORMAL for r in results)
    assert all(e.status == Status.UNKNOWN for r in results for e in r.evidences.values())


def test_scan_points_are_invariant_to_heading_noise():
    # 回転だけずれた姿勢では一致率が下がる (向きの誤りを捉えられる)
    store = _scan_store((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(20)))
    detector = _detector(ScanMap, 'scan', {'pose': 'amcl', 'reference': 'map_scan'})
    assert detector.evaluate(store.snapshot(1.0)).values['ratio'] < 0.7
    assert np.isfinite(detector.evaluate(store.snapshot(1.0)).score)
