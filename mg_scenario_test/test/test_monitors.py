"""MG-01 固有の monitor の単体テスト。"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from mg_scenario_test.monitors import (
    LocalizationHoldMonitor,
    LocalizationHoldSpec,
    LocalizationOutcomeMonitor,
    LocalizationOutcomeSpec,
    LocalizationReady,
    LocalizationReadySpec,
    LocalizationStateMonitor,
    LocalizationStateSpec,
)
from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.geometry import Pose


def _status(hold, speed_limit=0.0):
    return SimpleNamespace(hold_requested=hold, speed_limit=speed_limit)


def _monitor(spec):
    ctx = MagicMock()
    clock = {"t": 0.0}
    ctx.clock.now = lambda: clock["t"]
    return LocalizationHoldMonitor(ctx, spec), clock


def test_localization_hold_monitor_sums_hold_time():
    monitor, clock = _monitor(LocalizationHoldSpec(max_total_sec=5.0))
    assert monitor.result().status == ResultStatus.ERROR
    for t, hold in ((0.0, False), (1.0, True), (3.0, True), (4.0, False), (6.0, True), (8.0, False)):
        clock["t"] = t
        monitor._callback(_status(hold))
    result = monitor.result()
    assert result.status == ResultStatus.PASSED
    assert "2 time(s), 5.0 s in total" in result.message


def test_localization_hold_monitor_fails_when_still_holding_or_too_long():
    monitor, clock = _monitor(LocalizationHoldSpec())
    for t, hold in ((0.0, False), (1.0, True)):
        clock["t"] = t
        monitor._callback(_status(hold))
    clock["t"] = 4.0
    result = monitor.result()
    assert result.status == ResultStatus.FAILED and "still holding" in result.message
    limited, clock = _monitor(LocalizationHoldSpec(max_total_sec=1.0, ends_released=False))
    for t, hold in ((0.0, True), (2.0, False)):
        clock["t"] = t
        limited._callback(_status(hold))
    assert limited.result().status == ResultStatus.FAILED


def _loc_status(state=0, blame="", in_grace=False, checks=()):
    return SimpleNamespace(
        state=state, blame=blame, in_grace=in_grace,
        checks=[SimpleNamespace(name=name, status=status) for name, status in checks])


def test_localization_state_monitor_counts_only_matching_blame():
    monitor = LocalizationStateMonitor(MagicMock(), LocalizationStateSpec(state="RECOVERING", blame="amcl"))
    monitor._callback(_loc_status(state=3, blame="gnss"))
    assert monitor.result().status == ResultStatus.FAILED
    monitor._callback(_loc_status(state=3, blame="amcl"))
    result = monitor.result()
    assert result.status == ResultStatus.PASSED and "with blame amcl" in result.message


def test_localization_ready_waits_for_settled_normal_and_checks():
    ctx = MagicMock()
    clock = {"t": 0.0}
    ctx.clock.now = lambda: clock["t"]
    trigger = LocalizationReady(ctx, LocalizationReadySpec(for_sec=3.0, checks=["amcl_gnss"]))
    for t, msg in ((0.0, _loc_status(in_grace=True)),
                   (1.0, _loc_status(checks=[("amcl_gnss", 2)])),
                   (5.0, _loc_status(checks=[("amcl_gnss", 2)]))):
        clock["t"] = t
        trigger._callback(msg)
    # 落ち着いて 4 s たったが、GNSS の判定がまだ UNKNOWN のまま
    assert not trigger.poll()
    clock["t"] = 6.0
    trigger._callback(_loc_status(checks=[("amcl_gnss", 0)]))
    assert trigger.poll()
    # NORMAL でなくなると、落ち着いた時間を数え直す
    trigger._callback(_loc_status(state=1))
    trigger._callback(_loc_status())
    assert not trigger.poll()
    clock["t"] = 9.0
    assert trigger.poll()


def _outcome(**kwargs):
    ctx = MagicMock()
    clock = {"t": 0.0}
    ctx.clock.now = lambda: clock["t"]
    ctx.poses.to_map = lambda spec: Pose(spec.x, spec.y, spec.z, spec.yaw)
    truth = {"pose": Pose(0.0, 0.0)}
    ctx.backend.entity_pose = lambda _: truth["pose"]
    return LocalizationOutcomeMonitor(ctx, LocalizationOutcomeSpec(**kwargs)), clock, ctx, truth


def test_localization_outcome_separates_missed_driving_from_recovery():
    monitor, clock, ctx, truth = _outcome(max_missed_distance_m=1.5, require_deviation=True)
    # 1 m/s で進む。1〜3 s はずれて要求なし (見逃し)、3〜4 s はずれて要求あり (復旧中)
    for t, offset, hold in ((0, 0.0, False), (1, 0.0, False), (2, 2.0, False), (3, 2.0, False), (4, 2.0, True)):
        clock["t"] = float(t)
        truth["pose"] = Pose(float(t), 0.0)
        ctx.poses.robot.get.return_value = Pose(float(t), offset)
        monitor._on_status(_status(hold))
        monitor._callback(None)
    result = monitor.result()
    assert result.status == ResultStatus.FAILED
    assert "drove 2.00 m without a hold" in result.message and "hold while deviated 1.0 s" in result.message


def test_localization_outcome_separates_slowed_driving_from_driving_at_normal_speed():
    monitor, clock, ctx, truth = _outcome(max_missed_normal_speed_m=0.5)
    # 1 m/s で進む。区間は終わりの標本の状態で数える: 1〜2 s はずれて減速なし、2〜4 s はずれて減速あり、
    # 4〜6 s はずれずに減速あり
    for t, offset, slow in ((0, 0.0, 0.0), (1, 0.0, 0.0), (2, 2.0, 0.0), (3, 2.0, 0.3), (4, 2.0, 0.3),
                            (5, 0.0, 0.3), (6, 0.0, 0.3)):
        clock["t"] = float(t)
        truth["pose"] = Pose(float(t), 0.0)
        ctx.poses.robot.get.return_value = Pose(float(t), offset)
        monitor._on_status(_status(False, slow))
        monitor._callback(None)
    result = monitor.result()
    assert result.status == ResultStatus.FAILED
    assert "drove 3.00 m without a hold" in result.message
    assert "drove 1.00 m at normal speed while deviated" in result.message
    assert "slowed 4.0 s (2.0 s not deviated)" in result.message


def test_localization_outcome_counts_false_hold_and_reports_no_deviation():
    monitor, clock, ctx, truth = _outcome(max_false_hold_sec=0.5, require_deviation=True)
    ctx.poses.robot.get.return_value = Pose(0.0, 0.0)
    monitor._on_status(_status(True))
    for t in (0.0, 1.0):
        clock["t"] = t
        monitor._callback(None)
    assert monitor.result().status == ResultStatus.ERROR
    relaxed, clock, ctx, truth = _outcome(max_false_hold_sec=0.5)
    ctx.poses.robot.get.return_value = Pose(0.0, 0.0)
    relaxed._on_status(_status(True))
    for t in (0.0, 1.0):
        clock["t"] = t
        relaxed._callback(None)
    assert relaxed.result().status == ResultStatus.FAILED
