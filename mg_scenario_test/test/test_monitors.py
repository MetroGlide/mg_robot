"""MG-01 固有の monitor の単体テスト。"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from mg_scenario_test.monitors import (
    LocalizationHoldMonitor,
    LocalizationHoldSpec,
    LocalizationReady,
    LocalizationReadySpec,
    LocalizationStateMonitor,
    LocalizationStateSpec,
)
from sim_scenario_test.engine.result import ResultStatus


def _status(hold):
    return SimpleNamespace(hold_requested=hold)


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
