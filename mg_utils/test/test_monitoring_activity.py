import pytest

from mg_utils.monitoring.activity import ActivityMonitor, RateEstimator, TickGuard


def _monitor(**kwargs):
    params = dict(stale_timeout_sec=1.0, recover_hold_sec=2.0, start_time=0.0)
    params.update(kwargs)
    return ActivityMonitor(**params)


def test_healthy_source_is_not_faulted():
    m = _monitor()
    for t in range(0, 10):
        m.on_message(t * 0.1)
        assert not m.update(t * 0.1)


def test_stale_source_becomes_faulted():
    m = _monitor()
    m.on_message(0.0)
    assert not m.update(0.9)
    assert m.update(1.1)
    assert m.stale_sec(1.1) == pytest.approx(1.1)


def test_startup_grace_delays_fault_without_first_message():
    m = _monitor(startup_grace_sec=30.0)
    assert not m.update(20.0)
    assert m.stale_sec(20.0) == 0.0
    # 猶予が終わってからさらに stale_timeout_sec たっても来なければ異常
    assert not m.update(30.5)
    assert m.update(31.5)


def test_no_first_message_without_grace_is_faulted_after_timeout():
    m = _monitor()
    assert not m.update(0.5)
    assert m.update(1.5)


def test_armed_on_first_message_ignores_source_that_never_published():
    m = _monitor(armed_on_first_message=True)
    assert not m.update(100.0)
    assert m.stale_sec(100.0) == 0.0
    m.on_message(100.0)
    assert not m.update(100.5)
    assert m.update(102.0)


def test_recovery_requires_hold_time():
    m = _monitor()
    m.on_message(0.0)
    assert m.update(2.0)

    m.on_message(2.0)
    assert m.update(2.1)  # 戻ったが、まだ保持時間に達しない
    m.on_message(3.0)
    assert m.update(3.0)
    m.on_message(4.0)
    assert not m.update(4.2)  # 2.1 から 2 秒以上正常が続いた


def test_fault_during_hold_restarts_the_hold():
    m = _monitor()
    m.on_message(0.0)
    assert m.update(2.0)
    m.on_message(2.0)
    assert m.update(2.5)  # 保持の開始 (2.5)
    assert m.update(4.0)  # 再び途絶 (最後の受信は 2.0)。保持は振り出しに戻る
    m.on_message(4.0)
    assert m.update(4.1)  # 保持の開始 (4.1)
    m.on_message(5.0)
    assert m.update(5.0)
    m.on_message(6.0)
    assert not m.update(6.2)  # 4.1 から 2 秒以上正常が続いた


def test_not_ok_message_is_fault_even_if_fresh():
    m = _monitor()
    m.on_message(0.0, ok=False)
    assert m.update(0.1)
    assert m.disconnected
    m.on_message(0.2, ok=True)
    assert m.update(0.3)
    m.on_message(2.3, ok=True)
    assert not m.update(2.4)


def test_forgive_moves_last_receive_forward():
    m = _monitor()
    m.on_message(0.0)
    m.forgive(5.0)
    assert not m.update(5.5)
    assert m.stale_sec(5.5) == pytest.approx(0.5)


def test_forgive_does_not_arm_a_source_without_messages():
    m = _monitor(armed_on_first_message=True)
    m.forgive(5.0)
    assert not m.received
    assert not m.update(100.0)


def test_tick_guard_detects_large_gap():
    g = TickGuard(max_gap_sec=2.0)
    assert not g.check(0.0)
    assert not g.check(0.5)
    assert g.check(3.0)
    assert not g.check(3.5)


def test_rate_estimator_needs_two_messages():
    r = RateEstimator()
    assert r.hz(0.0) == 0.0
    r.tick(0.0)
    assert r.hz(0.0) == 0.0


def test_rate_estimator_computes_frequency():
    r = RateEstimator()
    for i in range(6):
        r.tick(i * 0.1)
    assert r.hz(0.5) == pytest.approx(10.0)


def test_rate_estimator_uses_only_the_window():
    r = RateEstimator(window=3)
    for t in (0.0, 10.0, 10.1, 10.2):
        r.tick(t)
    assert r.hz(10.2) == pytest.approx(10.0)


def test_rate_estimator_drops_to_zero_after_max_age():
    r = RateEstimator()
    for i in range(6):
        r.tick(i * 0.1)
    assert r.hz(1.0, max_age_sec=1.0) == pytest.approx(10.0)
    assert r.hz(1.6, max_age_sec=1.0) == 0.0
    # max_age_sec を指定しなければ、最後の頻度を返し続ける
    assert r.hz(100.0) == pytest.approx(10.0)


def test_activity_monitor_hz_drops_to_zero_when_stale():
    m = _monitor()
    for i in range(6):
        m.on_message(i * 0.1)
    assert m.hz(0.6) == pytest.approx(10.0)
    assert m.hz(2.0) == 0.0
