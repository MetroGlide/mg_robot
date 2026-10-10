import pytest

from mg_utils.monitoring.activity import ActivityMonitor
from mg_utils.monitoring.health import evaluate_topic_health


def _monitor():
    return ActivityMonitor(1.0, 2.0, 0.0)


def test_waiting_for_first_message_is_ok():
    h = evaluate_topic_health(_monitor(), 0.5, expected_hz=10.0)
    assert (h.level, h.message) == ('ok', 'waiting for first message')


def test_healthy_topic_is_ok_with_values():
    m = _monitor()
    for i in range(6):
        m.on_message(i * 0.1)
    h = evaluate_topic_health(m, 0.5, expected_hz=10.0)
    assert (h.level, h.message) == ('ok', '10.0 Hz')
    assert h.values == {'hz': '10.00', 'expected_hz': '10.00', 'stale_sec': '0.0'}


def test_low_rate_is_warn():
    m = _monitor()
    for i in range(6):
        m.on_message(i * 0.4)
    h = evaluate_topic_health(m, 2.0, expected_hz=10.0)
    assert (h.level, h.message) == ('warn', 'low rate: 2.5 Hz (expected 10.0)')


def test_stale_topic_is_error_and_hz_drops_to_zero():
    m = _monitor()
    for i in range(6):
        m.on_message(i * 0.1)
    h = evaluate_topic_health(m, 3.0, expected_hz=10.0)
    assert (h.level, h.message) == ('error', 'no data for 2.5 s')
    assert h.values['hz'] == '0.00'


def test_fault_level_can_be_warn():
    m = _monitor()
    m.on_message(0.0)
    assert evaluate_topic_health(m, 3.0, fault_level='warn').level == 'warn'


def test_invalid_fault_level():
    with pytest.raises(ValueError):
        evaluate_topic_health(_monitor(), 0.0, fault_level='fatal')


def test_disconnected_message():
    m = _monitor()
    m.on_message(0.0, ok=False)
    h = evaluate_topic_health(m, 0.1)
    assert (h.level, h.message) == ('error', 'reports disconnected')
    assert 'expected_hz' not in h.values


def test_without_expected_hz_does_not_check_rate():
    m = _monitor()
    m.on_message(0.0)
    m.on_message(0.9)
    h = evaluate_topic_health(m, 0.9)
    assert (h.level, h.message) == ('ok', 'ok')


def test_evaluate_updates_the_monitor():
    m = _monitor()
    m.on_message(0.0)
    evaluate_topic_health(m, 3.0)
    assert m.faulted
