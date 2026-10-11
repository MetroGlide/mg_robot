import pytest

from mg_utils.monitoring.thresholds import MovingAverage, level_for_high, worst


def test_level_for_high_boundaries():
    assert level_for_high(84.9, 85.0, 95.0) == 'ok'
    assert level_for_high(85.0, 85.0, 95.0) == 'warn'
    assert level_for_high(94.9, 85.0, 95.0) == 'warn'
    assert level_for_high(95.0, 85.0, 95.0) == 'error'


def test_level_for_high_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        level_for_high(1.0, 90.0, 80.0)


def test_worst():
    assert worst('ok', 'ok') == 'ok'
    assert worst('ok', 'warn', 'ok') == 'warn'
    assert worst('warn', 'error', 'ok') == 'error'


def test_moving_average_uses_window():
    m = MovingAverage(3)
    assert m.value == 0.0
    assert m.add(10.0) == 10.0
    assert m.add(20.0) == 15.0
    m.add(30.0)
    assert m.add(40.0) == pytest.approx(30.0)  # 20, 30, 40


def test_moving_average_rejects_invalid_window():
    with pytest.raises(ValueError):
        MovingAverage(0)
