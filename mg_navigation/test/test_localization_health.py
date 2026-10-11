import pytest

from mg_navigation.localization_health import localization_health, watchdog_state
from mg_navigation.localization_supervisor.state_machine import State


@pytest.mark.parametrize('state, level', [
    (State.NORMAL, 'ok'),
    (State.SUSPECT, 'warn'),
    (State.ISOLATED, 'warn'),
    (State.RECOVERING, 'warn'),
    (State.DEGRADED, 'error'),
])
def test_level_by_state(state, level):
    got_level, message, _ = localization_health(int(state), True, '', {})
    assert got_level == level
    assert message == state.name


def test_values_skip_unavailable_judgements():
    _, _, values = localization_health(
        0, False, 'isolated: amcl_gnss',
        {'amcl_gnss_d2': 1.5, 'amcl_jump_m': -1.0, 'scan_match_ratio': 0.8})
    assert values == {
        'amcl_attached': 'False', 'last_event': 'isolated: amcl_gnss',
        'amcl_gnss_d2': '1.500', 'scan_match_ratio': '0.800'}


@pytest.mark.parametrize('in_recovery, failed, consecutive, expected', [
    (False, False, 0, State.NORMAL),
    (False, False, 2, State.SUSPECT),
    (True, False, 0, State.RECOVERING),
    (True, True, 3, State.RECOVERING),
    (False, True, 0, State.DEGRADED),
    (False, True, 2, State.DEGRADED),
])
def test_watchdog_state(in_recovery, failed, consecutive, expected):
    assert watchdog_state(in_recovery, failed, consecutive) == expected
