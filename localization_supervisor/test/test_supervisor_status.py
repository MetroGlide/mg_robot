import pytest

from localization_supervisor.core.health import localization_health
from localization_supervisor.core.state_machine import State


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
