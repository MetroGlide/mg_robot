import pytest

from localization_supervisor.core.state_machine import State
from mg_navigation.amcl_watchdog.health import watchdog_state


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
