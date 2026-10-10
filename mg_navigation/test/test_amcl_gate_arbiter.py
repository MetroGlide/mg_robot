import pytest

from mg_navigation.amcl_gate_arbiter.arbiter import GateArbiter, gate_health


def test_gate_is_open_only_when_every_requester_attaches():
    arbiter = GateArbiter(['waypoint', 'supervisor'])
    assert arbiter.desired is True
    arbiter.request('waypoint', False)
    assert arbiter.desired is False
    assert arbiter.holders == ['waypoint']
    # 監督ノードが復帰させても、ウェイポイントが切っている間は開かない
    arbiter.request('supervisor', False)
    arbiter.request('supervisor', True)
    assert arbiter.desired is False
    arbiter.request('waypoint', True)
    assert arbiter.desired is True
    assert arbiter.holders == []


def test_waypoint_on_does_not_override_supervisor_isolation():
    arbiter = GateArbiter(['waypoint', 'supervisor'])
    arbiter.request('supervisor', False)
    arbiter.request('waypoint', True)
    assert arbiter.desired is False


def test_needs_apply_until_marked_and_after_change():
    arbiter = GateArbiter(['waypoint'])
    assert arbiter.needs_apply()
    arbiter.mark_applied(True)
    assert not arbiter.needs_apply()
    arbiter.request('waypoint', False)
    assert arbiter.needs_apply()
    arbiter.mark_applied(False)
    assert not arbiter.needs_apply()
    arbiter.mark_unknown()
    assert arbiter.needs_apply()


def test_applied_is_true_only_when_gate_matches_desired():
    arbiter = GateArbiter(['waypoint'])
    assert not arbiter.applied
    arbiter.mark_applied(True)
    assert arbiter.applied
    arbiter.request('waypoint', False)
    assert not arbiter.applied
    arbiter.mark_applied(False)
    assert arbiter.applied
    arbiter.mark_unknown()
    assert not arbiter.applied


def test_unknown_requester_is_rejected():
    with pytest.raises(KeyError):
        GateArbiter(['waypoint']).request('other', False)


def test_gate_health_ok_when_applied():
    assert gate_health(True, True, [])[:2] == ('ok', 'attached')
    level, message, values = gate_health(False, True, ['waypoint'])
    assert (level, message) == ('ok', 'detached (held by: waypoint)')
    assert values == {'desired': 'False', 'applied': 'True', 'holders': 'waypoint'}


def test_gate_health_warns_when_not_applied():
    level, message, _ = gate_health(False, False, ['supervisor'])
    assert level == 'warn'
    assert message == 'not applied: detached (held by: supervisor)'
