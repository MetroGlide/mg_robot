import pytest

from mg_navigation.amcl_gate_arbiter.arbiter import GateArbiter


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


def test_unknown_requester_is_rejected():
    with pytest.raises(KeyError):
        GateArbiter(['waypoint']).request('other', False)
