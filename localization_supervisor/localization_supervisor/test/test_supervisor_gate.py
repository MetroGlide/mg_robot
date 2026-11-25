"""監督に門 (設定の gate) を入れたときの分担のテスト。

部屋の中を円を描いて走るロボット (test_motion_gate と同じ) で、監督を門の周期で回す。照合の依頼は、次の周期の
結果として同じスレッドで返す。置き直しの指示は、送り先の役 (推定器 / 融合) に合わせて姿勢を移す。
"""
import copy
import math

import pytest

from localization_supervisor.core.config import parse_config
from localization_supervisor.core.geometry import compose
from localization_supervisor.core.interfaces import MotionSample, PoseSample
from localization_supervisor.core.map_match import MapMatcher
from localization_supervisor.core.motion_gate import GateState
from localization_supervisor.core.plugins import ConfigError
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.status import status_view
from localization_supervisor.core.supervisor import Supervisor, make_store

from supervisor_test_helpers import cov, room_scan, wall_map
from test_motion_gate import PERIOD, _relative, truth

RAW = {
    'version': 1,
    'period_sec': 1.0,
    'sources': {
        'amcl': {'kind': 'pose', 'input': {}},
        'ekf': {'kind': 'pose', 'derived_from': ['amcl', 'gnss'], 'input': {}},
        'gnss': {'kind': 'pose', 'input': {}},
        'odom': {'kind': 'motion', 'input': {}},
        'map_scan': {'kind': 'reference', 'input': {}},
    },
    'timeline': {'motion_priority': ['odom'], 'horizon_sec': 60.0},
    'detectors': {
        'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                      'params': {'metric': 'distance', 'threshold': 1.0}},
    },
    'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 4.0}},
    'state_machine': {'recovery_enabled': True, 'isolate_only': ['gnss']},
    'recovery': {
        'generators': {'gnss': {'type': 'pose_source', 'sources': {'pose': 'gnss'}}},
        'resetters': [{'type': 'amcl', 'role': 'estimator'}, {'type': 'robot_localization', 'role': 'fused'}],
    },
    'grace': {'startup_sec': 2.0, 'after_reset_sec': 5.0},
    'external_reinit': {'topics': ['/initialpose']},
    'gate': {
        'enabled': True,
        'period_sec': PERIOD,
        'settings': {'estimator': 'amcl', 'fused': 'ekf', 'motion': 'odom', 'reference': 'map_scan', 'gnss': 'gnss',
                     'confirm_distance_m': 1000.0, 'undecided_hold_sec': 4.0, 'undecided_drive_m': 2.0},
        'closers': {
            'gate_amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'},
                               'params': {'threshold_m': 0.5, 'threshold_rad': 0.3}},
            'gate_amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                               'params': {'metric': 'distance', 'threshold': 1.0, 'b_max_std_m': 0.5},
                               'close_after_sec': 2.0, 'active_in_grace': True},
        },
        'scan_buffer': {'keyframe_distance_m': 0.3, 'window_distance_m': 3.0},
    },
}


def _raw(**changes):
    raw = copy.deepcopy(RAW)
    for key, value in changes.items():
        raw[key] = value
    return raw


class Sim:
    def __init__(self, raw=None):
        self.config = parse_config(raw or RAW)
        self.supervisor = Supervisor(self.config)
        self.matcher = MapMatcher(self.config.gate.map_match)
        self.store = make_store(self.config)
        self.store.set_map('map_scan', wall_map())
        self.t = 0.0
        self.drive = 0.0
        self.moving = True
        self.true = truth(0.0)
        self.amcl_offset = (0.0, 0.0, 0.0)
        self.ekf_offset = (0.0, 0.0, 0.0)
        self.gnss_offset = (0.0, 0.0)
        self.pending = None
        self.results = []

    def step(self):
        if self.moving:
            self.drive += PERIOD
        self.true = truth(self.drive)
        self.t += PERIOD
        t = self.t
        self.store.add_motion('odom', MotionSample(t, self.true))
        self.store.add_pose('amcl', PoseSample(t, compose(self.true, self.amcl_offset), cov()))
        self.store.add_pose('ekf', PoseSample(t, compose(self.true, self.ekf_offset), cov()))
        self.store.add_pose('gnss', PoseSample(
            t, (self.true[0] + self.gnss_offset[0], self.true[1] + self.gnss_offset[1], 0.0), cov(0.2)))
        self.store.set_scan('map_scan', room_scan(t, self.true))
        result = self.supervisor.step(self.store.snapshot(t), self.pending)
        self.pending = None
        if result.gate.request is not None:
            request = result.gate.request
            self.pending = (request, self.matcher.decide(request.field_for, request.points, request.candidates))
        for directive in result.directives:
            if directive.kind == 'reinit':
                rel = _relative(self.true, directive.candidate.pose)
                self.ekf_offset = rel
                if directive.targets == 'all':
                    self.amcl_offset = rel
        self.results.append(result)
        return result

    def run(self, seconds):
        return [self.step() for _ in range(int(round(seconds / PERIOD)))]

    def events(self, name):
        return [e for r in self.results for e in r.events if e.name == f'localization.{name}']


def _opened():
    sim = Sim()
    sim.run(4.0)
    assert sim.supervisor.gate.state == GateState.OPEN
    sim.run(14.0)
    return sim


def test_detectors_run_every_period_and_the_gate_every_gate_period():
    sim = Sim()
    sim.run(3.0)
    # 判定の総合 (状態遷移のイベントや check の評価) は period_sec ごと、門は毎周期
    assert len(sim.results) == 15
    assert sum(1 for r in sim.results if r.gate is not None) == 15
    stamps = sorted({id(r.fusion) for r in sim.results})
    assert len(stamps) == 3


def test_gate_reset_discards_the_supervisor_accumulation_and_starts_grace():
    sim = _opened()
    assert not sim.results[-1].in_grace
    sim.amcl_offset = (0.0, 1.2, 0.0)
    sim.moving = False
    sim.run(4.0)
    assert sim.events('pose_reset')
    assert any(d.kind == 'reinit' for r in sim.results for d in r.directives)
    assert sim.supervisor.state == State.NORMAL
    assert sim.supervisor._grace_until > sim.t


def test_supervisor_recovery_closes_the_gate_holds_and_counts_the_gate_decision_as_reinit():
    sim = _opened()
    sim.supervisor.machine.state = State.RECOVERING
    sim.supervisor.machine.blame = 'amcl'
    sim.supervisor._reinit_pending = True
    sim.amcl_offset = (0.0, 1.2, 0.0)
    sim.ekf_offset = (0.0, 1.2, 0.0)
    sim.moving = False
    result = sim.step()
    assert sim.events('reinit_requested')[0].fields['via'] == 'gate'
    assert result.gate.state == GateState.CLOSED and result.hold_requested
    assert not result.gate.gate_open
    sim.run(4.0)
    assert sim.events('pose_reset')
    assert sim.supervisor.machine.reinit_done
    assert not sim.supervisor._awaiting_gate


def test_supervisor_isolation_closes_the_gate_before_asking_to_reinit():
    """閉じる理由の部品が気づかないゆっくりしたずれを監督が確かめたら、切り離した時点で門を閉じさせて止める。
    再初期化を求める前に門が決めれば、それを再初期化として数える。"""
    raw = _raw()
    raw['gate'] = copy.deepcopy(raw['gate'])
    del raw['gate']['closers']['gate_amcl_gnss']
    raw['state_machine'] = dict(raw['state_machine'], isolate_hold_sec=3.0)
    sim = Sim(raw)
    sim.run(18.0)
    assert sim.supervisor.gate.state == GateState.OPEN
    closed_at = None
    for i in range(1, 300):
        sim.amcl_offset = (0.0, min(0.01 * i, 2.0), 0.0)
        result = sim.step()
        if result.gate.state != GateState.OPEN:
            closed_at = result
            break
    assert closed_at is not None
    assert sim.supervisor.machine.state == State.ISOLATED
    assert sim.events('gate_closed')[-1].fields['reason'] == 'supervisor'
    assert closed_at.hold_requested
    sim.moving = False
    sim.run(8.0)
    assert sim.events('pose_reset')
    assert sim.supervisor.machine.reinit_done or sim.supervisor.state == State.NORMAL
    requested = sim.events('reinit_requested')
    assert requested and requested[0].fields.get('decided') is True


def test_gnss_isolated_by_the_supervisor_is_not_used_by_the_gate():
    """GNSS を切り離している (isolate_only の blame。止めず、再初期化もしない) 間は、門が GNSS との食い違いで閉じない。"""
    sim = _opened()
    machine = sim.supervisor.machine
    machine.state, machine.blame, machine._soft = State.ISOLATED, 'gnss', True
    sim.gnss_offset = (0.0, 2.5)
    sim.run(4.0)
    assert machine.state == State.ISOLATED and machine.blame == 'gnss'
    assert sim.supervisor._gate_distrust() == {'gnss'}
    assert not sim.events('gate_closed')


def test_external_reinit_is_ignored_with_the_gate():
    sim = _opened()
    sim.supervisor.machine.state = State.RECOVERING
    sim.supervisor.external_reinit(sim.t, '/initialpose')
    assert sim.supervisor.machine.state == State.RECOVERING
    assert not sim.events('external_reinit')


def test_status_shows_the_heavier_of_the_supervisor_and_the_gate():
    sim = _opened()
    sim.amcl_offset = (1.5, 0.0, 0.0)
    result = sim.step()
    assert result.state == State.NORMAL and result.gate.state == GateState.CLOSED
    view = status_view(result, sim.supervisor.config, '')
    assert view.state == State.ISOLATED
    assert 'amcl' in view.isolated_sources and not view.trusted
    assert {'amcl_gnss', 'gate_amcl_jump', 'gate_amcl_gnss'} <= {c.name for c in view.checks}


def test_without_the_gate_the_step_is_the_supervisor_alone():
    raw = _raw()
    raw['gate'] = dict(raw['gate'], enabled=False)
    config = parse_config(raw)
    assert config.gate is None
    supervisor = Supervisor(config)
    assert supervisor.gate is None


def test_with_gate_switches_the_gate_and_keeps_the_profile():
    raw = _raw()
    raw['gate'] = dict(raw['gate'], enabled=False)
    config = parse_config(raw, profile='calm', profiles={'calm': {'grace': {'startup_sec': 9.0}}})
    enabled = config.with_gate(True)
    assert enabled.gate is not None and enabled.startup_grace_sec == 9.0
    assert enabled.with_gate(False).gate is None


def test_config_rejects_a_closer_named_like_a_detector():
    raw = _raw()
    raw['gate'] = copy.deepcopy(raw['gate'])
    raw['gate']['closers']['amcl_gnss'] = raw['gate']['closers'].pop('gate_amcl_gnss')
    with pytest.raises(ConfigError, match='used by a detector'):
        parse_config(raw)


def test_config_requires_resetter_roles_for_the_gate():
    raw = _raw()
    raw['recovery'] = dict(raw['recovery'], resetters=[{'type': 'amcl'}, {'type': 'robot_localization'}])
    with pytest.raises(ConfigError, match='both roles'):
        parse_config(raw)
    raw['recovery'] = dict(raw['recovery'], resetters=[{'type': 'amcl', 'role': 'odometry'}])
    with pytest.raises(ConfigError, match='role'):
        parse_config(raw)


def test_profiles_cannot_change_the_gate():
    with pytest.raises(ConfigError, match='gate'):
        parse_config(RAW, profile='p', profiles={'p': {'gate': {'period_sec': 0.5}}})


def test_with_gate_needs_a_gate_section():
    raw = _raw()
    del raw['gate']
    with pytest.raises(ConfigError, match='no gate section'):
        parse_config(raw).with_gate(True)


def test_gate_period_must_not_exceed_the_supervisor_period():
    raw = _raw()
    raw['gate'] = dict(raw['gate'], period_sec=2.0)
    with pytest.raises(ConfigError, match='must not exceed'):
        parse_config(raw)
