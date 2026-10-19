"""疑わしい情報源の切り分け (正しいと示す判定 vouch) と、切り離すだけの blame (isolate_only) のテスト。"""
import numpy as np
import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.fusion import CusumFusion, attribute
from localization_supervisor.core.interfaces import (
    CONTRACT_VERSION, Candidate, CandidateGenerator, CandidateScorer, Evidence, FusionResult, InstanceInfo,
    ParamSpec, PoseSample, RecoveryContext, Status)
from localization_supervisor.core.recovery.planner import RecoveryPlanner, ScorerConfig, SelectionConfig
from localization_supervisor.core.state_machine import Action, MachineConfig, State, StateMachine
from localization_supervisor.core.supervisor import Supervisor, make_store
from localization_supervisor.ros.actuators import apply_actuators

from supervisor_test_helpers import Drive, cov, pose_sample


def _infos(*specs):
    return [InstanceInfo(name=n, subjects=list(s), sources=list(s), **kw) for n, s, kw in specs]


def _ev(status, score=0.0):
    return Evidence(Status(status), score)


# ---------------------------------------------------------------- vouch (切り分け)

def _attribution(vouched, values=None):
    instances = {i.name: i for i in _infos(
        ('amcl_gnss', ['amcl', 'gnss'], {}), ('amcl_jump', ['amcl'], {}))}
    values = {'amcl_gnss': 9.0, 'amcl_jump': 0.0, **(values or {})}
    return attribute(values, values, instances, ['amcl', 'gnss'], {}, 1.0, vouched)


def test_a_vouched_source_is_not_suspected_by_a_comparison():
    assert _attribution(set()) == ('unknown', [])
    assert _attribution({'amcl'}) == ('gnss', [])
    assert _attribution({'gnss'}) == ('amcl', [])
    # 両方が正しいと示されているときは、片方に寄せられない
    assert _attribution({'amcl', 'gnss'}) == ('unknown', [])


def test_a_vouched_source_is_still_suspected_by_its_own_detector():
    # AMCL だけを疑う判定 (1 つの情報源) は、vouch に関わらず AMCL を疑う
    assert _attribution({'amcl'}, {'amcl_gnss': 0.0, 'amcl_jump': 3.0}) == ('amcl', [])


def _vouching(hold_ticks=3):
    infos = _infos(('amcl_gnss', ['amcl', 'gnss'], {}), ('scan', ['amcl'], {'vouches': ['amcl']}))
    return CusumFusion('cusum', {'suspect_level': 1.0, 'confirm_level': 5.0, 'blame_margin': 1.0,
                                 'vouch_hold_ticks': hold_ticks}, infos, ['amcl', 'gnss'], {})


def test_fusion_blames_the_other_source_while_the_vouching_detector_is_normal():
    fusion = _vouching()
    fusion.update({'amcl_gnss': _ev('normal'), 'scan': _ev('normal')}, True)
    result = fusion.update({'amcl_gnss': _ev('anomaly', 5.0), 'scan': _ev('normal')}, True)
    assert result.blame == 'gnss'


def test_the_vouch_lasts_while_the_detector_cannot_judge_then_expires():
    fusion = _vouching(hold_ticks=3)
    fusion.update({'amcl_gnss': _ev('normal'), 'scan': _ev('normal')}, True)
    blames = [fusion.update({'amcl_gnss': _ev('anomaly', 5.0), 'scan': _ev('unknown')}, True).blame
              for _ in range(6)]
    # 判定できない周期が hold_ticks を超えると、AMCL も疑う側に戻る
    assert blames[:3] == ['gnss'] * 3 and blames[-1] == 'unknown'


def test_the_vouch_is_withdrawn_when_the_detector_turns_anomalous():
    fusion = _vouching()
    fusion.update({'amcl_gnss': _ev('normal'), 'scan': _ev('normal')}, True)
    result = fusion.update({'amcl_gnss': _ev('anomaly', 5.0), 'scan': _ev('anomaly', 5.0)}, True)
    assert result.blame == 'amcl'


def test_the_vouch_does_not_change_during_the_grace_period():
    fusion = _vouching()
    fusion.update({'amcl_gnss': _ev('normal'), 'scan': _ev('normal')}, True)
    fusion.update({'amcl_gnss': _ev('anomaly', 5.0), 'scan': _ev('anomaly', 5.0)}, False)
    result = fusion.update({'amcl_gnss': _ev('anomaly', 5.0), 'scan': _ev('normal')}, True)
    assert result.blame == 'gnss'


# ---------------------------------------------------------------- isolate_only (状態遷移)

def _fr(suspect=False, confirmed=False, cleared=False, anomaly=False, blame=''):
    return FusionResult(level=0.0, suspect=suspect, confirmed=confirmed, cleared=cleared, anomaly=anomaly,
                        blame=blame)


def _soft_machine():
    machine = StateMachine(MachineConfig(
        recovery_enabled=True, isolate_hold_sec=1.0, clear_ticks=2, isolate_only=['gnss']))
    machine.step(0.0, _fr(suspect=True, anomaly=True))
    return machine


def test_an_isolate_only_blame_is_isolated_without_holding_or_reinitializing():
    machine = _soft_machine()
    d = machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert d.state == State.ISOLATED and d.actions == [Action.ISOLATE]
    assert machine.isolating and not machine.hold_requested
    # 切り離してから待っても、再初期化に進まない
    for t in (2.0, 3.0, 10.0):
        d = machine.step(t, _fr(suspect=True, anomaly=True, blame='gnss'))
        assert d.state == State.ISOLATED and not d.actions
    assert not machine.hold_requested


def test_an_isolate_only_blame_returns_when_the_anomaly_clears():
    machine = _soft_machine()
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert machine.step(2.0, _fr(cleared=True)).state == State.ISOLATED
    d = machine.step(3.0, _fr(cleared=True))
    assert d.state == State.NORMAL and d.actions == [Action.RESTORE] and d.event == 'cleared'
    assert not machine.isolating


def test_another_blame_takes_over_from_an_isolate_only_blame():
    machine = _soft_machine()
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    d = machine.step(2.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='amcl'))
    assert d.state == State.ISOLATED and d.actions == [Action.ISOLATE] and d.blame == 'amcl'
    assert machine.hold_requested
    # 以降は、通常の流れ (待って再初期化)
    assert machine.step(2.5, _fr(anomaly=True)).state == State.ISOLATED
    d = machine.step(3.0, _fr(anomaly=True))
    assert d.state == State.RECOVERING and d.actions == [Action.REINIT]


def test_an_undecided_blame_also_takes_over():
    machine = _soft_machine()
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    d = machine.step(2.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='unknown'))
    assert d.actions == [Action.ISOLATE] and machine.hold_requested


def test_a_blame_narrowed_before_the_reinit_becomes_an_isolate_only_isolation():
    # 正しいと示す判定がまだ判定できず、決めきれずに止めた (blame が unknown)。後から gnss と分かった
    machine = _soft_machine()
    d = machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='unknown'))
    assert machine.hold_requested and d.state == State.ISOLATED
    d = machine.step(1.5, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert d.state == State.ISOLATED and d.actions == [Action.ISOLATE] and d.blame == 'gnss'
    assert machine.isolating and not machine.hold_requested
    for t in (2.0, 9.0):
        assert machine.step(t, _fr(suspect=True, anomaly=True, blame='gnss')).state == State.ISOLATED


def test_a_blame_narrowed_while_recovering_before_the_reinit_also_stops_the_recovery():
    machine = _soft_machine()
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='unknown'))
    assert machine.step(2.0, _fr(confirmed=True, anomaly=True, blame='unknown')).state == State.RECOVERING
    d = machine.step(3.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert d.state == State.ISOLATED and not machine.hold_requested
    # 再初期化した後は、blame が分かっても収束の確認を続ける
    machine = _soft_machine()
    machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='unknown'))
    machine.step(2.0, _fr(confirmed=True, anomaly=True, blame='unknown'))
    machine.note_reinit()
    assert machine.step(3.0, _fr(confirmed=True, anomaly=True, blame='gnss')).state == State.RECOVERING


def test_isolate_only_is_ignored_when_recovery_is_off():
    machine = StateMachine(MachineConfig(recovery_enabled=False, isolate_only=['gnss']))
    machine.step(0.0, _fr(suspect=True, anomaly=True))
    d = machine.step(1.0, _fr(confirmed=True, suspect=True, anomaly=True, blame='gnss'))
    assert d.state == State.DEGRADED and not machine.isolating


# ---------------------------------------------------------------- 復旧の候補から、疑わしい情報源を外す

class _Gen(CandidateGenerator):
    PARAMS = {'x': ParamSpec(float, 0.0)}

    def generate(self, context):
        self.status = 'ok'
        return [Candidate((self.params['x'], 0.0, 0.0), np.eye(3) * 0.01, self.name, info={'value': 0.6})]


class _Score(CandidateScorer):
    def score(self, candidate, context):
        return 1.0 if self.sources['pose'] == 'gnss' else 0.2


def _two_sources_planner():
    generators = [_Gen('from_gnss', {'x': 5.0}, {'pose': 'gnss'}), _Gen('from_odom', {'x': 0.0}, {'motion': 'odom'})]
    scorers = [_Score('gnss_agreement', {}, {'pose': 'gnss'}), _Score('other', {}, {'pose': 'amcl'})]
    configs = [ScorerConfig(s.name, 'x', _Score, s.sources, {}) for s in scorers]
    return RecoveryPlanner(generators, scorers, configs, SelectionConfig(ambiguity_margin=0.0), None)


def test_the_planner_does_not_use_the_blamed_source():
    planner = _two_sources_planner()
    context = RecoveryContext(snapshot=None)
    plan = planner.plan(context)
    assert [c.origin for c in plan.candidates] == ['from_gnss', 'from_odom']
    assert set(plan.candidates[0].scores) == {'gnss_agreement', 'other'}
    plan = planner.plan(RecoveryContext(snapshot=None), exclude='gnss')
    assert [c.origin for c in plan.candidates] == ['from_odom']
    assert set(plan.candidates[0].scores) == {'other'}
    assert plan.generators == {'from_gnss': 'excluded:gnss', 'from_odom': 'ok'}
    # 'unknown' のように、情報源の名前でないものは何も外さない
    assert len(planner.plan(RecoveryContext(snapshot=None), exclude='unknown').candidates) == 2


# ---------------------------------------------------------------- 設定

def _raw(**overrides):
    raw = {
        'version': CONTRACT_VERSION,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'}},
        'timeline': {'motion_priority': ['odom']},
        'detectors': {
            'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 6.0}},
            'amcl_cov': {'type': 'covariance', 'sources': {'pose': 'amcl'}, 'params': {'threshold': 100.0},
                         'vouch': ['amcl']},
        },
    }
    raw.update(overrides)
    return raw


def test_vouch_must_name_a_source_the_detector_suspects():
    assert parse_config(_raw()).detectors['amcl_cov'].vouch == ['amcl']
    raw = _raw()
    raw['detectors']['amcl_cov']['vouch'] = ['gnss']
    with pytest.raises(ConfigError, match='vouch'):
        parse_config(raw)


def test_isolate_only_must_name_sources():
    with pytest.raises(ConfigError, match='isolate_only'):
        parse_config(_raw(state_machine={'isolate_only': ['nope']}))
    assert parse_config(_raw(state_machine={'isolate_only': ['gnss']})).machine.isolate_only == ['gnss']


def test_disabled_actuators_must_name_actuators():
    actuators = [{'type': 'gate', 'params': {'service': '/g'}}, {'type': 'variance_scale', 'name': 'scale'}]
    config = parse_config(_raw(recovery={'actuators': actuators, 'disabled_actuators': ['gate', 'scale']}))
    assert config.disabled_actuators == ['gate', 'scale']
    with pytest.raises(ConfigError, match='disabled_actuators'):
        parse_config(_raw(recovery={'actuators': actuators, 'disabled_actuators': ['variance_scale']}))


# ---------------------------------------------------------------- 対処の無効化

class _Recorder:
    def __init__(self, name):
        self.name = name
        self.log = []

    def apply(self, result):
        self.log.append('apply')

    def release(self):
        self.log.append('release')


def test_disabled_actuators_are_released_once_and_resume_when_enabled_again():
    a, b = _Recorder('a'), _Recorder('b')
    released = set()
    apply_actuators([a, b], {'b'}, released, None)
    apply_actuators([a, b], {'b'}, released, None)
    assert a.log == ['apply', 'apply'] and b.log == ['release']
    apply_actuators([a, b], set(), released, None)
    assert b.log == ['release', 'apply']
    apply_actuators([a, b], {'b'}, released, None)
    assert b.log == ['release', 'apply', 'release']


# ---------------------------------------------------------------- Supervisor 全体 (GNSS だけがずれる)

def _supervisor_config(vouch, isolate_only):
    raw = _raw(
        fusion={'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 5.0}},
        state_machine={'recovery_enabled': True, 'isolate_hold_sec': 2.0, 'clear_ticks': 3,
                       'isolate_only': isolate_only},
        recovery={'generators': {'gnss': {'type': 'pose_source', 'sources': {'pose': 'gnss'},
                                          'params': {'max_std_m': 1.0, 'consecutive': 2, 'max_age_sec': 3.0}}}},
        residuals={'gnss_vs_amcl': {'a': 'gnss', 'b': 'amcl', 'max_scale': 50.0}})
    raw['detectors']['amcl_cov']['vouch'] = vouch
    return parse_config(raw)


def _run(config, bias_until, seconds=40):
    """GNSS は bias_until までずれている。受け取る側のように、出した倍率を GNSS の共分散にかけて渡す。"""
    supervisor = Supervisor(config)
    store = make_store(config)
    drive = Drive()
    results = []
    scale = 1.0
    for t in range(seconds):
        stamp = float(t)
        for half in (stamp - 0.5, stamp):
            store.add_motion('odom', drive.motion(half))
        truth = drive.map(stamp)
        bias = 3.0 if t < bias_until else 0.0
        store.add_pose('amcl', pose_sample(stamp, truth, std_xy=0.05))
        store.add_pose('gnss', PoseSample(stamp, (truth[0] + bias, truth[1], truth[2]), cov(0.1) * scale))
        result = supervisor.step(store.snapshot(stamp))
        scale = result.applied_scales['gnss_vs_amcl']
        store.note_scale('gnss', stamp, scale)
        results.append(result)
    return results


def test_a_biased_gnss_is_isolated_without_stopping_or_reinitializing_amcl():
    results = _run(_supervisor_config(['amcl'], ['gnss']), bias_until=20)
    isolated = [r for r in results if r.state == State.ISOLATED]
    assert isolated and all(r.blame == 'gnss' for r in isolated)
    assert not any(r.hold_requested for r in results)
    assert not [d for r in results for d in r.directives if d.kind == 'reinit']
    assert all(r.applied_scales['gnss_vs_amcl'] == 50.0 for r in isolated)
    assert [r.isolated_sources for r in isolated][0] == ['gnss']
    # ずれがなくなれば戻る
    assert results[-1].state == State.NORMAL
    assert [d.kind for r in results for d in r.directives if d.kind == 'restore'] == ['restore']


def test_without_a_vouch_the_same_bias_is_not_attributed_and_stops_the_robot():
    results = _run(_supervisor_config([], ['gnss']), bias_until=20)
    assert any(r.hold_requested for r in results)
    assert {r.blame for r in results if r.state == State.ISOLATED} == {'unknown'}


def test_the_scale_applied_while_isolated_does_not_hide_the_bias_from_the_detector():
    # 切り離してスケールを上限にしても、判定は倍率をかける前の分散で行うので、ずれが続くかぎり異常のまま
    results = _run(_supervisor_config(['amcl'], ['gnss']), bias_until=40, seconds=40)
    assert results[-1].state == State.ISOLATED
    assert results[-1].evidences['amcl_gnss'].status == Status.ANOMALY
