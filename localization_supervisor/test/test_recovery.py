"""復旧 (候補を作る・評価する・選ぶ) と、Supervisor が再初期化を指示するまでのテスト。"""
import math

import numpy as np
import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.interfaces import (
    Candidate, CandidateGenerator, CandidateScorer, ParamSpec, RecoveryContext, SourceKind)
from localization_supervisor.core.recovery.planner import RecoveryPlanner, ScorerConfig, SelectionConfig
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.supervisor import Supervisor, make_store

from supervisor_test_helpers import Drive, pose_sample, room_scan, wall_map


# ---------------------------------------------------------------- 計画 (選び方)

class Fixed(CandidateGenerator):
    PARAMS = {'x': ParamSpec(float, 0.0), 'value': ParamSpec(float, 0.5)}

    def generate(self, context):
        cov = np.eye(3) * 0.01
        return [Candidate((self.params['x'], 0.0, 0.0), cov, self.name, info={'value': self.params['value']})]


class FromInfo(CandidateScorer):
    def score(self, candidate, context):
        return candidate.info.get('value')


def _planner(candidates, selection=None, veto_below=None, scorer=True):
    generators = [Fixed(f'g{i}', {'x': x, 'value': v}, {}) for i, (x, v) in enumerate(candidates)]
    scorers = [FromInfo('s', {}, {})] if scorer else []
    configs = [ScorerConfig('s', 'x', FromInfo, {}, {}, veto_below=veto_below)] if scorer else []
    return RecoveryPlanner(generators, scorers, configs, selection or SelectionConfig(), None)


def _plan(planner):
    return planner.plan(RecoveryContext(snapshot=None))


def test_selects_the_best_candidate():
    plan = _plan(_planner([(0.0, 0.5), (5.0, 0.9)]))
    assert plan.reason == 'selected' and plan.chosen.pose[0] == 5.0


def test_ambiguous_when_a_far_rival_is_almost_as_good():
    plan = _plan(_planner([(0.0, 0.80), (5.0, 0.75)], SelectionConfig(ambiguity_margin=0.1)))
    assert plan.reason == 'ambiguous' and plan.chosen is None


def test_a_near_rival_is_the_same_place_not_a_rival():
    plan = _plan(_planner([(0.0, 0.80), (0.5, 0.79)], SelectionConfig(ambiguity_margin=0.1, separation_m=1.0)))
    assert plan.reason == 'selected' and plan.chosen.pose[0] == 0.0


def test_low_score_is_not_reinitialized():
    plan = _plan(_planner([(0.0, 0.2)], SelectionConfig(min_score=0.5)))
    assert plan.reason == 'low_score' and plan.chosen is None


def test_vetoed_candidates_are_dropped():
    plan = _plan(_planner([(0.0, 0.1), (5.0, 0.9)], veto_below=0.5))
    assert plan.chosen.pose[0] == 5.0
    assert plan.candidates[0].vetoed_by == 's'
    assert _plan(_planner([(0.0, 0.1)], veto_below=0.5)).reason == 'all_vetoed'


def test_without_scorers_the_first_candidate_is_used():
    plan = _plan(_planner([(3.0, 0.0), (5.0, 0.0)], scorer=False))
    assert plan.reason == 'unscored' and plan.chosen.pose[0] == 3.0


def test_no_candidates():
    assert _plan(_planner([], scorer=False)).reason == 'no_candidates'


# ---------------------------------------------------------------- 設定の検証

def _raw(recovery=None, anchors=True, enabled=True):
    raw = {
        'version': 0,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'},
                    'map_scan': {'kind': 'reference'}},
        'timeline': {'motion_priority': ['odom'], 'alignment_std_per_m': 0.0, 'alignment_std_per_rad': 0.0},
        'detectors': {'amcl_gnss': {
            'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
            'params': {'threshold': 6.0, 'b_max_std_m': 2.0}}},
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 3.0}},
        'state_machine': {'recovery_enabled': enabled, 'isolate_hold_sec': 3.0, 'recover_ok_ticks': 3,
                          'recover_timeout_sec': 20.0, 'max_reinit_attempts': 2, 'clear_ticks': 3},
        'recovery': {
            'convergence': ['amcl_gnss'],
            'scan_buffer': {'reference': 'map_scan', 'motion': 'odom', 'window_distance_m': 20.0},
            'generators': {'gnss': {'type': 'pose_source', 'sources': {'pose': 'gnss'},
                                    'params': {'max_std_m': 1.0, 'consecutive': 2}}},
            'scorers': {'scan': {'type': 'scan_match', 'sources': {'reference': 'map_scan', 'motion': 'odom'},
                                 'params': {'min_distance_m': 100.0}}},
            'selection': {'min_score': 0.9}},
    }
    if anchors:
        raw['anchors'] = {'pose': 'amcl', 'motion': 'odom', 'min_interval_sec': 1.0}
    if recovery is not None:
        raw['recovery'] = recovery
    return raw


def test_recovery_config_is_parsed():
    config = parse_config(_raw())
    assert list(config.generators) == ['gnss'] and list(config.scorers) == ['scan']
    assert config.selection.min_score == 0.9 and config.scan_buffer.window_distance_m == 20.0


def test_recovery_enabled_requires_generators():
    with pytest.raises(ConfigError, match='requires recovery.generators'):
        parse_config(_raw(recovery={'convergence': ['amcl_gnss']}))


def test_generator_that_needs_anchors_is_rejected_without_them():
    recovery = _raw()['recovery']
    recovery['generators'] = {'dr': {'type': 'anchor_motion', 'sources': {'motion': 'odom'}}}
    with pytest.raises(ConfigError, match="requires 'anchors'"):
        parse_config(_raw(recovery=recovery, anchors=False))


def test_generator_motion_must_match_anchor_motion():
    raw = _raw()
    raw['sources']['wheel'] = {'kind': 'motion'}
    raw['recovery']['generators'] = {'dr': {'type': 'anchor_motion', 'sources': {'motion': 'wheel'}}}
    with pytest.raises(ConfigError, match='differs from anchors.motion'):
        parse_config(raw)


def test_unknown_generator_source_and_scorer_params_are_rejected():
    raw = _raw()
    raw['recovery']['generators']['gnss']['sources'] = {'pose': 'nope'}
    with pytest.raises(ConfigError, match='unknown source'):
        parse_config(raw)
    raw = _raw()
    raw['recovery']['scorers']['scan']['params'] = {'nope': 1}
    with pytest.raises(ConfigError, match='unknown params'):
        parse_config(raw)


def test_resetters_and_actuators_need_a_type():
    raw = _raw()
    raw['recovery']['resetters'] = [{'topic': '/x'}]
    with pytest.raises(ConfigError, match='type: required'):
        parse_config(raw)


# ---------------------------------------------------------------- Supervisor が再初期化を指示する

class Scene:
    """部屋の中を円を描いて走るロボット。AMCL はずらせる。GNSS は真値 (標準偏差 0.3 m)。"""

    def __init__(self, raw):
        self.drive = Drive(map_odom=(1.0, -2.0, 0.3), speed=0.5, yaw_rate=0.2)
        self.config = parse_config(raw)
        self.supervisor = Supervisor(self.config)
        self.store = make_store(self.config)
        self.store.set_map('map_scan', wall_map())
        self.amcl_offset = (0.0, 0.0, 0.0)
        self.gnss = True
        self.results = []

    def truth(self, t):
        return self.drive.map(t)

    def run(self, start, stop):
        for step in range(int(start), int(stop)):
            t = float(step)
            for half in (t - 0.5, t):
                self.store.add_motion('odom', self.drive.motion(half))
            truth = self.truth(t)
            self.store.set_scan('map_scan', room_scan(t, truth))
            amcl = (truth[0] + self.amcl_offset[0], truth[1] + self.amcl_offset[1], truth[2] + self.amcl_offset[2])
            self.store.add_pose('amcl', pose_sample(t, amcl, std_xy=0.1))
            if self.gnss:
                self.store.add_pose('gnss', pose_sample(t, truth, std_xy=0.3))
            result = self.supervisor.step(self.store.snapshot(t))
            self.results.append((t, result))
            yield t, result


def _reinit(scene, start, stop):
    for _, result in scene.run(start, stop):
        for directive in result.directives:
            if directive.kind == 'reinit':
                return result, directive
    return None, None


def test_gnss_fallback_reinitializes_to_the_gnss_pose_and_recovers():
    scene = Scene(_raw())
    for _ in scene.run(0, 40):
        pass
    assert scene.supervisor.state == State.NORMAL
    scene.amcl_offset = (4.0, 0.0, 0.0)
    result, directive = _reinit(scene, 40, 80)
    assert directive is not None, 'no reinit directive'
    truth = scene.truth(result.stamp)
    assert math.hypot(directive.candidate.pose[0] - truth[0], directive.candidate.pose[1] - truth[1]) < 0.1
    assert directive.candidate.origin == 'gnss'
    assert directive.candidate.scores['scan'] > 0.5
    assert result.state == State.RECOVERING
    names = [e.name for e in result.events]
    assert 'localization.reinit_requested' in names and 'localization.recovery_candidate' in names

    # 推定器が初期化し直され、AMCL が真値に戻れば、収束して NORMAL に戻る
    scene.amcl_offset = (0.0, 0.0, 0.0)
    for _ in scene.run(int(result.stamp) + 1, int(result.stamp) + 40):
        pass
    assert scene.supervisor.state == State.NORMAL


def test_isolate_and_restore_directives_are_issued():
    scene = Scene(_raw())
    for _ in scene.run(0, 40):
        pass
    scene.amcl_offset = (4.0, 0.0, 0.0)
    kinds = []
    for _, result in scene.run(40, 60):
        kinds += [d.kind for d in result.directives]
    assert kinds[0] == 'isolate'
    scene.amcl_offset = (0.0, 0.0, 0.0)
    for _, result in scene.run(60, 120):
        kinds += [d.kind for d in result.directives]
    assert 'reinit' in kinds and kinds[-1] == 'restore'


def test_reinit_waits_until_a_candidate_can_be_made():
    scene = Scene(_raw())
    for _ in scene.run(0, 40):
        pass
    scene.gnss = False
    scene.amcl_offset = (4.0, 0.0, 0.0)
    skipped = 0
    for _, result in scene.run(40, 70):
        skipped += sum(e.name == 'localization.reinit_skipped' for e in result.events)
    assert skipped > 0
    scene.gnss = True
    result, directive = _reinit(scene, 70, 100)
    assert directive is not None and directive.candidate.origin == 'gnss'


def test_low_scan_match_is_not_reinitialized():
    # GNSS が壁の外を指す (スキャンと合わない) とき、再初期化しない
    scene = Scene(_raw())
    for _ in scene.run(0, 40):
        pass
    scene.amcl_offset = (4.0, 0.0, 0.0)
    original = scene.store.add_pose

    def wrong_gnss(name, sample):
        if name == 'gnss':
            sample.pose = (sample.pose[0] + 6.0, sample.pose[1], sample.pose[2])
        original(name, sample)
    scene.store.add_pose = wrong_gnss
    _, directive = _reinit(scene, 40, 70)
    assert directive is None
    reasons = [e.fields.get('reason') for _, r in scene.results for e in r.events
               if e.name == 'localization.reinit_skipped']
    assert 'low_score' in reasons or 'all_vetoed' in reasons


def test_dead_reckoning_candidates_from_the_anchor():
    raw = _raw()
    raw['recovery']['generators'] = {
        'dr': {'type': 'anchor_motion', 'sources': {'motion': 'odom'}},
        'submap': {'type': 'submap_match', 'sources': {'motion': 'odom', 'reference': 'map_scan'},
                   'params': {'peaks': 2, 'min_range_m': 0.6, 'step_m': 0.3, 'min_distance_m': 100.0}}}
    scene = Scene(raw)
    for _ in scene.run(0, 40):
        pass
    scene.amcl_offset = (4.0, 0.0, 0.0)
    result, directive = _reinit(scene, 40, 80)
    assert directive is not None
    truth = scene.truth(result.stamp)
    assert math.hypot(directive.candidate.pose[0] - truth[0], directive.candidate.pose[1] - truth[1]) < 0.5
    origins = {c.origin for c in result.plan.candidates}
    assert origins == {'dr', 'submap'}
