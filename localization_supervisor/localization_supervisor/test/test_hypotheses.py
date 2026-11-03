"""仮説の集合の情報源: アダプタ (particle_cloud)、Snapshot の読み出し、故障の注入、判定 hypothesis_split、
候補を作るもの hypothesis_modes。"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from localization_supervisor.core.config import parse_config
from localization_supervisor.core.detectors import STANDARD_DETECTORS
from localization_supervisor.core.faults import FaultInjectingStore, FaultInjector, parse_faults, validate_sources
from localization_supervisor.core.interfaces import (
    Hypothesis, HypothesisSample, MotionSample, RecoveryContext, SourceKind, Status)
from localization_supervisor.core.recovery import STANDARD_GENERATORS
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.ros.adapters import AdapterContext, build_adapters

KINDS = {'amcl': SourceKind.POSE, 'particles': SourceKind.HYPOTHESES, 'odom': SourceKind.MOTION}


def _config(extra=None):
    raw = {
        'version': 1,
        'sources': {
            'amcl': {'kind': 'pose', 'input': {'type': 'pose_with_covariance', 'topic': '/amcl_pose'}},
            'particles': {'kind': 'hypotheses',
                          'input': {'type': 'particle_cloud', 'topic': '/particle_cloud', 'min_interval_sec': 0.5}},
            'odom': {'kind': 'motion', 'input': {'type': 'odometry', 'topic': '/odom'}},
        },
        'detectors': {
            'split': {'type': 'hypothesis_split', 'sources': {'pose': 'amcl', 'hypotheses': 'particles'},
                      'weight': 0.0},
        },
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 4.0}},
        'state_machine': {'recovery_enabled': False},
    }
    raw.update(extra or {})
    return parse_config(raw)


def _header(t):
    sec = int(math.floor(t))
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9))), frame_id='map')


def _particle(x, y, yaw):
    pose = SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)))
    return SimpleNamespace(pose=pose, weight=1.0)


def _cloud(t, groups, seed=0):
    """groups は (x, y, yaw, 粒子の数) の列。"""
    rng = np.random.default_rng(seed)
    particles = []
    for x, y, yaw, n in groups:
        for dx, dy, dyaw in rng.normal(0.0, [0.1, 0.1, 0.03], size=(n, 3)):
            particles.append(_particle(x + dx, y + dy, yaw + dyaw))
    return SimpleNamespace(header=_header(t), particles=particles)


def _hypotheses(t, items):
    """items は (重み, x, y, yaw) の列。"""
    return HypothesisSample(t, [Hypothesis(w, (x, y, yaw), np.diag([0.01, 0.01, 0.001])) for w, x, y, yaw in items])


def _store():
    store = SampleStore(KINDS, ('odom',))
    for i in range(19):
        store.add_motion('odom', MotionSample(90.0 + i * 0.5, (0.0, 0.0, 0.0)))
    return store


def test_hypotheses_source_is_not_independent():
    config = _config()
    assert config.kinds()['particles'] == SourceKind.HYPOTHESES
    assert 'particles' not in config.independent_sources()
    split = config.detectors['split']
    assert STANDARD_DETECTORS['hypothesis_split'](split.name, split.params, split.sources).subjects() == ['amcl']


def test_particle_cloud_adapter_splits_modes_and_throttles():
    config = _config()
    store = SampleStore(config.kinds(), ('odom',))
    adapter = build_adapters(config, store, AdapterContext())['particles']
    handler = adapter.subscriptions()[0].handler
    handler(_cloud(100.0, [(0.0, 0.0, 0.0, 300), (0.0, 0.0, math.pi / 2, 100)]))
    handler(_cloud(100.2, [(5.0, 5.0, 0.0, 400)]))
    sample = store.snapshot(100.3).hypotheses('particles')
    assert sample.stamp == pytest.approx(100.0)
    assert [h.weight for h in sample.hypotheses] == pytest.approx([0.75, 0.25])
    assert sample.hypotheses[1].pose[2] == pytest.approx(math.pi / 2, abs=0.02)
    handler(_cloud(100.6, [(5.0, 5.0, 0.0, 400)]))
    assert store.snapshot(100.7).hypotheses('particles').hypotheses[0].pose[0] == pytest.approx(5.0, abs=0.05)
    assert store.snapshot(110.0).hypotheses('particles', max_age=3.0) is None


def test_pose_fault_moves_hypotheses_that_follow():
    faults = parse_faults({'faults': [
        {'type': 'pose_offset', 'source': 'amcl', 'at': 10.0, 'offset': {'x': 3.0, 'yaw': 0.5},
         'followers': {'particles': 1.0}}]})
    validate_sources(faults, KINDS)
    store = FaultInjectingStore(FaultInjector(faults, 0.0), KINDS, ('odom',))
    store.add_hypotheses('particles', _hypotheses(5.0, [(1.0, 1.0, 2.0, 0.0)]))
    assert store.snapshot(5.0).hypotheses('particles').hypotheses[0].pose == pytest.approx((1.0, 2.0, 0.0))
    store.add_hypotheses('particles', _hypotheses(11.0, [(1.0, 1.0, 2.0, 0.0)]))
    assert store.snapshot(11.0).hypotheses('particles').hypotheses[0].pose == pytest.approx((4.0, 2.0, 0.5))


def test_hypotheses_cannot_be_the_source_of_a_pose_fault():
    faults = parse_faults({'faults': [{'type': 'pose_offset', 'source': 'particles', 'at': 1.0, 'offset': {'x': 1.0}}]})
    with pytest.raises(ValueError):
        validate_sources(faults, KINDS)


def _split(**params):
    cls = STANDARD_DETECTORS['hypothesis_split']
    return cls('split', {**{k: s.default for k, s in cls.PARAMS.items()}, **params},
               {'pose': 'amcl', 'hypotheses': 'particles'})


def test_split_scores_the_largest_separated_hypothesis():
    store = _store()
    store.add_hypotheses('particles', _hypotheses(100.0, [(0.6, 0.0, 0.0, 0.0), (0.3, 0.1, 0.0, 1.5),
                                                          (0.1, 0.2, 0.0, 0.0)]))
    evidence = _split(threshold=0.2).evaluate(store.snapshot(100.5))
    assert evidence.status == Status.ANOMALY
    assert evidence.score == pytest.approx(1.5)
    assert evidence.values['second_weight'] == pytest.approx(0.3)
    assert evidence.values['modes'] == 3


def test_split_is_normal_with_one_hypothesis_and_unknown_without():
    store = _store()
    assert _split().evaluate(store.snapshot(100.0)).status == Status.UNKNOWN
    store.add_hypotheses('particles', _hypotheses(100.0, [(0.9, 0.0, 0.0, 0.0), (0.1, 0.1, 0.0, 0.0)]))
    evidence = _split().evaluate(store.snapshot(100.0))
    assert evidence.status == Status.NORMAL
    assert evidence.score == 0.0


def _modes(**params):
    cls = STANDARD_GENERATORS['hypothesis_modes']
    return cls('hypotheses', {**{k: s.default for k, s in cls.PARAMS.items()}, **params}, {'hypotheses': 'particles'})


def test_modes_proposes_separated_hypotheses_moved_to_now():
    store = _store()
    store.add_hypotheses('particles', _hypotheses(99.0, [
        (0.5, 0.0, 0.0, 0.0), (0.3, 0.2, 0.0, 0.05), (0.15, 3.0, 0.0, 0.0), (0.05, 6.0, 0.0, 0.0)]))
    # 99 s から 100 s のあいだに x へ 1 m 動いた
    store.add_motion('odom', MotionSample(100.0, (1.0, 0.0, 0.0)))
    generator = _modes(min_weight=0.1, max_candidates=3)
    candidates = generator.generate(RecoveryContext(store.snapshot(100.0)))
    assert generator.status == 'ok'
    assert [c.info['weight'] for c in candidates] == pytest.approx([0.5, 0.15])
    assert candidates[1].pose[0] == pytest.approx(4.0, abs=1e-6)
    assert candidates[0].cov[0, 0] == pytest.approx(0.3 ** 2)


def test_modes_reports_why_it_has_no_candidate():
    store = _store()
    generator = _modes()
    assert generator.generate(RecoveryContext(store.snapshot(100.0))) == []
    assert generator.status == 'no_recent_sample'
