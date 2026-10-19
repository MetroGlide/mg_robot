"""残差から求める分散の倍率 (信頼できる周期に、2 つの情報源の位置の差を共分散で正規化して残す) と、倍率を出す対処のテスト。"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.interfaces import CONTRACT_VERSION, PoseSample
from localization_supervisor.core.residuals import ResidualConfig, ResidualStats, ResidualTracker
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.supervisor import Supervisor, make_store
from localization_supervisor.ros.actuators import build_actuators, variance_scale

from supervisor_test_helpers import Drive, cov, pose_sample


def test_tracker_updates_the_scale_only_with_enough_new_samples():
    tracker = ResidualTracker(ResidualConfig(window_sec=10.0, min_samples=3, gain=1.0))
    # Δ² の平均 8、a の分散 1、b の分散 0 -> 期待される Δ² は 2 -> 倍率 4 なら Δ² の平均 8 に一致する
    tracker.add(0.0, 8.0, 1.0, 0.0)
    tracker.add(1.0, 8.0, 1.0, 0.0)
    assert tracker.update(1.0).scale == 1.0
    tracker.add(2.0, 8.0, 1.0, 0.0)
    stats = tracker.update(2.0)
    assert stats.count == 3 and stats.scale == pytest.approx(4.0)
    # 倍率をかけた後の分散で正規化した Δ² の平均は、期待値の 2 に近い
    assert stats.mean_d2 == pytest.approx(2.0)
    # 新しい標本がない周期 (信頼できない周期) は更新しない
    assert tracker.update(3.0).scale == pytest.approx(4.0)


def test_other_variance_is_subtracted_before_the_scale():
    tracker = ResidualTracker(ResidualConfig(min_samples=1, gain=1.0))
    # Δ² = 8 のうち、b などの分散が 2 * 3 = 6 を説明し、a の分散 1 で足りない 2 * 1 = 2 を倍率 1 で説明する
    tracker.add(0.0, 8.0, 1.0, 3.0)
    assert tracker.update(0.0).scale == pytest.approx(1.0)


def test_scale_is_clamped_and_comes_back_to_one():
    tracker = ResidualTracker(ResidualConfig(window_sec=30.0, min_samples=1, gain=1.0, max_scale=10.0))
    for t in range(30):
        tracker.add(float(t), 200.0, 1.0, 0.0)
        tracker.update(float(t))
    assert tracker.scale == 10.0
    # 残差が小さくなると、窓の中の標本が入れ替わって、1 まで戻る
    for t in range(30, 100):
        tracker.add(float(t), 0.0, 1.0, 0.0)
        tracker.update(float(t))
    assert tracker.scale == 1.0


def test_consistent_residual_does_not_raise_the_scale():
    tracker = ResidualTracker(ResidualConfig(min_samples=2, gain=1.0))
    for t in range(5):
        tracker.add(float(t), 4.0, 1.0, 1.0)
        tracker.update(float(t))
    assert tracker.scale == pytest.approx(1.0)


def test_the_store_divides_the_scale_we_asked_for_out_of_the_variance():
    # 受け取る側が倍率 s をかけた分散で届くので、標本を入れるときに、そのとき効いていた倍率で割る
    config = _config(scale_delay_sec=0.5)
    store = make_store(config)
    store.note_scale('gnss', 10.0, 4.0)
    for stamp in (10.2, 10.5, 11.0):
        store.add_pose('gnss', PoseSample(stamp, (0.0, 0.0, 0.0), cov(0.2) * 4.0))
    variances = [float(s.cov[0, 0]) for s in store.poses['gnss'].samples()]
    # 出してから 0.5 s 後に効き始める。それより前の標本は、倍率がかかっていない
    assert variances == pytest.approx([0.16, 0.04, 0.04])
    assert store.poses['gnss'].samples()[0].cov[1, 1] == pytest.approx(0.16)


def test_the_store_does_not_divide_when_scales_are_not_tracked():
    store = make_store(_config(), track_scales=False)
    store.note_scale('gnss', 10.0, 4.0)
    store.add_pose('gnss', PoseSample(11.0, (0.0, 0.0, 0.0), cov(0.2)))
    assert float(store.poses['gnss'].latest().cov[0, 0]) == pytest.approx(0.04)


def test_tracker_forgets_old_samples_and_clears():
    tracker = ResidualTracker(ResidualConfig(window_sec=3.0, min_samples=1, gain=1.0))
    tracker.add(0.0, 20.0, 1.0, 0.0)
    tracker.add(10.0, 2.0, 1.0, 0.0)
    assert tracker.update(10.0).count == 1
    tracker.clear()
    assert tracker.update(10.0) == ResidualStats(0, 0.0, 1.0)


def _config(**residual):
    raw = {
        'version': CONTRACT_VERSION,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'}},
        'timeline': {'motion_priority': ['odom'], 'alignment_std_per_m': 0.0, 'alignment_std_per_rad': 0.0},
        'detectors': {'amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'}}},
        'fusion': {'type': 'cusum', 'params': {'suspect_level': 1.0, 'confirm_level': 5.0}},
        'residuals': {'gnss_vs_amcl': {'a': 'gnss', 'b': 'amcl', 'min_samples': 5, 'window_sec': 30.0,
                                       'notify_scale': 2.0, 'gain': 0.2, **residual}},
    }
    return parse_config(raw)


def _run(config, bias_x, seconds=60, apply_scale=False):
    """gnss は真値から bias_x ずれていて、自己申告の標準偏差は 0.1 m。apply_scale なら、受け取る側のように、
    出した倍率を gnss の共分散にかけて渡す。"""
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
        store.add_pose('amcl', pose_sample(stamp, truth, std_xy=0.05))
        store.add_pose('gnss', PoseSample(stamp, (truth[0] + bias_x, truth[1], truth[2]), cov(0.1) * scale))
        results.append(supervisor.step(store.snapshot(stamp)))
        if apply_scale:
            scale = results[-1].applied_scales['gnss_vs_amcl']
            store.note_scale('gnss', stamp, scale)
    return results


def test_a_biased_source_raises_the_scale_and_notifies_once():
    results = _run(_config(), bias_x=1.0)
    stats = results[-1].residuals['gnss_vs_amcl']
    # 自己申告の標準偏差 0.1 m に対して 1 m ずれている
    assert stats.scale > 3.0 and stats.count >= 5
    events = [e for r in results for e in r.events if e.name == 'localization.residual_changed']
    assert len(events) == 1 and events[0].fields['high'] is True and events[0].level == 'warn'


def test_the_scale_settles_when_the_receiver_applies_it():
    results = _run(_config(max_scale=1000.0), bias_x=1.0, seconds=400, apply_scale=True)
    scales = [r.residuals['gnss_vs_amcl'].scale for r in results]
    # 1 / (0.01 s + 0.0025 + 0.09) = 2 となる s = (0.5 - 0.0925) / 0.01 = 40.75 で落ち着く
    assert scales[-1] == pytest.approx(40.75, rel=0.1)
    assert abs(scales[-1] - scales[-20]) / scales[-1] < 0.05


def test_a_consistent_source_keeps_the_scale_at_one_without_events():
    results = _run(_config(), bias_x=0.0)
    assert results[-1].residuals['gnss_vs_amcl'].scale == pytest.approx(1.0, abs=0.3)
    assert not [e for r in results for e in r.events if e.name == 'localization.residual_changed']


def test_residuals_are_only_collected_while_trusted():
    # 起動の猶予の間は信頼できる周期にならないので、標本がたまらない
    config = parse_config({
        'version': CONTRACT_VERSION,
        'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'}},
        'timeline': {'motion_priority': ['odom']},
        'detectors': {'amcl_jump': {'type': 'pose_jump', 'sources': {'pose': 'amcl', 'motion': 'odom'}}},
        'grace': {'startup_sec': 100.0},
        'residuals': {'gnss_vs_amcl': {'a': 'gnss', 'b': 'amcl', 'min_samples': 1}}})
    results = _run(config, bias_x=1.0, seconds=10)
    assert results[-1].residuals['gnss_vs_amcl'].count == 0


def test_residual_config_is_validated():
    with pytest.raises(ConfigError, match='not a pose source'):
        _config(a='odom')
    with pytest.raises(ConfigError, match='must differ'):
        _config(a='amcl')
    with pytest.raises(ConfigError, match='unknown keys'):
        _config(nope=1)


# ---------------------------------------------------------------- 対処

def _actuator(**params):
    node = MagicMock()
    (actuator,) = build_actuators([{'type': 'variance_scale', 'params': {
        'topic': '/scale', 'residual': 'r', **params}}], node)
    return actuator, node.create_publisher.return_value


def _result(scale=None):
    return SimpleNamespace(applied_scales={} if scale is None else {'r': scale})


def test_variance_scale_actuator_publishes_the_applied_scale(monkeypatch):
    monkeypatch.setattr(variance_scale, 'Float64', SimpleNamespace)
    actuator, pub = _actuator()
    actuator.apply(_result(4.0))
    actuator.apply(_result(None))
    assert [call.args[0].data for call in pub.publish.call_args_list] == [4.0, 1.0]


def test_variance_scale_actuator_publishes_one_when_released(monkeypatch):
    monkeypatch.setattr(variance_scale, 'Float64', SimpleNamespace)
    actuator, pub = _actuator()
    actuator.release()
    assert [call.args[0].data for call in pub.publish.call_args_list] == [1.0]


def test_the_applied_scale_is_the_maximum_while_the_source_is_isolated():
    # a (gnss) を切り離しているあいだは、残差の倍率に関わらず上限にする
    supervisor = Supervisor(_config(max_scale=50.0))
    machine = supervisor.machine
    machine.config.recovery_enabled = True
    machine.state = State.ISOLATED
    machine.blame = 'gnss'
    stats = {'gnss_vs_amcl': ResidualStats(30, 2.0, 1.5)}
    assert supervisor._applied_scales(stats) == {'gnss_vs_amcl': 50.0}
    machine.blame = 'amcl'
    assert supervisor._applied_scales(stats) == {'gnss_vs_amcl': 1.5}
    machine.state = State.NORMAL
    machine.blame = ''
    assert supervisor._applied_scales(stats) == {'gnss_vs_amcl': 1.5}

