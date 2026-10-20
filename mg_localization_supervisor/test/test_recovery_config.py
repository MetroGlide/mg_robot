"""このロボットの自動復旧の構成 (supervision.yaml の recovery とプロファイル) のテスト。

プロファイル gnss_fallback は、以前の「AMCL と GNSS が乖離したら GNSS で引き戻す」動作と同じになることを確かめる。
"""
import pathlib
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import yaml

from localization_supervisor.core.config import load_config
from localization_supervisor.core.interfaces import MotionSample, PoseSample
from localization_supervisor.core.state_machine import State
from localization_supervisor.core.supervisor import Supervisor, make_store
from localization_supervisor.ros.actuators import build_actuators
from localization_supervisor.ros.resetters import build_resetters
from localization_supervisor.ros.own_messages import OwnMessages
from localization_supervisor.ros.supervisor_node import check_node_supports
from mg_localization_supervisor.actuators import pause_request
from mg_localization_supervisor.actuators.clear_costmap import ClearCostmap
from mg_localization_supervisor.actuators.pause_request import PauseRequestActuator

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
SUPERVISION = str(PACKAGE_DIR / 'config' / 'supervision.yaml')


def _yaml(name):
    with open(PACKAGE_DIR / 'config' / name, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_base_config_recovers_and_asks_to_stop():
    config = load_config(SUPERVISION)
    assert config.machine.recovery_enabled and config.machine.request_hold
    assert list(config.generators) == ['gnss', 'dead_reckoning', 'submap']
    assert list(config.scorers) == ['scan', 'gnss_agreement', 'dead_reckoning_agreement', 'free']
    assert config.scorers['free'].veto_below is not None


def test_notify_only_profile_only_turns_recovery_off():
    base = load_config(SUPERVISION)
    config = load_config(SUPERVISION, 'notify_only')
    assert not config.machine.recovery_enabled
    assert list(config.enabled_detectors()) == list(base.enabled_detectors())
    assert list(config.generators) == list(base.generators)


def test_node_can_run_every_profile_with_recovery():
    check_node_supports(load_config(SUPERVISION))


def test_actuators_and_resetters_are_built_from_the_config():
    config = load_config(SUPERVISION)
    node = MagicMock()
    actuators = build_actuators(config.actuators, node)
    assert [type(a) for a in actuators][1:3] == [PauseRequestActuator, ClearCostmap]
    assert [a.name for a in actuators][3:] == ['variance_scale']
    resetters = build_resetters(config.resetters, node, OwnMessages())
    assert [r.name for r in resetters] == ['amcl', 'robot_localization']


def test_gate_service_matches_the_arbiter_requester():
    config = load_config(SUPERVISION)
    gate = config.actuators[0]
    arbiter = _yaml('gate_arbiter.yaml')['amcl_gate_arbiter']['ros__parameters']
    requester = gate['params']['service'].split('/')[2]
    assert requester in arbiter['requesters']
    assert gate['params']['service'].startswith('/amcl_gate_arbiter/')


def test_variance_scale_topic_matches_the_gnss_bridge_parameters():
    config = load_config(SUPERVISION)
    (scale,) = [a for a in config.actuators if a['type'] == 'variance_scale']
    topic = scale['params']['topic']
    for path in ('slam_gnss_2d/slam_gnss_2d/params/nav_bridge.yaml', 'mg_simulation/config/nav_bridge_sim.yaml'):
        with open(PACKAGE_DIR.parent / path, encoding='utf-8') as f:
            bridge = yaml.safe_load(f)['slam_gnss_nav_bridge']['ros__parameters']
        assert bridge['variance_scale_topic'] == topic, path
    # 倍率を求める情報源は、監督の設定にある
    residual = config.residuals[scale['params']['residual']]
    assert residual.a == 'gnss'


def test_the_gnss_bridge_accepts_the_largest_scale_the_supervisor_publishes():
    # ブリッジが上限で切ると、監督が割り戻す倍率とかけられた倍率が食い違い、GNSS の誤差を小さく見積もる
    config = load_config(SUPERVISION)
    residual = config.residuals['gnss_vs_ekf']
    for path in ('slam_gnss_2d/slam_gnss_2d/params/nav_bridge.yaml', 'mg_simulation/config/nav_bridge_sim.yaml'):
        with open(PACKAGE_DIR.parent / path, encoding='utf-8') as f:
            bridge = yaml.safe_load(f)['slam_gnss_nav_bridge']['ros__parameters']
        # 書いていなければ、ブリッジのコードの既定 (100)
        assert bridge.get('variance_scale_max', 100.0) >= residual.max_scale, path


def test_gnss_fallback_does_not_scale_the_gnss_variance():
    assert load_config(SUPERVISION, 'gnss_fallback').residuals['gnss_vs_ekf'].max_scale == 1.0


def test_a_gnss_that_disagrees_is_only_isolated_and_amcl_is_vouched_for_by_the_submap_check():
    config = load_config(SUPERVISION)
    assert config.machine.isolate_only == ['gnss']
    # AMCL と独立でない scan_amcl は vouch しない
    assert {name: d.vouch for name, d in config.detectors.items() if d.vouch} == {'submap_amcl': ['amcl']}
    assert 'submap_amcl' in config.enabled_detectors()


def test_actuators_are_the_same_in_every_profile():
    base = load_config(SUPERVISION)
    assert [a.get('name', a['type']) for a in base.actuators] == [
        'gate', 'pause_request', 'clear_costmap', 'variance_scale']
    for profile in base.profiles:
        assert load_config(SUPERVISION, profile).actuators == base.actuators


def test_gnss_fallback_neither_holds_nor_scales_nor_isolates():
    config = load_config(SUPERVISION, 'gnss_fallback')
    assert not config.machine.request_hold
    assert config.residuals['gnss_vs_ekf'].max_scale == 1.0
    assert config.machine.isolate_only == []


def test_pause_request_is_active_only_while_holding(monkeypatch):
    monkeypatch.setattr(pause_request, 'PauseRequest', SimpleNamespace)
    node = MagicMock()
    actuator = PauseRequestActuator('pause', {'topic': '/t', 'requester_id': 'localization'}, node)
    actuator.apply(SimpleNamespace(hold_requested=True, state=State.RECOVERING, blame='amcl'))
    actuator.apply(SimpleNamespace(hold_requested=False, state=State.NORMAL, blame=''))
    first, second = [call.args[0] for call in node.create_publisher.return_value.publish.call_args_list]
    assert first.active is True and 'RECOVERING' in first.reason and first.requester_id == 'localization'
    assert second.active is False


# ---------------------------------------------------------------- gnss_fallback = 以前の動作

def test_gnss_fallback_matches_the_previous_initializer():
    config = load_config(SUPERVISION, 'gnss_fallback')
    initializer = _yaml('initializer.yaml')['gnss_amcl_initializer_node']['ros__parameters']
    assert list(config.enabled_detectors()) == ['amcl_gnss']
    assert list(config.generators) == ['gnss'] and not config.scorers
    gnss = config.generators['gnss'].params
    assert gnss['consecutive'] == initializer['required_consecutive_good']
    cov = initializer['pose_covariance']
    assert gnss['min_std_m'] ** 2 == cov[0] and abs(gnss['yaw_std_rad'] ** 2 - cov[35]) < 1e-3
    assert [r['type'] for r in config.resetters] == ['amcl', 'robot_localization']
    # 切り離しも一時停止もせず、すぐ初期化し直す
    assert config.machine.recovery_enabled and not config.machine.request_hold
    assert config.machine.isolate_hold_sec == 0.0


def test_gnss_fallback_reinitializes_to_gnss_when_amcl_diverges():
    config = load_config(SUPERVISION, 'gnss_fallback')
    supervisor = Supervisor(config)
    store = make_store(config)
    directives = []
    for t in range(0, 120):
        stamp = float(t)
        store.add_motion('odom', MotionSample(stamp, (0.0, 0.0, 0.0)))
        amcl_x = 0.0 if t < 60 else 8.0
        store.add_pose('amcl', PoseSample(stamp, (amcl_x, 0.0, 0.0), np.diag([0.01, 0.01, 0.01])))
        store.add_pose('gnss', PoseSample(stamp, (0.0, 0.0, 0.0), np.diag([0.04, 0.04, 0.01])))
        result = supervisor.step(store.snapshot(stamp))
        assert not result.hold_requested
        directives += [(t, d) for d in result.directives]
        if any(d.kind == 'reinit' for _, d in directives):
            break
    reinit = [(t, d) for t, d in directives if d.kind == 'reinit']
    assert reinit, 'no reinit'
    t, directive = reinit[0]
    assert 60 <= t <= 70
    assert directive.candidate.origin == 'gnss' and directive.candidate.pose[:2] == (0.0, 0.0)
    assert directive.candidate.total is None   # 評価はしない
