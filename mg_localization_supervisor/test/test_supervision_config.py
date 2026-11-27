"""このロボットの自己位置監視の構成 (config/supervision.yaml) が、フレームワークの検証を通り、他の設定と合っていることを確かめる。"""
import ast
import pathlib
from unittest.mock import MagicMock

import yaml

from localization_supervisor.core.config import load_config
from localization_supervisor.core.fusion import ConsecutiveFusion, CusumFusion
from localization_supervisor.core.interfaces import SourceKind
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.ros.adapters import AdapterContext, build_adapters
from localization_supervisor.ros.event_sinks import build_event_sink
from localization_supervisor.ros.supervisor_node import check_node_supports
from mg_localization_supervisor.event_sinks.event_log import EventLogSink
from mg_utils.event_log import parse_event

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO_DIR = PACKAGE_DIR.parent
SUPERVISION = str(PACKAGE_DIR / 'config' / 'supervision.yaml')


def _yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_supervision_and_all_profiles_load():
    config = load_config(SUPERVISION)
    assert config.fusion_cls is CusumFusion
    assert config.independent_sources() == ['amcl', 'gnss', 'odom']
    assert config.derived_sources() == {'ekf': ['amcl', 'gnss', 'odom']}
    for profile in config.profiles:
        assert config.with_profile(profile).enabled_detectors()


def test_every_source_has_a_valid_input():
    config = load_config(SUPERVISION)
    adapters = build_adapters(config.sources, SampleStore(config.kinds()), AdapterContext())
    assert sorted(adapters) == sorted(config.sources)


def test_inputs_match_robot_topics():
    config = load_config(SUPERVISION)
    nav2 = _yaml(REPO_DIR / 'mg_navigation' / 'params' / 'nav2_params.yaml')['amcl']['ros__parameters']
    reference = config.sources['map_scan'].input
    assert reference['scan_topic'] == nav2['scan_topic']
    assert reference['base_frame'] == nav2['base_frame_id']
    assert config.sources['map_scan'].kind == SourceKind.REFERENCE



def test_particles_are_recorded_for_the_hypotheses():
    """仮説の判定と候補をオフライン評価で確かめられるよう、粒子のトピックを走行の記録に入れている。"""
    config = load_config(SUPERVISION)
    recorded = (REPO_DIR / 'mg_navigation' / 'params' / 'record_topic_list.txt').read_text(encoding='utf-8').split()
    particles = [s for s in config.sources.values() if s.kind == SourceKind.HYPOTHESES]
    assert particles
    assert all(s.input['topic'] in recorded for s in particles)
    assert 'hypotheses' not in config.generators
    assert 'hypotheses' in config.with_profile('hypotheses').generators


def test_gnss_fallback_only_compares_amcl_and_gnss():
    config = load_config(SUPERVISION, 'gnss_fallback')
    assert list(config.enabled_detectors()) == ['amcl_gnss']
    assert config.fusion_cls is ConsecutiveFusion


def test_node_can_run_every_profile():
    check_node_supports(load_config(SUPERVISION))


def test_gate_loads_with_every_profile():
    """AMCL 検証ゲート (use_gate) を有効にしても、基本設定とすべてのプロファイルが動かせる。基本設定では AMCL 検証ゲートを使わない。"""
    config = load_config(SUPERVISION)
    assert config.gate is None
    gated = config.with_gate(True)
    assert gated.gate is not None
    check_node_supports(gated)
    for profile in config.profiles:
        assert gated.with_profile(profile).gate is not None


def test_gate_grace_after_reset_covers_the_window_closer():
    """姿勢補正の後の猶予は、時間の窓で比べる遮断条件が、姿勢補正の前の標本と比べなくなるまで続く。"""
    gate = load_config(SUPERVISION).with_gate(True).gate
    window = next(c for c in gate.closers if c.name == 'gate_amcl_window').detector.params
    assert gate.settings.grace_after_reset_sec >= window['window_sec'] + window['max_gap_sec']


def test_only_the_gate_gnss_closer_is_active_in_grace():
    """姿勢補正の前の標本と比べる部品 (動きの比較) は猶予の間に使わず、今の AMCL と GNSS を比べる部品は使う。"""
    gate = load_config(SUPERVISION).with_gate(True).gate
    assert {c.name for c in gate.closers if c.active_in_grace} == {'gate_amcl_gnss'}


def test_gate_manual_search_range_is_within_the_match_cap():
    gate = load_config(SUPERVISION).with_gate(True).gate
    assert gate.settings.manual_xy_m <= gate.map_match.cap_xy_m
    assert gate.settings.manual_yaw_rad <= gate.map_match.cap_yaw_rad


def test_gate_manual_pose_topic_matches_the_launch_relay_and_the_web_ui():
    topic = load_config(SUPERVISION).with_gate(True).manual_pose_topic
    tree = ast.parse((PACKAGE_DIR / 'launch' / 'supervisor.launch.py').read_text(encoding='utf-8'))
    values = {node.targets[0].id: node.value.value for node in tree.body
              if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)}
    assert values['MANUAL_POSE_TOPIC'] == topic
    topics = (REPO_DIR / 'mg_ui' / 'mg_web_ui' / 'frontend' / 'src' / 'ros' / 'topics.ts').read_text(
        encoding='utf-8')
    assert f"'{topic}'" in topics


def test_gate_resetter_roles_send_amcl_and_ekf():
    config = load_config(SUPERVISION)
    assert {item['type']: item['role'] for item in config.resetters} == {
        'amcl': 'estimator', 'robot_localization': 'fused'}


def test_safety_footprint_matches_costmap():
    config = load_config(SUPERVISION)
    params = _yaml(REPO_DIR / 'mg_navigation' / 'params' / 'nav2_params.yaml')
    for costmap in ('global_costmap', 'local_costmap'):
        footprint = yaml.safe_load(params[costmap][costmap]['ros__parameters']['footprint'])
        assert [tuple(p) for p in footprint] == config.safety.footprint, costmap


def test_safety_uses_the_planning_map_of_the_global_costmap():
    config = load_config(SUPERVISION)
    params = _yaml(REPO_DIR / 'mg_navigation' / 'params' / 'nav2_params.yaml')
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
    assert config.sources[config.safety.map].input['topic'] == global_costmap['map_topic']
    assert config.safety.pose == 'ekf'


def test_event_sink_writes_project_events():
    config = load_config(SUPERVISION)
    logger = MagicMock()
    sink = build_event_sink(config.event_sink, logger)
    assert isinstance(sink, EventLogSink)
    sink.emit('localization.state_changed', 'error', {'from': 'SUSPECT', 'to': 'DEGRADED'})
    text = logger.error.call_args.args[0]
    assert parse_event(text) == ('localization.state_changed', {'from': 'SUSPECT', 'to': 'DEGRADED'})
