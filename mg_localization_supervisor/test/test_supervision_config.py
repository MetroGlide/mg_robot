"""このロボットの監督の構成 (config/supervision.yaml) が、フレームワークの検証を通り、他の設定と合っていることを確かめる。"""
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
    adapters = build_adapters(config, SampleStore(config.kinds()), AdapterContext())
    assert sorted(adapters) == sorted(config.sources)


def test_inputs_match_robot_topics():
    config = load_config(SUPERVISION)
    nav2 = _yaml(REPO_DIR / 'mg_navigation' / 'params' / 'nav2_params.yaml')['amcl']['ros__parameters']
    reference = config.sources['map_scan'].input
    assert reference['scan_topic'] == nav2['scan_topic']
    assert reference['base_frame'] == nav2['base_frame_id']
    assert config.sources['map_scan'].kind == SourceKind.REFERENCE


def test_gnss_fallback_only_compares_amcl_and_gnss():
    config = load_config(SUPERVISION, 'gnss_fallback')
    assert list(config.enabled_detectors()) == ['amcl_gnss']
    assert config.fusion_cls is ConsecutiveFusion


def test_consecutive_profile_uses_the_counting_fusion():
    config = load_config(SUPERVISION, 'consecutive')
    assert config.fusion_cls is ConsecutiveFusion
    assert sorted(config.enabled_detectors()) == ['amcl_ekf', 'amcl_gnss', 'amcl_jump', 'scan_ekf']


def test_node_can_run_every_profile():
    check_node_supports(load_config(SUPERVISION))


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
