"""このロボットの監督の構成 (config/supervision.yaml) が、フレームワークの検証を通り、他の設定と合っていることを確かめる。"""
import pathlib

import yaml

from localization_supervisor.core.config import load_config
from localization_supervisor.core.fusion import ConsecutiveFusion, CusumFusion
from localization_supervisor.core.interfaces import SourceKind
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.ros.adapters import AdapterContext, build_adapters

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


def test_consecutive_profile_matches_current_supervisor_thresholds():
    # 今の監督ノード (supervisor.yaml) と同じ判定のしきい値と回数であること
    current = _yaml(PACKAGE_DIR / 'config' / 'supervisor.yaml')['localization_supervisor_node']['ros__parameters']
    config = load_config(SUPERVISION, 'consecutive')
    detectors = config.enabled_detectors()
    assert detectors['amcl_gnss'].params['threshold'] == current['gnss_d2_threshold']
    assert detectors['amcl_gnss'].params['b_max_std_m'] == current['gnss_max_sigma_m']
    assert detectors['amcl_jump'].params['threshold_m'] == current['jump_threshold_m']
    assert detectors['amcl_jump'].params['threshold_rad'] == current['jump_threshold_rad']
    assert detectors['amcl_ekf'].params['threshold'] == current['diff_threshold_m']
    assert detectors['scan_ekf'].params['ratio_factor'] == current['scan_ratio_factor']
    options = {name: d.options for name, d in detectors.items()}
    assert options['amcl_gnss'] == {'suspect_after': current['gnss_suspect_ticks'],
                                    'confirm_after': current['gnss_isolate_ticks']}
    assert options['scan_ekf'] == {'suspect_after': current['scan_suspect_ticks'],
                                   'confirm_after': current['scan_isolate_ticks']}
    assert options['amcl_ekf'] == {'suspect_after': current['diff_suspect_ticks'],
                                   'confirm_after': current['diff_isolate_ticks']}
    assert options['amcl_jump']['corroborate_after'] == current['corroborate_ticks']
    assert config.machine.clear_ticks == current['clear_ticks']
