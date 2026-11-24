"""このロボットの門のノードの設定 (config/motion_gate.yaml) と、その起動の組み合わせを確かめる。"""
import ast
import pathlib

import yaml

from localization_supervisor.core.config import load_config
from localization_supervisor.core.interfaces import CONTRACT_VERSION
from localization_supervisor.core.motion_gate_config import load_gate_config
from localization_supervisor.core.plugins import resolve
from localization_supervisor.core.snapshot import SampleStore
from localization_supervisor.ros.adapters import AdapterContext, build_adapters
from localization_supervisor.ros.hold_outputs import HoldOutput
from mg_localization_supervisor.hold_outputs.pause_request import PauseRequestHold

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
CONFIG = str(PACKAGE_DIR / 'config' / 'motion_gate.yaml')
SUPERVISION = str(PACKAGE_DIR / 'config' / 'supervision.yaml')
LAUNCH = PACKAGE_DIR / 'launch' / 'supervisor.launch.py'


def _yaml(name):
    with open(PACKAGE_DIR / 'config' / name, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_motion_gate_config_loads_and_builds_inputs():
    config = load_gate_config(CONFIG)
    assert config.gate.settings.estimator == 'amcl'
    assert {c.name for c in config.gate.closers} == {'amcl_jump', 'amcl_window', 'amcl_gnss'}
    adapters = build_adapters(config.sources, SampleStore(config.gate.kinds), AdapterContext())
    assert sorted(adapters) == sorted(config.sources)
    assert _yaml('motion_gate.yaml')['version'] == CONTRACT_VERSION


def test_inputs_are_the_same_as_the_supervisor():
    gate = load_gate_config(CONFIG)
    supervisor = load_config(SUPERVISION)
    for name, source in gate.sources.items():
        assert source.input == supervisor.sources[name].input, name


def test_gate_service_is_a_requester_of_the_arbiter():
    gate = load_gate_config(CONFIG)
    arbiter = _yaml('gate_arbiter.yaml')['amcl_gate_arbiter']['ros__parameters']
    assert gate.gate_service == '/amcl_gate_arbiter/motion_gate/change_publish_state'
    assert 'motion_gate' in arbiter['requesters'] and 'motion_gate' in arbiter['warn_holders']


def test_grace_after_reset_covers_the_window_closer():
    """置き直しの後の猶予は、時間の窓で比べる閉じる理由が、置き直しの前の標本と比べなくなるまで続く。"""
    config = load_gate_config(CONFIG)
    window = next(c for c in config.gate.closers if c.name == 'amcl_window').detector.params
    assert config.gate.settings.grace_after_reset_sec >= window['window_sec'] + window['max_gap_sec']


def test_only_the_gnss_closer_is_active_in_grace():
    """置き直しの前の標本と比べる部品 (動きの比較) は猶予の間に使わず、今の AMCL と GNSS を比べる部品は使う。"""
    config = load_gate_config(CONFIG)
    assert {c.name for c in config.gate.closers if c.active_in_grace} == {'amcl_gnss'}


def test_manual_search_range_is_within_the_match_cap():
    """手動の初期姿勢の探す範囲は、照合の範囲の上限で切られない。"""
    config = load_gate_config(CONFIG)
    settings, match = config.gate.settings, config.gate.map_match
    assert settings.manual_xy_m <= match.cap_xy_m and settings.manual_yaw_rad <= match.cap_yaw_rad


def test_hold_output_is_the_pause_request():
    config = load_gate_config(CONFIG)
    assert resolve(config.hold_output['type'], {}, HoldOutput, 'hold_output') is PauseRequestHold


def test_manual_pose_topic_matches_the_launch_relay():
    config = load_gate_config(CONFIG)
    tree = ast.parse(LAUNCH.read_text(encoding='utf-8'))
    values = {node.targets[0].id: node.value.value for node in tree.body
              if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)}
    assert values['MANUAL_POSE_TOPIC'] == config.manual_pose_topic


def test_web_ui_sends_manual_pose_to_the_topic():
    config = load_gate_config(CONFIG)
    topics = (PACKAGE_DIR.parent / 'mg_ui' / 'mg_web_ui' / 'frontend' / 'src' / 'ros' / 'topics.ts').read_text(
        encoding='utf-8')
    assert f"'{config.manual_pose_topic}'" in topics
