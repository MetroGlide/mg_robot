"""窓口の設定ファイルが読み込め、ノード名とサービス名が互いに合っていることを確かめる。"""
import pathlib

import pytest
import yaml

CONFIG_DIR = pathlib.Path(__file__).resolve().parents[1] / 'config'


def _load(name):
    with open(CONFIG_DIR / name, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('name, node', [
    ('gate_arbiter.yaml', 'amcl_gate_arbiter'),
    ('initializer.yaml', 'gnss_amcl_initializer_node'),
])
def test_config_has_parameters_for_launched_node(name, node):
    config = _load(name)
    assert list(config) == [node]
    assert isinstance(config[node]['ros__parameters'], dict)


def test_gate_arbiter_keeps_a_requester_for_the_supervisor():
    gate = _load('gate_arbiter.yaml')['amcl_gate_arbiter']['ros__parameters']
    assert 'supervisor' in gate['requesters']


def test_gate_arbiter_accepts_waypoint_requests():
    # ウェイポイントの amcl_on / amcl_off は /amcl_gate_arbiter/waypoint/change_publish_state を呼ぶ
    gate = _load('gate_arbiter.yaml')['amcl_gate_arbiter']['ros__parameters']
    assert 'waypoint' in gate['requesters']


def test_initializer_pose_covariance_is_6x6():
    params = _load('initializer.yaml')['gnss_amcl_initializer_node']['ros__parameters']
    assert len(params['pose_covariance']) == 36
    assert all(isinstance(v, float) for v in params['pose_covariance'])
