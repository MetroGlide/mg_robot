"""自己位置推定の構成の名前から launch の引数を決める (mg_navigation/localization_config.py) のテスト。"""
import os

import pytest

from mg_navigation.localization_config import CONFIGS, DEFAULT_CONFIG, resolve
from mg_navigation.localization_preset import PRESET_SUBDIR, REQUIRED_MONITOR

NAVIGATION_DIR = os.path.join(os.path.dirname(__file__), '..')
SUPERVISOR_PROFILES = os.path.join(NAVIGATION_DIR, '..', 'mg_localization_supervisor', 'config', 'profiles')
EMPTY = {key: '' for key in CONFIGS[DEFAULT_CONFIG]}


def test_default_is_supervised():
    assert DEFAULT_CONFIG == 'supervised'
    assert resolve('', EMPTY) == CONFIGS['supervised']


def test_empty_overrides_follow_the_config():
    assert resolve('legacy', EMPTY) == CONFIGS['legacy']


def test_overrides_win_and_none_clears_preset_and_profile():
    values = resolve('supervised', {**EMPTY, 'localization_preset': 'none', 'localization_profile': 'notify_only'})
    assert values['localization_preset'] == ''
    assert values['localization_profile'] == 'notify_only'
    assert values['localization_gate'] == 'true'


def test_unknown_config_and_key_are_errors():
    with pytest.raises(RuntimeError):
        resolve('new', EMPTY)
    with pytest.raises(RuntimeError):
        resolve('legacy', {'use_navigation': 'true'})


def test_every_config_is_a_consistent_combination():
    """プリセットは自己位置監視と組。AMCL 検証ゲートが補正するのは自己位置監視の監視のときだけ。observe は watchdog と並走する自己位置監視で判定だけ。"""
    for name, values in CONFIGS.items():
        monitor = values['localization_monitor']
        if values['localization_preset']:
            assert monitor == REQUIRED_MONITOR, name
            assert os.path.isfile(os.path.join(NAVIGATION_DIR, PRESET_SUBDIR, values['localization_preset'] + '.yaml'))
        if values['localization_gate'] == 'true':
            assert monitor == 'supervisor', name
        if values['localization_gate'] == 'observe':
            assert monitor == 'supervisor' or values['localization_supervisor'] == 'true', name
        if values['localization_profile']:
            assert os.path.isfile(os.path.join(SUPERVISOR_PROFILES, values['localization_profile'] + '.yaml'))
