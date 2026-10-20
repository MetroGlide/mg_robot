"""設定ファイルのスキーマの検証と、プラグインの読み込みのテスト。"""
import copy
import pathlib

import pytest

from localization_supervisor.core.config import ConfigError, load_config, merge, parse_config
from localization_supervisor.core.detectors.pose_consistency import PoseConsistency
from localization_supervisor.core.fusion import ConsecutiveFusion
from localization_supervisor.core.interfaces import CONTRACT_VERSION

from supervisor_test_helpers import AlwaysNormal

EXAMPLES = sorted((pathlib.Path(__file__).resolve().parents[1] / 'config' / 'examples').glob('*.yaml'))

BASE = {
    'version': CONTRACT_VERSION,
    'sources': {'amcl': {'kind': 'pose'}, 'gnss': {'kind': 'pose'}, 'odom': {'kind': 'motion'},
                'ekf': {'kind': 'pose', 'derived_from': ['amcl', 'gnss', 'odom']}},
    'detectors': {
        'amcl_gnss': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                      'params': {'threshold': 6.0}},
    },
}


def _raw(**changes):
    raw = copy.deepcopy(BASE)
    raw.update(changes)
    return raw


def test_examples_load():
    assert EXAMPLES
    for path in EXAMPLES:
        config = load_config(str(path))
        assert config.enabled_detectors()


def test_parse_fills_defaults():
    config = parse_config(_raw())
    detector = config.detectors['amcl_gnss']
    assert detector.cls is PoseConsistency
    assert detector.params['max_age_sec'] == 3.0
    assert config.independent_sources() == ['amcl', 'gnss', 'odom']
    assert config.derived_sources() == {'ekf': ['amcl', 'gnss', 'odom']}


@pytest.mark.parametrize('changes, message', [
    ({'version': 0}, 'version'),
    ({'unknown_key': 1}, 'unknown keys'),
    ({'sources': {'x': {'kind': 'laser'}}}, 'kind'),
    ({'sources': {'x': {'kind': 'pose', 'derived_from': ['y']}}}, 'derived_from'),
    ({'detectors': {'d': {'type': 'no_such', 'sources': {}}}}, 'unknown type'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'nope'},
                          'params': {'threshold': 1.0}}}}, 'unknown source'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'odom'},
                          'params': {'threshold': 1.0}}}}, 'is motion, expected pose'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl'},
                          'params': {'threshold': 1.0}}}}, 'expected arguments'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'}}}}, 'required'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 'x'}}}}, 'expected float'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 1.0, 'typo': 1}}}}, 'unknown params'),
    ({'detectors': {'d': {'type': 'pose_consistency', 'sources': {'a': 'amcl', 'b': 'gnss'},
                          'params': {'threshold': 1.0, 'metric': 'bad'}}}}, 'metric'),
    ({'detectors': {'d': {'type': 'scan_map', 'sources': {'pose': 'amcl', 'reference': 'amcl'}}}},
     'expected reference'),
    ({'timeline': {'motion_priority': ['amcl']}}, 'not a motion source'),
    ({'fusion': {'type': 'cusum', 'params': {'nope': 1}}}, 'unknown params'),
    ({'trust': {'require_normal': ['nope']}}, 'unknown detector'),
    ({'anchors': {'pose': 'odom', 'motion': 'odom'}}, 'not a pose source'),
    ({'state_machine': {'clear_ticks': 'many'}}, 'integer'),
    ({'enabled_detectors': ['nope']}, 'unknown detector'),
])
def test_invalid_configs_fail_at_load(changes, message):
    with pytest.raises(ConfigError, match=message):
        parse_config(_raw(**changes))


def test_external_plugin_by_module_path():
    raw = _raw(detectors={'d': {'type': 'supervisor_test_helpers:AlwaysNormal', 'sources': {'pose': 'amcl'},
                                'params': {'gain': 2}}})
    config = parse_config(raw)
    assert config.detectors['d'].cls is AlwaysNormal
    assert config.detectors['d'].params == {'gain': 2.0}
    with pytest.raises(ConfigError, match='cannot import'):
        parse_config(_raw(detectors={'d': {'type': 'no_such_module:X', 'sources': {}}}))
    with pytest.raises(ConfigError, match='is not a Detector'):
        parse_config(_raw(detectors={'d': {'type': 'supervisor_test_helpers:NotAPlugin', 'sources': {}}}))


def test_a_detector_that_requires_anchors_needs_them_configured():
    detectors = {'d': {'type': 'supervisor_test_helpers:NeedsAnchors', 'sources': {'pose': 'amcl'}}}
    with pytest.raises(ConfigError, match="requires 'anchors'"):
        parse_config(_raw(detectors=detectors))
    parse_config(_raw(detectors=detectors, anchors={'pose': 'ekf', 'motion': 'odom'}))


def test_profiles_overlay_and_replace_plugin_type():
    profiles = {'strict': {'detectors': {'amcl_gnss': {'params': {'threshold': 3.0}}}},
                'counting': {'fusion': {'type': 'consecutive'}, 'enabled_detectors': []}}
    base = parse_config(_raw(fusion={'type': 'cusum', 'params': {'confirm_level': 3.0}}), profiles=profiles)
    strict = base.with_profile('strict')
    assert strict.detectors['amcl_gnss'].params['threshold'] == 3.0
    assert base.detectors['amcl_gnss'].params['threshold'] == 6.0
    counting = base.with_profile('counting')
    assert counting.fusion_cls is ConsecutiveFusion and counting.enabled_detectors() == {}
    with pytest.raises(ConfigError, match='not defined'):
        base.with_profile('nope')


def test_merge_replaces_lists_and_other_types():
    assert merge({'a': {'b': 1, 'c': [1]}}, {'a': {'c': [2]}}) == {'a': {'b': 1, 'c': [2]}}
    assert merge({'f': {'type': 'x', 'params': {'p': 1}}}, {'f': {'type': 'y'}}) == {'f': {'type': 'y'}}


def test_load_config_validates_every_profile(tmp_path):
    (tmp_path / 'bad.yaml').write_text('detectors: {amcl_gnss: {params: {threshold: x}}}\n')
    main = tmp_path / 'main.yaml'
    main.write_text(
        f'version: {CONTRACT_VERSION}\n' 'sources: {amcl: {kind: pose}, gnss: {kind: pose}}\n'
        'detectors: {amcl_gnss: {type: pose_consistency, sources: {a: amcl, b: gnss}, params: {threshold: 6.0}}}\n'
        'profiles: {bad: bad.yaml}\n')
    with pytest.raises(ConfigError, match='expected float'):
        load_config(str(main))
