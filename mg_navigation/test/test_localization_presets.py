"""自己位置推定のプリセット (params/localization_presets/) を検査する。

プリセットは元の params の後に重ねるので、元にないキーを書くと黙って新しいパラメータになる。書いたキーが元にあることを確かめる。
"""
from pathlib import Path

import pytest
import yaml

from mg_navigation.localization_preset import preset_files

PACKAGE_DIR = Path(__file__).resolve().parents[1]
PRESET_DIR = PACKAGE_DIR / 'params' / 'localization_presets'
# プリセットに書いてよいノードと、そのノードの元の params
BASE_PARAMS = {
    'amcl': PACKAGE_DIR / 'params' / 'nav2_params.yaml',
    'ekf_global_node': PACKAGE_DIR.parent / 'mg_drivers' / 'params' / 'ekf_global.yaml',
}
PRESETS = sorted(PRESET_DIR.glob('*.yaml'))


def _load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_presets_exist():
    assert PRESETS


@pytest.mark.parametrize('preset', PRESETS, ids=lambda p: p.stem)
def test_preset_keys_exist_in_base_params(preset):
    for node, params in _load(preset).items():
        assert node in BASE_PARAMS, f'{preset.name}: {node} はプリセットで変えるノードではない'
        base = _load(BASE_PARAMS[node])[node]['ros__parameters']
        for key in params['ros__parameters']:
            assert key in base, f'{preset.name}: {node}.{key} が {BASE_PARAMS[node].name} にない'


def test_empty_preset_adds_nothing():
    assert preset_files('', 'none', str(PACKAGE_DIR)) == []


def test_preset_resolves_to_file():
    assert preset_files('robust', 'supervisor', str(PACKAGE_DIR)) == [str(PRESET_DIR / 'robust.yaml')]


def test_preset_requires_supervisor():
    with pytest.raises(RuntimeError):
        preset_files('robust', 'watchdog', str(PACKAGE_DIR))


def test_unknown_preset_fails():
    with pytest.raises(RuntimeError):
        preset_files('no_such_preset', 'supervisor', str(PACKAGE_DIR))
