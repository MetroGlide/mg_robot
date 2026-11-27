"""自己位置推定のプリセット (AMCL と EKF の params に重ねる設定) を、launch で解決する。

プリセットは mg_navigation の share の params/localization_presets/<名前>.yaml。各ノードの params の後に重ねるので、
書いたキーだけが変わる。
"""
import os
from typing import List

# mg_navigation の share からの、プリセットの置き場
PRESET_SUBDIR = os.path.join('params', 'localization_presets')

# プリセットは、外れた AMCL を EKF が受け付けない設定を含む。受け付けなくなったまま戻れなくならないよう、
# 自己位置監視が初期化し直す構成と組で使う
REQUIRED_MONITOR = 'supervisor'


def preset_files(preset: str, monitor: str, navigation_share: str) -> List[str]:
    """プリセットの名前から、重ねる params のファイルの並びを返す。名前が空なら重ねない。"""
    if not preset:
        return []
    if monitor != REQUIRED_MONITOR:
        raise RuntimeError(
            f'自己位置推定のプリセット {preset} は、localization_monitor:={REQUIRED_MONITOR} と組で使います '
            f'(今は {monitor})')
    path = os.path.join(navigation_share, PRESET_SUBDIR, preset + '.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(f'自己位置推定のプリセットがありません: {path}')
    return [path]
