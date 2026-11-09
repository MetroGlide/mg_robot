import importlib.util
import os

import numpy as np

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "degrade_localization_map.py")
spec = importlib.util.spec_from_file_location("degrade_localization_map", SCRIPT)
dm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dm)


def _map():
    # 解像度 1 m、原点 (0, 0)、10x10。画像の行 7 (y = 2..3)、列 4 (x = 4..5) を占有にする
    pixels = np.full((10, 10), dm.FREE_PIXEL, dtype=np.uint8)
    pixels[7, 4] = dm.OCCUPIED_PIXEL
    return pixels


def test_erase_only_inside_box():
    result = dm.erase(_map(), 1.0, 0.0, 0.0, (3.0, 1.0, 6.0, 4.0))
    assert np.all(result == dm.FREE_PIXEL)
    untouched = dm.erase(_map(), 1.0, 0.0, 0.0, (0.0, 5.0, 2.0, 7.0))
    assert untouched[7, 4] == dm.OCCUPIED_PIXEL


def test_shift_moves_occupied_in_map_coordinates():
    result = dm.shift(_map(), 1.0, 0.0, 0.0, (3.0, 1.0, 6.0, 4.0), -2.0, 3.0)
    assert result[7, 4] == dm.FREE_PIXEL
    # x は 2 m 左へ (列 2)、y は 3 m 上へ (行 4)
    assert result[4, 2] == dm.OCCUPIED_PIXEL
    assert np.sum(result == dm.OCCUPIED_PIXEL) == 1


def test_thicken_grows_by_margin():
    result = dm.thicken(_map(), 1.0, 1.0)
    assert result[7, 4] == dm.OCCUPIED_PIXEL
    assert result[7, 5] == dm.OCCUPIED_PIXEL and result[6, 4] == dm.OCCUPIED_PIXEL
    assert result[6, 5] == dm.FREE_PIXEL
