import argparse

import numpy as np
import pytest

from tools.scripts import add_keepout_to_map as keepout


def test_box_is_marked_in_map_coordinates():
    # 10x10 セル、解像度 1 m、原点 (-5, -5)。行 0 が上 (y 最大)
    pixels = np.full((10, 10), 254, dtype=np.uint8)
    result = keepout.add_boxes(pixels, 1.0, -5.0, -5.0, [(-1.0, -1.0, 1.0, 1.0)])
    occupied = np.argwhere(result == keepout.OCCUPIED_PIXEL)
    # x の範囲は列 4..5、y の範囲は行 4..5 (y=0 付近は画像の中央)
    assert sorted(set(occupied[:, 1])) == [4, 5]
    assert sorted(set(occupied[:, 0])) == [4, 5]
    assert result.shape == pixels.shape and (pixels == 254).all()


def test_box_is_clipped_to_the_map():
    pixels = np.full((4, 4), 254, dtype=np.uint8)
    result = keepout.add_boxes(pixels, 1.0, 0.0, 0.0, [(-10.0, -10.0, 100.0, 100.0)])
    assert (result == keepout.OCCUPIED_PIXEL).all()


def test_parse_box():
    assert keepout.parse_box('-1,-2,3,4') == (-1.0, -2.0, 3.0, 4.0)
    with pytest.raises(argparse.ArgumentTypeError):
        keepout.parse_box('1,1,0,0')
    with pytest.raises(argparse.ArgumentTypeError):
        keepout.parse_box('1,2,3')
