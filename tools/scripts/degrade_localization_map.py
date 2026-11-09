#!/usr/bin/env python3
"""
degrade_localization_map.py

測位用の地図 (AMCL が使う地図) を加工して、AMCL が弱い状況を作る。計画用の地図は変えない。

使い方:
  degrade_localization_map.py <map.yaml> <出力ディレクトリ> [--thicken M] [--erase X0,Y0,X1,Y1 ...]
      [--shift X0,Y0,X1,Y1,DX,DY ...] [--name 名前]

- --thicken: 占有の線を M [m] だけ太らせる (尤度の山が平らになり、収束しにくくなる)
- --erase: 長方形の中の占有を消す (地図にない物がある状況)
- --shift: 長方形の中の占有を (DX, DY) [m] 動かす。測位用だけをずらすので、AMCL がこれに合うと、
  推定の footprint が計画用地図の障害物にかかる (実際には接触していない)
- 座標は地図座標 [m]。負の値で始まるときは --erase=-1,-1,1,1 と書く。適用の順は erase、shift、thicken
- 出力: <出力ディレクトリ>/<名前>.yaml と <名前>.pgm (名前の既定は localization)。yaml の image 以外は元のまま
"""
import argparse
import os
from typing import List, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

OCCUPIED_PIXEL = 0
FREE_PIXEL = 254

Box = Tuple[float, float, float, float]


def parse_box(text: str) -> Box:
    values = [float(v) for v in text.split(',')]
    if len(values) != 4 or not (values[0] < values[2] and values[1] < values[3]):
        raise argparse.ArgumentTypeError(f'X0,Y0,X1,Y1 (X0<X1, Y0<Y1) で指定してください: {text}')
    return values[0], values[1], values[2], values[3]


def parse_shift(text: str) -> Tuple[Box, float, float]:
    values = text.split(',')
    if len(values) != 6:
        raise argparse.ArgumentTypeError(f'X0,Y0,X1,Y1,DX,DY で指定してください: {text}')
    return parse_box(','.join(values[:4])), float(values[4]), float(values[5])


def box_slices(shape: Tuple[int, int], resolution: float, origin_x: float, origin_y: float,
               box: Box) -> Tuple[slice, slice]:
    """地図座標の長方形に対応する画像 (行 0 が上 = y 最大) の範囲。"""
    x0, y0, x1, y1 = box
    height, width = shape
    col0 = max(int(np.floor((x0 - origin_x) / resolution)), 0)
    col1 = min(int(np.ceil((x1 - origin_x) / resolution)), width)
    row_top = max(height - int(np.ceil((y1 - origin_y) / resolution)), 0)
    row_bottom = min(height - int(np.floor((y0 - origin_y) / resolution)), height)
    return slice(row_top, row_bottom), slice(col0, col1)


def erase(pixels: np.ndarray, resolution: float, origin_x: float, origin_y: float,
          box: Box) -> np.ndarray:
    result = pixels.copy()
    rows, cols = box_slices(result.shape, resolution, origin_x, origin_y, box)
    region = result[rows, cols]
    region[region == OCCUPIED_PIXEL] = FREE_PIXEL
    return result


def shift(pixels: np.ndarray, resolution: float, origin_x: float, origin_y: float,
          box: Box, dx: float, dy: float) -> np.ndarray:
    result = pixels.copy()
    rows, cols = box_slices(result.shape, resolution, origin_x, origin_y, box)
    mask = np.zeros(result.shape, dtype=bool)
    mask[rows, cols] = result[rows, cols] == OCCUPIED_PIXEL
    result[mask] = FREE_PIXEL
    row_index, col_index = np.nonzero(mask)
    new_rows = row_index - int(round(dy / resolution))
    new_cols = col_index + int(round(dx / resolution))
    inside = (new_rows >= 0) & (new_rows < result.shape[0]) & (new_cols >= 0) & (new_cols < result.shape[1])
    result[new_rows[inside], new_cols[inside]] = OCCUPIED_PIXEL
    return result


def thicken(pixels: np.ndarray, resolution: float, margin: float) -> np.ndarray:
    cells = int(round(margin / resolution))
    y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
    grown = ndimage.binary_dilation(pixels == OCCUPIED_PIXEL, structure=x * x + y * y <= cells * cells)
    result = pixels.copy()
    result[grown] = OCCUPIED_PIXEL
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description='測位用の地図を加工して、AMCL が弱い状況を作る')
    parser.add_argument('map_yaml')
    parser.add_argument('out_dir')
    parser.add_argument('--thicken', type=float, default=0.0)
    parser.add_argument('--erase', action='append', type=parse_box, default=[])
    parser.add_argument('--shift', action='append', type=parse_shift, default=[])
    parser.add_argument('--name', default='localization')
    args = parser.parse_args()

    with open(args.map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), image_path)
    result: np.ndarray = np.array(Image.open(image_path))
    resolution = float(meta['resolution'])
    origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])

    boxes: List[Box] = args.erase
    for box in boxes:
        result = erase(result, resolution, origin_x, origin_y, box)
    for box, dx, dy in args.shift:
        result = shift(result, resolution, origin_x, origin_y, box, dx, dy)
    if args.thicken > 0.0:
        result = thicken(result, resolution, args.thicken)

    os.makedirs(args.out_dir, exist_ok=True)
    Image.fromarray(result).save(os.path.join(args.out_dir, f'{args.name}.pgm'))
    meta['image'] = f'{args.name}.pgm'
    with open(os.path.join(args.out_dir, f'{args.name}.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f'保存: {args.out_dir}/{args.name}.yaml')


if __name__ == '__main__':
    main()
