#!/usr/bin/env python3
"""
add_keepout_to_map.py

既存の地図 (map.yaml と画像) に、占有(進入禁止)の長方形を足した地図を作る。
計画用地図にだけ進入禁止の領域がある状況 (実際の世界には何もない) を、シナリオテストで再現するために使う。

使い方:
  add_keepout_to_map.py <map.yaml> <出力ディレクトリ> --box X0,Y0,X1,Y1 [--box ...] [--name 名前]

- 長方形は地図座標 [m] (x0 < x1, y0 < y1)。複数指定できる。負の値で始まるときは --box=-1,-1,1,1 と書く
- 出力: <出力ディレクトリ>/<名前>.yaml と <名前>.pgm (名前の既定は planning)。yaml の image 以外は元のまま
- 元の地図の画像は、yaml の image が相対パスなら yaml と同じ場所から読む

例 (ウェアハウスの計画用地図の原点付近に 2m 四方):
  tools/scripts/add_keepout_to_map.py mg_simulation/maps/warehouse/planning.yaml \
      mg_scenario_test/scenarios/data/keepout --box=-1,-1,1,1
"""
import argparse
import os
from typing import List, Tuple

import numpy as np
import yaml
from PIL import Image

# trinary の地図で占有 (進入禁止) を表す画素値
OCCUPIED_PIXEL = 0


def parse_box(text: str) -> Tuple[float, float, float, float]:
    values = [float(v) for v in text.split(',')]
    if len(values) != 4 or not (values[0] < values[2] and values[1] < values[3]):
        raise argparse.ArgumentTypeError(f'--box は X0,Y0,X1,Y1 (X0<X1, Y0<Y1): {text}')
    return values[0], values[1], values[2], values[3]


def add_boxes(pixels: np.ndarray, resolution: float, origin_x: float, origin_y: float,
              boxes: List[Tuple[float, float, float, float]]) -> np.ndarray:
    """画像 (行 0 が上 = y 最大) の、地図座標の長方形の中を占有にする。"""
    result = pixels.copy()
    height = result.shape[0]
    for x0, y0, x1, y1 in boxes:
        col0 = max(int(np.floor((x0 - origin_x) / resolution)), 0)
        col1 = min(int(np.ceil((x1 - origin_x) / resolution)), result.shape[1])
        row_top = max(height - int(np.ceil((y1 - origin_y) / resolution)), 0)
        row_bottom = min(height - int(np.floor((y0 - origin_y) / resolution)), height)
        result[row_top:row_bottom, col0:col1] = OCCUPIED_PIXEL
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description='地図に進入禁止の長方形を足す')
    parser.add_argument('map_yaml')
    parser.add_argument('out_dir')
    parser.add_argument('--box', action='append', type=parse_box, required=True)
    parser.add_argument('--name', default='planning')
    args = parser.parse_args()

    with open(args.map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), image_path)
    pixels = np.array(Image.open(image_path))
    origin = meta['origin']
    result = add_boxes(pixels, float(meta['resolution']), float(origin[0]), float(origin[1]), args.box)

    os.makedirs(args.out_dir, exist_ok=True)
    Image.fromarray(result).save(os.path.join(args.out_dir, f'{args.name}.pgm'))
    meta['image'] = f'{args.name}.pgm'
    with open(os.path.join(args.out_dir, f'{args.name}.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f'保存: {args.out_dir}/{args.name}.yaml ({len(args.box)} 個の長方形)')


if __name__ == '__main__':
    main()
