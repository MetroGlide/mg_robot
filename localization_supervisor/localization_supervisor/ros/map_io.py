"""nav2 の map_server と同じ形式の地図 (YAML と画像) を読む (オフライン評価用)。

map_server の trinary の変換と同じく、画素の濃さ p が occupied_thresh を超えれば占有 (100)、
free_thresh 未満なら空き (0)、それ以外は未知 (-1) にする。
"""
import os

import numpy as np
import yaml
from PIL import Image

from ..core.grid import OccupancyMap


def load_map_yaml(path: str, occupied_threshold: int = 65) -> OccupancyMap:
    with open(path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(path)), image_path)
    pixels = np.asarray(Image.open(image_path), dtype=float)
    if pixels.ndim == 3:
        pixels = pixels[:, :, :3].mean(axis=2)
    p = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
    data = np.full(p.shape, -1, dtype=np.int8)
    data[p > float(meta.get('occupied_thresh', 0.65))] = 100
    data[p < float(meta.get('free_thresh', 0.25))] = 0
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    # 画像は上の行が y の大きい側。格子は下の行 (原点の側) から並べる
    return OccupancyMap(np.flipud(data).copy(), float(meta['resolution']), float(origin[0]), float(origin[1]),
                        occupied_threshold)
