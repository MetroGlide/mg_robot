"""地図・座標系変換に関する共通ユーティリティモジュール。"""

import math
import os
from typing import List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image


def load_map_info(map_yaml_path: str) -> Tuple[np.ndarray, List[float], float, str]:
    """map.yaml を読み込み、画像配列 (NumPy)、origin [x, y, yaw]、resolution [m/px]、画像パスを返却する。"""
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    resolution_val = data.get("resolution")
    if resolution_val is None:
        raise ValueError(f"map YAML に必須キー 'resolution' がありません: {map_yaml_path}")
    resolution = float(resolution_val)

    origin = [float(v) for v in data.get("origin", [0.0, 0.0, 0.0])]

    image_rel_path = data.get("image")
    if not image_rel_path:
        raise ValueError(f"map YAML に 'image' キーがありません: {map_yaml_path}")

    if not os.path.isabs(image_rel_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_rel_path)
    else:
        image_path = image_rel_path

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"地図画像ファイルが存在しません: {image_path}")

    img = Image.open(image_path)
    img_arr = np.array(img)
    return img_arr, origin, resolution, image_path


def estimate_rigid_transform(
    utm_xy: np.ndarray,
    map_xy: np.ndarray,
) -> Optional[Tuple[float, float, float]]:
    """Procrustes解析 (SVD) により UTM座標群から地図座標群への剛体変換 (x, y, yaw) を推定する。

    map_xy = R(yaw) * utm_xy + [x, y]
    """
    if utm_xy.shape[0] < 2:
        return None

    mu_utm = np.mean(utm_xy, axis=0)
    mu_map = np.mean(map_xy, axis=0)
    x_centered = utm_xy - mu_utm
    y_centered = map_xy - mu_map

    u, _, vt = np.linalg.svd(np.dot(y_centered.T, x_centered))
    r = np.dot(u, vt)
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = np.dot(u, vt)

    theta = math.atan2(r[1, 0], r[0, 0])
    t = mu_map - np.dot(r, mu_utm)
    return float(t[0]), float(t[1]), float(theta)


def load_transform_from_yaml(
    yaml_path: str,
    label: Optional[str] = None,
) -> Optional[Tuple[float, float, float]]:
    """static_transforms YAML ファイルから指定 label の transform (x, y, yaw) を取得する。"""
    if not os.path.exists(yaml_path):
        return None

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    transforms = data.get("transforms", [])
    if not transforms:
        return None

    if label:
        for t in transforms:
            if t.get("label") == label:
                tf = t.get("transform")
                return float(tf[0]), float(tf[1]), float(tf[2])

    tf = transforms[0].get("transform")
    return float(tf[0]), float(tf[1]), float(tf[2])


def pixel_to_map_coordinates(
    u: float,
    v: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    image_height: int,
) -> Tuple[float, float]:
    """画像ピクセル座標 (u, v) [左上原点] を地図実世界座標 (x, y) [m, 左下原点] に変換する。"""
    map_x = origin_x + (u * resolution)
    map_y = origin_y + ((image_height - v) * resolution)
    return float(map_x), float(map_y)
