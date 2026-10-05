#!/usr/bin/env python3
"""
generate_static_transforms.py

gnss_to_map_base_data.yaml (ピクセル座標と緯度経度の対応点リスト) を読み込み、
map.yaml の解像度・原点情報および WGS84->UTM 変換を用いて、
UTM 座標から地図座標 (map frame) への剛体変換 (x, y, yaw) を推定し YAML に出力するツール。
"""

import argparse
import os
import sys

import numpy as np
import yaml

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.geo import lat_lon_to_utm  # noqa: E402
from tools.common.map import (  # noqa: E402
    estimate_rigid_transform,
    load_map_info,
    pixel_to_map_coordinates,
)


def main():
    parser = argparse.ArgumentParser(
        description="対応点から UTM -> map 剛体変換 (x, y, yaw) を推定するツール"
    )
    parser.add_argument(
        "--base-yaml-name",
        default="gnss_to_map_base_data.yaml",
        help="ベースデータ YAML ファイル名",
    )
    parser.add_argument(
        "--map-dir",
        default="/root/ros2_data/map",
        help="map.yaml およびベースデータを探索するディレクトリ",
    )
    parser.add_argument(
        "--out-yaml-name",
        default="gnss_to_map_static_transforms.yaml",
        help="出力先 YAML ファイル名",
    )
    parser.add_argument(
        "--utm-zone",
        type=int,
        default=54,
        help="WGS84->UTM 変換ゾーン (デフォルト: 54)",
    )
    args = parser.parse_args()

    base_yaml_path = os.path.join(args.map_dir, args.base_yaml_name)
    if not os.path.exists(base_yaml_path):
        print(f"ベースデータ YAML が見つかりません: {base_yaml_path}", file=sys.stderr)
        return

    with open(base_yaml_path, "r", encoding="utf-8") as f:
        base = yaml.safe_load(f)

    if not base:
        print("ベースデータ YAML が空です。")
        return

    base_data = base.get("base_data", [])
    out = {"transforms": []}

    for rec in base_data:
        map_name = rec.get("map_name")
        label = rec.get("label")
        points = rec.get("points", [])
        if not map_name or not label or len(points) < 2:
            print(f"Skipping label {label} for map {map_name}: need >=2 points")
            continue

        map_yaml_path = os.path.join(args.map_dir, map_name)
        if not os.path.isfile(map_yaml_path):
            print(f"map yaml not found: {map_yaml_path}")
            continue

        try:
            img_arr, origin, resolution, _ = load_map_info(map_yaml_path)
        except Exception as e:
            print(f"Failed to load map info for {map_yaml_path}: {e}")
            continue

        origin_x, origin_y = float(origin[0]), float(origin[1])
        image_height = img_arr.shape[0]

        utm_xy = []
        map_xy = []
        for pnt in points:
            pixel = pnt.get("pixel")
            latlon = pnt.get("latlon")
            if pixel is None or latlon is None:
                continue
            u, v = float(pixel[0]), float(pixel[1])
            lat, lon = float(latlon[0]), float(latlon[1])

            x, y = lat_lon_to_utm(lat, lon, zone=args.utm_zone)
            mx, my = pixel_to_map_coordinates(
                u, v, resolution, origin_x, origin_y, image_height
            )
            utm_xy.append([x, y])
            map_xy.append([mx, my])

        utm_arr = np.array(utm_xy)
        map_arr = np.array(map_xy)
        est = estimate_rigid_transform(utm_arr, map_arr)
        if est is None:
            print(f"Failed to estimate for label {label}")
            continue

        out["transforms"].append({
            "label": label,
            "map_name": map_name,
            "transform": [est[0], est[1], est[2]],
        })
        print(f"Estimated {label}: x={est[0]:.3f}, y={est[1]:.3f}, yaw={est[2]:.3f}")

    out_path = os.path.join(args.map_dir, args.out_yaml_name)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
