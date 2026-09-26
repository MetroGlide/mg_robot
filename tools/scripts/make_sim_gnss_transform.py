#!/usr/bin/env python3
"""
make_sim_gnss_transform.py

シミュレータのワールド SDF の spherical_coordinates (ワールド原点の緯度経度) から、GNSS ブリッジ
(slam_gnss_nav_bridge_node) が使う gnss_transform.yaml を作る。

シミュレータの地図はワールドの東・北に揃っていて、UTM のグリッドとは子午線収束角の分だけずれる。
そのずれを rotation_rad に入れる (ブリッジの use_file_rotation: true で map 座標を回転して補正する)。
実機の gnss_transform.yaml の rotation_rad (SLAM の初期方位の回転で、ブリッジは使わない) とは意味が違う。

使い方:
  make_sim_gnss_transform.py mg_simulation/worlds/warehouse.sdf -o mg_simulation/maps/warehouse/gnss_transform.yaml
"""

import argparse
import math
import xml.etree.ElementTree as ET
from typing import Any, Dict, Tuple

import pyproj
import yaml

# 収束角を求めるために、真北へ進む距離 [m]
PROBE_DISTANCE_M = 100.0


def read_spherical_coordinates(sdf_path: str) -> Tuple[float, float]:
    """ワールド SDF の spherical_coordinates から (緯度, 経度) [deg] を返す。"""
    root = ET.parse(sdf_path).getroot()
    node = root.find(".//spherical_coordinates")
    if node is None:
        raise SystemExit(f"エラー: {sdf_path} に spherical_coordinates がありません。")
    heading = float(node.findtext("heading_deg", default="0"))
    if abs(heading) > 1e-9:
        raise SystemExit("エラー: heading_deg が 0 でないワールドには対応していません。")
    return float(node.findtext("latitude_deg")), float(node.findtext("longitude_deg"))


def build_transform(latitude: float, longitude: float) -> Dict[str, Any]:
    """原点の緯度経度から gnss_transform.yaml の内容を作る。"""
    zone = int((longitude + 180.0) // 6.0) + 1
    south = latitude < 0.0
    proj = pyproj.Proj(proj="utm", zone=zone, ellps="WGS84", south=south)
    easting, northing = proj(longitude, latitude)

    # 原点から真北へ進んだ点を UTM に投影して、グリッドの北と真北のずれ (収束角) を求める
    north_lon, north_lat, _ = pyproj.Geod(ellps="WGS84").fwd(longitude, latitude, 0.0, PROBE_DISTANCE_M)
    north_e, north_n = proj(north_lon, north_lat)
    # UTM の座標のずれ (de, dn) を、東・北に揃った座標 (0, |d|) に回す角度
    rotation = math.atan2(north_e - easting, north_n - northing)
    return {
        "anchor": {"latitude": latitude, "longitude": longitude},
        "anchor_utm": {
            "easting": float(easting), "northing": float(northing),
            "zone": zone, "hemisphere": "south" if south else "north",
        },
        "rotation_rad": float(rotation),
        "metadata": {"source": "make_sim_gnss_transform.py (シミュレータのワールドの原点。東・北に揃った座標)"},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="シミュレータのワールドから gnss_transform.yaml を作る")
    parser.add_argument("sdf", help="ワールドの SDF")
    parser.add_argument("-o", "--output", required=True, help="出力する gnss_transform.yaml")
    args = parser.parse_args()

    latitude, longitude = read_spherical_coordinates(args.sdf)
    transform = build_transform(latitude, longitude)
    with open(args.output, "w", encoding="utf-8") as f:
        yaml.safe_dump(transform, f, sort_keys=False, allow_unicode=True)
    print(f"保存: {args.output} (zone {transform['anchor_utm']['zone']} {transform['anchor_utm']['hemisphere']}, "
          f"rotation {math.degrees(transform['rotation_rad']):.3f} deg)")


if __name__ == "__main__":
    main()
