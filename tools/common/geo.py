"""幾何・地理座標変換に関する共通ユーティリティモジュール。"""

import math
from typing import Dict, Tuple

import pyproj

# ゾーンごとの pyproj.Proj インスタンスキャッシュ
_UTM_PROJ_CACHE: Dict[int, pyproj.Proj] = {}


def get_utm_proj(zone: int = 54) -> pyproj.Proj:
    """指定ゾーンの WGS84 -> UTM 投影プロジェクションを取得する (キャッシュ付き)。"""
    if zone not in _UTM_PROJ_CACHE:
        _UTM_PROJ_CACHE[zone] = pyproj.Proj(proj="utm", zone=zone, ellps="WGS84")
    return _UTM_PROJ_CACHE[zone]


def lat_lon_to_utm(lat: float, lon: float, zone: int = 54) -> Tuple[float, float]:
    """WGS84 緯度経度から UTM 座標 (Easting, Northing) [m] へ変換する。"""
    proj = get_utm_proj(zone)
    easting, northing = proj(lon, lat)
    return float(easting), float(northing)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """クォータニオン (qx, qy, qz, qw) から 2D 平面上の Yaw 角 [rad] を算出する。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2地点の緯度経度から大圏距離 (Haversine距離) [m] を算出する。"""
    r_earth = 6371000.0  # 地球半径 [m]
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r_earth * c
