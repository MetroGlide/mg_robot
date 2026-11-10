"""故障注入の計算 (numpy のみに依存)。fault_injector.py から使う。"""

import math
from typing import Tuple

import numpy as np

from tools.common.faults import Fault

# 地表の緯度 1 度あたりの距離 [m] (WGS84 の平均)
METERS_PER_DEGREE_LAT = 111_320.0


def is_active(fault: Fault, t_rel: float) -> bool:
    """再生開始からの経過秒 t_rel に、区間の故障 (kidnap 以外) が作用しているか。"""
    return (not fault.is_instant) and fault.start <= t_rel < fault.end


def is_gated_on(fault: Fault, abs_w: float) -> bool:
    """旋回の速さの条件 min_abs_w があるとき、それを満たしているか (条件がなければ常に真)。"""
    return abs_w >= float(fault.params.get("min_abs_w", 0.0))


def thin_scan(ranges: np.ndarray, keep_every: int) -> np.ndarray:
    """LaserScan の距離を keep_every 本に 1 本だけ残し、他を無限遠 (欠測) にする。"""
    masked = np.full(ranges.shape, np.inf)
    masked[::keep_every] = ranges[::keep_every]
    return masked


def limit_scan_range(ranges: np.ndarray, max_range: float) -> np.ndarray:
    """max_range より遠い距離を無限遠 (欠測) にする。"""
    masked = ranges.copy()
    masked[~(masked <= max_range)] = np.inf
    return masked


def jitter_scan(ranges: np.ndarray, ratio: float, depth_m: float, rng: np.random.Generator) -> np.ndarray:
    """ratio の割合のビームを選び、半分を無限遠 (欠測) に、残りを 0〜depth_m だけ遠くする。"""
    result = ranges.copy()
    chosen = rng.uniform(size=ranges.size) < ratio
    drop = chosen & (rng.uniform(size=ranges.size) < 0.5)
    stretch = chosen & ~drop
    result[stretch] += rng.uniform(0.0, depth_m, size=int(stretch.sum()))
    result[drop] = np.inf
    return result


def mask_scan_sector(
    ranges: np.ndarray, angle_min: float, angle_increment: float, sector_deg: Tuple[float, float],
) -> np.ndarray:
    """LaserScan の距離のうち、角度が sector_deg (度、センサ座標系) の範囲のものを無限遠 (欠測) にする。"""
    angles = np.degrees(angle_min + angle_increment * np.arange(ranges.size))
    # 角度を [-180, 180) に正規化して比べる
    angles = (angles + 180.0) % 360.0 - 180.0
    low, high = sector_deg
    inside = (angles >= low) & (angles <= high)
    masked = ranges.copy()
    masked[inside] = np.inf
    return masked


def shift_latlon(lat_deg: float, lon_deg: float, dx_m: float, dy_m: float) -> Tuple[float, float]:
    """緯度経度を、東へ dx_m、北へ dy_m だけずらす (地図座標は UTM 由来で東・北にほぼ揃っているとみなす)。"""
    dlat = dy_m / METERS_PER_DEGREE_LAT
    dlon = dx_m / (METERS_PER_DEGREE_LAT * math.cos(math.radians(lat_deg)))
    return lat_deg + dlat, lon_deg + dlon


def set_carrier_solution(flags: int, carrier: str) -> int:
    """u-blox NavPVT の flags の搬送波位相の解 (bit6-7) を、fixed / float / none に書き換える。"""
    code = {"none": 0, "float": 1, "fixed": 2}[carrier]
    return (flags & ~(0x03 << 6)) | (code << 6)
