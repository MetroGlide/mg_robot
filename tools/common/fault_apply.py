"""故障注入の計算 (numpy のみに依存)。fault_injector.py から使う。"""

import math
from typing import Optional, Sequence, Tuple

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


def _beam_angles_deg(size: int, angle_min: float, angle_increment: float) -> np.ndarray:
    """ビームの角度 (度、センサ座標系) を [-180, 180) に正規化して返す。"""
    angles = np.degrees(angle_min + angle_increment * np.arange(size))
    return (angles + 180.0) % 360.0 - 180.0


def mask_scan_sector(
    ranges: np.ndarray, angle_min: float, angle_increment: float, sector_deg: Tuple[float, float],
) -> np.ndarray:
    """LaserScan の距離のうち、角度が sector_deg (度、センサ座標系) の範囲のものを無限遠 (欠測) にする。"""
    angles = _beam_angles_deg(ranges.size, angle_min, angle_increment)
    low, high = sector_deg
    inside = (angles >= low) & (angles <= high)
    masked = ranges.copy()
    masked[inside] = np.inf
    return masked


def keep_scan_sectors(
    ranges: np.ndarray, angle_min: float, angle_increment: float, sectors_deg: Sequence[Sequence[float]],
) -> np.ndarray:
    """角度が sectors_deg ([low, high] の並び、度、センサ座標系) のどれかに入るビームだけを残し、他を無限遠にする。"""
    angles = _beam_angles_deg(ranges.size, angle_min, angle_increment)
    keep = np.zeros(ranges.size, dtype=bool)
    for low, high in sectors_deg:
        keep |= (angles >= low) & (angles <= high)
    masked = ranges.copy()
    masked[~keep] = np.inf
    return masked


class ScanOccluder:
    """人混み。ロボットの周りに幅 width_m の遮蔽物 (人) を count 個置き、その方向のビームを遮蔽物までの距離にする。

    遮蔽物の方向は sectors_deg ([low, high] の並び、度、センサ座標系) から、距離は range_m ([近, 遠]) から一様に選び、
    hold_sec ごとに選び直す (人がロボットに付いて歩き、ときどき入れ替わる状況)。遮蔽物より手前の当たりは残す。
    """

    def __init__(self, rng: np.random.Generator) -> None:
        self._rng = rng
        self._t: Optional[float] = None
        # 遮蔽物ごとの (方向 [度], 距離 [m])
        self._occluders = np.zeros((0, 2))

    def apply(
        self, ranges: np.ndarray, angle_min: float, angle_increment: float, t: float, count: int,
        width_m: float, range_m: Sequence[float], sectors_deg: Sequence[Sequence[float]], hold_sec: float,
    ) -> np.ndarray:
        if self._t is None or t < self._t or t - self._t >= hold_sec or len(self._occluders) != count:
            self._occluders = self._draw(count, range_m, sectors_deg)
            self._t = t
        angles = np.radians(_beam_angles_deg(ranges.size, angle_min, angle_increment))
        result = ranges.copy()
        for center_deg, distance in self._occluders:
            half_width = math.atan2(0.5 * width_m, distance)
            diff = (angles - math.radians(center_deg) + math.pi) % (2.0 * math.pi) - math.pi
            hit = (np.abs(diff) <= half_width) & ~(result <= distance)
            result[hit] = distance
        return result

    def _draw(self, count: int, range_m: Sequence[float], sectors_deg: Sequence[Sequence[float]]) -> np.ndarray:
        sectors = np.asarray(sectors_deg, dtype=float).reshape(-1, 2)
        widths = sectors[:, 1] - sectors[:, 0]
        chosen = self._rng.choice(len(sectors), size=count, p=widths / widths.sum())
        directions = sectors[chosen, 0] + self._rng.uniform(size=count) * widths[chosen]
        distances = self._rng.uniform(float(range_m[0]), float(range_m[1]), size=count)
        return np.column_stack([directions, distances])


def shift_latlon(lat_deg: float, lon_deg: float, dx_m: float, dy_m: float) -> Tuple[float, float]:
    """緯度経度を、東へ dx_m、北へ dy_m だけずらす (地図座標は UTM 由来で東・北にほぼ揃っているとみなす)。"""
    dlat = dy_m / METERS_PER_DEGREE_LAT
    dlon = dx_m / (METERS_PER_DEGREE_LAT * math.cos(math.radians(lat_deg)))
    return lat_deg + dlat, lon_deg + dlon


def local_east_north(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> Tuple[float, float]:
    """緯度経度を、基準の緯度経度からの東・北の距離 [m] に直す (数百 m の範囲の近似)。"""
    east = (lon_deg - lon0_deg) * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0_deg))
    north = (lat_deg - lat0_deg) * METERS_PER_DEGREE_LAT
    return east, north


def bias_field(points: list, x: float, y: float, length_m: float) -> Tuple[float, float]:
    """points ([x, y, bx, by] の並び) のずれを、(x, y) からの距離のガウスの重みで補間する。"""
    data = np.asarray(points, dtype=float).reshape(-1, 4)
    d2 = (data[:, 0] - x) ** 2 + (data[:, 1] - y) ** 2
    weights = np.exp(-0.5 * d2 / (length_m * length_m))
    total = float(weights.sum())
    if total < 1e-12:
        # どの点からも遠いときは、最も近い点のずれを使う
        nearest = int(np.argmin(d2))
        return float(data[nearest, 2]), float(data[nearest, 3])
    return float(weights @ data[:, 2] / total), float(weights @ data[:, 3] / total)


class GnssNoise:
    """GNSS の位置の雑音 (東, 北) [m]。tau [s] が正なら 1 次のガウス・マルコフ過程 (偏りが tau 程度続く単独測位の
    誤差)、0 なら毎回独立な白色雑音。どちらも各軸の標準偏差は sigma。

    同じ測位から出る 2 つのメッセージ (/navpvt と /gps/fix) に同じ値を返すため、前回から same_sec [s] 以内の
    呼び出しには前回の値を返す。
    """

    def __init__(self, rng: np.random.Generator, same_sec: float = 0.2) -> None:
        self._rng = rng
        self._same_sec = same_sec
        self._t: Optional[float] = None
        self._value = np.zeros(2)

    def sample(self, t: float, sigma: float, tau: float) -> np.ndarray:
        if self._t is not None and abs(t - self._t) <= self._same_sec:
            return self._value
        if tau > 0.0 and self._t is not None:
            rho = math.exp(-max(t - self._t, 0.0) / tau)
            self._value = rho * self._value + math.sqrt(1.0 - rho * rho) * self._rng.normal(0.0, sigma, 2)
        else:
            self._value = self._rng.normal(0.0, sigma, 2)
        self._t = t
        return self._value


def set_carrier_solution(flags: int, carrier: str) -> int:
    """u-blox NavPVT の flags の搬送波位相の解 (bit6-7) を、fixed / float / none に書き換える。"""
    code = {"none": 0, "float": 1, "fixed": 2}[carrier]
    return (flags & ~(0x03 << 6)) | (code << 6)


def horizontal_covariance(covariance: list, std_m: float) -> list:
    """NavSatFix の position_covariance (3x3、行優先) の水平の分散を std_m^2 にし、水平の相関を消す。"""
    out = list(covariance)
    variance = std_m * std_m
    out[0] = variance
    out[4] = variance
    out[1] = out[3] = 0.0
    return out
