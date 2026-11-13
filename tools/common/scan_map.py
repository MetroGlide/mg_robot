"""2D LiDAR のスキャンと占有格子の地図を、姿勢を介して突き合わせる共通処理。

- OccupancyMap: 地図の占有と、占有までの距離。光線追跡 (raycast) もここで行う
- interpolate_pose: 姿勢列 (記録の /tf からは tools.common.bag.read_map_base_poses) を時刻で補間する
- beam_geometry: スキャンのビームの、地図座標での始点と向き
"""
import argparse
import os
from typing import List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

from tools.common import loc_metrics as lm
from tools.common.map import load_map_info

# 補間に使う姿勢の間隔の上限 [s]。これより離れていたら姿勢なしとする
MAX_POSE_GAP_SEC = 0.5


class OccupancyMap:
    def __init__(self, map_yaml: str, close_radius: float = 0.0, occupied_thresh: float = 0.65) -> None:
        pixels, origin, resolution, _ = load_map_info(map_yaml)
        if pixels.ndim == 3:
            pixels = pixels[..., 0]
        self._pixels = pixels
        self._occupied_thresh = occupied_thresh
        # 画像の行 0 が上 (y 最大) なので、行 0 が y 最小になるよう反転する
        self.occupied = ((255.0 - pixels.astype(float)) / 255.0 > occupied_thresh)[::-1]
        self.resolution = resolution
        self.origin = np.array(origin[:2])
        self._close_cells = int(round(close_radius / resolution))
        self._update()

    def _update(self) -> None:
        """占有から、距離と点の間を埋めた占有を作り直す。"""
        # 占有までの距離 [m]
        self.distance = ndimage.distance_transform_edt(~self.occupied) * self.resolution
        cells = self._close_cells
        y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
        self.closed = ndimage.binary_closing(self.occupied, structure=x * x + y * y <= cells * cells) \
            if cells > 0 else self.occupied
        # 点の間を埋めた占有までの距離 [m] (光線追跡で、まばらな占有の帯の隙間を光線が抜けないようにする)
        self.closed_distance = ndimage.distance_transform_edt(~self.closed) * self.resolution \
            if cells > 0 else self.distance

    def clear_near(self, xy: np.ndarray, radius: float) -> None:
        """点列 xy から radius [m] 以内の占有を消す (地図を作った走行で一緒に映った人などを、光線追跡の前に除く)。"""
        mask = np.zeros_like(self.occupied)
        row, col, inside = self.cells(xy)
        mask[row[inside], col[inside]] = True
        near = ndimage.distance_transform_edt(~mask) * self.resolution <= radius
        self.occupied = self.occupied & ~near
        self._update()

    def save(self, out_yaml: str, bounds: Optional[List[float]] = None) -> None:
        """clear_near で消した占有を空き (254) にした地図を、元の地図と同じ形式 (pgm と map.yaml) で保存する。

        未知などそれ以外の画素は元のまま。光線追跡の世界と同じ地図で測位する対照に使う。
        bounds [x_min, y_min, x_max, y_max] [m] を与えると、その範囲 (地図の内側) だけを切り出す
        (大きな地図は AMCL が尤度場を作る間、最初のスキャンの処理が止まるため)。
        """
        pixels = self._pixels.copy()
        source = (255.0 - pixels.astype(float)) / 255.0 > self._occupied_thresh
        pixels[source & ~self.occupied[::-1]] = 254
        rows, cols = self.occupied.shape
        row0, row1, col0, col1 = 0, rows, 0, cols
        if bounds is not None:
            col0 = max(int(np.floor((bounds[0] - self.origin[0]) / self.resolution)), 0)
            row0 = max(int(np.floor((bounds[1] - self.origin[1]) / self.resolution)), 0)
            col1 = min(int(np.ceil((bounds[2] - self.origin[0]) / self.resolution)), cols)
            row1 = min(int(np.ceil((bounds[3] - self.origin[1]) / self.resolution)), rows)
        # 画像の行は上が y 最大
        pixels = pixels[rows - row1:rows - row0, col0:col1]
        origin = self.origin + self.resolution * np.array([col0, row0])
        image = os.path.splitext(os.path.basename(out_yaml))[0] + ".pgm"
        Image.fromarray(pixels.astype(np.uint8)).save(os.path.join(os.path.dirname(out_yaml), image))
        with open(out_yaml, "w", encoding="utf-8") as f:
            yaml.safe_dump({"image": image, "mode": "trinary", "resolution": float(self.resolution),
                            "origin": [float(origin[0]), float(origin[1]), 0.0], "negate": 0,
                            "occupied_thresh": self._occupied_thresh, "free_thresh": 0.25}, f)

    def cells(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (row >= 0) & (row < self.occupied.shape[0]) & (col >= 0) & (col < self.occupied.shape[1])
        return row, col, inside

    def distance_at(self, xy: np.ndarray, closed: bool = False) -> np.ndarray:
        """占有 (closed なら点の間を埋めた占有) までの距離 [m]。地図の外は inf。"""
        row, col, inside = self.cells(xy)
        result = np.full(xy.shape[0], np.inf)
        result[inside] = (self.closed_distance if closed else self.distance)[row[inside], col[inside]]
        return result

    def closed_at(self, xy: np.ndarray) -> np.ndarray:
        row, col, inside = self.cells(xy)
        result = np.zeros(xy.shape[0], dtype=bool)
        result[inside] = self.closed[row[inside], col[inside]]
        return result

    def run_length(self, xy: np.ndarray, direction: np.ndarray, max_len: float) -> np.ndarray:
        """各点から、方向 direction の前後に (点の間を埋めた) 占有が続く長さ [m]。占有の上にない点は nan。"""
        step = self.resolution / 2.0
        offsets = np.arange(step, max_len + 1e-9, step)
        result = np.full(xy.shape[0], np.nan)
        on = self.closed_at(xy)
        for i in np.nonzero(on)[0]:
            length = 0.0
            for sign in (1.0, -1.0):
                points = xy[i] + sign * offsets[:, None] * direction[i]
                occupied = self.closed_at(points)
                broken = np.nonzero(~occupied)[0]
                length += offsets[broken[0]] - step if broken.size else max_len
            result[i] = length + step
        return result

    def raycast(self, origin: np.ndarray, yaws: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
        """origin (x, y) から各向き yaws へ光線を飛ばし、占有に当たるまでの距離 [m] を返す。当たらなければ inf。

        占有までの距離だけ進む (sphere tracing)。占有から半格子以内に入ったら当たりとする。
        close_radius を与えて作った地図では、点の間を埋めた占有に当てる。
        """
        hit_dist = self.resolution * 0.5
        direction = np.stack([np.cos(yaws), np.sin(yaws)], axis=1)
        travelled = np.full(yaws.size, float(min_range))
        result = np.full(yaws.size, np.inf)
        active = np.ones(yaws.size, dtype=bool)
        while np.any(active):
            idx = np.nonzero(active)[0]
            points = origin[None, :] + travelled[idx, None] * direction[idx]
            dist = self.distance_at(points, closed=True)
            hit = dist <= hit_dist
            result[idx[hit]] = travelled[idx[hit]]
            # 地図の外 (inf) と最大距離を超えたものは当たりなし
            gone = ~np.isfinite(dist)
            active[idx[hit | gone]] = False
            step = np.maximum(dist, hit_dist)
            travelled[idx] += np.where(hit | gone, 0.0, step)
            active[idx[travelled[idx] > max_range]] = False
        return result


def interpolate_pose(times: np.ndarray, poses: np.ndarray, t: float,
                     max_gap: float = MAX_POSE_GAP_SEC) -> Optional[np.ndarray]:
    """時刻 t の姿勢を前後の姿勢から線形に補間する。前後が無いか、間が max_gap [s] より空けば None。"""
    i = int(np.searchsorted(times, t))
    if i <= 0 or i >= times.size or times[i] - times[i - 1] > max_gap:
        return None
    a = (t - times[i - 1]) / (times[i] - times[i - 1])
    yaw = poses[i - 1, 2] + a * lm.wrap_angle(poses[i, 2] - poses[i - 1, 2])
    xy = poses[i - 1, :2] + a * (poses[i, :2] - poses[i - 1, :2])
    return np.array([xy[0], xy[1], yaw])


def valid_ranges(ranges, range_min: float, range_max: float) -> np.ndarray:
    """LaserScan の距離を配列にし、範囲外と nan を inf にする。"""
    result = np.asarray(ranges, dtype=float).copy()
    result[~np.isfinite(result) | (result < range_min) | (result > range_max)] = np.inf
    return result


def keep_matched(ranges: np.ndarray, sensor_xy: np.ndarray, beam_yaw: np.ndarray, grid: OccupancyMap,
                 match_dist: float) -> np.ndarray:
    """端点が地図の占有から match_dist より遠いビームを inf にした距離を返す (地図にない物の点を除く)。"""
    result = ranges.copy()
    valid = np.nonzero(np.isfinite(ranges))[0]
    points = sensor_xy + ranges[valid, None] * np.stack([np.cos(beam_yaw[valid]), np.sin(beam_yaw[valid])], axis=1)
    result[valid[grid.distance_at(points) > match_dist]] = np.inf
    return result


def raycast_ranges(ranges: np.ndarray, sensor_xy: np.ndarray, beam_yaw: np.ndarray, grid: OccupancyMap,
                   min_range: float, max_range: float, noise_std: float, rng: np.random.Generator) -> np.ndarray:
    """有効なビームの距離を、地図への光線追跡の距離 (+ 雑音) に置き換える。当たらないビームと元の欠測は inf。"""
    result = np.full(ranges.size, np.inf)
    valid = np.nonzero(np.isfinite(ranges))[0]
    hit = grid.raycast(sensor_xy, beam_yaw[valid], min_range, max_range)
    if noise_std > 0.0:
        hit = hit + rng.normal(0.0, noise_std, hit.size)
    hit[(hit < min_range) | (hit > max_range)] = np.inf
    result[valid] = hit
    return result


def parse_pose(text: str) -> List[float]:
    """コマンドラインの "X,Y,YAW" を [x, y, yaw] にする (argparse の type に使う)。"""
    values = [float(v) for v in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError(f"X,Y,YAW で指定してください: {text}")
    return values


def beam_geometry(pose: np.ndarray, lidar_pose: np.ndarray, angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """base の姿勢 pose と、base 座標系での LiDAR の姿勢から、LiDAR の位置 (x, y) と各ビームの向き [rad] を返す。"""
    sensor = lm.compose(pose[None, :], lidar_pose[None, :])[0]
    return sensor[:2], sensor[2] + angles
