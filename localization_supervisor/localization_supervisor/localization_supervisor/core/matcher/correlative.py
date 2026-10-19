"""総当たりの照合 (correlative scan matching)。numpy だけで書き、ROS に依存しない。

中心の姿勢の周りの位置と向きを格子で総当たりし、点群が地図の占有セルの近くに載る割合 (一致率) が最も高い姿勢を探す。
粗い格子で探し、上位の互いに離れた姿勢を、細かい格子で絞り込む。上位が複数あれば、似た場所の取り違え
(別の場所にも同じくらい合う) を、呼び出し側が判断できる。
"""
import math
from typing import List, Tuple

import numpy as np

from ..geometry import Pose2, wrap_angle
from ..grid import LocalDistanceField
from ..interfaces import MatchPeak, Matcher, ParamSpec, SearchWindow

# 一度に評価する姿勢の数 (メモリの上限)
CHUNK = 256
# 向きがこれ以上違えば、位置が近くても別の候補とする [rad]
DISTINCT_YAW_RAD = 0.5


class CorrelativeMatcher(Matcher):
    PARAMS = {
        'tolerance_m': ParamSpec(float, 0.3, '占有セルからこの距離以内の点を一致とする'),
        'min_points': ParamSpec(int, 30, '距離場の中にある点がこれ未満なら判断しない'),
        'separation_m': ParamSpec(float, 1.0, '上位の候補どうしが離れているべき距離'),
        'refine_ratio': ParamSpec(int, 4, '絞り込みで格子の刻みを何分の 1 にするか'),
    }

    def match(
        self, field: LocalDistanceField, points_base: np.ndarray, center: Pose2, window: SearchWindow,
        max_peaks: int = 1,
    ) -> List[MatchPeak]:
        p = self.params
        if points_base.shape[0] < p['min_points']:
            return []
        coarse = self._grid(center, window.range_xy_m, window.step_xy_m, window.range_yaw_rad, window.step_yaw_rad)
        scored = self._evaluate(field, points_base, coarse)
        peaks = self._separate(scored, max_peaks)
        refined: List[MatchPeak] = []
        for peak in peaks:
            fine_step = window.step_xy_m / p['refine_ratio']
            fine_yaw = window.step_yaw_rad / p['refine_ratio']
            yaw_range = window.step_yaw_rad / 2.0 if window.range_yaw_rad > 0 else 0.0
            grid = self._grid(peak.pose, window.step_xy_m / 2.0, fine_step, yaw_range, fine_yaw)
            best = self._evaluate(field, points_base, grid)
            refined.append(best[0] if best else peak)
        refined.sort(key=lambda q: (-q.ratio, q.mean_distance))
        return refined

    @staticmethod
    def _grid(center: Pose2, range_xy: float, step_xy: float, range_yaw: float, step_yaw: float) -> np.ndarray:
        """探す姿勢 (K, 3)。"""
        xy = np.arange(-range_xy, range_xy + 1e-9, step_xy) if step_xy > 0 else np.zeros(1)
        yaw = np.arange(-range_yaw, range_yaw + 1e-9, step_yaw) if range_yaw > 0 and step_yaw > 0 else np.zeros(1)
        gx, gy, gt = np.meshgrid(xy, xy, yaw, indexing='ij')
        poses = np.stack([gx.ravel() + center[0], gy.ravel() + center[1], gt.ravel() + center[2]], axis=1)
        return poses

    def _evaluate(self, field: LocalDistanceField, points_base: np.ndarray, poses: np.ndarray) -> List[MatchPeak]:
        """姿勢ごとの一致率。一致率の高い順 (同じなら距離の平均の小さい順)。判断できない姿勢は除く。"""
        p = self.params
        tolerance, min_points = p['tolerance_m'], p['min_points']
        results: List[MatchPeak] = []
        for start in range(0, poses.shape[0], CHUNK):
            chunk = poses[start:start + CHUNK]
            c, s = np.cos(chunk[:, 2])[:, None], np.sin(chunk[:, 2])[:, None]
            px, py = points_base[None, :, 0], points_base[None, :, 1]
            xs = chunk[:, 0:1] + c * px - s * py
            ys = chunk[:, 1:2] + s * px + c * py
            distances = field.lookup(np.stack([xs.ravel(), ys.ravel()], axis=1)).reshape(xs.shape)
            valid = np.isfinite(distances)
            count = valid.sum(axis=1)
            near = ((distances <= tolerance) & valid).sum(axis=1)
            total = np.where(valid, distances, 0.0).sum(axis=1)
            for i in np.nonzero(count >= min_points)[0]:
                results.append(MatchPeak(
                    (float(chunk[i, 0]), float(chunk[i, 1]), wrap_angle(float(chunk[i, 2]))),
                    float(near[i] / count[i]), float(total[i] / count[i])))
        results.sort(key=lambda q: (-q.ratio, q.mean_distance))
        return results

    def _separate(self, scored: List[MatchPeak], max_peaks: int) -> List[MatchPeak]:
        """一致の良い順に、互いに離れたものだけを max_peaks 個選ぶ。"""
        chosen: List[MatchPeak] = []
        for peak in scored:
            if all(_distinct(peak.pose, other.pose, self.params['separation_m']) for other in chosen):
                chosen.append(peak)
                if len(chosen) >= max_peaks:
                    break
        return chosen


def _distinct(a: Pose2, b: Pose2, separation: float) -> bool:
    return (math.hypot(a[0] - b[0], a[1] - b[1]) >= separation
            or abs(wrap_angle(a[2] - b[2])) >= DISTINCT_YAW_RAD)


def window_size(cov: np.ndarray, sigmas: float, minimum: float, maximum: float) -> Tuple[float, float]:
    """共分散から、探す範囲 (位置 [m]、向き [rad]) を決める。sigmas 倍の標準偏差を minimum と maximum で挟む。"""
    std_xy = math.sqrt(max(float(cov[0, 0]), float(cov[1, 1]), 0.0))
    std_yaw = math.sqrt(max(float(cov[2, 2]), 0.0))
    return min(max(sigmas * std_xy, minimum), maximum), sigmas * std_yaw
