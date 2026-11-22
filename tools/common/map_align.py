"""地図と GNSS のずれの計測に使う共通処理。

- 点群 (車体の座標系) を占有格子の地図に照合して、地図の上の姿勢を求める (match_cloud)
- 複数の地図 (区画) から、点を含む地図を選ぶ (select_map)
- waypoint-tool のプロジェクト (.wptroj) のグループと、その範囲・手のずらし量を読む (load_wptroj_groups)
- 地図の座標どうしの剛体のずれを当てはめる (fit_offsets)

numpy と scipy のみに依存する。
"""

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from tools.common import loc_metrics as lm
from tools.common.map import estimate_rigid_transform
from tools.common.scan_map import OccupancyMap


def transform_points(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    """姿勢 pose [x, y, yaw] で点列 points (N, 2) を移す。"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def inverse_pose(pose: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.array([-c * pose[0] - s * pose[1], s * pose[0] - c * pose[1], -pose[2]])


def voxel_downsample(points: np.ndarray, voxel: float, max_points: int,
                     rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """voxel [m] の格子ごとに 1 点を残し、max_points を超えたら間引く。"""
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    kept = points[np.sort(idx)]
    if kept.shape[0] > max_points:
        rng = rng or np.random.default_rng(0)
        kept = kept[np.sort(rng.choice(kept.shape[0], max_points, replace=False))]
    return kept


def select_map(maps: Sequence[OccupancyMap], xy: np.ndarray) -> Optional[int]:
    """点 xy を含み、端までの余裕が最も大きい地図の番号。どれにも入らなければ None。"""
    best, best_margin = None, 0.0
    for i, grid in enumerate(maps):
        rows, cols = grid.occupied.shape
        lo = grid.origin
        hi = grid.origin + grid.resolution * np.array([cols, rows])
        margin = float(np.min(np.concatenate([xy - lo, hi - xy])))
        if margin > best_margin:
            best, best_margin = i, margin
    return best


def _score(grid: OccupancyMap, points: np.ndarray, sigma: float) -> np.ndarray:
    """点ごとの尤度 exp(-d^2 / 2 sigma^2) (地図の外は 0)。points は (..., 2)。"""
    flat = points.reshape(-1, 2)
    d = grid.distance_at(flat)
    like = np.where(np.isfinite(d), np.exp(-0.5 * (d / sigma) ** 2), 0.0)
    return like.reshape(points.shape[:-1])


def _grid_search(grid: OccupancyMap, cloud: np.ndarray, center: np.ndarray, half_xy: float, step_xy: float,
                 half_yaw: float, step_yaw: float, sigma: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """center の周りで (dx, dy, dyaw) を格子で探す。返り値は候補の姿勢 (K, 3)、平均の尤度 (K,)、最良の番号。"""
    offsets = np.arange(-half_xy, half_xy + 1e-9, step_xy)
    yaws = center[2] + np.arange(-half_yaw, half_yaw + 1e-12, step_yaw) if half_yaw > 0 else np.array([center[2]])
    shifts = np.stack(np.meshgrid(offsets, offsets, indexing="ij"), axis=-1).reshape(-1, 2)
    poses, scores = [], []
    for yaw in yaws:
        rotated = transform_points(np.array([center[0], center[1], yaw]), cloud)
        candidate = rotated[None, :, :] + shifts[:, None, :]
        scores.append(_score(grid, candidate, sigma).mean(axis=1))
        poses.append(np.column_stack([center[0] + shifts[:, 0], center[1] + shifts[:, 1],
                                      np.full(shifts.shape[0], yaw)]))
    poses_arr = np.concatenate(poses)
    scores_arr = np.concatenate(scores)
    return poses_arr, scores_arr, int(np.argmax(scores_arr))


@dataclass
class MatchResult:
    pose: np.ndarray
    score: float
    inlier_ratio: float
    ambiguity: float
    cov: np.ndarray
    n_points: int
    extra: Dict[str, float] = field(default_factory=dict)


def match_cloud(grid: OccupancyMap, cloud: np.ndarray, init: np.ndarray, search_xy: float = 3.0,
                search_yaw: float = math.radians(3.0), coarse_step: float = 0.2,
                coarse_yaw_step: float = math.radians(0.5), coarse_sigma: float = 0.3,
                fine_sigma: float = 0.1, inlier_dist: float = 0.15, ambiguity_radius: float = 1.0,
                gn_iterations: int = 10) -> MatchResult:
    """車体の座標系の点群 cloud (N, 2) を地図に照合し、init の周りで最も合う姿勢を返す。

    粗い格子の探索 → 細かい格子の探索 → 距離場での Gauss-Newton の順に詰める。
    ambiguity: 最良から ambiguity_radius [m] 以上離れた候補の最良の尤度 / 最良の尤度 (粗い探索)。1 に近いほど曖昧。
    cov: Gauss-Newton の (J^T J)^-1 に残差の分散をかけた姿勢の共分散 (点どうしの相関を無視するので小さめに出る)。
    """
    poses, scores, best = _grid_search(grid, cloud, init, search_xy, coarse_step, search_yaw, coarse_yaw_step,
                                       coarse_sigma)
    far = np.hypot(poses[:, 0] - poses[best, 0], poses[:, 1] - poses[best, 1]) >= ambiguity_radius
    ambiguity = float(np.max(scores[far]) / scores[best]) if np.any(far) and scores[best] > 0 else 0.0

    poses_f, scores_f, best_f = _grid_search(grid, cloud, poses[best], coarse_step * 1.5, 0.05,
                                             coarse_yaw_step * 1.5, math.radians(0.1), fine_sigma)
    pose = poses_f[best_f].copy()

    eps = grid.resolution
    cov = np.full((3, 3), np.nan)
    for _ in range(gn_iterations):
        pts = transform_points(pose, cloud)
        d = grid.distance_at(pts)
        use = np.isfinite(d) & (d < 3.0 * fine_sigma)
        if np.count_nonzero(use) < 10:
            break
        p = pts[use]
        gx = (grid.distance_at(p + [eps, 0.0]) - grid.distance_at(p - [eps, 0.0])) / (2 * eps)
        gy = (grid.distance_at(p + [0.0, eps]) - grid.distance_at(p - [0.0, eps])) / (2 * eps)
        ok = np.isfinite(gx) & np.isfinite(gy)
        if np.count_nonzero(ok) < 10:
            break
        p, gx, gy, r = p[ok], gx[ok], gy[ok], d[use][ok]
        rel = p - pose[:2]
        jac = np.column_stack([gx, gy, -rel[:, 1] * gx + rel[:, 0] * gy])
        # 通路のように 1 方向に決まらない形でも解けるよう、わずかに減衰をかける (その方向の分散は大きく出る)
        h = jac.T @ jac
        h = h + 1e-6 * np.trace(h) * np.eye(3)
        delta = -np.linalg.solve(h, jac.T @ r)
        delta[:2] = np.clip(delta[:2], -0.1, 0.1)
        delta[2] = float(np.clip(delta[2], -math.radians(0.5), math.radians(0.5)))
        pose = pose + delta
        cov = np.linalg.inv(h) * max(float(np.mean(r ** 2)), (0.5 * grid.resolution) ** 2)
        if np.hypot(delta[0], delta[1]) < 1e-3 and abs(delta[2]) < 1e-4:
            break
    pose[2] = float(lm.wrap_angle(pose[2]))
    final = grid.distance_at(transform_points(pose, cloud))
    inlier = float(np.mean(np.isfinite(final) & (final < inlier_dist)))
    return MatchResult(pose=pose, score=float(_score(grid, transform_points(pose, cloud), fine_sigma).mean()),
                       inlier_ratio=inlier, ambiguity=ambiguity, cov=cov, n_points=int(cloud.shape[0]))


@dataclass
class WptGroup:
    name: str
    offset: Tuple[float, float, float]
    rects: List[Tuple[float, float, float, float]]
    order: int

    def contains(self, xy: np.ndarray) -> np.ndarray:
        inside = np.zeros(xy.shape[0], dtype=bool)
        for x, y, w, h in self.rects:
            inside |= (xy[:, 0] >= x) & (xy[:, 0] <= x + w) & (xy[:, 1] >= y) & (xy[:, 1] <= y + h)
        return inside


def load_wptroj_groups(path: str) -> Tuple[List[WptGroup], List[Tuple[str, Tuple[float, float, float, float]]]]:
    """waypoint-tool のプロジェクトから、表示しているグループ (範囲 = 子の地図レイヤーの切り抜き) と出力の区画を読む。

    order は layer_order で最も上にある子の位置 (小さいほど上に描かれる)。
    切り抜きのない子の地図は、範囲に数えない (地図全体を覆うレイヤーは、区画の境目を決めない)。
    """
    with open(path, "r", encoding="utf-8") as f:
        project = json.load(f)
    layers = {layer["id"]: layer for layer in project.get("map_layers", [])}
    order = {lid: i for i, lid in enumerate(project.get("layer_order", []))}
    groups = []
    for group in project.get("layer_groups", {}).values():
        if not group.get("visible", True):
            continue
        rects, ranks = [], []
        for child in group.get("children_ids", []):
            layer = layers.get(child)
            if layer is None or not layer.get("visible", True):
                continue
            ranks.append(order.get(child, len(order)))
            for r in (layer.get("clip") or {}).get("rects", []):
                rects.append((float(r["x"]), float(r["y"]), float(r["width"]), float(r["height"])))
        if not rects:
            continue
        off = group.get("pose_offset") or {}
        groups.append(WptGroup(name=group["name"],
                               offset=(float(off.get("x", 0.0)), float(off.get("y", 0.0)),
                                       float(off.get("yaw", 0.0))),
                               rects=rects, order=min(ranks) if ranks else len(order)))
    regions = [(r["name"], (float(r["rect"]["x"]), float(r["rect"]["y"]), float(r["rect"]["width"]),
                            float(r["rect"]["height"]))) for r in project.get("export_regions", [])]
    return groups, regions


def assign_group(groups: Sequence[WptGroup], xy: np.ndarray) -> List[Optional[str]]:
    """点ごとに、それを含むグループのうち最も上に描かれるものの名前 (どれにも入らなければ None)。"""
    names: List[Optional[str]] = [None] * xy.shape[0]
    best = np.full(xy.shape[0], np.iinfo(np.int64).max)
    for group in groups:
        inside = group.contains(xy) & (group.order < best)
        best[inside] = group.order
        for i in np.nonzero(inside)[0]:
            names[i] = group.name
    return names


def fit_offsets(ref_xy: np.ndarray, map_xy: np.ndarray, rotation: bool = True,
                min_extent: float = 20.0) -> Tuple[np.ndarray, float]:
    """map = R(theta) * ref + t となる (t, theta) を当てはめる。

    点の広がりが min_extent [m] より小さいときは、回転を決められないので並進だけにする。
    """
    if ref_xy.shape[0] == 0:
        raise ValueError("当てはめる点がありません。")
    extent = float(np.max(np.ptp(ref_xy, axis=0))) if ref_xy.shape[0] > 1 else 0.0
    if rotation and ref_xy.shape[0] >= 3 and extent >= min_extent:
        result = estimate_rigid_transform(ref_xy, map_xy)
        if result is not None:
            return np.array(result[:2]), float(result[2])
    return np.mean(map_xy - ref_xy, axis=0), 0.0


def apply_offsets(xy: np.ndarray, t: np.ndarray, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return xy @ np.array([[c, s], [-s, c]]) + t
