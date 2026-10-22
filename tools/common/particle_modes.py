"""AMCL の粒子 (nav2_msgs/ParticleCloud) を仮説 (峰) に分ける共通モジュール (numpy のみに依存)。

粒子を (x, y, yaw) の格子に入れ、隣り合う格子 (yaw は一周つながる) をまとめた連結成分を 1 つの仮説とする。
仮説ごとに、重みの和、重みつきの平均 (yaw は円周の平均) と共分散を求める。
"""

from collections import deque
from typing import Any, Dict, List, Tuple

import numpy as np

from tools.common.loc_metrics import wrap_angle


def cluster_particles(
    poses: np.ndarray, weights: np.ndarray, cell_m: float, cell_yaw_rad: float,
) -> List[Dict[str, Any]]:
    """粒子を仮説に分け、重みの大きい順に返す。

    Args:
      poses: shape (N, 3) -> [x, y, yaw]
      weights: shape (N,)。和が 0 なら均等とみなす
      cell_m, cell_yaw_rad: 格子の大きさ。これより離れた粒子の群れは別の仮説になる
    Returns:
      仮説ごとの {"weight" (和を 1 に正規化), "mean" [x, y, yaw], "cov" 3x3, "n"}
    """
    if poses.shape[0] == 0:
        return []
    w = np.asarray(weights, dtype=float)
    w = w / w.sum() if w.sum() > 0.0 else np.full(w.shape, 1.0 / w.size)
    n_yaw = max(1, int(round(2.0 * np.pi / cell_yaw_rad)))
    keys = np.stack([
        np.floor(poses[:, 0] / cell_m).astype(int),
        np.floor(poses[:, 1] / cell_m).astype(int),
        np.floor((wrap_angle(poses[:, 2]) + np.pi) / (2.0 * np.pi) * n_yaw).astype(int) % n_yaw,
    ], axis=1)
    cells: Dict[Tuple[int, int, int], List[int]] = {}
    for i, key in enumerate(map(tuple, keys)):
        cells.setdefault(key, []).append(i)

    label: Dict[Tuple[int, int, int], int] = {}
    n_labels = 0
    for start in cells:
        if start in label:
            continue
        label[start] = n_labels
        queue = deque([start])
        while queue:
            cx, cy, ck = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dk in (-1, 0, 1):
                        nb = (cx + dx, cy + dy, (ck + dk) % n_yaw)
                        if nb in cells and nb not in label:
                            label[nb] = n_labels
                            queue.append(nb)
        n_labels += 1

    members: List[List[int]] = [[] for _ in range(n_labels)]
    for key, idx in cells.items():
        members[label[key]].extend(idx)
    modes = [summarize_mode(poses[idx], w[idx]) for idx in members]
    return sorted(modes, key=lambda m: m["weight"], reverse=True)


def summarize_mode(poses: np.ndarray, weights: np.ndarray) -> Dict[str, Any]:
    total = float(weights.sum())
    nw = weights / total if total > 0.0 else np.full(weights.shape, 1.0 / weights.size)
    yaw = float(np.arctan2(np.sum(nw * np.sin(poses[:, 2])), np.sum(nw * np.cos(poses[:, 2]))))
    mean = np.array([np.sum(nw * poses[:, 0]), np.sum(nw * poses[:, 1]), yaw])
    d = poses - mean
    d[:, 2] = wrap_angle(d[:, 2])
    cov = (d * nw[:, None]).T @ d
    return {"weight": total, "mean": mean, "cov": cov, "n": int(poses.shape[0])}
