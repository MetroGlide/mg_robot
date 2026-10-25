"""AMCL の粒子 (nav2_msgs/ParticleCloud) を仮説 (峰) に分ける共通モジュール (numpy のみに依存)。

粒子を (x, y, yaw) の格子に入れ、隣り合う格子 (yaw は一周つながる) をまとめた連結成分に分ける。
向きに扇形に広がった粒子は 1 つの連結成分につながるため、連結成分の中の向きの分布に谷があれば、
谷で区切ってさらに分ける。仮説ごとに、重みの和、重みつきの平均 (yaw は円周の平均) と共分散を求める。
nav2 の AMCL はリサンプリングの後に粒子を出すので、重みはほぼ均一で、仮説の重みは粒子の数の割合になる
(尤度の比ではない)。
"""

from collections import deque
from typing import Any, Dict, List, Tuple

import numpy as np

from tools.common.loc_metrics import wrap_angle


def cluster_particles(
    poses: np.ndarray, weights: np.ndarray, cell_m: float, cell_yaw_rad: float,
    yaw_bin_rad: float = np.radians(10.0), valley_ratio: float = 0.5,
) -> List[Dict[str, Any]]:
    """粒子を仮説に分け、重みの大きい順に返す。

    Args:
      poses: shape (N, 3) -> [x, y, yaw]
      weights: shape (N,)。和が 0 なら均等とみなす
      cell_m, cell_yaw_rad: 格子の大きさ。これより離れた粒子の群れは別の仮説になる
      yaw_bin_rad: 連結成分の中で向きの分布を見る区間の幅
      valley_ratio: 2 つの山の間の谷が、低い方の山のこの割合未満なら別の仮説に分ける (0 で分けない)
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
    modes = []
    for idx in members:
        idx = np.asarray(idx)
        for part in split_by_yaw_valleys(poses[idx, 2], w[idx], yaw_bin_rad, valley_ratio):
            modes.append(summarize_mode(poses[idx[part]], w[idx[part]]))
    return sorted(modes, key=lambda m: m["weight"], reverse=True)


def split_by_yaw_valleys(
    yaw: np.ndarray, weights: np.ndarray, bin_rad: float, valley_ratio: float,
) -> List[np.ndarray]:
    """向きの重みつき分布 (一周つながる) を、深い谷で区切った粒子の添字の組に分ける。"""
    n_bins = max(3, int(round(2.0 * np.pi / bin_rad)))
    bins = np.floor((wrap_angle(yaw) + np.pi) / (2.0 * np.pi) * n_bins).astype(int) % n_bins
    if valley_ratio <= 0.0:
        return [np.arange(yaw.size)]
    hist = np.bincount(bins, weights=weights, minlength=n_bins)
    # 隣の区間と平均して、区間の境目で山が欠けて見えるのを抑える
    smooth = (np.roll(hist, 1) + hist + np.roll(hist, -1)) / 3.0
    peaks = [i for i in range(n_bins)
             if smooth[i] > 0.0 and smooth[i] >= smooth[i - 1] and smooth[i] > smooth[(i + 1) % n_bins]]
    cuts = []
    for a, b in zip(peaks, peaks[1:] + peaks[:1]):
        span = (b - a) % n_bins or n_bins
        between = [(a + k) % n_bins for k in range(1, span)]
        if not between:
            continue
        valley = min(between, key=lambda i: smooth[i])
        if smooth[valley] < valley_ratio * min(smooth[a], smooth[b]):
            cuts.append(valley)
    if len(cuts) < 2:
        return [np.arange(yaw.size)]
    # 谷の区間 cuts[k] から次の谷の手前までを 1 つの仮説にする (谷の区間は始まりの側に入れる)
    cuts = sorted(cuts)
    region = np.searchsorted(cuts, bins, side="right") % len(cuts)
    return [np.nonzero(region == r)[0] for r in range(len(cuts)) if np.any(region == r)]


def summarize_mode(poses: np.ndarray, weights: np.ndarray) -> Dict[str, Any]:
    total = float(weights.sum())
    nw = weights / total if total > 0.0 else np.full(weights.shape, 1.0 / weights.size)
    yaw = float(np.arctan2(np.sum(nw * np.sin(poses[:, 2])), np.sum(nw * np.cos(poses[:, 2]))))
    mean = np.array([np.sum(nw * poses[:, 0]), np.sum(nw * poses[:, 1]), yaw])
    d = poses - mean
    d[:, 2] = wrap_angle(d[:, 2])
    cov = (d * nw[:, None]).T @ d
    return {"weight": total, "mean": mean, "cov": cov, "n": int(poses.shape[0])}
