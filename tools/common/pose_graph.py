"""slam_gnss_2d の出力 (pose_graph.json / gnss_transform.yaml) を扱う共通モジュール。

rclpy には依存しないため、単体テストや rosbag を読まない解析からも利用できる。
"""

import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

from tools.common.map import estimate_rigid_transform

# SLAM ノード間の補間を許す最大ギャップ [s]
NODE_MAX_GAP_SEC = 2.0
# 旋回中とみなす、逐次エッジ 1 本あたりの回転量 [rad]
TURNING_DYAW_RAD = 0.03


def load_slam_output(slam_dir: str) -> Tuple[np.ndarray, Dict[str, Any], Dict[str, Any]]:
    """pose_graph.json と gnss_transform.yaml を読み込む。

    Returns:
      nodes: shape (N, 4) -> [timestamp, x, y, yaw] (timestamp 昇順)
      transform: gnss_transform.yaml の内容
      matching: 逐次エッジのスキャンマッチング品質の統計
    """
    graph_path = os.path.join(slam_dir, "pose_graph.json")
    transform_path = os.path.join(slam_dir, "gnss_transform.yaml")
    with open(graph_path, "r", encoding="utf-8") as f:
        graph = json.load(f)
    with open(transform_path, "r", encoding="utf-8") as f:
        transform = yaml.safe_load(f)

    nodes = np.array(
        [[n["timestamp"], n["x"], n["y"], n["yaw"]] for n in graph["nodes"]],
        dtype=float,
    )
    nodes = nodes[np.argsort(nodes[:, 0])]
    return nodes, transform, matching_stats(graph.get("sequential_edges", []))


def matching_stats(seq_edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    """逐次エッジのマッチングスコア (CSM: 高いほど良い) の統計を返す。旋回中は別集計する。"""
    if not seq_edges:
        return {"n": 0}
    scores = np.array([e["score"] for e in seq_edges], dtype=float)
    dyaw = np.abs(np.array([e["dyaw"] for e in seq_edges], dtype=float))
    fallback = np.array([bool(e.get("is_odom_fallback", False)) for e in seq_edges])
    turning = dyaw > TURNING_DYAW_RAD
    stats: Dict[str, Any] = {
        "n": int(scores.size),
        "odom_fallback": int(fallback.sum()),
        "mean_score": float(np.mean(scores)),
        "median_score": float(np.median(scores)),
        "p10_score": float(np.percentile(scores, 10)),
        "n_turning": int(turning.sum()),
    }
    if np.any(turning):
        stats["mean_score_turning"] = float(np.mean(scores[turning]))
        stats["p10_score_turning"] = float(np.percentile(scores[turning], 10))
    return stats


def anchor_utm_xy(transform: Dict[str, Any]) -> np.ndarray:
    """gnss_transform.yaml のアンカーの UTM 座標 [easting, northing] を返す。"""
    anchor = transform["anchor_utm"]
    return np.array([anchor["easting"], anchor["northing"]], dtype=float)


def shift_nodes_to_anchor(
    nodes: np.ndarray,
    from_transform: Dict[str, Any],
    to_transform: Dict[str, Any],
) -> np.ndarray:
    """ノード列を from 側のアンカー座標から to 側のアンカー座標へ移す。

    両方の SLAM 結果は UTM に整合済み (SLAM 座標 = UTM - anchor_utm) なので、
    アンカー同士の UTM 差を並進として加えるだけでよい。UTM ゾーンが違う場合は移せない。
    """
    if from_transform["anchor_utm"]["zone"] != to_transform["anchor_utm"]["zone"]:
        raise ValueError("UTM ゾーンが異なる SLAM 結果同士は座標を移せません。")
    shift = anchor_utm_xy(from_transform) - anchor_utm_xy(to_transform)
    moved = nodes.copy()
    moved[:, 1:3] += shift
    return moved


def interpolate_nodes(nodes: np.ndarray, times: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """SLAM ノード列を times で線形補間する (yaw は単位ベクトル補間)。

    Returns:
      poses: shape (M, 3) -> [x, y, yaw]
      valid: shape (M,) 補間可能 (範囲内かつ隣接ノード間隔が NODE_MAX_GAP_SEC 以下) か
    """
    t = nodes[:, 0]
    idx = np.searchsorted(t, times, side="right")
    valid = (idx > 0) & (idx < len(t))
    idx = np.clip(idx, 1, len(t) - 1)
    t0, t1 = t[idx - 1], t[idx]
    valid &= (t1 - t0) <= NODE_MAX_GAP_SEC
    denom = np.where(t1 - t0 > 0.0, t1 - t0, 1.0)
    w = np.clip((times - t0) / denom, 0.0, 1.0)

    x = nodes[idx - 1, 1] * (1 - w) + nodes[idx, 1] * w
    y = nodes[idx - 1, 2] * (1 - w) + nodes[idx, 2] * w
    c = np.cos(nodes[idx - 1, 3]) * (1 - w) + np.cos(nodes[idx, 3]) * w
    s = np.sin(nodes[idx - 1, 3]) * (1 - w) + np.sin(nodes[idx, 3]) * w
    yaw = np.arctan2(s, c)
    return np.stack([x, y, yaw], axis=1), valid


def antenna_positions(poses: np.ndarray, lever_arm: Tuple[float, float]) -> np.ndarray:
    """base_link 姿勢とレバーアーム (base_link 座標系) から GNSS アンテナ位置を求める。"""
    lx, ly = lever_arm
    c = np.cos(poses[:, 2])
    s = np.sin(poses[:, 2])
    ax = poses[:, 0] + c * lx - s * ly
    ay = poses[:, 1] + s * lx + c * ly
    return np.stack([ax, ay], axis=1)


def fit_rigid(src_xy: np.ndarray, dst_xy: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    """dst = R(theta) * src + t となる剛体変換を推定し、(t, theta) を返す。"""
    result = estimate_rigid_transform(src_xy, dst_xy)
    if result is None:
        return None
    return np.array([result[0], result[1]]), result[2]


def apply_rigid(xy: np.ndarray, t: np.ndarray, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return xy @ rot.T + t
