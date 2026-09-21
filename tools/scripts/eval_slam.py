#!/usr/bin/env python3
"""
eval_slam.py

slam_gnss_2d のオフラインSLAM出力 (pose_graph.json / gnss_transform.yaml) を、
rosbag の NavPVT (RTK Fix / Float / Single) と比較して精度を評価するツール。

比較は GNSS アンテナ位置 (base_link + R(yaw) * レバーアーム) 対 NavPVT 位置で行う。
  - raw     : gnss_transform.yaml の anchor_utm をそのまま使った残差 (UTM整合性)
  - aligned : 全 Fix サンプルへ SE(2) 剛体整合した後の残差 (地図の形状精度)
  - segment : 連続した Fix 区間ごとに個別整合した残差 (局所形状精度)
"""

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from ublox_msgs.msg import NavPVT

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import open_reader  # noqa: E402
from tools.common.cli import add_output_args, resolve_output_path  # noqa: E402
from tools.common.geo import lat_lon_to_utm  # noqa: E402
from tools.common.map import estimate_rigid_transform, load_map_info  # noqa: E402

CARRIER_NAMES = {2: "fix", 1: "float", 0: "none"}
# 連続した Fix 区間とみなす最大サンプル間隔 [s]
SEGMENT_MAX_GAP_SEC = 2.5
# SLAM ノード間の補間を許す最大ギャップ [s]
NODE_MAX_GAP_SEC = 2.0
# 局所整合の対象とする最小サンプル数
SEGMENT_MIN_SAMPLES = 10
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


def read_navpvt(bag_path: str, utm_zone: int) -> np.ndarray:
    """rosbag の /navpvt を読み込む。

    Returns:
      shape (M, 5) -> [bag_time, easting, northing, carr_soln, h_acc]
    """
    reader = open_reader(bag_path, topics=["/navpvt"])
    records = []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        msg = deserialize_message(data, NavPVT)
        if not (msg.flags & 0x01):
            continue
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        if abs(lat) < 0.1 or abs(lon) < 0.1:
            continue
        easting, northing = lat_lon_to_utm(lat, lon, zone=utm_zone)
        carr_soln = (msg.flags >> 6) & 0x03
        records.append([stamp * 1e-9, easting, northing, carr_soln, msg.h_acc * 1e-3])
    return np.array(records, dtype=float).reshape(-1, 5)


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


def residual_stats(err: np.ndarray, h_acc: Optional[np.ndarray] = None) -> Dict[str, Any]:
    """残差ベクトル (N, 2) の統計量を返す。h_acc があれば hAcc 正規化 RMS も付加する。"""
    if err.shape[0] == 0:
        return {"n": 0}
    dist = np.linalg.norm(err, axis=1)
    stats = {
        "n": int(dist.size),
        "rms_m": float(np.sqrt(np.mean(dist ** 2))),
        "mean_m": float(np.mean(dist)),
        "median_m": float(np.median(dist)),
        "p95_m": float(np.percentile(dist, 95)),
        "max_m": float(np.max(dist)),
    }
    if h_acc is not None:
        mask = h_acc > 0.0
        if np.any(mask):
            stats["hacc_normalized_rms"] = float(
                np.sqrt(np.mean((dist[mask] / h_acc[mask]) ** 2)))
    return stats


def residuals_by_carrier(
    err: np.ndarray, carr: np.ndarray, h_acc: np.ndarray
) -> Dict[str, Dict[str, Any]]:
    """残差ベクトル (N, 2) を RTK 状態 (Fix / Float / None) ごとに集計する。"""
    return {
        name: residual_stats(err[carr == code], h_acc[carr == code])
        for code, name in CARRIER_NAMES.items()
    }


def split_segments(times: np.ndarray, indices: np.ndarray) -> List[np.ndarray]:
    """indices (times 昇順) を、時間ギャップが SEGMENT_MAX_GAP_SEC を超える所で分割する。"""
    if indices.size == 0:
        return []
    breaks = np.where(np.diff(times[indices]) > SEGMENT_MAX_GAP_SEC)[0] + 1
    return np.split(indices, breaks)


def wall_thickness_stats(map_yaml: str) -> Dict[str, Any]:
    """占有セルの平均壁厚 [m] を、面積/周長から近似する (鮮鋭な壁ほど小さい)。"""
    img, _, resolution, _ = load_map_info(map_yaml)
    occupied = img < 100
    padded = np.pad(occupied, 1, constant_values=False)
    interior = (
        padded[1:-1, 1:-1]
        & padded[:-2, 1:-1] & padded[2:, 1:-1]
        & padded[1:-1, :-2] & padded[1:-1, 2:]
    )
    area = int(occupied.sum())
    perimeter = int((occupied & ~interior).sum())
    thickness_px = 2.0 * area / perimeter if perimeter > 0 else float("nan")
    return {
        "occupied_cells": area,
        "mean_wall_thickness_m": float(thickness_px * resolution),
    }


def evaluate(
    nodes: np.ndarray,
    transform: Dict[str, Any],
    gnss: np.ndarray,
    lever_arm: Tuple[float, float],
    time_offset: float,
) -> Dict[str, Any]:
    anchor = transform["anchor_utm"]
    anchor_xy = np.array([anchor["easting"], anchor["northing"]])

    times = gnss[:, 0] + time_offset
    poses, valid = interpolate_nodes(nodes, times)
    gnss = gnss[valid]
    poses = poses[valid]
    times = times[valid]

    # UTM 原点周りの回転で並進が巨大になるのを避けるため、anchor 基準のローカル座標で扱う
    slam_xy = antenna_positions(poses, lever_arm)
    gnss_xy = gnss[:, 1:3] - anchor_xy
    carr = gnss[:, 3].astype(int)
    h_acc = gnss[:, 4]

    report: Dict[str, Any] = {
        "slam": {
            "num_nodes": int(nodes.shape[0]),
            "duration_s": float(nodes[-1, 0] - nodes[0, 0]),
            "path_length_m": float(
                np.sum(np.linalg.norm(np.diff(nodes[:, 1:3], axis=0), axis=1))),
        },
        "gnss_transform": {
            "anchor_utm": [float(anchor_xy[0]), float(anchor_xy[1])],
            "rotation_rad": float(transform.get("rotation_rad", 0.0)),
        },
        "num_compared": int(gnss.shape[0]),
        "lever_arm_m": list(lever_arm),
        "time_offset_s": time_offset,
    }

    # raw: anchor_utm をそのまま使った UTM 整合性
    report["raw"] = residuals_by_carrier(slam_xy - gnss_xy, carr, h_acc)

    # aligned: 全 Fix サンプルへの SE(2) 整合 (Fix が無ければ Float を使う)
    fit_code = 2 if np.count_nonzero(carr == 2) >= SEGMENT_MIN_SAMPLES else 1
    fit_mask = carr == fit_code
    fit = fit_rigid(slam_xy[fit_mask], gnss_xy[fit_mask])
    report["aligned_fit_carrier"] = CARRIER_NAMES[fit_code]
    if fit is not None:
        t, theta = fit
        aligned_xy = apply_rigid(slam_xy, t, theta)
        report["aligned_transform"] = {
            "translation_m": [float(t[0]), float(t[1])],
            "rotation_deg": float(math.degrees(theta)),
        }
        report["aligned"] = residuals_by_carrier(aligned_xy - gnss_xy, carr, h_acc)

    # segment: 連続 Fix 区間ごとの個別整合
    segments = []
    fix_idx = np.where(carr == 2)[0]
    for seg in split_segments(times, fix_idx):
        if seg.size < SEGMENT_MIN_SAMPLES:
            continue
        seg_fit = fit_rigid(slam_xy[seg], gnss_xy[seg])
        if seg_fit is None:
            continue
        seg_xy = apply_rigid(slam_xy[seg], *seg_fit)
        stats = residual_stats(seg_xy - gnss_xy[seg])
        stats["t_start_rel_s"] = float(times[seg[0]] - nodes[0, 0])
        stats["t_end_rel_s"] = float(times[seg[-1]] - nodes[0, 0])
        stats["rotation_deg"] = float(math.degrees(seg_fit[1]))
        stats["translation_m"] = [float(seg_fit[0][0]), float(seg_fit[0][1])]
        segments.append(stats)
    report["fix_segments"] = segments
    return report


def format_report(report: Dict[str, Any]) -> str:
    def row(name: str, s: Dict[str, Any]) -> str:
        if s.get("n", 0) == 0:
            return f"| {name} | 0 | - | - | - | - | - | - |"
        norm = s.get("hacc_normalized_rms")
        norm_str = f"{norm:.2f}" if norm is not None else "-"
        return (
            f"| {name} | {s['n']} | {s['rms_m']:.3f} | {s['mean_m']:.3f} | "
            f"{s['median_m']:.3f} | {s['p95_m']:.3f} | {s['max_m']:.3f} | {norm_str} |"
        )

    header = (
        "| 区分 | N | RMS[m] | 平均[m] | 中央値[m] | p95[m] | 最大[m] | hAcc正規化RMS |\n"
        "|---|---|---|---|---|---|---|---|"
    )
    lines = ["# SLAM 評価レポート (RTK 比較)", ""]
    slam = report["slam"]
    lines += [
        f"- ノード数: {slam['num_nodes']} / 時間: {slam['duration_s']:.1f} s / "
        f"経路長: {slam['path_length_m']:.1f} m",
        f"- 比較サンプル数: {report['num_compared']} "
        f"(レバーアーム {report['lever_arm_m']}, 時刻オフセット {report['time_offset_s']} s)",
        f"- anchor_utm: {report['gnss_transform']['anchor_utm']} / "
        f"rotation_rad: {report['gnss_transform']['rotation_rad']:.4f}",
        "",
        "## raw (gnss_transform.yaml のまま: UTM整合性)",
        "",
        header,
    ]
    lines += [row(k, v) for k, v in report["raw"].items()]

    if "aligned" in report:
        tf = report["aligned_transform"]
        lines += [
            "",
            f"## aligned ({report['aligned_fit_carrier']} 全体へ SE(2) 整合: 形状精度)",
            "",
            f"- 整合量: 並進 {tf['translation_m']} m, 回転 {tf['rotation_deg']:.3f} deg",
            "",
            header,
        ]
        lines += [row(k, v) for k, v in report["aligned"].items()]

    if report["fix_segments"]:
        lines += [
            "",
            "## Fix 区間ごとの個別整合 (局所形状精度)",
            "",
            "| 区間[s] | N | RMS[m] | 最大[m] | 個別整合の回転[deg] |",
            "|---|---|---|---|---|",
        ]
        for s in report["fix_segments"]:
            lines.append(
                f"| {s['t_start_rel_s']:.0f}-{s['t_end_rel_s']:.0f} | {s['n']} | "
                f"{s['rms_m']:.3f} | {s['max_m']:.3f} | {s['rotation_deg']:.3f} |"
            )

    if report.get("matching", {}).get("n", 0) > 0:
        mt = report["matching"]
        lines += [
            "",
            "## スキャンマッチング品質 (逐次エッジのスコア: 高いほど良い)",
            "",
            f"- エッジ数: {mt['n']} (オドメトリフォールバック {mt['odom_fallback']})",
            f"- 全体: 平均 {mt['mean_score']:.4f} / 中央値 {mt['median_score']:.4f} / p10 {mt['p10_score']:.4f}",
        ]
        if "mean_score_turning" in mt:
            lines.append(
                f"- 旋回中 (|dyaw|>{TURNING_DYAW_RAD} rad, {mt['n_turning']} 本): "
                f"平均 {mt['mean_score_turning']:.4f} / p10 {mt['p10_score_turning']:.4f}")

    if "map" in report:
        m = report["map"]
        lines += [
            "",
            "## 地図",
            "",
            f"- 占有セル数: {m['occupied_cells']} / 平均壁厚(面積/周長近似): "
            f"{m['mean_wall_thickness_m']:.3f} m",
        ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="slam_gnss_2d の出力を rosbag の NavPVT (RTK) と比較して評価する")
    parser.add_argument("bag_path", type=str, help="評価に使う rosbag のパス")
    parser.add_argument(
        "--slam-dir", type=str, required=True,
        help="pose_graph.json と gnss_transform.yaml を含む SLAM 出力ディレクトリ")
    parser.add_argument(
        "--map", type=str, default=None,
        help="壁厚を算出する map.yaml (省略時は --slam-dir/map.yaml があれば使用)")
    parser.add_argument(
        "--lever-arm", type=float, nargs=2, default=[0.26, -0.13],
        metavar=("X", "Y"), help="base_link から gps_link までのオフセット [m]")
    parser.add_argument(
        "--time-offset", type=float, default=0.0,
        help="GNSS 時刻に加える時間オフセット [s]")
    add_output_args(parser, "eval_slam.json")
    args = parser.parse_args()

    nodes, transform, matching = load_slam_output(args.slam_dir)
    zone = int(transform["anchor_utm"]["zone"])
    gnss = read_navpvt(args.bag_path, zone)
    if gnss.shape[0] == 0:
        raise SystemExit("エラー: rosbag に有効な /navpvt がありません。")

    report = evaluate(
        nodes, transform, gnss, tuple(args.lever_arm), args.time_offset)

    report["matching"] = matching

    map_yaml = args.map or os.path.join(args.slam_dir, "map.yaml")
    if os.path.exists(map_yaml):
        report["map"] = wall_thickness_stats(map_yaml)

    text = format_report(report)
    print(text)

    output_path = resolve_output_path(
        args.bag_path, "eval_slam.json", args.output,
        args.output_to_bag_dir, args.output_dir)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    md_path = os.path.splitext(output_path)[0] + ".md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"保存: {output_path}, {md_path}")


if __name__ == "__main__":
    main()
