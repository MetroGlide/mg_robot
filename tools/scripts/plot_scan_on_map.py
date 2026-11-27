#!/usr/bin/env python3
"""
plot_scan_on_map.py

rosbag の LiDAR スキャンを、指定した時刻の真値の姿勢 (SLAM 出力) と、推定した姿勢 (再生評価の記録の
/amcl_pose など) に置いて、地図に重ねて描く。自己位置がずれた時刻に、スキャンが地図のどこに合っているか
(真値の姿勢で地図と合うか、推定の姿勢で何に引き寄せられたか) を確かめるのに使う。
地図を複数渡すと、地図ごとに 1 行で並べる (自己位置推定用の地図と、その走行の SLAM 地図の比較など)。

使い方:
  plot_scan_on_map.py <スキャンの bag> --map <map.yaml> [--map ...] --gt-dir <SLAM 出力>
      [--estimate <再生評価の記録の bag>] --times 340,350,356 [-o scan_on_map.png]

時刻は、スキャンの bag の最初のスキャンからの経過秒。
"""

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.map import load_map_info  # noqa: E402
from tools.common.pose_graph import interpolate_nodes, load_slam_output  # noqa: E402

# 真値の補間で、止まっているとみなす隣接ノード間の移動 [m] (eval_localization.py と同じ)
STATIONARY_DIST_M = 0.6


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sensor_to_base(static_tfs: Dict[str, Tuple[str, np.ndarray]], frame: str, base_frame: str) -> np.ndarray:
    """/tf_static をたどり、センサの座標系から base_frame への 2D の姿勢 [x, y, yaw] を返す。"""
    pose = np.zeros(3)
    while frame != base_frame:
        if frame not in static_tfs:
            raise SystemExit(f"エラー: /tf_static に {frame} から {base_frame} へのつながりがありません。")
        frame, parent_pose = static_tfs[frame]
        pose = lm.compose(parent_pose[None, :], pose[None, :])[0]
    return pose


def read_scans(bag_path: str, topic: str, times: List[float], base_frame: str) -> List[Tuple[float, np.ndarray]]:
    """各時刻の直後のスキャンを、base_frame の座標の点 (N, 2) で返す。時刻は bag の時刻 [s]。"""
    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise SystemExit(f"エラー: {bag_path} に {topic} がありません。")
    deserializer = MessageDeserializer(type_map)
    static_tfs: Dict[str, Tuple[str, np.ndarray]] = {}
    t0: Optional[float] = None
    found: Dict[float, Tuple[float, np.ndarray, str]] = {}
    while reader.has_next() and len(found) < len(times):
        name, raw, _ = reader.read_next()
        if name == "/tf_static":
            for tf in deserializer.deserialize(name, raw).transforms:
                q = tf.transform.rotation
                static_tfs[tf.child_frame_id] = (tf.header.frame_id, np.array([
                    tf.transform.translation.x, tf.transform.translation.y,
                    quaternion_to_yaw(q.x, q.y, q.z, q.w)]))
        if name != topic:
            continue
        msg = deserializer.deserialize(name, raw)
        t = stamp_sec(msg.header.stamp)
        t0 = t if t0 is None else t0
        for rel in times:
            if rel not in found and t - t0 >= rel:
                ranges = np.array(msg.ranges, dtype=float)
                angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
                ok = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
                pts = np.c_[ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])]
                found[rel] = (t, pts, msg.header.frame_id)
    scans = []
    for rel in times:
        if rel not in found:
            raise SystemExit(f"エラー: {rel} s のスキャンがありません。")
        t, pts, frame = found[rel]
        scans.append((t, transform_points(pts, sensor_to_base(static_tfs, frame, base_frame))))
    return scans


def transform_points(pts: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return pts @ np.array([[c, s], [-s, c]]) + pose[:2]


def read_estimates(bag_path: str, topic: str) -> np.ndarray:
    """推定の姿勢の列 (N, 4) [t, x, y, yaw] を返す。"""
    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise SystemExit(f"エラー: {bag_path} に {topic} がありません。")
    deserializer = MessageDeserializer(type_map)
    rows = []
    while reader.has_next():
        name, raw, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserializer.deserialize(name, raw)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rows.append([stamp_sec(msg.header.stamp), p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)])
    arr = np.array(rows, dtype=float).reshape(-1, 4)
    return arr[np.argsort(arr[:, 0], kind="stable")]


def main() -> None:
    parser = argparse.ArgumentParser(description="スキャンを真値と推定の姿勢に置いて地図に重ねる")
    parser.add_argument("bag_path", help="スキャンと /tf_static を含む bag")
    parser.add_argument("--map", action="append", required=True, help="地図の yaml (複数可)")
    parser.add_argument("--gt-dir", required=True, help="真値 (pose_graph.json) の SLAM 出力")
    parser.add_argument("--estimate", default=None, help="推定の姿勢を含む bag (再生評価の run_N/output)")
    parser.add_argument("--estimate-topic", default="/amcl_pose")
    parser.add_argument("--scan-topic", default="/scan_top_lidar")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--times", required=True, help="描く時刻 (最初のスキャンからの経過秒、カンマ区切り)")
    parser.add_argument("--span", type=float, default=25.0, help="真値の位置から描く範囲 [m]")
    parser.add_argument("-o", "--output", default="scan_on_map.png")
    args = parser.parse_args()

    times = [float(v) for v in args.times.split(",")]
    gt_nodes = load_slam_output(args.gt_dir)[0]
    scans = read_scans(args.bag_path, args.scan_topic, times, args.base_frame)
    estimates = read_estimates(args.estimate, args.estimate_topic) if args.estimate else None
    maps = [load_map_info(path) for path in args.map]

    fig, axes = plt.subplots(len(maps), len(times), figsize=(5 * len(times), 5 * len(maps)), squeeze=False)
    for col, (rel, (t, pts)) in enumerate(zip(times, scans)):
        gt, valid = interpolate_nodes(gt_nodes, np.array([t]), stationary_dist=STATIONARY_DIST_M)
        if not valid[0]:
            raise SystemExit(f"エラー: {rel} s の真値がありません。")
        poses = [("GT", gt[0], "tab:green")]
        if estimates is not None and estimates.size:
            idx = min(np.searchsorted(estimates[:, 0], t), len(estimates) - 1)
            poses.append(("estimate", estimates[idx, 1:4], "tab:red"))
        title = f"{rel:g} s  " + "  ".join(f"{label} yaw {np.degrees(p[2]):.0f}" for label, p, _ in poses)
        for row, (img, origin, resolution, path) in enumerate(maps):
            ax = axes[row][col]
            h, w = img.shape[:2]
            ax.imshow(img, cmap="gray", origin="upper",
                      extent=[origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution])
            for label, pose, color in poses:
                world = transform_points(pts, pose)
                ax.scatter(world[:, 0], world[:, 1], s=1, color=color, label=f"scan @ {label}")
                ax.arrow(pose[0], pose[1], 2.0 * np.cos(pose[2]), 2.0 * np.sin(pose[2]), color=color, width=0.2)
            ax.set_xlim(gt[0, 0] - args.span, gt[0, 0] + args.span)
            ax.set_ylim(gt[0, 1] - args.span, gt[0, 1] + args.span)
            ax.set_title(f"{os.path.basename(path)}\n{title}", fontsize=9)
            ax.legend(loc="lower right", markerscale=8, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output, dpi=80)
    print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
