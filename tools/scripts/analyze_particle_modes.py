#!/usr/bin/env python3
"""
analyze_particle_modes.py

再生評価の記録 (run_localization_variant.sh の run_N/output) の /particle_cloud を、粒子の仮説 (峰) に分け、
仮説の数・重みと、真値に近い仮説 (正しい仮説) の重みを時系列で出す。
AMCL の粒子が角で割れて誤った方に収束するときに、正しい仮説がいつまで残っていたかを調べるのに使う。

使い方:
  analyze_particle_modes.py <記録の bag> --gt-dir <SLAM 出力> [--start S --end E] [-o modes.csv]

出力:
  CSV (粒子の配信ごとの 1 行) と、同名の .md (仮説が割れた時間、正しい仮説が負けた・消えた時間の要約)
"""

import argparse
import csv
import os
import sys
from typing import Any, Dict, List, Optional

import numpy as np

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.particle_modes import cluster_particles  # noqa: E402
from tools.common.pose_graph import interpolate_nodes, load_slam_output  # noqa: E402

# 真値の補間で、止まっているとみなす隣接ノード間の移動 [m] (eval_localization.py と同じ)
STATIONARY_DIST_M = 0.6


def read_clouds(bag_path: str, topic: str) -> List[Dict[str, Any]]:
    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise SystemExit(f"エラー: {bag_path} に {topic} がありません。")
    deserializer = MessageDeserializer(type_map)
    clouds = []
    while reader.has_next():
        name, raw, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserializer.deserialize(name, raw)
        poses = np.array([[p.pose.position.x, p.pose.position.y,
                           quaternion_to_yaw(p.pose.orientation.x, p.pose.orientation.y,
                                             p.pose.orientation.z, p.pose.orientation.w)]
                          for p in msg.particles]).reshape(-1, 3)
        weights = np.array([p.weight for p in msg.particles], dtype=float)
        clouds.append({"t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                       "poses": poses, "weights": weights})
    return clouds


def analyze(clouds: List[Dict[str, Any]], gt_nodes: Optional[np.ndarray],
            args: argparse.Namespace) -> List[Dict[str, Any]]:
    rows = []
    for cloud in clouds:
        modes = cluster_particles(cloud["poses"], cloud["weights"], args.cell, np.radians(args.cell_yaw_deg))
        major = [m for m in modes if m["weight"] >= args.min_weight]
        row: Dict[str, Any] = {"t": cloud["t"], "n_particles": int(cloud["poses"].shape[0]),
                               "n_modes": len(major), "top_weight": major[0]["weight"] if major else 0.0}
        if gt_nodes is not None and modes:
            gt, valid = interpolate_nodes(gt_nodes, np.array([cloud["t"]]), stationary_dist=STATIONARY_DIST_M)
            if valid[0]:
                means = np.array([m["mean"] for m in modes])
                pos_err, yaw_err = lm.pose_errors(means, np.repeat(gt, len(modes), axis=0))
                correct = (pos_err <= args.correct_position) & (np.abs(yaw_err) <= np.radians(args.correct_yaw_deg))
                row.update({
                    "top_position_err_m": float(pos_err[0]),
                    "top_yaw_err_deg": float(np.degrees(abs(yaw_err[0]))),
                    "correct_weight": float(sum(m["weight"] for m, ok in zip(modes, correct) if ok)),
                })
        rows.append(row)
    return rows


def spans(times: np.ndarray, flags: np.ndarray, max_gap: float) -> List[List[float]]:
    """flags が真の時刻を、max_gap 以内の間隔でつないだ区間にする。"""
    result: List[List[float]] = []
    for t in times[flags]:
        if result and t - result[-1][1] <= max_gap:
            result[-1][1] = t
        else:
            result.append([t, t])
    return result


def format_summary(rows: List[Dict[str, Any]], t0: float, args: argparse.Namespace) -> str:
    times = np.array([r["t"] for r in rows])
    gap = args.span_gap
    lines = ["# AMCL の粒子の仮説", "",
             f"- 粒子の配信: {len(rows)} 回 / 仮説とみなす重み {args.min_weight:g} 以上 / "
             f"格子 {args.cell:g} m・{args.cell_yaw_deg:g} deg",
             f"- 正しい仮説: 真値から {args.correct_position:g} m・{args.correct_yaw_deg:g} deg 以内", ""]

    def section(title: str, flags: np.ndarray) -> None:
        lines.extend([f"## {title}", ""])
        found = spans(times, flags, gap)
        if not found:
            lines.extend(["- なし", ""])
            return
        lines.extend(f"- {a - t0:.1f}–{b - t0:.1f} s" for a, b in found)
        lines.append("")

    section("仮説が 2 つ以上に割れた時間", np.array([r["n_modes"] >= 2 for r in rows]))
    if any("correct_weight" in r for r in rows):
        cw = np.array([r.get("correct_weight", np.nan) for r in rows])
        section("正しい仮説が負けた時間 (重み 0.5 未満)", (cw < 0.5) & (cw > 0.0))
        section("正しい仮説が消えた時間 (重み 0)", cw == 0.0)
    lines.append("時刻は記録の先頭からの経過秒。")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="AMCL の粒子を仮説に分けて時系列で出す")
    parser.add_argument("bag_path", help="/particle_cloud を記録した bag")
    parser.add_argument("--gt-dir", default=None, help="真値 (pose_graph.json) の SLAM 出力")
    parser.add_argument("--topic", default="/particle_cloud")
    parser.add_argument("--start", type=float, default=0.0, help="解析の開始 (記録の先頭からの経過秒)")
    parser.add_argument("--end", type=float, default=0.0, help="解析の終了 (0 で末尾まで)")
    parser.add_argument("--cell", type=float, default=0.5, help="仮説を分ける格子の大きさ [m]")
    parser.add_argument("--cell-yaw-deg", type=float, default=15.0, help="仮説を分ける格子の向きの大きさ [deg]")
    parser.add_argument("--min-weight", type=float, default=0.05, help="仮説として数える重みの下限")
    parser.add_argument("--correct-position", type=float, default=2.5, help="正しい仮説とみなす位置の差 [m]")
    parser.add_argument("--correct-yaw-deg", type=float, default=20.0, help="正しい仮説とみなす向きの差 [deg]")
    parser.add_argument("--span-gap", type=float, default=2.0, help="区間をつなぐ最大の間隔 [s]")
    parser.add_argument("-o", "--output", default=None, help="CSV の出力先 (既定: <bag>/../particle_modes.csv)")
    args = parser.parse_args()

    clouds = read_clouds(args.bag_path, args.topic)
    if not clouds:
        raise SystemExit("エラー: 粒子の配信がありません。")
    t0 = clouds[0]["t"]
    t_end = t0 + args.end if args.end > 0.0 else np.inf
    clouds = [c for c in clouds if t0 + args.start <= c["t"] < t_end]
    gt_nodes = load_slam_output(args.gt_dir)[0] if args.gt_dir else None
    rows = analyze(clouds, gt_nodes, args)

    output = args.output or os.path.join(os.path.dirname(os.path.abspath(args.bag_path)), "particle_modes.csv")
    fields = ["t", "n_particles", "n_modes", "top_weight", "top_position_err_m", "top_yaw_err_deg",
              "correct_weight"]
    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, "t": f"{row['t'] - t0:.3f}"})
    text = format_summary(rows, t0, args)
    md_path = os.path.splitext(output)[0] + ".md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    print(f"保存: {output}, {md_path}")


if __name__ == "__main__":
    main()
