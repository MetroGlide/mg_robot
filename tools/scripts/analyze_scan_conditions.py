#!/usr/bin/env python3
"""
analyze_scan_conditions.py

2D LiDAR のスキャンが、測位用の地図に対してどれだけ「位置を決めやすい」観測かを時系列で測る。
角で観測が大きく変わる、植栽などで点が不安定、地図の線が太い、といった AMCL が弱い条件を数値にして、
実機の記録とシミュレーションを比べるのに使う。

使い方:
  analyze_scan_conditions.py <スキャンの bag> --pose-bag <推定の bag> --map <測位用 map.yaml>
      [--start S --end E] [--lidar-pose X,Y,YAW] [-o scan_conditions.csv]

- 姿勢は --pose-bag の /tf (map->odom と odom->base) から取る (例: ずれなかった再生評価の出力。
  地図に合った姿勢で、地図との照合を見るため)。同じ bag に両方があれば、同じパスを渡す
- --lidar-pose は base 座標系での LiDAR の姿勢 (x, y [m], yaw [rad])

スキャンごとの指標:
  n_valid          : 有効な点の数
  match_ratio      : 端点が地図の占有から --match-dist 以内にある割合
  thickness_m      : 地図の占有にかかった端点で、ビームの向きに沿った占有の連続の長さの中央値。
                     植栽のように点がまばらに散った帯も 1 本の帯として測るよう、--close-radius で
                     点の間を埋めた地図で測る (帯が太いほど、ビームの向きに位置が決まりにくい)
  instability_m    : 端点から、--compare-sec 前のスキャンの端点への最近傍距離の中央値 (地図座標。
                     同じ物を見ていて安定なら小さい。植栽で手前と奥の当たりが入れ替わると大きい)
  view_overlap     : 端点が当たった格子 (--view-cell) の、--view-sec 前のスキャンとの重なり (Jaccard)
  yaw_rate         : 姿勢の向きの変化の速さ [rad/s] (角の判定用)
"""

import argparse
import csv
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.map import load_map_info  # noqa: E402


class OccupancyMap:
    def __init__(self, map_yaml: str, close_radius: float, occupied_thresh: float = 0.65) -> None:
        pixels, origin, resolution, _ = load_map_info(map_yaml)
        if pixels.ndim == 3:
            pixels = pixels[..., 0]
        # 画像の行 0 が上 (y 最大) なので、行 0 が y 最小になるよう反転する
        self.occupied = ((255.0 - pixels.astype(float)) / 255.0 > occupied_thresh)[::-1]
        self.resolution = resolution
        self.origin = np.array(origin[:2])
        # 占有までの距離 [m]
        self.distance = ndimage.distance_transform_edt(~self.occupied) * resolution
        cells = int(round(close_radius / resolution))
        y, x = np.ogrid[-cells:cells + 1, -cells:cells + 1]
        self.closed = ndimage.binary_closing(self.occupied, structure=x * x + y * y <= cells * cells) \
            if cells > 0 else self.occupied

    def cells(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (row >= 0) & (row < self.occupied.shape[0]) & (col >= 0) & (col < self.occupied.shape[1])
        return row, col, inside

    def distance_at(self, xy: np.ndarray) -> np.ndarray:
        row, col, inside = self.cells(xy)
        result = np.full(xy.shape[0], np.inf)
        result[inside] = self.distance[row[inside], col[inside]]
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


def read_poses(bag_path: str, map_frame: str, odom_frame: str, base_frame: str) -> Tuple[np.ndarray, np.ndarray]:
    """/tf の map->odom と odom->base を合成した map->base の姿勢列 (時刻, [x, y, yaw])。"""
    reader = open_reader(bag_path, topics=["/tf"])
    deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    map_odom: List[List[float]] = []
    odom_base: List[List[float]] = []
    while reader.has_next():
        name, raw, _ = reader.read_next()
        for tf in deserializer.deserialize(name, raw).transforms:
            pair = (tf.header.frame_id, tf.child_frame_id)
            if pair not in ((map_frame, odom_frame), (odom_frame, base_frame)):
                continue
            q = tf.transform.rotation
            row = [tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9, tf.transform.translation.x,
                   tf.transform.translation.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)]
            (map_odom if pair[0] == map_frame else odom_base).append(row)
    if not map_odom or not odom_base:
        raise SystemExit(f"エラー: {bag_path} の /tf に {map_frame}->{odom_frame} と {odom_frame}->{base_frame} が必要です。")
    mo = np.array(sorted(map_odom))
    ob = np.array(sorted(odom_base))
    idx = lm.hold_index(mo[:, 0], ob[:, 0])
    valid = idx >= 0
    return ob[valid, 0], lm.compose(mo[idx[valid], 1:4], ob[valid, 1:4])


def read_scans(bag_path: str, topic: str, every: int) -> List[Dict]:
    reader = open_reader(bag_path, topics=[topic])
    deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    scans = []
    count = 0
    while reader.has_next():
        name, raw, _ = reader.read_next()
        count += 1
        if (count - 1) % every:
            continue
        msg = deserializer.deserialize(name, raw)
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        scans.append({"t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                      "ranges": ranges[valid], "angles": angles[valid]})
    return scans


def interpolate_pose(times: np.ndarray, poses: np.ndarray, t: float) -> Optional[np.ndarray]:
    i = int(np.searchsorted(times, t))
    if i <= 0 or i >= times.size or times[i] - times[i - 1] > 0.5:
        return None
    a = (t - times[i - 1]) / (times[i] - times[i - 1])
    yaw = poses[i - 1, 2] + a * lm.wrap_angle(poses[i, 2] - poses[i - 1, 2])
    xy = poses[i - 1, :2] + a * (poses[i, :2] - poses[i - 1, :2])
    return np.array([xy[0], xy[1], yaw])


def jaccard(a: set, b: set) -> float:
    return len(a & b) / len(a | b) if (a or b) else float("nan")


def analyze(scans: List[Dict], times: np.ndarray, poses: np.ndarray, grid: OccupancyMap,
            args: argparse.Namespace) -> List[Dict]:
    lidar = np.array(args.lidar_pose)
    history: List[Dict] = []
    rows = []
    for scan in scans:
        pose = interpolate_pose(times, poses, scan["t"])
        if pose is None:
            continue
        sensor = lm.compose(pose[None, :], lidar[None, :])[0]
        beam_yaw = sensor[2] + scan["angles"]
        direction = np.stack([np.cos(beam_yaw), np.sin(beam_yaw)], axis=1)
        points = sensor[:2] + scan["ranges"][:, None] * direction
        near = grid.distance_at(points) <= args.match_dist
        run = grid.run_length(points[near], direction[near], args.max_thickness)
        cells = set(map(tuple, np.floor(points / args.view_cell).astype(int)))
        entry = {"t": scan["t"], "pose": pose, "points": points, "cells": cells}

        def before(sec: float) -> Optional[Dict]:
            older = [h for h in history if h["t"] <= scan["t"] - sec]
            return older[-1] if older else None

        prev = before(args.compare_sec)
        view = before(args.view_sec)
        instability = float("nan")
        if prev is not None and points.shape[0] and prev["points"].shape[0]:
            distances, _ = cKDTree(prev["points"]).query(points[near] if np.any(near) else points)
            instability = float(np.median(distances))
        yaw_rate = float("nan")
        if prev is not None:
            yaw_rate = float(lm.wrap_angle(pose[2] - prev["pose"][2]) / (scan["t"] - prev["t"]))
        rows.append({
            "t": scan["t"], "x": pose[0], "y": pose[1], "yaw": pose[2],
            "n_valid": int(points.shape[0]),
            "match_ratio": float(np.mean(near)) if points.shape[0] else float("nan"),
            "thickness_m": float(np.nanmedian(run)) if np.any(np.isfinite(run)) else float("nan"),
            "instability_m": instability,
            "view_overlap": jaccard(cells, view["cells"]) if view is not None else float("nan"),
            "yaw_rate": yaw_rate,
        })
        history.append(entry)
        history = [h for h in history if h["t"] >= scan["t"] - max(args.compare_sec, args.view_sec) - 1.0]
    return rows


METRICS = ["n_valid", "match_ratio", "thickness_m", "instability_m", "view_overlap"]


def format_summary(rows: List[Dict], t0: float, args: argparse.Namespace) -> str:
    turning = np.array([abs(r["yaw_rate"]) >= args.turn_rate for r in rows])
    lines = ["# スキャンの観測条件", "",
             f"- 旋回中とみなす向きの変化の速さ: {args.turn_rate} rad/s。旋回中 {int(turning.sum())} / 全 {len(rows)} スキャン", "",
             "| 指標 | 直進 中央値 | 直進 p10〜p90 | 旋回中 中央値 | 旋回中 p10〜p90 |", "| :--- | ---: | ---: | ---: | ---: |"]
    for key in METRICS:
        values = np.array([r[key] for r in rows], dtype=float)
        cells = []
        for mask in (~turning, turning):
            v = values[mask & np.isfinite(values)]
            cells += ([f"{np.median(v):.3g}", f"{np.percentile(v, 10):.3g}〜{np.percentile(v, 90):.3g}"]
                      if v.size else ["-", "-"])
        lines.append(f"| {key} | " + " | ".join(cells) + " |")
    lines.append("")
    if turning.any():
        lines += ["## 旋回の区間 (記録の先頭からの経過秒)", ""]
        start = None
        for i, flag in enumerate(list(turning) + [False]):
            if flag and start is None:
                start = i
            elif not flag and start is not None:
                part = rows[start:i]
                lines.append(
                    f"- {part[0]['t'] - t0:.1f}〜{part[-1]['t'] - t0:.1f} s: 向きの変化 "
                    f"{math.degrees(lm.wrap_angle(part[-1]['yaw'] - part[0]['yaw'])):.0f}°、"
                    f"視界の重なりの最小 {np.nanmin([r['view_overlap'] for r in part] + [np.inf]):.2f}、"
                    f"不安定さの中央値 {np.nanmedian([r['instability_m'] for r in part]):.2f} m")
                start = None
        lines.append("")
    return "\n".join(lines)


def parse_pose(text: str) -> List[float]:
    values = [float(v) for v in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError(f"X,Y,YAW で指定してください: {text}")
    return values


def main() -> None:
    parser = argparse.ArgumentParser(description="2D LiDAR の観測が、地図に対して位置を決めやすいかを時系列で測る")
    parser.add_argument("scan_bag", help="スキャンを記録した bag")
    parser.add_argument("--pose-bag", required=True, help="/tf (map->odom、odom->base) を記録した bag")
    parser.add_argument("--map", required=True, help="測位用の地図 (map.yaml)")
    parser.add_argument("--scan-topic", default="/scan_top_lidar")
    parser.add_argument("--lidar-pose", type=parse_pose, default=[0.0, 0.0, 0.0],
                        help="base 座標系での LiDAR の姿勢 X,Y,YAW [m, m, rad]")
    parser.add_argument("--start", type=float, default=0.0, help="解析の開始 (スキャンの bag の先頭からの経過秒)")
    parser.add_argument("--end", type=float, default=0.0, help="解析の終了 (0 で末尾まで)")
    parser.add_argument("--every", type=int, default=5, help="この本数に 1 本のスキャンだけを解析する")
    parser.add_argument("--match-dist", type=float, default=0.2, help="地図に一致したとみなす占有までの距離 [m]")
    parser.add_argument("--max-thickness", type=float, default=3.0, help="線の太さを測る片側の上限 [m]")
    parser.add_argument("--close-radius", type=float, default=0.25,
                        help="線の太さを測る前に、占有の点の間を埋める半径 [m] (0 で埋めない)")
    parser.add_argument("--compare-sec", type=float, default=0.5, help="不安定さを比べる前のスキャンとの間隔 [s]")
    parser.add_argument("--view-sec", type=float, default=3.0, help="視界の重なりを比べる前のスキャンとの間隔 [s]")
    parser.add_argument("--view-cell", type=float, default=0.5, help="視界の重なりを数える格子 [m]")
    parser.add_argument("--turn-rate", type=float, default=0.15, help="旋回中とみなす向きの変化の速さ [rad/s]")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("-o", "--output", default=None, help="CSV の出力先 (既定: <pose-bag>/../scan_conditions.csv)")
    args = parser.parse_args()

    times, poses = read_poses(args.pose_bag, args.map_frame, args.odom_frame, args.base_frame)
    scans = read_scans(args.scan_bag, args.scan_topic, args.every)
    if not scans:
        raise SystemExit("エラー: スキャンがありません。")
    t0 = scans[0]["t"]
    t_end = t0 + args.end if args.end > 0.0 else np.inf
    scans = [s for s in scans if t0 + args.start <= s["t"] < t_end]
    rows = analyze(scans, times, poses, OccupancyMap(args.map, args.close_radius), args)
    if not rows:
        raise SystemExit("エラー: 姿勢と時刻の合うスキャンがありません。")

    output = args.output or os.path.join(os.path.dirname(os.path.abspath(args.pose_bag)), "scan_conditions.csv")
    fields = ["t", "x", "y", "yaw", "yaw_rate"] + METRICS
    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({**{k: row[k] for k in fields}, "t": f"{row['t'] - t0:.3f}"})
    text = format_summary(rows, t0, args)
    md_path = os.path.splitext(output)[0] + ".md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    print(f"保存: {output}, {md_path}")


if __name__ == "__main__":
    main()
