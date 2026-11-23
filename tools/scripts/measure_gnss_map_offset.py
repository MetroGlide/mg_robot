#!/usr/bin/env python3
"""
measure_gnss_map_offset.py

運用の地図 (占有格子) と GNSS (gnss_transform.yaml でブリッジと同じに変換した座標) の、場所ごとのずれを測る。

measure: 走行の rosbag のスキャンを、基準の軌跡で数 m の局所の点群にまとめ、地図に照合する。
  照合した姿勢 (地図が言う位置) − 基準の姿勢 (GNSS が言う位置) を、場所ごとのずれとして出す。
  基準の軌跡は次のどちらか。
  - --slam-dir: slam_gnss_2d の出力 (UTM に揃った軌跡)。RTK のない区間は SLAM の形で前後の RTK からつなぐ
  - --odom-fit: /odom の軌跡を RTK の測位に剛体で当てはめたもの (短い走行向け)
analyze: measure の出力をまとめ、ずれを「全体の剛体」「グループ (waypoint-tool) ごとの剛体」に分けて当てはめ、
  残りを区画ごとに集計する。決めるのと確かめるのに、道のりの区切りを交互に振り分けた別の側を使う。
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader, read_static_transform  # noqa: E402
from tools.common.cli import add_output_args, resolve_output_path  # noqa: E402
from tools.common.geo import lat_lon_to_utm, quaternion_to_yaw  # noqa: E402
from tools.common.map import load_gnss_transform, load_map_info  # noqa: E402
from tools.common.map_align import (  # noqa: E402
    apply_offsets,
    correction_motion,
    assign_group,
    fit_offsets,
    inverse_pose,
    load_wptroj_groups,
    match_cloud,
    select_map,
    transform_points,
    voxel_downsample,
)
from tools.common.pose_graph import antenna_positions, interpolate_nodes, load_slam_output  # noqa: E402
from tools.common.scan_map import OccupancyMap  # noqa: E402

# RTK の測位とみなす hAcc の上限 [m]
RTK_HACC_M = 0.1
# スキャンと GNSS の時刻を対応づける許容 [s]
GNSS_MATCH_SEC = 0.6


# ---------------------------------------------------------------- measure

def read_bag(bag_path: str, scan_topic: str, need_odom: bool) -> Dict[str, Any]:
    """スキャン (時刻はヘッダ)、NavPVT (時刻は受信)、/odom を 1 回の走査で読む。"""
    topics = [scan_topic, "/navpvt"] + (["/odom"] if need_odom else [])
    reader = open_reader(bag_path, topics=topics)
    des = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    scan_t, scans, gnss, odom = [], [], [], []
    geometry = None
    while reader.has_next():
        name, raw, stamp = reader.read_next()
        msg = des.deserialize(name, raw)
        if name == scan_topic:
            if geometry is None:
                geometry = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            scan_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            scans.append(np.asarray(msg.ranges, dtype=np.float32))
        elif name == "/navpvt":
            if not (msg.flags & 0x01) or abs(msg.lat) < 1e6:
                continue
            gnss.append([stamp * 1e-9, msg.lat * 1e-7, msg.lon * 1e-7, (msg.flags >> 6) & 0x03, msg.h_acc * 1e-3])
        else:
            q = msg.pose.pose.orientation
            odom.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.pose.pose.position.x,
                         msg.pose.pose.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)])
    if geometry is None:
        raise SystemExit(f"エラー: {bag_path} に {scan_topic} がありません。")
    if not gnss:
        raise SystemExit(f"エラー: {bag_path} に有効な /navpvt がありません。")
    return {"scan_t": np.array(scan_t), "scans": scans, "geometry": geometry, "gnss": np.array(gnss),
            "odom": np.array(sorted(odom)) if odom else np.zeros((0, 4))}


def gnss_to_map(gnss: np.ndarray, map_tf: Tuple[float, float, float], zone: int) -> np.ndarray:
    """NavPVT [t, lat, lon, carr, hAcc] を、ブリッジと同じ変換で地図の座標 [t, x, y, carr, hAcc] にする。"""
    tx, ty, yaw = map_tf
    utm = np.array([lat_lon_to_utm(lat, lon, zone=zone) for lat, lon in gnss[:, 1:3]])
    xy = apply_offsets(utm, np.array([tx, ty]), yaw)
    return np.column_stack([gnss[:, 0], xy, gnss[:, 3:5]])


def slam_reference(slam_dir: str, map_tf: Tuple[float, float, float]) -> Tuple[np.ndarray, Dict[str, Any]]:
    """SLAM のノード列を、地図の座標の基準の姿勢列 [t, x, y, yaw] にする (SLAM の座標 = UTM − アンカー)。"""
    nodes, transform, _ = load_slam_output(slam_dir)
    anchor = transform["anchor_utm"]
    utm = nodes[:, 1:3] + np.array([anchor["easting"], anchor["northing"]])
    tx, ty, yaw = map_tf
    xy = apply_offsets(utm, np.array([tx, ty]), yaw)
    ref = np.column_stack([nodes[:, 0], xy, lm.wrap_angle(nodes[:, 3] + yaw)])
    return ref, {"source": "slam", "slam_dir": slam_dir}


def odom_fit_reference(odom: np.ndarray, gnss_map: np.ndarray, lever: Tuple[float, float]
                       ) -> Tuple[np.ndarray, Dict[str, Any]]:
    """/odom の軌跡を、RTK の測位 (アンテナの位置) に剛体で当てはめた基準の姿勢列。"""
    rtk = gnss_map[gnss_map[:, 4] < RTK_HACC_M]
    if rtk.shape[0] < 10 or odom.shape[0] == 0:
        raise SystemExit("エラー: --odom-fit には /odom と 10 点以上の RTK の測位が要ります。")
    nodes = odom.copy()
    poses, valid = interpolate_nodes(nodes, rtk[:, 0], stationary_dist=0.05)
    ant = antenna_positions(poses[valid], lever)
    t, theta = fit_offsets(ant, rtk[valid, 1:3], min_extent=0.0)
    xy = apply_offsets(odom[:, 1:3], t, theta)
    ref = np.column_stack([odom[:, 0], xy, lm.wrap_angle(odom[:, 3] + theta)])
    residual = apply_offsets(ant, t, theta) - rtk[valid, 1:3]
    return ref, {"source": "odom_fit", "n_rtk": int(np.count_nonzero(valid)),
                 "fit_rms_m": float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))}


def local_refit(pose: np.ndarray, t: float, ref_t: np.ndarray, ref_poses: np.ndarray, rtk: np.ndarray,
                lever: Tuple[float, float], window_sec: float
                ) -> Tuple[Optional[np.ndarray], Optional[float]]:
    """時刻 t の前後 window_sec [s] の RTK の測位に、基準の軌跡を剛体で当て直した姿勢と、当てはめの残差 RMS。

    RTK の点が広がっていなければ並進だけを直す。点が 3 つ未満なら (None, None)。
    """
    near = rtk[np.abs(rtk[:, 0] - t) <= window_sec]
    if near.shape[0] < 3:
        return None, None
    nodes = np.column_stack([ref_t, ref_poses])
    poses, valid = interpolate_nodes(nodes, near[:, 0], stationary_dist=0.05)
    if np.count_nonzero(valid) < 3:
        return None, None
    ant = antenna_positions(poses[valid], lever)
    shift, theta = fit_offsets(ant, near[valid, 1:3], min_extent=5.0)
    xy = apply_offsets(pose[None, :2], shift, theta)[0]
    residual = apply_offsets(ant, shift, theta) - near[valid, 1:3]
    return (np.array([xy[0], xy[1], lm.wrap_angle(pose[2] + theta)]),
            float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))


def scan_points(ranges: np.ndarray, geometry: Tuple[float, float, float, float], max_range: float,
                beam_step: int) -> np.ndarray:
    angle_min, inc, rmin, rmax = geometry
    idx = np.arange(0, ranges.size, beam_step)
    r = ranges[idx]
    ok = np.isfinite(r) & (r > rmin) & (r < min(rmax, max_range))
    ang = angle_min + inc * idx[ok]
    return np.column_stack([r[ok] * np.cos(ang), r[ok] * np.sin(ang)])


def measure(args: argparse.Namespace) -> None:
    map_tf, zone = load_gnss_transform(args.gnss_transform)
    data = read_bag(args.bag, args.scan_topic, args.odom_fit)
    gnss_map = gnss_to_map(data["gnss"], map_tf, zone)
    lever = tuple(args.lever_arm)
    if args.slam_dir:
        ref_nodes, ref_info = slam_reference(args.slam_dir, map_tf)
    else:
        ref_nodes, ref_info = odom_fit_reference(data["odom"], gnss_map, lever)
    print(f"基準: {ref_info}")

    lidar_t = read_static_transform(args.bag, args.base_frame, args.lidar_frame)
    lidar_pose = np.array([lidar_t[0, 3], lidar_t[1, 3], math.atan2(lidar_t[1, 0], lidar_t[0, 0])])
    # 上下が逆の取り付け (z 軸が下向き) では、ビームの角の向きが逆になる
    flip = lidar_t[2, 2] < 0

    maps = [OccupancyMap(path) for path in args.maps]
    names = [os.path.splitext(os.path.basename(p))[0] for p in args.maps]

    scan_t = data["scan_t"]
    scan_ref, valid = interpolate_nodes(ref_nodes, scan_t, stationary_dist=0.05)
    keep = np.nonzero(valid)[0]
    scan_t, scan_ref = scan_t[keep], scan_ref[keep]
    scans = [data["scans"][i] for i in keep]
    path = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(scan_ref[:, 0]), np.diff(scan_ref[:, 1])))])

    # 局所の点群に使うスキャン: 移動 scan_spacing [m] か回転 0.1 rad ごと
    chosen = [0]
    for i in range(1, len(scans)):
        j = chosen[-1]
        if path[i] - path[j] >= args.scan_spacing or abs(lm.wrap_angle(scan_ref[i, 2] - scan_ref[j, 2])) >= 0.1:
            chosen.append(i)
    chosen = np.array(chosen)
    lidar_pts = {}

    # RTK の測位の、基準の軌跡の上での位置 (道のり)
    rtk = gnss_map[gnss_map[:, 4] < RTK_HACC_M]
    rtk_path = np.interp(rtk[:, 0], scan_t, path, left=np.nan, right=np.nan) if rtk.size else np.zeros(0)
    rtk_path = rtk_path[np.isfinite(rtk_path)]

    samples = []
    next_path = 0.0
    rng = np.random.default_rng(0)
    for i in range(len(scans)):
        if path[i] < next_path:
            continue
        next_path = path[i] + args.step_m
        p0 = scan_ref[i]
        ref_fit_rms = None
        if args.odom_fit:
            # オドメトリは向きが流れるので、前後の RTK の測位だけで当て直す (点群の形は相対の動きだけで決まる)
            p0, ref_fit_rms = local_refit(p0, scan_t[i], scan_t, scan_ref, rtk, lever, args.refit_window_sec)
            if p0 is None:
                continue
        grid_idx = select_map(maps, p0[:2])
        if grid_idx is None:
            continue
        window = chosen[np.abs(path[chosen] - path[i]) <= args.window_m]
        if window.size < 3:
            continue
        inv0 = inverse_pose(p0)
        parts = []
        for k in window:
            if k not in lidar_pts:
                pts = scan_points(scans[k], data["geometry"], args.max_range, args.beam_step)
                if flip:
                    pts[:, 1] *= -1.0
                lidar_pts[k] = pts
            sensor = lm.compose(scan_ref[k][None, :], lidar_pose[None, :])[0]
            rel = lm.compose(inv0[None, :], sensor[None, :])[0]
            parts.append(transform_points(rel, lidar_pts[k]))
        cloud = voxel_downsample(np.concatenate(parts), args.voxel, args.max_points, rng)
        if cloud.shape[0] < 50:
            continue
        result = match_cloud(maps[grid_idx], cloud, p0, search_xy=args.search_xy,
                             search_yaw=math.radians(args.search_yaw_deg))
        offset = result.pose - p0
        offset[2] = lm.wrap_angle(offset[2])

        # 同じ時刻の GNSS (アンテナの位置) と基準の差。基準の質の目安
        g = int(np.argmin(np.abs(gnss_map[:, 0] - scan_t[i])))
        gnss_info = None
        if abs(gnss_map[g, 0] - scan_t[i]) <= GNSS_MATCH_SEC:
            ant = antenna_positions(p0[None, :], lever)[0]
            gnss_info = {"carr": int(gnss_map[g, 3]), "h_acc": float(gnss_map[g, 4]),
                         "ref_minus_gnss": [float(ant[0] - gnss_map[g, 1]), float(ant[1] - gnss_map[g, 2])]}
        dist_rtk = float(np.min(np.abs(rtk_path - path[i]))) if rtk_path.size else None
        samples.append({
            "t": float(scan_t[i]), "t_rel": float(scan_t[i] - scan_t[0]), "path_m": float(path[i]),
            "map": names[grid_idx], "ref": p0.tolist(), "matched": result.pose.tolist(),
            "offset": [float(offset[0]), float(offset[1]), float(offset[2])],
            "score": result.score, "inlier_ratio": result.inlier_ratio, "ambiguity": result.ambiguity,
            "cov": np.nan_to_num(result.cov, nan=-1.0).tolist(), "n_points": result.n_points,
            "gnss": gnss_info, "path_to_rtk_m": dist_rtk, "ref_fit_rms_m": ref_fit_rms,
        })
        print(f"{scan_t[i] - scan_t[0]:7.1f} s {names[grid_idx]:18s} off=({offset[0]:+.2f},{offset[1]:+.2f}) m "
              f"{math.degrees(offset[2]):+.2f} deg inlier={result.inlier_ratio:.2f} amb={result.ambiguity:.2f} "
              f"rtk_dist={dist_rtk if dist_rtk is None else round(dist_rtk, 1)}")

    out = resolve_output_path(args.bag, "gnss_map_offset.json", args.output, args.output_to_bag_dir,
                              args.output_dir)
    report = {"bag": args.bag, "gnss_transform": args.gnss_transform, "maps": args.maps, "reference": ref_info,
              "params": {k: v for k, v in vars(args).items() if k not in ("func",)}, "samples": samples}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    print(f"保存: {out} ({len(samples)} 点)")


# ---------------------------------------------------------------- analyze

def load_samples(paths: List[str], block_m: float) -> Tuple[List[Dict[str, Any]], List[str]]:
    """measure の出力を読み、道のりで block_m [m] ごとの区切りを交互に A と B に分ける。

    走行は区画を順に通るので、時間で前後半に分けるとグループが重ならない。交互の区切りなら、
    どのグループにも両方が入り、隣の区切りで確かめることになる。
    """
    samples, maps = [], []
    for run, path in enumerate(paths):
        with open(path, "r", encoding="utf-8") as f:
            report = json.load(f)
        name = os.path.basename(os.path.dirname(os.path.abspath(path))) or str(run)
        for s in report["samples"]:
            s["run"] = name
            s["half"] = "A" if int(s["path_m"] // block_m) % 2 == 0 else "B"
            samples.append(s)
        maps.extend(m for m in report["maps"] if m not in maps)
    return samples, maps


def sample_ok(s: Dict[str, Any], args: argparse.Namespace) -> bool:
    if s["inlier_ratio"] < args.min_inlier or s["ambiguity"] > args.max_ambiguity:
        return False
    cov = np.array(s["cov"])
    if np.any(cov[:2, :2] < -0.5):
        return False
    eig = np.linalg.eigvalsh(cov[:2, :2])
    return bool(eig[0] > 0 and math.sqrt(eig[1] / eig[0]) <= args.max_degeneracy)


def rms(v: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum(v ** 2, axis=1)))) if v.size else float("nan")


def fit_model(kind: str, ref: np.ndarray, mat: np.ndarray, groups: List[Optional[str]]) -> Dict[str, Any]:
    if kind == "none":
        return {}
    if kind == "global":
        return {"*": fit_offsets(ref, mat)}
    model = {}
    for name in set(groups):
        idx = [i for i, g in enumerate(groups) if g == name]
        if len(idx) >= 3:
            model[name] = fit_offsets(ref[idx], mat[idx])
    return model


def predict(model: Dict[str, Any], kind: str, ref: np.ndarray, groups: List[Optional[str]]) -> np.ndarray:
    if kind == "none":
        return ref.copy()
    out = np.full(ref.shape, np.nan)
    for i, g in enumerate(groups):
        key = "*" if kind == "global" else g
        if key in model:
            t, theta = model[key]
            out[i] = apply_offsets(ref[i:i + 1], t, theta)[0]
    return out


def draw_background(ax, map_yamls: List[str], step: int = 8) -> None:
    for path in map_yamls:
        pixels, origin, res, _ = load_map_info(path)
        if pixels.ndim == 3:
            pixels = pixels[..., 0]
        img = pixels[::step, ::step]
        h, w = pixels.shape
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, alpha=0.5,
                  extent=[origin[0], origin[0] + w * res, origin[1], origin[1] + h * res], origin="upper")


def analyze(args: argparse.Namespace) -> None:
    samples, maps = load_samples(args.inputs, args.block_m)
    groups_def, regions = load_wptroj_groups(args.wptroj) if args.wptroj else ([], [])
    good = [s for s in samples if sample_ok(s, args)]
    if not good:
        raise SystemExit("エラー: 条件を満たす照合がありません。")
    ref = np.array([s["ref"][:2] for s in good])
    mat = np.array([s["matched"][:2] for s in good])
    off = mat - ref
    dyaw = np.degrees([s["offset"][2] for s in good])
    group = assign_group(groups_def, ref) if groups_def else [None] * len(good)
    group = [g or "(none)" for g in group]

    def region_of(xy: np.ndarray) -> str:
        names = [n for n, (x, y, w, h) in regions if x <= xy[0] <= x + w and y <= xy[1] <= y + h]
        return "/".join(names) if names else "(outside)"

    region = [region_of(p) for p in ref]
    extrap = np.array([(s["path_to_rtk_m"] is None) or (s["path_to_rtk_m"] > args.rtk_near_m) for s in good])

    lines = ["# 地図と GNSS のずれの計測", ""]
    lines += [f"- 入力: {', '.join(args.inputs)}",
              f"- 照合 {len(samples)} 点のうち、条件 (inlier ≥ {args.min_inlier}、曖昧さ ≤ {args.max_ambiguity}、"
              f"縮退 ≤ {args.max_degeneracy}) を満たす {len(good)} 点を使う",
              f"- ずれ = 照合した姿勢 (地図) − 基準の姿勢 (GNSS)。RTK から道のりで {args.rtk_near_m} m より離れた点は"
              "「外挿」(基準が SLAM の形でつないだ位置)", ""]

    # 基準の質: 同じ時刻の GNSS との差
    lines += ["## 基準の質 (基準のアンテナ位置 − GNSS、区画ごと)", "",
              "| 区画 | RTK (hAcc<0.1) の N | RMS [m] | Float などの N | RMS [m] | Float の hAcc 中央値 [m] |",
              "| :--- | ---: | ---: | ---: | ---: | ---: |"]
    by_region = defaultdict(lambda: {"rtk": [], "other": [], "hacc": []})
    for s, r in zip(good, region):
        if s["gnss"] is None:
            continue
        key = "rtk" if s["gnss"]["h_acc"] < RTK_HACC_M else "other"
        by_region[r][key].append(s["gnss"]["ref_minus_gnss"])
        if key == "other":
            by_region[r]["hacc"].append(s["gnss"]["h_acc"])
    for r in sorted(by_region):
        d = by_region[r]
        rr, oo = np.array(d["rtk"]).reshape(-1, 2), np.array(d["other"]).reshape(-1, 2)
        hacc = f"{np.median(d['hacc']):.2f}" if d["hacc"] else "-"
        lines.append(f"| {r} | {len(rr)} | {rms(rr):.2f} | {len(oo)} | {rms(oo):.2f} | {hacc} |")
    lines.append("")

    # 生のずれ: 区画とグループごと
    lines += ["## ずれ (当てはめ前)", "",
              "| グループ | 区画 | N | 外挿の N | 平均 dx [m] | 平均 dy [m] | 大きさの中央値 [m] | 最大 [m] | 平均 dyaw [deg] |",
              "| :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    cells = defaultdict(list)
    for i in range(len(good)):
        cells[(group[i], region[i])].append(i)
    for (g, r), idx in sorted(cells.items()):
        d = off[idx]
        mag = np.hypot(d[:, 0], d[:, 1])
        lines.append(f"| {g} | {r} | {len(idx)} | {int(extrap[idx].sum())} | {d[:, 0].mean():+.2f} | "
                     f"{d[:, 1].mean():+.2f} | {np.median(mag):.2f} | {mag.max():.2f} | {np.mean(dyaw[idx]):+.2f} |")
    lines.append("")

    # 当てはめ: 時間で分けた半分で決め、残りの半分で確かめる
    lines += [f"## 当てはめ (道のり {args.block_m:g} m ごとの交互の区切りの片方で決め、もう片方で確かめる)", "",
              "| モデル | 決めた半分 | 決めた側の RMS [m] | 確かめた側の RMS [m] | 確かめた側の外挿の RMS [m] |",
              "| :--- | :--- | ---: | ---: | ---: |"]
    halves = np.array([s["half"] for s in good])
    for kind in ("none", "global", "group"):
        for fit_half in ("A", "B"):
            tr = np.nonzero(halves == fit_half)[0]
            te = np.nonzero(halves != fit_half)[0]
            model = fit_model(kind, ref[tr], mat[tr], [group[i] for i in tr])
            pr_tr = predict(model, kind, ref[tr], [group[i] for i in tr])
            pr_te = predict(model, kind, ref[te], [group[i] for i in te])
            ok_te = np.all(np.isfinite(pr_te), axis=1)
            res_tr = (mat[tr] - pr_tr)[np.all(np.isfinite(pr_tr), axis=1)]
            res_te = (mat[te] - pr_te)[ok_te]
            res_ex = (mat[te] - pr_te)[ok_te & extrap[te]]
            lines.append(f"| {kind} | {fit_half} | {rms(res_tr):.2f} | {rms(res_te):.2f} "
                         f"({int(ok_te.sum())}/{len(te)} 点) | {rms(res_ex):.2f} |")
    lines.append("")

    # 全点で当てはめたモデル (報告と図に使う)
    full_global = fit_model("global", ref, mat, group)
    full_group = fit_model("group", ref, mat, group)
    t, theta = full_global["*"]
    lines += ["## 全点での当てはめ", "",
              f"- 全体の剛体: 地図 = R({math.degrees(theta):+.3f} deg) · GNSS + ({t[0]:+.2f}, {t[1]:+.2f}) m "
              "(回転は地図の原点 = アンカーの周り)", "",
              "| グループ | N | 中心での平均のずれ dx, dy [m] | 当てはめの回転 [deg] | 当てはめ後の RMS [m] | "
              "手のずらし量 x, y [m], yaw [deg] | 直す動き dx, dy [m], dyaw [deg] |",
              "| :--- | ---: | :--- | ---: | ---: | :--- | :--- |"]
    manual = {g.name: g.offset for g in groups_def}
    for name in sorted(full_group):
        idx = [i for i, g in enumerate(group) if g == name]
        tg, thg = full_group[name]
        pred = apply_offsets(ref[idx], tg, thg)
        center = ref[idx].mean(axis=0)
        shift = apply_offsets(center[None, :], tg, thg)[0] - center
        m = manual.get(name)
        mtxt = f"{m[0]:+.2f}, {m[1]:+.2f}, {math.degrees(m[2]):+.2f}" if m else "-"
        cx, cy, cyaw = correction_motion(tg, thg)
        lines.append(f"| {name} | {len(idx)} | {shift[0]:+.2f}, {shift[1]:+.2f} | {math.degrees(thg):+.3f} | "
                     f"{rms(mat[idx] - pred):.2f} | {mtxt} | {cx:+.3f}, {cy:+.3f}, {math.degrees(cyaw):+.3f} |")
    lines += ["", "- 直す動き: グループを地図の原点の周りに R(dyaw)·p + (dx, dy) で動かすと、当てはめたずれが消える。"
              "`move_wptroj_groups.py --move <グループ> dx dy dyaw` にそのまま渡せる。", ""]

    os.makedirs(args.output_dir, exist_ok=True)
    md = os.path.join(args.output_dir, "gnss_map_offset_report.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    resid = mat - predict(full_group, "group", ref, group)
    fig, axes = plt.subplots(1, 2, figsize=(22, 9))
    names_sorted = sorted(set(group))
    colors = {n: plt.cm.tab10(i % 10) for i, n in enumerate(names_sorted)}
    panels = ((axes[0], off, "offset (map - GNSS)"), (axes[1], resid, "residual after per-group rigid fit"))
    for ax, vec, title in panels:
        draw_background(ax, maps)
        for n in names_sorted:
            idx = np.array([i for i, g in enumerate(group) if g == n])
            ok = np.all(np.isfinite(vec[idx]), axis=1)
            idx = idx[ok]
            for ex, face in ((False, colors[n]), (True, "none")):
                sel = idx[extrap[idx] == ex]
                if sel.size == 0:
                    continue
                ax.scatter(ref[sel, 0], ref[sel, 1], s=12, facecolors=face, edgecolors=colors[n],
                           label=f"{n}{' (extrapolated)' if ex else ''}")
                ax.quiver(ref[sel, 0], ref[sel, 1], vec[sel, 0], vec[sel, 1], color=colors[n], angles="xy",
                          scale_units="xy", scale=1.0 / args.arrow_scale, width=0.002)
        for name, (x, y, w, h) in regions:
            ax.add_patch(plt.Rectangle((x, y), w, h, fill=False, ls="--", lw=0.8, ec="gray"))
            ax.text(x + 2, y + h - 6, name, fontsize=8, color="gray")
        ax.set_aspect("equal")
        ax.set_title(f"{title} (arrows x{args.arrow_scale:g})")
        ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    png = os.path.join(args.output_dir, "gnss_map_offset.png")
    fig.savefig(png, dpi=120)
    print("\n".join(lines))
    print(f"保存: {md}, {png}")


def main() -> None:
    parser = argparse.ArgumentParser(description="地図と GNSS の場所ごとのずれを測る")
    sub = parser.add_subparsers(dest="command", required=True)

    m = sub.add_parser("measure", help="rosbag のスキャンを地図に照合して、場所ごとのずれを出す")
    m.add_argument("bag")
    m.add_argument("--maps", nargs="+", required=True, help="照合する地図 (区画) の map.yaml")
    m.add_argument("--gnss-transform", required=True, help="地図の gnss_transform.yaml")
    ref = m.add_mutually_exclusive_group(required=True)
    ref.add_argument("--slam-dir", help="基準の軌跡にする slam_gnss_2d の出力 (pose_graph.json と gnss_transform.yaml)")
    ref.add_argument("--odom-fit", action="store_true", help="/odom を RTK の測位に当てはめた軌跡を基準にする")
    m.add_argument("--refit-window-sec", type=float, default=10.0,
                   help="--odom-fit で、各点の前後この秒数の RTK の測位で当て直す [s]")
    m.add_argument("--scan-topic", default="/scan_top_lidar")
    m.add_argument("--base-frame", default="base_link")
    m.add_argument("--lidar-frame", default="top_lrf_link")
    m.add_argument("--lever-arm", type=float, nargs=2, default=[0.26, -0.13], metavar=("X", "Y"),
                   help="base_link から gps_link までのオフセット [m]")
    m.add_argument("--step-m", type=float, default=2.0, help="照合する間隔 (道のり) [m]")
    m.add_argument("--window-m", type=float, default=8.0, help="局所の点群にまとめる前後の道のり [m]")
    m.add_argument("--scan-spacing", type=float, default=0.3, help="点群に使うスキャンの間隔 [m]")
    m.add_argument("--beam-step", type=int, default=3, help="ビームの間引き")
    m.add_argument("--max-range", type=float, default=25.0)
    m.add_argument("--voxel", type=float, default=0.1)
    m.add_argument("--max-points", type=int, default=3000)
    m.add_argument("--search-xy", type=float, default=4.0, help="照合で探す範囲 (±) [m]")
    m.add_argument("--search-yaw-deg", type=float, default=3.0, help="照合で探す向きの範囲 (±) [deg]")
    add_output_args(m, "gnss_map_offset.json")
    m.set_defaults(func=measure)

    a = sub.add_parser("analyze", help="measure の出力を、全体とグループごとの剛体に分けて当てはめる")
    a.add_argument("inputs", nargs="+", help="measure の出力 (json)")
    a.add_argument("--wptroj", help="waypoint-tool のプロジェクト (グループと区画を読む)")
    a.add_argument("--min-inlier", type=float, default=0.5)
    a.add_argument("--max-ambiguity", type=float, default=0.9)
    a.add_argument("--max-degeneracy", type=float, default=4.0, help="共分散の長軸と短軸の比 (標準偏差) の上限")
    a.add_argument("--rtk-near-m", type=float, default=30.0, help="これより RTK から離れた点を外挿とする [m]")
    a.add_argument("--block-m", type=float, default=20.0,
                   help="決めるのと確かめるのに交互に振り分ける、道のりの区切りの長さ [m]")
    a.add_argument("--arrow-scale", type=float, default=10.0)
    a.add_argument("--output-dir", default=os.path.join(REPO_ROOT, "tools", "data", "gnss_map"))
    a.set_defaults(func=analyze)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
