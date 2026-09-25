#!/usr/bin/env python3
"""
eval_localization.py

自己位置推定 (オドメトリ + AMCL + GNSS の EKF 融合) の出力を、rosbag から評価するツール。
実機のナビ走行の bag と、再生評価 (run_localization_variant.sh) の出力 bag のどちらにも使える。

推定値は /tf の map->odom と odom->base_footprint を合成して求める (Nav2 が使う姿勢と同じ)。
真値は slam_gnss_2d の pose_graph.json (--gt-dir) で、省略すると滑らかさなど真値なしの指標だけを出す。
別走行を評価する場合は、評価 bag の SLAM 出力を --gt-dir、地図を作った走行の SLAM 出力を
--map-gt-dir に渡す (両者を UTM 経由で同じ座標系に揃える)。

指標:
  - accuracy    : 位置・yaw の真値に対する誤差 (全体と時間窓ごと)
  - smoothness  : map->odom の 1 ステップあたりの飛び (AMCL / GNSS の補正が Nav2 に与える影響)
  - nees        : EKF の共分散が誤差に見合っているか (自由度 3 に近いほど一貫)
  - ekf_vs_odom : EKF の速度とホイールオドメトリ速度の遅れ・差
  - amcl        : AMCL の共分散・遅延・EKF との差
  - faults      : --faults 指定時の検知・復旧の指標と、故障のない区間での誤検知
"""

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402
from tools.common.cli import add_output_args, resolve_output_path  # noqa: E402
from tools.common.faults import Fault, load_faults  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.pose_graph import (  # noqa: E402
    apply_rigid,
    fit_rigid,
    interpolate_nodes,
    load_slam_output,
    shift_nodes_to_anchor,
)

# 3 自由度の x^2 分布の上側 5% 点 (NEES が一貫した推定なら 95% がこの値以下)
CHI2_95_DOF3 = 7.815
# SLAM のキーフレームは 0.5 m 動くごとに作られるので、これ未満なら止まっているとみなして補間する [m]
STATIONARY_DIST_M = 0.6
# bag の記録時刻と header.stamp の差がこれを超えるときは、別の時計 (シミュレーション時刻など) とみなす [s]
MAX_PLAUSIBLE_LATENCY_SEC = 5.0


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def pose_of(pose) -> Tuple[float, float, float]:
    q = pose.orientation
    return pose.position.x, pose.position.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)


class BagData:
    """評価に使うトピックの時系列。すべて header.stamp 基準の秒。"""

    def __init__(self) -> None:
        self.map_odom: List[List[float]] = []    # [t, x, y, yaw]
        self.odom_base: List[List[float]] = []   # [t, x, y, yaw]
        self.ekf: List[List[float]] = []         # [t, x, y, yaw, vx, wz] + cov36
        self.odom: List[List[float]] = []        # [t, vx, wz]
        self.amcl: List[List[float]] = []        # [t, record_t, x, y, yaw] + cov36
        self.gps_count = 0
        self.status: List[List[float]] = []      # [t, state]


def read_bag(bag_path: str, args: argparse.Namespace) -> BagData:
    reader = open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    deserializer = MessageDeserializer(type_map)
    wanted = {args.tf_topic, args.ekf_topic, args.odom_topic, args.amcl_topic,
              args.gps_topic, args.status_topic}
    data = BagData()
    while reader.has_next():
        topic, raw, record_ns = reader.read_next()
        if topic not in wanted or topic not in type_map:
            continue
        msg = deserializer.deserialize(topic, raw)
        if topic == args.tf_topic:
            for tf in msg.transforms:
                frame, child = tf.header.frame_id, tf.child_frame_id
                t = stamp_sec(tf.header.stamp)
                q = tf.transform.rotation
                row = [t, tf.transform.translation.x, tf.transform.translation.y,
                       quaternion_to_yaw(q.x, q.y, q.z, q.w)]
                if frame == args.map_frame and child == args.odom_frame:
                    data.map_odom.append(row)
                elif frame == args.odom_frame and child == args.base_frame:
                    data.odom_base.append(row)
        elif topic == args.ekf_topic:
            x, y, yaw = pose_of(msg.pose.pose)
            tw = msg.twist.twist
            data.ekf.append([stamp_sec(msg.header.stamp), x, y, yaw, tw.linear.x, tw.angular.z]
                            + list(msg.pose.covariance))
        elif topic == args.odom_topic:
            tw = msg.twist.twist
            data.odom.append([stamp_sec(msg.header.stamp), tw.linear.x, tw.angular.z])
        elif topic == args.amcl_topic:
            x, y, yaw = pose_of(msg.pose.pose)
            data.amcl.append([stamp_sec(msg.header.stamp), record_ns * 1e-9, x, y, yaw]
                             + list(msg.pose.covariance))
        elif topic == args.gps_topic:
            data.gps_count += 1
        elif topic == args.status_topic:
            data.status.append([stamp_sec(msg.header.stamp), float(msg.state)])
    return data


def as_array(rows: List[List[float]], cols: int) -> np.ndarray:
    arr = np.array(rows, dtype=float).reshape(-1, cols)
    return arr[np.argsort(arr[:, 0], kind="stable")] if arr.size else arr


def estimate_base_poses(data: BagData) -> Tuple[np.ndarray, np.ndarray]:
    """map->odom と odom->base を合成した map->base の姿勢列を返す。

    Returns:
      times: odom->base の時刻 (N,)
      poses: (N, 3) [x, y, yaw]
    """
    map_odom = as_array(data.map_odom, 4)
    odom_base = as_array(data.odom_base, 4)
    if map_odom.size == 0 or odom_base.size == 0:
        raise SystemExit("エラー: /tf に map->odom と odom->base の両方が必要です。")
    idx = lm.hold_index(map_odom[:, 0], odom_base[:, 0])
    valid = idx >= 0
    times = odom_base[valid, 0]
    poses = lm.compose(map_odom[idx[valid], 1:4], odom_base[valid, 1:4])
    return times, poses


def load_ground_truth(args: argparse.Namespace) -> Optional[np.ndarray]:
    if not args.gt_dir:
        return None
    nodes, transform, _ = load_slam_output(args.gt_dir)
    if args.map_gt_dir:
        _, map_transform, _ = load_slam_output(args.map_gt_dir)
        nodes = shift_nodes_to_anchor(nodes, transform, map_transform)
    return nodes


def window_stats(times: np.ndarray, pos_err: np.ndarray, t0: float, window: float) -> List[Dict[str, Any]]:
    rows = []
    if times.size == 0:
        return rows
    n_windows = int(np.ceil((times[-1] - t0) / window))
    for i in range(n_windows):
        mask = (times >= t0 + i * window) & (times < t0 + (i + 1) * window)
        if not np.any(mask):
            continue
        rows.append({
            "t_start_rel_s": float(i * window),
            "n": int(mask.sum()),
            "rms_m": float(np.sqrt(np.mean(pos_err[mask] ** 2))),
            "max_m": float(np.max(pos_err[mask])),
        })
    return rows


def evaluate_accuracy(
    times: np.ndarray, poses: np.ndarray, gt_nodes: np.ndarray, t0: float, window: float,
) -> Tuple[Dict[str, Any], np.ndarray, np.ndarray]:
    gt_poses, valid = interpolate_nodes(gt_nodes, times, stationary_dist=STATIONARY_DIST_M)
    if not np.any(valid):
        return {"n": 0}, times[:0], np.zeros(0)
    t = times[valid]
    pos_err, yaw_err = lm.pose_errors(poses[valid], gt_poses[valid])
    report = {
        "n": int(valid.sum()),
        "coverage": float(valid.mean()),
        "position_m": lm.summarize(pos_err),
        "yaw_rad": lm.summarize(np.abs(yaw_err)),
        "by_window": window_stats(t, pos_err, t0, window),
    }
    # 走行全体で SE(2) 整合した後の残差。別走行の評価では、真値と地図の座標系が GNSS の精度の分だけ
    # ずれるため、そのずれ (並進・回転) を除いた、地図に対する自己位置の整合性を見る
    est_xy = poses[valid][:, :2]
    gt_xy = gt_poses[valid][:, :2]
    fit = fit_rigid(est_xy, gt_xy)
    if fit is not None:
        translation, theta = fit
        aligned_err = np.hypot(*(apply_rigid(est_xy, translation, theta) - gt_xy).T)
        # 並進は座標原点まわりの回転の分を含んで大きく見えるため、軌跡の重心でのずれで表す
        centroid_offset = gt_xy.mean(axis=0) - est_xy.mean(axis=0)
        report["aligned"] = {
            "centroid_offset_m": [float(centroid_offset[0]), float(centroid_offset[1])],
            "rotation_deg": float(np.degrees(theta)),
            "position_m": lm.summarize(aligned_err),
        }
    return report, t, pos_err


def clip_time(arr: np.ndarray, t_from: float, t_to: float) -> np.ndarray:
    """先頭の列 (時刻) が [t_from, t_to) の行だけを残す。"""
    if arr.size == 0:
        return arr
    return arr[(arr[:, 0] >= t_from) & (arr[:, 0] < t_to)]


def evaluate_smoothness(map_odom: np.ndarray, odom_base: np.ndarray) -> Dict[str, Any]:
    # 各 map->odom 更新の時刻の odom->base (直前の値)
    idx = lm.hold_index(odom_base[:, 0], map_odom[1:, 0])
    valid = idx >= 0
    kept = np.concatenate([[True], valid])
    pos, yaw = lm.correction_jumps(map_odom[kept, 1:4], odom_base[idx[valid], 1:4])
    return {
        "n_map_odom": int(map_odom.shape[0]),
        "jump_position_m": lm.summarize(pos),
        "jump_yaw_rad": lm.summarize(yaw),
    }


def evaluate_nees(ekf: np.ndarray, gt_nodes: np.ndarray) -> Dict[str, Any]:
    gt_poses, valid = interpolate_nodes(gt_nodes, ekf[:, 0], stationary_dist=STATIONARY_DIST_M)
    if not np.any(valid):
        return {"n": 0}
    est = ekf[valid, 1:4]
    err = np.stack([
        est[:, 0] - gt_poses[valid, 0],
        est[:, 1] - gt_poses[valid, 1],
        lm.wrap_angle(est[:, 2] - gt_poses[valid, 2]),
    ], axis=1)
    values = lm.nees(err, ekf[valid, 6:42])
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"n": 0}
    return {
        "n": int(values.size),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "fraction_within_chi2_95": float(np.mean(values <= CHI2_95_DOF3)),
        "expected_mean": 3.0,
    }


def evaluate_ekf_vs_odom(ekf: np.ndarray, odom: np.ndarray) -> Dict[str, Any]:
    if ekf.shape[0] < 2 or odom.shape[0] < 2:
        return {"n": 0}
    lag_v, rmse_v = lm.best_lag(odom[:, 0], odom[:, 1], ekf[:, 0], ekf[:, 4])
    lag_w, rmse_w = lm.best_lag(odom[:, 0], odom[:, 2], ekf[:, 0], ekf[:, 5])
    v_i = np.interp(ekf[:, 0], odom[:, 0], odom[:, 1])
    w_i = np.interp(ekf[:, 0], odom[:, 0], odom[:, 2])
    return {
        "n": int(ekf.shape[0]),
        "v_diff_mps": lm.summarize(np.abs(ekf[:, 4] - v_i)),
        "w_diff_radps": lm.summarize(np.abs(ekf[:, 5] - w_i)),
        "v_lag_s": lag_v, "v_lag_rmse": rmse_v,
        "w_lag_s": lag_w, "w_lag_rmse": rmse_w,
        "ekf_yaw_sigma_rad": lm.summarize(np.sqrt(np.maximum(ekf[:, 6 + 35], 0.0))),
        "ekf_x_sigma_m": lm.summarize(np.sqrt(np.maximum(ekf[:, 6 + 0], 0.0))),
    }


def evaluate_amcl(amcl: np.ndarray, est_times: np.ndarray, est_poses: np.ndarray) -> Dict[str, Any]:
    if amcl.shape[0] == 0:
        return {"n": 0}
    cov = amcl[:, 5:41]
    report: Dict[str, Any] = {
        "n": int(amcl.shape[0]),
        "x_sigma_m": lm.summarize(np.sqrt(np.maximum(cov[:, 0], 0.0))),
        "yaw_sigma_rad": lm.summarize(np.sqrt(np.maximum(cov[:, 35], 0.0))),
    }
    latency = amcl[:, 1] - amcl[:, 0]
    if abs(float(np.median(latency))) < MAX_PLAUSIBLE_LATENCY_SEC:
        report["latency_s"] = lm.summarize(latency)
    else:
        report["latency_s"] = "n/a (bag 記録時刻と header.stamp が別の時計)"
    # AMCL が届いた時点の、その直前の推定姿勢との差 (補正の大きさ)
    if est_times.size:
        idx = lm.hold_index(est_times, amcl[:, 0])
        ok = idx >= 0
        if np.any(ok):
            pos, yaw = lm.pose_errors(amcl[ok, 2:5], est_poses[idx[ok]])
            report["diff_to_estimate_position_m"] = lm.summarize(pos)
            report["diff_to_estimate_yaw_rad"] = lm.summarize(np.abs(yaw))
    return report


def evaluate_faults(
    faults: List[Fault], t0: float, t_end: float, gt_times: np.ndarray, gt_pos_err: np.ndarray,
    status: np.ndarray, args: argparse.Namespace,
) -> Dict[str, Any]:
    results = []
    windows = []
    for i, fault in enumerate(faults):
        start = t0 + fault.start
        next_start = t0 + faults[i + 1].start if i + 1 < len(faults) else t_end
        entry: Dict[str, Any] = {"type": fault.type, "start_s": fault.start, "end_s": fault.end}
        if gt_times.size:
            entry.update(lm.recovery_metrics(
                gt_times, gt_pos_err, start, next_start, args.ok_threshold, args.hold_sec))
        if status.shape[0]:
            entry["detection_delay_s"] = lm.first_detection_delay(
                status[:, 0], status[:, 1], start, next_start)
        results.append(entry)
        windows.append((start - 1.0, t0 + fault.end + args.fault_margin))
    report: Dict[str, Any] = {"per_fault": results}
    if status.shape[0]:
        report["status"] = lm.count_episodes(status[:, 0], status[:, 1], windows)
    return report


def format_summary(report: Dict[str, Any]) -> str:
    def stat_row(name: str, s: Dict[str, Any], unit: str) -> str:
        if s.get("n", 0) == 0:
            return f"| {name} | 0 | - | - | - | - |"
        return (f"| {name} [{unit}] | {s['n']} | {s['median']:.3f} | {s['p95']:.3f} | "
                f"{s['p99']:.3f} | {s['max']:.3f} |")

    header = "| 指標 | N | 中央値 | p95 | p99 | 最大 |\n|---|---|---|---|---|---|"
    lines = ["# 自己位置推定 評価レポート", ""]
    meta = report["meta"]
    lines += [
        f"- bag: `{meta['bag']}`",
        f"- 推定姿勢の数: {meta['n_estimates']} / 時間: {meta['duration_s']:.1f} s",
        f"- 真値: {meta['gt_dir'] or 'なし'}"
        + (f" (地図: {meta['map_gt_dir']})" if meta.get("map_gt_dir") else ""),
        "",
    ]

    sm = report["smoothness"]
    lines += ["## 滑らかさ (map→odom の更新でロボットの姿勢が飛ぶ大きさ)", "", header,
              stat_row("位置の飛び", sm["jump_position_m"], "m"),
              stat_row("yaw の飛び", sm["jump_yaw_rad"], "rad"), ""]

    acc = report.get("accuracy")
    if acc and acc.get("n", 0) > 0:
        lines += [f"## 精度 (真値との差、比較できた割合 {acc['coverage'] * 100:.0f}%)", "", header,
                  stat_row("位置誤差", acc["position_m"], "m"),
                  stat_row("yaw 誤差", acc["yaw_rad"], "rad"), ""]
        if "aligned" in acc:
            al = acc["aligned"]
            offset = al["centroid_offset_m"]
            lines += [f"走行全体で SE(2) 整合すると (軌跡の重心でのずれ [{offset[0]:.2f}, {offset[1]:.2f}] m、"
                      f"回転 {al['rotation_deg']:.2f} deg): 位置誤差 中央値 {al['position_m']['median']:.3f} m / "
                      f"p95 {al['position_m']['p95']:.3f} m / 最大 {al['position_m']['max']:.3f} m。",
                      "整合量が大きいときは、推定ではなく真値と地図の座標系のずれ (GNSS の精度) が誤差の主因。", ""]
        lines += ["| 区間[s] | N | RMS[m] | 最大[m] |", "|---|---|---|---|"]
        for w in acc["by_window"]:
            lines.append(f"| {w['t_start_rel_s']:.0f}- | {w['n']} | {w['rms_m']:.3f} | {w['max_m']:.3f} |")
        lines.append("")

    nees = report.get("nees")
    if nees and nees.get("n", 0) > 0:
        lines += ["## NEES (自由度 3 の一貫した推定なら平均 3、95% が 7.8 以下)", "",
                  f"- 平均 {nees['mean']:.2f} / 中央値 {nees['median']:.2f} / "
                  f"7.8 以下の割合 {nees['fraction_within_chi2_95'] * 100:.0f}% (N={nees['n']})", ""]

    ev = report.get("ekf_vs_odom")
    if ev and ev.get("n", 0) > 0:
        lines += ["## EKF とホイールオドメトリの速度", "",
                  f"- 速度の差: 中央値 {ev['v_diff_mps']['median']:.3f} m/s / p95 {ev['v_diff_mps']['p95']:.3f}",
                  f"- 角速度の差: 中央値 {ev['w_diff_radps']['median']:.3f} rad/s / p95 {ev['w_diff_radps']['p95']:.3f}",
                  f"- 速度の遅れ {ev['v_lag_s']:.2f} s (RMSE {ev['v_lag_rmse']:.3f}) / "
                  f"角速度の遅れ {ev['w_lag_s']:.2f} s (RMSE {ev['w_lag_rmse']:.3f})",
                  f"- EKF の σ: x 中央値 {ev['ekf_x_sigma_m']['median']:.3f} m / "
                  f"yaw 中央値 {ev['ekf_yaw_sigma_rad']['median']:.3f} rad", ""]

    amcl = report.get("amcl")
    if amcl and amcl.get("n", 0) > 0:
        lines += ["## AMCL", "", f"- 件数 {amcl['n']} / σ: x 中央値 {amcl['x_sigma_m']['median']:.3f} m, "
                  f"yaw 中央値 {amcl['yaw_sigma_rad']['median']:.3f} rad"]
        lat = amcl["latency_s"]
        if isinstance(lat, dict):
            lines.append(f"- 遅延: 中央値 {lat['median']:.3f} s / p95 {lat['p95']:.3f} s / 最大 {lat['max']:.3f} s")
        else:
            lines.append(f"- 遅延: {lat}")
        if "diff_to_estimate_position_m" in amcl:
            dp, dy = amcl["diff_to_estimate_position_m"], amcl["diff_to_estimate_yaw_rad"]
            lines.append(f"- 届いた時点の推定姿勢との差: 位置 中央値 {dp['median']:.3f} m / p95 {dp['p95']:.3f} m, "
                         f"yaw 中央値 {dy['median']:.3f} rad / p95 {dy['p95']:.3f} rad")
        lines.append("")

    lines += ["## GNSS", "", f"- `/odom/gps` の件数: {report['gps']['count']}", ""]

    faults = report.get("faults")
    if faults:
        lines += ["## 故障注入", "",
                  "| 種類 | 開始[s] | 終了[s] | 最大誤差[m] | 悪化 | 復旧[s] | 検知[s] |",
                  "|---|---|---|---|---|---|---|"]

        def fmt(v, spec=".2f"):
            return "-" if v is None else format(v, spec)

        for f in faults["per_fault"]:
            lines.append(
                f"| {f['type']} | {f['start_s']:.0f} | {f['end_s']:.0f} | {fmt(f.get('max_error_m'))} | "
                f"{'はい' if f.get('deviated') else 'いいえ'} | {fmt(f.get('recovery_sec'), '.1f')} | "
                f"{fmt(f.get('detection_delay_s'), '.1f')} |")
        if "status" in faults:
            st = faults["status"]
            lines += ["", f"- 状態遷移 (NORMAL 以外へ入った回数): 故障の前後 {st['episodes_in_fault_windows']} 回 / "
                      f"**誤検知 {st['false_detections']} 回**"]
        lines.append("")
    elif "status" in report:
        st = report["status"]
        lines += ["## 監督ノードの状態", "",
                  f"- NORMAL 以外へ入った回数 (誤検知): {st['false_detections']} 回", ""]
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="自己位置推定の出力を rosbag から評価する")
    parser.add_argument("bag_path", type=str, help="評価する rosbag のパス")
    parser.add_argument("--gt-dir", type=str, default=None,
                        help="真値 (pose_graph.json と gnss_transform.yaml) の SLAM 出力ディレクトリ")
    parser.add_argument("--map-gt-dir", type=str, default=None,
                        help="地図を作った走行の SLAM 出力。指定すると真値をこの座標系へ移す")
    parser.add_argument("--faults", type=str, default=None, help="故障定義 YAML")
    parser.add_argument("--start", type=float, default=0.0, help="評価の開始時刻 (先頭からの経過秒)")
    parser.add_argument("--end", type=float, default=0.0, help="評価の終了時刻 (0 で末尾まで)")
    parser.add_argument("--window", type=float, default=20.0, help="区間ごとの誤差の窓幅 [s]")
    parser.add_argument("--ok-threshold", type=float, default=0.5, help="復旧とみなす位置誤差 [m]")
    parser.add_argument("--hold-sec", type=float, default=3.0, help="復旧とみなすために誤差が閾値以下でいる時間 [s]")
    parser.add_argument("--fault-margin", type=float, default=60.0,
                        help="故障の終了後、状態遷移を誤検知に数えない時間 [s]")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--ekf-topic", default="/ekf_global_odom")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--gps-topic", default="/odom/gps")
    parser.add_argument("--status-topic", default="/localization/status")
    add_output_args(parser, "eval_localization.json")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    data = read_bag(args.bag_path, args)
    est_times, est_poses = estimate_base_poses(data)
    t0 = float(est_times[0])
    t_last = float(est_times[-1])
    t_from = t0 + args.start
    t_to = t0 + args.end if args.end > 0.0 else t_last + 1e-6
    keep = (est_times >= t_from) & (est_times < t_to)
    est_times, est_poses = est_times[keep], est_poses[keep]

    # --start / --end で指定した区間だけを評価する (初期姿勢による最初の飛びなどの過渡を除く)
    ekf = clip_time(as_array(data.ekf, 42), t_from, t_to)
    odom = clip_time(as_array(data.odom, 3), t_from, t_to)
    amcl = clip_time(as_array(data.amcl, 41), t_from, t_to)
    status = clip_time(as_array(data.status, 2), t_from, t_to)
    map_odom = clip_time(as_array(data.map_odom, 4), t_from, t_to)

    report: Dict[str, Any] = {
        "meta": {
            "bag": args.bag_path, "gt_dir": args.gt_dir, "map_gt_dir": args.map_gt_dir,
            "n_estimates": int(est_times.size), "t0": t0,
            "duration_s": float(t_to - t_from) if args.end > 0.0 else float(t_last - t_from),
        },
        "smoothness": evaluate_smoothness(map_odom, as_array(data.odom_base, 4)),
        "ekf_vs_odom": evaluate_ekf_vs_odom(ekf, odom),
        "amcl": evaluate_amcl(amcl, est_times, est_poses),
        "gps": {"count": data.gps_count},
    }

    gt_nodes = load_ground_truth(args)
    gt_times = np.zeros(0)
    gt_pos_err = np.zeros(0)
    if gt_nodes is not None:
        report["accuracy"], gt_times, gt_pos_err = evaluate_accuracy(
            est_times, est_poses, gt_nodes, t0, args.window)
        if ekf.size:
            report["nees"] = evaluate_nees(ekf, gt_nodes)

    if args.faults:
        report["faults"] = evaluate_faults(
            load_faults(args.faults), t0, t_last, gt_times, gt_pos_err, status, args)
    elif status.shape[0]:
        report["status"] = lm.count_episodes(status[:, 0], status[:, 1], [])

    text = format_summary(report)
    print(text)

    output_path = resolve_output_path(
        args.bag_path, "eval_localization.json", args.output,
        args.output_to_bag_dir, args.output_dir)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    md_path = os.path.splitext(output_path)[0] + ".md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"保存: {output_path}, {md_path}")


if __name__ == "__main__":
    main()
