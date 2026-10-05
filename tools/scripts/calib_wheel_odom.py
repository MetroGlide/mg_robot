#!/usr/bin/env python3
"""
calib_wheel_odom.py

走行ログの /odom (ホイールオドメトリの生の姿勢) と、slam_gnss_2d の SLAM 出力 (pose_graph.json) を比べて、
ホイールオドメトリの並進スケール k_v、旋回スケール k_w、走行距離あたりの曲がり yaw_bias_per_meter、
時刻の遅れ time_offset を推定し、EKF に入れる速度の共分散の目安を出す。

真値の経路で --window-m ごとに区切った窓について、窓の始点から終点への相対移動を比べる
(全体を一度に積算すると誤差が累積して崩れるため)。推定は Huber 損失で外れ値の窓の影響を抑える。
結果は wheel_odom_corrector_node のパラメータ (YAML) として出力できる。

注意: /odom は補正前の生の値である必要がある (補正ノードを通した bag には使えない)。
推定値は路面や荷重で変わりうるので、走行ごとに推定して、ばらつきを確認すること (前半・後半の推定も出す)。
"""

import argparse
import os
import sys
from typing import Any, Dict, List, Tuple

import numpy as np
import yaml

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import odom_calib as oc  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.pose_graph import load_slam_output  # noqa: E402

# 速度の共分散の標準偏差の下限。真値の精度や量子化の限界より小さくしない
MIN_SIGMA_V = 0.02
MIN_SIGMA_W = 0.02


def read_odom(bag_path: str, topic: str) -> np.ndarray:
    """/odom を [t, x, y, yaw (unwrap 済み)] の配列で読む。"""
    reader = open_reader(bag_path, topics=[topic])
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise SystemExit(f"エラー: bag に {topic} がありません。")
    deserializer = MessageDeserializer(type_map)
    rows: List[List[float]] = []
    while reader.has_next():
        name, raw, _ = reader.read_next()
        msg = deserializer.deserialize(name, raw)
        q = msg.pose.pose.orientation
        rows.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     msg.pose.pose.position.x, msg.pose.pose.position.y,
                     quaternion_to_yaw(q.x, q.y, q.z, q.w)])
    odom = np.array(rows, dtype=float)
    odom = odom[np.argsort(odom[:, 0], kind="stable")]
    odom[:, 3] = np.unwrap(odom[:, 3])
    return odom


def collect(
    nodes: np.ndarray, odom: np.ndarray, args: argparse.Namespace, time_offset: float,
) -> Tuple[List[oc.Window], List[np.ndarray], np.ndarray]:
    windows = oc.build_windows(nodes, args.window_m, args.max_gap)
    kept, steps_list, gt_rel = [], [], []
    for w in windows:
        if not (args.start <= w.t_a - nodes[0, 0] and (args.end <= 0.0 or w.t_b - nodes[0, 0] <= args.end)):
            continue
        steps = oc.steps_between(odom, w.t_a, w.t_b, time_offset)
        if steps is None:
            continue
        kept.append(w)
        steps_list.append(steps)
        gt_rel.append(oc.gt_relative(nodes, w))
    return kept, steps_list, np.array(gt_rel)


def window_errors(steps_list, gt_rel, theta, yaw_weight) -> Dict[str, Any]:
    res = oc.residual_vector(np.asarray(theta), steps_list, gt_rel, yaw_weight).reshape(-1, 3)
    pos = np.hypot(res[:, 0], res[:, 1])
    yaw_deg = np.degrees(np.abs(res[:, 2] / yaw_weight))
    return {
        "position_m_median": float(np.median(pos)), "position_m_p95": float(np.percentile(pos, 95)),
        "yaw_deg_median": float(np.median(yaw_deg)), "yaw_deg_p95": float(np.percentile(yaw_deg, 95)),
    }


def fit_at(steps_list, gt_rel, args) -> Tuple[np.ndarray, float]:
    return oc.fit_parameters(steps_list, gt_rel, yaw_weight=args.yaw_weight, huber_delta=args.huber)


def main() -> None:
    parser = argparse.ArgumentParser(description="ホイールオドメトリのスケール・バイアス・遅れを走行ログから推定する")
    parser.add_argument("bag_path", help="生の /odom を含む rosbag")
    parser.add_argument("--slam-dir", required=True, help="真値 (pose_graph.json / gnss_transform.yaml) の SLAM 出力")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--window-m", type=float, default=5.0, help="窓の経路長 [m]")
    parser.add_argument("--max-gap", type=float, default=2.0, help="窓に含めるノード間の最大時間 [s] (停止や欠損を除く)")
    parser.add_argument("--yaw-weight", type=float, default=5.0, help="yaw 残差を長さに換算する係数 [m/rad]")
    parser.add_argument("--huber", type=float, default=0.1, help="Huber 損失のしきい値 [m]")
    parser.add_argument("--offset-range", type=float, nargs=3, default=[-0.2, 0.6, 0.05],
                        metavar=("MIN", "MAX", "STEP"), help="時刻の遅れの探索範囲 [s]")
    parser.add_argument("--inflate", type=float, default=3.0,
                        help="速度の共分散の標準偏差にかける係数 (誤差が窓の間続く低周波成分なので余裕を見る)")
    parser.add_argument("--start", type=float, default=0.0, help="使う区間の開始 (真値の先頭からの経過秒)")
    parser.add_argument("--end", type=float, default=0.0, help="使う区間の終了 (0 で最後まで)")
    parser.add_argument("-o", "--output", default=None, help="wheel_odom_corrector のパラメータ YAML の出力先")
    args = parser.parse_args()

    nodes, _, _ = load_slam_output(args.slam_dir)
    odom = read_odom(args.bag_path, args.odom_topic)

    # 時刻の遅れを探索し、損失が最小の遅れで推定する
    offsets = np.arange(args.offset_range[0], args.offset_range[1] + 1e-9, args.offset_range[2])
    curve = []
    for offset in offsets:
        _, steps_list, gt_rel = collect(nodes, odom, args, float(offset))
        if len(steps_list) < 10:
            continue
        _, cost = fit_at(steps_list, gt_rel, args)
        curve.append((float(offset), cost / len(steps_list)))
    if not curve:
        raise SystemExit("エラー: 使える窓が 10 個未満です。--window-m や --max-gap を見直してください。")
    best_offset = min(curve, key=lambda c: c[1])[0]

    windows, steps_list, gt_rel = collect(nodes, odom, args, best_offset)
    theta, _ = fit_at(steps_list, gt_rel, args)
    durations = np.array([w.t_b - w.t_a for w in windows])
    v_err, w_err = oc.window_velocity_errors(steps_list, gt_rel, durations, theta)
    v_err_raw, w_err_raw = oc.window_velocity_errors(steps_list, gt_rel, durations, np.array([1.0, 1.0, 0.0]))
    sigma_v = float(np.sqrt(np.mean(v_err ** 2)))
    sigma_w = float(np.sqrt(np.mean(w_err ** 2)))
    cov_vx = (max(sigma_v, MIN_SIGMA_V) * args.inflate) ** 2
    cov_vyaw = (max(sigma_w, MIN_SIGMA_W) * args.inflate) ** 2

    half = len(windows) // 2
    halves = []
    for name, sl in (("前半", slice(0, half)), ("後半", slice(half, None))):
        if len(steps_list[sl]) >= 10:
            t_half, _ = fit_at(steps_list[sl], gt_rel[sl], args)
            halves.append((name, len(steps_list[sl]), t_half))

    before = window_errors(steps_list, gt_rel, [1.0, 1.0, 0.0], args.yaw_weight)
    after = window_errors(steps_list, gt_rel, theta, args.yaw_weight)
    path_length = float(sum(np.sum(np.hypot(s[:, 0], s[:, 1])) for s in steps_list))

    lines = [
        "# ホイールオドメトリの校正", "",
        f"- bag: `{args.bag_path}` / 真値: `{args.slam_dir}`",
        f"- 窓: {len(windows)} 個 (経路長 {args.window_m} m ごと、合計 {path_length:.0f} m)", "",
        "## 推定値", "",
        f"- k_v (並進スケール): **{theta[0]:.4f}**",
        f"- k_w (旋回スケール): **{theta[1]:.4f}**",
        f"- yaw_bias_per_meter (走行距離あたりの曲がり): **{theta[2]:.5f} rad/m** "
        f"(10 m 直進で {np.degrees(theta[2] * 10):.2f} deg)",
        f"- time_offset (オドメトリの遅れ): **{best_offset:.2f} s**", "",
        "## 窓ごとの相対移動の誤差 (補正前 → 補正後)", "",
        "| | 位置 中央値[m] | 位置 p95[m] | yaw 中央値[deg] | yaw p95[deg] |", "|---|---|---|---|---|",
        f"| 補正前 | {before['position_m_median']:.3f} | {before['position_m_p95']:.3f} | "
        f"{before['yaw_deg_median']:.2f} | {before['yaw_deg_p95']:.2f} |",
        f"| 補正後 | {after['position_m_median']:.3f} | {after['position_m_p95']:.3f} | "
        f"{after['yaw_deg_median']:.2f} | {after['yaw_deg_p95']:.2f} |", "",
        "## 推定値の安定性 (前半と後半で別々に推定)", "",
        "| | 窓 | k_v | k_w | yaw_bias_per_meter |", "|---|---|---|---|---|",
    ]
    lines += [f"| {name} | {n} | {t[0]:.4f} | {t[1]:.4f} | {t[2]:.5f} |" for name, n, t in halves]
    lines += [
        "", "## EKF に入れる速度の共分散の目安", "",
        f"- 窓の平均速度の誤差 (補正後): 進行方向 σ {sigma_v:.4f} m/s (補正前 {np.sqrt(np.mean(v_err_raw ** 2)):.4f}) / "
        f"角速度 σ {sigma_w:.4f} rad/s (補正前 {np.sqrt(np.mean(w_err_raw ** 2)):.4f})",
        f"- 推奨: covariance_vx = **{cov_vx:.4f}**、covariance_vyaw = **{cov_vyaw:.4f}** "
        f"(σ を {args.inflate} 倍、下限 {MIN_SIGMA_V} m/s / {MIN_SIGMA_W} rad/s)", "",
        "## 時刻の遅れの探索 (窓あたりの損失)", "",
        "| 遅れ[s] | " + " | ".join(f"{o:.2f}" for o, _ in curve) + " |",
        "|---|" + "---|" * len(curve),
        "| 損失 | " + " | ".join(f"{c:.4f}" for _, c in curve) + " |", "",
    ]
    print("\n".join(lines))

    if args.output:
        params = {"wheel_odom_corrector_node": {"ros__parameters": {
            "enabled": True,
            "k_v": round(float(theta[0]), 4),
            "k_w": round(float(theta[1]), 4),
            "yaw_bias_per_meter": round(float(theta[2]), 5),
            "time_offset": round(float(best_offset), 3),
            "covariance_vx": round(cov_vx, 4),
            "covariance_vyaw": round(cov_vyaw, 4),
        }}}
        with open(args.output, "w", encoding="utf-8") as f:
            yaml.safe_dump(params, f, sort_keys=False)
        print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
