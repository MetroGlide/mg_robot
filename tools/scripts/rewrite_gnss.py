#!/usr/bin/env python3
"""
rewrite_gnss.py

記録の GNSS (/gps/fix と /navpvt) の位置を、真値 (SLAM 出力のキーフレーム) のアンテナ位置に置き換えた bag を作る。
ほかのトピックと、GNSS の精度の欄 (共分散、hAcc、測位の状態) はそのまま写す。
自己位置推定のずれが、GNSS の誤差と地図の食い違いのどちらから来るかを再生で切り分けるのに使う。雑音を足すときは、
再生の故障の注入 (gnss_noise。tools/common/faults.py) で足す。

使い方:
  rewrite_gnss.py <入力 bag> -o <出力 bag> --pose-graph <SLAM 出力> [--gnss-transform <yaml>]
      [--lever-arm X,Y] [--end E] [--max-pose-gap S]

- 真値の座標系は SLAM 出力の地図。GNSS との変換は --gnss-transform (既定: SLAM 出力の gnss_transform.yaml)
- /gps/fix はヘッダの時刻で真値を補間する。/navpvt は受信の時刻が最も近い /gps/fix と同じ位置にする
  (同じ測位から出る 2 つが食い違わないように)
- 真値が無い時刻の GNSS は書き換えずに写し、件数を表示する
- 置き換える前の記録の GNSS の、真値に対する誤差 (東・北の標準偏差、1 s 離れた値の相関と、そこから見積もった
  相関時間) を表示する。gnss_noise の sigma と tau を決める目安にする
"""

import argparse
import math
import os
import sys
from typing import Dict, List, Tuple

import numpy as np
from rclpy.serialization import serialize_message

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import MessageDeserializer, detect_storage_id, open_reader, open_writer  # noqa: E402
from tools.common.fault_apply import shift_latlon  # noqa: E402
from tools.common.geo import lat_lon_to_utm  # noqa: E402
from tools.common.map import load_gnss_transform  # noqa: E402
from tools.common.pose_graph import antenna_positions, load_slam_output  # noqa: E402
from tools.common.scan_map import interpolate_pose  # noqa: E402


def parse_xy(text: str) -> Tuple[float, float]:
    values = [float(v) for v in text.split(",")]
    if len(values) != 2:
        raise argparse.ArgumentTypeError(f"X,Y で指定してください: {text}")
    return values[0], values[1]


def to_map(transform: Tuple[float, float, float], zone: int, lat: float, lon: float) -> np.ndarray:
    """緯度経度を地図座標にする (slam_gnss_nav_bridge と同じ変換)。"""
    tx, ty, yaw = transform
    e, n = lat_lon_to_utm(lat, lon, zone)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * e - s * n + tx, s * e + c * n + ty])


def map_delta_to_east_north(yaw: float, delta: np.ndarray) -> Tuple[float, float]:
    """地図座標での変位を、東・北の変位にする (地図は UTM を yaw だけ回したもの)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]


def report_error(times: np.ndarray, error: np.ndarray) -> None:
    """記録の GNSS の真値に対する誤差の統計を表示する。"""
    print(f"記録の GNSS の誤差 ({times.size} 点): 平均 東 {error[:, 0].mean():+.2f} 北 {error[:, 1].mean():+.2f} m、"
          f"標準偏差 東 {error[:, 0].std():.2f} 北 {error[:, 1].std():.2f} m、"
          f"距離の中央値 {np.median(np.hypot(error[:, 0], error[:, 1])):.2f} m")
    # 1 s ごとの隣り合う測位 (間が空いた所は除く) の相関
    near = np.abs(np.diff(times) - 1.0) < 0.2
    centered = error - error.mean(axis=0)
    for axis, name in enumerate(("東", "北")):
        if near.sum() < 10:
            break
        rho = float(np.corrcoef(centered[:-1, axis][near], centered[1:, axis][near])[0, 1])
        tau = -1.0 / math.log(rho) if 0.0 < rho < 1.0 else float("inf")
        print(f"  {name}: 1 s 離れた値の相関 {rho:.3f} (相関時間の見積もり {tau:.1f} s)")


def main() -> None:
    parser = argparse.ArgumentParser(description="記録の GNSS の位置を真値のアンテナ位置に置き換えた bag を作る")
    parser.add_argument("input", help="入力 bag")
    parser.add_argument("-o", "--output", required=True, help="出力 bag (存在しないこと)")
    parser.add_argument("--pose-graph", required=True, help="真値の姿勢を pose_graph.json のキーフレームから取る SLAM 出力")
    parser.add_argument("--gnss-transform", default=None, help="真値の地図と GNSS の変換 (既定: SLAM 出力の gnss_transform.yaml)")
    parser.add_argument("--lever-arm", type=parse_xy, default=(0.26, -0.13),
                        help="base 座標系での GNSS アンテナの位置 X,Y [m] (URDF の base_link -> gps_link)")
    parser.add_argument("--end", type=float, default=0.0, help="写す区間の終わり (入力 bag の先頭からの経過秒。0 で末尾まで)")
    parser.add_argument("--max-pose-gap", type=float, default=2.0, help="キーフレームを補間してよい間隔の上限 [s]")
    parser.add_argument("--fix-topic", default="/gps/fix")
    parser.add_argument("--navpvt-topic", default="/navpvt")
    args = parser.parse_args()
    if os.path.exists(args.output):
        raise SystemExit(f"エラー: 出力 {args.output} が既にあります。")

    nodes, _, _ = load_slam_output(args.pose_graph)
    transform, zone = load_gnss_transform(args.gnss_transform or os.path.join(args.pose_graph, "gnss_transform.yaml"))
    storage_id = detect_storage_id(args.input)

    # 1 回目: /gps/fix ごとに、真値の位置にするための東・北のずらし量を決める
    reader = open_reader(args.input, storage_id=storage_id, topics=[args.fix_topic])
    deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    shifts: Dict[int, Tuple[float, float]] = {}
    fix_stamps: List[int] = []
    error_times: List[float] = []
    errors: List[np.ndarray] = []
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        msg = deserializer.deserialize(topic, raw)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = interpolate_pose(nodes[:, 0], nodes[:, 1:4], t, args.max_pose_gap)
        if pose is None:
            continue
        truth = antenna_positions(pose[None, :], args.lever_arm)[0]
        delta = truth - to_map(transform, zone, msg.latitude, msg.longitude)
        shifts[stamp_ns] = map_delta_to_east_north(transform[2], delta)
        fix_stamps.append(stamp_ns)
        error_times.append(t)
        errors.append(-delta)
    del reader
    if not fix_stamps:
        raise SystemExit("エラー: 真値で書き換えられる GNSS がありません。")
    report_error(np.array(error_times), np.array(errors))
    fix_array = np.array(fix_stamps)

    # 2 回目: 書き換えて写す
    reader = open_reader(args.input, storage_id=storage_id)
    topics = reader.get_all_topics_and_types()
    deserializer = MessageDeserializer({t.name: t.type for t in topics})
    writer = open_writer(args.output, storage_id=storage_id)
    for meta in topics:
        writer.create_topic(meta)
    start_ns = None
    counts = {"fix": 0, "navpvt": 0, "kept": 0}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        start_ns = stamp_ns if start_ns is None else start_ns
        if args.end > 0.0 and (stamp_ns - start_ns) * 1e-9 > args.end:
            break
        if topic == args.fix_topic:
            msg = deserializer.deserialize(topic, raw)
            if stamp_ns in shifts:
                msg.latitude, msg.longitude = shift_latlon(msg.latitude, msg.longitude, *shifts[stamp_ns])
                raw = serialize_message(msg)
                counts["fix"] += 1
            else:
                counts["kept"] += 1
        elif topic == args.navpvt_topic:
            msg = deserializer.deserialize(topic, raw)
            nearest = int(fix_array[np.argmin(np.abs(fix_array - stamp_ns))])
            if abs(nearest - stamp_ns) < 0.5e9:
                lat, lon = shift_latlon(msg.lat * 1e-7, msg.lon * 1e-7, *shifts[nearest])
                msg.lat = int(round(lat * 1e7))
                msg.lon = int(round(lon * 1e7))
                raw = serialize_message(msg)
                counts["navpvt"] += 1
            else:
                counts["kept"] += 1
        writer.write(topic, raw, stamp_ns)
    del writer
    print(f"書き換えた {args.fix_topic} {counts['fix']} 件、{args.navpvt_topic} {counts['navpvt']} 件、"
          f"真値が無く写しただけの GNSS {counts['kept']} 件")
    print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
