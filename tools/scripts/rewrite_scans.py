#!/usr/bin/env python3
"""
rewrite_scans.py

記録の 2D LiDAR のスキャンを、地図と姿勢を使って書き換えた bag を作る。ほかのトピックはそのまま写す。
自己位置推定のずれが、どの観測の性質から来るかを再生で切り分けるのに使う (run_localization_variant.sh の
データセットの eval.bag に出力を書く)。

使い方:
  rewrite_scans.py <入力 bag> -o <出力 bag> (--pose-bag <姿勢の bag> | --pose-graph <SLAM 出力>) --mode keep-matched|raycast --map <map.yaml>
      [--lidar-pose X,Y,YAW] [--end E] [--match-dist D] [--noise-std S] [--seed N]

モード:
  keep-matched : 正しい姿勢で、端点が地図 (--map) の占有から --match-dist より遠いビームを欠測 (inf) にする。
                 地図にない物の点を除いた観測になる
  raycast      : 正しい姿勢から地図 (--map) へ光線を飛ばした距離に置き換える (地図を世界とした雑音のない観測。
                 --noise-std で距離に正規分布の雑音を足す)。元の記録で欠測のビームは欠測のまま
                 (車体に遮られる向きなど)

- 正しい姿勢は --pose-bag の /tf (map->odom と odom->base) から取る (例: ずれなかった再生評価の出力)。
  --pose-graph に SLAM 出力を渡すと、そのキーフレームの姿勢 (その SLAM 地図の座標系) を補間して使う
  (SLAM 地図を世界とする raycast 用)。姿勢が無い時刻のスキャンは書き換えずに写し、件数を表示する
- --end は入力 bag の先頭からの経過秒で、そこまでのメッセージだけを写す (0 で末尾まで)
"""

import argparse
import os
import sys

import numpy as np
from rclpy.serialization import serialize_message

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import (  # noqa: E402
    MessageDeserializer,
    detect_storage_id,
    open_reader,
    open_writer,
    read_reference_poses,
)
from tools.common.scan_map import (  # noqa: E402
    OccupancyMap,
    beam_geometry,
    interpolate_pose,
    keep_matched,
    parse_pose,
    raycast_ranges,
    valid_ranges,
)

MODES = ("keep-matched", "raycast")


def main() -> None:
    parser = argparse.ArgumentParser(description="記録のスキャンを地図と姿勢で書き換えた bag を作る")
    parser.add_argument("input", help="入力 bag")
    parser.add_argument("-o", "--output", required=True, help="出力 bag (存在しないこと)")
    parser.add_argument("--pose-bag", default=None, help="正しい姿勢の /tf (map->odom、odom->base) を記録した bag")
    parser.add_argument("--pose-graph", default=None, help="正しい姿勢を pose_graph.json のキーフレームから取る SLAM 出力")
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--map", required=True, help="keep-matched は照合する地図、raycast は世界とする地図 (map.yaml)")
    parser.add_argument("--scan-topic", default="/scan_top_lidar")
    parser.add_argument("--lidar-pose", type=parse_pose, default=[0.0, 0.0, 0.0],
                        help="base 座標系での LiDAR の姿勢 X,Y,YAW [m, m, rad]")
    parser.add_argument("--end", type=float, default=0.0, help="写す区間の終わり (入力 bag の先頭からの経過秒。0 で末尾まで)")
    parser.add_argument("--match-dist", type=float, default=0.2, help="keep-matched で地図に一致したとみなす距離 [m]")
    parser.add_argument("--close-radius", type=float, default=0.0,
                        help="raycast の前に占有の点の間を埋める半径 [m] (まばらな帯の隙間を光線が抜けないように。0 で埋めない)")
    parser.add_argument("--max-pose-gap", type=float, default=0.0,
                        help="姿勢を補間してよい間隔の上限 [s] (0 で姿勢の種類ごとの既定。キーフレームは止まっている間に"
                             "出ないので、ほぼ止まっている区間も書き換えたいときに広げる)")
    parser.add_argument("--clear-path-radius", type=float, default=0.0,
                        help="raycast の前に、正しい姿勢の軌跡からこの半径 [m] 以内の占有を消す (地図を作った走行で"
                             "台車を押す人などが軌跡沿いに映った地図用。0 で消さない)")
    parser.add_argument("--noise-std", type=float, default=0.0, help="raycast で距離に足す雑音の標準偏差 [m]")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args()
    if os.path.exists(args.output):
        raise SystemExit(f"エラー: 出力 {args.output} が既にあります。")

    times, poses, max_gap = read_reference_poses(
        args.pose_bag, args.pose_graph, args.map_frame, args.odom_frame, args.base_frame)
    max_gap = args.max_pose_gap or max_gap
    grid = OccupancyMap(args.map, args.close_radius)
    if args.clear_path_radius > 0.0:
        grid.clear_near(poses[:, :2], args.clear_path_radius)
    lidar = np.array(args.lidar_pose)
    rng = np.random.default_rng(args.seed)

    storage_id = detect_storage_id(args.input)
    reader = open_reader(args.input, storage_id=storage_id)
    topics = reader.get_all_topics_and_types()
    deserializer = MessageDeserializer({t.name: t.type for t in topics})
    writer = open_writer(args.output, storage_id=storage_id)
    for meta in topics:
        writer.create_topic(meta)

    start_ns = None
    rewritten = kept = 0
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        start_ns = stamp_ns if start_ns is None else start_ns
        if args.end > 0.0 and (stamp_ns - start_ns) * 1e-9 > args.end:
            break
        if topic == args.scan_topic:
            msg = deserializer.deserialize(topic, raw)
            pose = interpolate_pose(times, poses, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, max_gap)
            if pose is None:
                kept += 1
            else:
                ranges = valid_ranges(msg.ranges, msg.range_min, msg.range_max)
                angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
                sensor_xy, beam_yaw = beam_geometry(pose, lidar, angles)
                if args.mode == "keep-matched":
                    ranges = keep_matched(ranges, sensor_xy, beam_yaw, grid, args.match_dist)
                else:
                    ranges = raycast_ranges(ranges, sensor_xy, beam_yaw, grid, msg.range_min, msg.range_max,
                                            args.noise_std, rng)
                msg.ranges = ranges.astype(np.float32).tolist()
                raw = serialize_message(msg)
                rewritten += 1
        writer.write(topic, raw, stamp_ns)
    del writer
    print(f"書き換えたスキャン {rewritten} 本、姿勢が無く写しただけのスキャン {kept} 本")
    print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
