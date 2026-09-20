#!/usr/bin/env python3
"""
plot_lidar_scans.py

rosbag (MCAP / SQLite3) から LiDAR スキャン (/scan_*) とオドメトリ (/odom) を読み込み、
ロボット移動量ごとのノードを抽出して、指定したノード区間のスキャン点群を 2D 画像に描画するデバッグツール。
"""

import argparse
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import open_reader  # noqa: E402
from tools.common.cli import resolve_output_path  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402


def extract_nodes(
    bag_path: str,
    scan_topic: str = "/scan_top_lidar",
    odom_topic: str = "/odom",
    min_trans: float = 0.2,
    min_rot: float = 0.1,
) -> List[Dict[str, Any]]:
    """rosbag から一定変位ごとのノード（オドメトリ姿勢とレーザースキャン）を抽出する。"""
    reader = open_reader(bag_path, topics=[scan_topic, odom_topic])

    last_odom = None
    last_node_odom = None
    node_count = 0
    nodes = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                qz,
                qw,
            )
            last_odom = (t * 1e-9, px, py, yaw)

        elif topic == scan_topic:
            if last_odom is None:
                continue
            msg = deserialize_message(data, LaserScan)
            ot, ox, oy, oyaw = last_odom

            if last_node_odom is None:
                node_count += 1
                last_node_odom = (ox, oy, oyaw)
                nodes.append({
                    "id": node_count,
                    "t": ot,
                    "odom": (ox, oy, oyaw),
                    "angle_min": msg.angle_min,
                    "angle_inc": msg.angle_increment,
                    "range_min": msg.range_min,
                    "range_max": msg.range_max,
                    "ranges": list(msg.ranges),
                })
                continue

            lx, ly, lyaw = last_node_odom
            dx_w = ox - lx
            dy_w = oy - ly
            dist = math.hypot(dx_w, dy_w)
            dyaw = (oyaw - lyaw + math.pi) % (2.0 * math.pi) - math.pi

            if dist >= min_trans or abs(dyaw) >= min_rot:
                node_count += 1
                last_node_odom = (ox, oy, oyaw)
                nodes.append({
                    "id": node_count,
                    "t": ot,
                    "odom": (ox, oy, oyaw),
                    "angle_min": msg.angle_min,
                    "angle_inc": msg.angle_increment,
                    "range_min": msg.range_min,
                    "range_max": msg.range_max,
                    "ranges": list(msg.ranges),
                })

    return nodes


def render_scans(
    nodes: List[Dict[str, Any]],
    output_path: str,
    resolution: float = 0.05,
    max_range: float = 20.0,
    margin_m: float = 5.0,
):
    """ノードのスキャン点群を2D画像として描画・保存する。"""
    if not nodes:
        print("描画対象のノードがありません。")
        return

    # 基準座標: 最初のノードのオドメトリ姿勢
    x0, y0, yaw0 = nodes[0]["odom"]

    # 点群の世界座標（x0, y0 基準）を収集して画像サイズを決定
    all_pts_x = [0.0]
    all_pts_y = [0.0]

    for d in nodes:
        ox, oy, oyaw = d["odom"]
        amin = d["angle_min"]
        ainc = d["angle_inc"]
        rmin = d["range_min"]
        rmax = d["range_max"]

        all_pts_x.append(ox - x0)
        all_pts_y.append(oy - y0)

        for i, r in enumerate(d["ranges"]):
            if rmin <= r <= min(rmax, max_range):
                th = oyaw + amin + i * ainc
                px_w = ox + r * math.cos(th) - x0
                py_w = oy + r * math.sin(th) - y0
                all_pts_x.append(px_w)
                all_pts_y.append(py_w)

    min_x = min(all_pts_x) - margin_m
    max_x = max(all_pts_x) + margin_m
    min_y = min(all_pts_y) - margin_m
    max_y = max(all_pts_y) + margin_m

    scale = 1.0 / resolution  # px/m
    img_w = int(math.ceil((max_x - min_x) * scale))
    img_h = int(math.ceil((max_y - min_y) * scale))

    # 白背景画像
    img = np.full((img_h, img_w, 3), 255, dtype=np.uint8)

    # カラーパレット（ノードごとに異なる色）
    color_palette = [
        (0, 0, 255),      # 赤
        (0, 128, 255),    # 橙
        (0, 200, 200),    # 黄
        (0, 200, 0),      # 緑
        (255, 0, 0),      # 青
        (255, 0, 255),    # マゼンタ
        (128, 0, 128),    # 紫
        (128, 128, 0),    # オリーブ
        (0, 128, 128),    # ティール
        (50, 50, 50),     # 濃灰
    ]

    for k, d in enumerate(nodes):
        node_id = d["id"]
        ox, oy, oyaw = d["odom"]
        color = color_palette[k % len(color_palette)]

        # ロボット位置の画像座標
        rx = int((ox - x0 - min_x) * scale)
        ry = img_h - int((oy - y0 - min_y) * scale)
        if 0 <= rx < img_w and 0 <= ry < img_h:
            cv2.circle(img, (rx, ry), 4, color, -1)
            cv2.putText(
                img,
                str(node_id),
                (rx + 5, ry - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
            )

        # スキャン点群の描画
        amin = d["angle_min"]
        ainc = d["angle_inc"]
        rmin = d["range_min"]
        rmax = d["range_max"]

        for i, r in enumerate(d["ranges"]):
            if rmin <= r <= min(rmax, max_range):
                th = oyaw + amin + i * ainc
                px_w = ox + r * math.cos(th) - x0
                py_w = oy + r * math.sin(th) - y0

                ix = int((px_w - min_x) * scale)
                iy = img_h - int((py_w - min_y) * scale)
                if 0 <= ix < img_w and 0 <= iy < img_h:
                    img[iy, ix] = color

    cv2.imwrite(output_path, img)
    print(f"スキャン画像を保存しました ({img_w}x{img_h}px, {len(nodes)}ノード): {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbag 内の LiDAR スキャン点群を 2D 画像に描画するツール"
    )
    parser.add_argument("bag_path", type=str, help="rosbag ディレクトリまたはファイルパス")
    parser.add_argument(
        "-o", "--output", type=str, default="lidar_scans_plot.png", help="出力画像ファイルパス"
    )
    parser.add_argument(
        "--output-to-bag-dir",
        action="store_true",
        help="bagと同じディレクトリに出力画像を保存",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="指定したディレクトリに出力画像を保存",
    )
    parser.add_argument(
        "--nodes",
        type=str,
        default=None,
        help="描画するノード番号の範囲 (例: 705:715 または 705)。未指定時は先頭30ノード",
    )
    parser.add_argument(
        "--scan-topic",
        type=str,
        default="/scan_top_lidar",
        help="LiDAR スキャン トピック名 (default: /scan_top_lidar)",
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/odom",
        help="オドメトリ トピック名 (default: /odom)",
    )
    parser.add_argument(
        "--min-trans",
        type=float,
        default=0.2,
        help="ノード作成の最小並進移動量 [m] (default: 0.2)",
    )
    parser.add_argument(
        "--min-rot",
        type=float,
        default=0.1,
        help="ノード作成の最小回転角 [rad] (default: 0.1)",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="描画解像度 [m/pixel] (default: 0.05)",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=20.0,
        help="プロットする最大距離 [m] (default: 20.0)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = os.path.abspath(args.bag_path)
    if not os.path.exists(bag_path):
        print(f"エラー: 指定されたパスが存在しません: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"ノード抽出中: {bag_path} ...")
    nodes = extract_nodes(
        bag_path,
        scan_topic=args.scan_topic,
        odom_topic=args.odom_topic,
        min_trans=args.min_trans,
        min_rot=args.min_rot,
    )
    print(f"抽出された総ノード数: {len(nodes)}")

    if not nodes:
        print("ノードが抽出されませんでした。トピック名を確認してください。", file=sys.stderr)
        sys.exit(1)

    # ノード範囲のフィルタリング
    selected_nodes = nodes
    if args.nodes:
        if ":" in args.nodes:
            parts = args.nodes.split(":")
            start_id = int(parts[0]) if parts[0] else 1
            end_id = int(parts[1]) if parts[1] else len(nodes)
            selected_nodes = [n for n in nodes if start_id <= n["id"] <= end_id]
        else:
            nid = int(args.nodes)
            selected_nodes = [n for n in nodes if n["id"] == nid]
    else:
        # デフォルトは先頭30ノード
        if len(nodes) > 30:
            print("注意: --nodes が未指定のため、先頭30ノードを描画します。")
            selected_nodes = nodes[:30]

    output_path = resolve_output_path(
        bag_path,
        default_filename="lidar_scans_plot.png",
        output=args.output,
        output_to_bag_dir=args.output_to_bag_dir,
        output_dir=args.output_dir,
    )
    render_scans(
        selected_nodes,
        output_path,
        resolution=args.resolution,
        max_range=args.max_range,
    )


if __name__ == "__main__":
    main()
