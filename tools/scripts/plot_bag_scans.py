#!/usr/bin/env python3
"""
rosbag内のオドメトリおよびLiDARスキャン点群を2D平面にプロットして可視化する汎用スクリプト。

使用例:
  python3 tools/scripts/plot_bag_scans.py \
    --bag /root/ros2_data/rosbag/TC2026/20260913/record_slam_20260913_043837 \
    --start-node 700 --end-node 730 \
    --output /app/scans_plot_700_730.png
"""

import argparse
import math
import os
import sys
import numpy as np

try:
    import cv2
except ImportError:
    print("Error: opencv-python is required. Run: pip install opencv-python", file=sys.stderr)
    sys.exit(1)

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
except ImportError:
    print("Error: ROS2 Python libraries (rosbag2_py, rclpy, sensor_msgs, nav_msgs) are required.", file=sys.stderr)
    print("Run inside Docker container: source /opt/ros/humble/setup.bash", file=sys.stderr)
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Plot odometry and LiDAR scans from a rosbag.")
    parser.add_argument("--bag", required=True, help="Path to ROS2 bag directory or mcap file")
    parser.add_argument("--start-node", type=int, default=700, help="Starting node index (default: 700)")
    parser.add_argument("--end-node", type=int, default=730, help="Ending node index (default: 730)")
    parser.add_argument("--output", default="scans_plot.png", help="Output PNG path (default: scans_plot.png)")
    parser.add_argument("--lidar-yaw-deg", type=float, default=180.0, help="LiDAR yaw mounting angle in degrees (default: 180.0)")
    parser.add_argument("--lidar-x", type=float, default=0.230, help="LiDAR x mounting offset in meters (default: 0.230)")
    parser.add_argument("--scale", type=float, default=20.0, help="Pixels per meter (default: 20.0, i.e. 0.05m/pixel)")
    parser.add_argument("--img-size", type=int, default=1600, help="Output image width/height (default: 1600)")
    parser.add_argument("--min-trans", type=float, default=0.2, help="Keyframe translation threshold (default: 0.2)")
    parser.add_argument("--min-rot", type=float, default=0.1, help="Keyframe rotation threshold (default: 0.1)")
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.bag):
        print(f"Error: Bag path does not exist: {args.bag}", file=sys.stderr)
        sys.exit(1)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    node_count = 0
    last_node_odom = None
    last_odom = None
    target_nodes = []

    lidar_yaw_rad = math.radians(args.lidar_yaw_deg)

    print(f"Reading bag: {args.bag}")
    print(f"Target node range: [{args.start_node}, {args.end_node}]")

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/odom":
            msg = deserialize_message(data, Odometry)
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            last_odom = (t * 1e-9, px, py, yaw)

        elif topic == "/scan_top_lidar":
            if last_odom is None:
                continue
            msg = deserialize_message(data, LaserScan)
            ot, ox, oy, oyaw = last_odom

            if last_node_odom is None:
                node_count += 1
                last_node_odom = (ox, oy, oyaw)
                continue

            lx, ly, lyaw = last_node_odom
            dx_w = ox - lx
            dy_w = oy - ly
            dist = math.hypot(dx_w, dy_w)
            dyaw = (oyaw - lyaw + math.pi) % (2.0 * math.pi) - math.pi

            if dist >= args.min_trans or abs(dyaw) >= args.min_rot:
                node_count += 1
                last_node_odom = (ox, oy, oyaw)

                if args.start_node <= node_count <= args.end_node:
                    ranges = [0.0 if math.isinf(r) or math.isnan(r) else float(r) for r in msg.ranges]
                    target_nodes.append({
                        "node": node_count,
                        "t": ot,
                        "odom": (ox, oy, oyaw),
                        "angle_min": msg.angle_min + lidar_yaw_rad,
                        "angle_inc": msg.angle_increment,
                        "range_min": msg.range_min,
                        "range_max": msg.range_max,
                        "ranges": ranges
                    })

                if node_count > args.end_node:
                    break

    if not target_nodes:
        print(f"No nodes found in range [{args.start_node}, {args.end_node}]. Total nodes: {node_count}")
        sys.exit(1)

    print(f"Collected {len(target_nodes)} nodes. Rendering image...")

    img = np.full((args.img_size, args.img_size, 3), 255, dtype=np.uint8)
    center = args.img_size // 2

    # 原点を先頭ノードの位置とする
    first_ox, first_oy, _ = target_nodes[0]["odom"]

    palette = [
        (0, 0, 255),    # 赤
        (0, 140, 255),  # 橙
        (0, 215, 255),  # 金
        (0, 255, 0),    # 緑
        (255, 100, 0),  # 青
        (255, 0, 255),  # マゼンタ
        (128, 0, 128),  # 紫
        (128, 128, 0),  # オリーブ
        (0, 128, 128),  # ティール
        (50, 50, 50),   # 濃灰
    ]

    for idx, node in enumerate(target_nodes):
        color = palette[idx % len(palette)]
        ox, oy, oyaw = node["odom"]
        dx_w = ox - first_ox
        dy_w = oy - first_oy

        rx = int(center + dx_w * args.scale)
        ry = int(center - dy_w * args.scale)

        cv2.circle(img, (rx, ry), 4, color, -1)
        cv2.putText(img, str(node["node"]), (rx + 6, ry - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        amin = node["angle_min"]
        ainc = node["angle_inc"]
        rmin = node["range_min"]
        rmax = node["range_max"]
        ranges = node["ranges"]

        for i, r in enumerate(ranges):
            if rmin <= r <= rmax and r < 30.0:
                th = oyaw + amin + i * ainc
                px_w = ox + r * math.cos(th) + args.lidar_x * math.cos(oyaw) - first_ox
                py_w = oy + r * math.sin(th) + args.lidar_x * math.sin(oyaw) - first_oy
                ix = int(center + px_w * args.scale)
                iy = int(center - py_w * args.scale)
                if 0 <= ix < args.img_size and 0 <= iy < args.img_size:
                    img[iy, ix] = color

    cv2.imwrite(args.output, img)
    print(f"Successfully saved plot to: {args.output}")


if __name__ == "__main__":
    main()
