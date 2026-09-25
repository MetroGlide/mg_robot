#!/usr/bin/env python3
"""
loc_init_pose.py

再生評価 (run_localization_variant.sh) で、AMCL と EKF の初期姿勢を真値 (pose_graph.json) から与える。
シミュレーション時刻 (/clock) が進み、odom->base の TF が届いてから、現在時刻の真値を
/initialpose (AMCL) と /set_pose (EKF) に 1 回ずつ送って終了する。

別走行を評価する場合は --map-gt-dir に地図を作った走行の SLAM 出力を渡し、真値をその座標系へ移す。
"""

import argparse
import math
import os
import sys

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.pose_graph import (  # noqa: E402
    interpolate_nodes,
    load_slam_output,
    shift_nodes_to_anchor,
)

# SLAM のキーフレームは 0.5 m 動くごとに作られるので、これ未満なら止まっているとみなして補間する
STATIONARY_DIST_M = 0.6

# 初期姿勢の共分散 (x, y は σ 0.3 m、yaw は σ 0.2 rad)
INITIAL_POSITION_VARIANCE = 0.09
INITIAL_YAW_VARIANCE = 0.04


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class InitPose(Node):
    def __init__(self, nodes: np.ndarray, args: argparse.Namespace) -> None:
        super().__init__(
            "loc_init_pose",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._nodes = nodes
        self._map_frame = args.map_frame
        self._odom_frame = args.odom_frame
        self._base_frame = args.base_frame
        self._timeout = args.timeout
        self._start = None
        self.done = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1)
        self._set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/set_pose", 1)
        self._timer = self.create_timer(0.5, self._on_timer)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        if self._start is None:
            self._start = now
        if (now - self._start).nanoseconds * 1e-9 > self._timeout:
            raise SystemExit("エラー: 初期姿勢を送る条件が時間内にそろいませんでした。")
        if (self._initialpose_pub.get_subscription_count() == 0
                or self._set_pose_pub.get_subscription_count() == 0):
            return
        if not self._tf_buffer.can_transform(self._odom_frame, self._base_frame, Time()):
            return
        poses, valid = interpolate_nodes(
            self._nodes, np.array([now.nanoseconds * 1e-9]), stationary_dist=STATIONARY_DIST_M)
        if not valid[0]:
            return
        x, y, yaw = (float(v) for v in poses[0])

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        msg.pose.covariance[0] = INITIAL_POSITION_VARIANCE
        msg.pose.covariance[7] = INITIAL_POSITION_VARIANCE
        msg.pose.covariance[35] = INITIAL_YAW_VARIANCE
        self._initialpose_pub.publish(msg)
        self._set_pose_pub.publish(msg)
        self.get_logger().info(f"初期姿勢を送信しました: x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
        self._timer.cancel()
        self.done = True


def main() -> None:
    parser = argparse.ArgumentParser(description="真値から AMCL と EKF の初期姿勢を与える")
    parser.add_argument("--gt-dir", required=True, help="真値の SLAM 出力ディレクトリ")
    parser.add_argument("--map-gt-dir", default=None, help="地図を作った走行の SLAM 出力ディレクトリ")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=60.0, help="待つ時間 (シミュレーション時刻) [s]")
    args = parser.parse_args()

    nodes, transform, _ = load_slam_output(args.gt_dir)
    if args.map_gt_dir:
        _, map_transform, _ = load_slam_output(args.map_gt_dir)
        nodes = shift_nodes_to_anchor(nodes, transform, map_transform)

    rclpy.init()
    node = InitPose(nodes, args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    # 送信したメッセージが相手へ届くまで少し待つ
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
