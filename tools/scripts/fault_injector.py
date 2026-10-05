#!/usr/bin/env python3
"""
fault_injector.py

自己位置推定の再生評価 (run_localization_variant.sh --faults) で、センサ入力に故障を注入する。
故障の定義 (YAML) は tools/common/faults.py を参照。時刻は再生開始 (最初の /fault/odom) からの経過秒。

rosbag の再生では、注入の対象のトピックを /fault/ 以下へ付け替えて配信し、このノードが中継する。
  /fault/odom            -> /odom/raw           odom_scale (ホイールのスリップ)
  /fault/navpvt          -> /navpvt             gnss_bias / gnss_drop
  /fault/scan_top_lidar  -> /scan_top_lidar     scan_drop
kidnap は、その時刻の推定姿勢 (EKF) をずらした姿勢を /initialpose に送り、AMCL だけをずらす。
"""

import argparse
import os
import sys
from typing import List, Optional

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from ublox_msgs.msg import NavPVT

# tools パッケージルートの解決と、ホイールオドメトリの補正計算 (ROS 非依存) の利用
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for path in (REPO_ROOT, os.path.join(REPO_ROOT, "mg_drivers", "scripts")):
    if path not in sys.path:
        sys.path.insert(0, path)

from tools.common import fault_apply as fa  # noqa: E402
from tools.common.faults import Fault, load_faults  # noqa: E402
from wheel_odom_correction import OdomCorrector, yaw_from_quaternion  # noqa: E402

# kidnap で送る姿勢の共分散 (AMCL に、ずらした位置を確かなものとして与える)
KIDNAP_POSITION_VARIANCE = 0.05
KIDNAP_YAW_VARIANCE = 0.02


class FaultInjector(Node):
    def __init__(self, faults: List[Fault], args: argparse.Namespace) -> None:
        super().__init__(
            "fault_injector",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._faults = faults
        self._t0: Optional[float] = None
        self._logged = set()
        self._kidnap_done = set()
        # 注入用のホイールオドメトリの積算。スリップで生じたずれは、その後も残る
        self._odom_integrator = OdomCorrector(1.0, 1.0, 0.0, reset_jump_m=1e9)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._map_frame = args.map_frame
        self._base_frame = args.base_frame

        self._odom_pub = self.create_publisher(Odometry, "/odom/raw", 10)
        self._navpvt_pub = self.create_publisher(NavPVT, "/navpvt", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan_top_lidar", qos_profile_sensor_data)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_subscription(Odometry, "/fault/odom", self._on_odom, 10)
        self.create_subscription(NavPVT, "/fault/navpvt", self._on_navpvt, 10)
        self.create_subscription(LaserScan, "/fault/scan_top_lidar", self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            "fault_injector started: " + ", ".join(f"{f.type}@{f.start:g}" for f in faults))

    def _t_rel(self) -> Optional[float]:
        if self._t0 is None:
            return None
        return self.get_clock().now().nanoseconds * 1e-9 - self._t0

    def _active(self, fault_type: str) -> Optional[Fault]:
        t = self._t_rel()
        if t is None:
            return None
        for index, fault in enumerate(self._faults):
            if fault.type == fault_type and fa.is_active(fault, t):
                if index not in self._logged:
                    self._logged.add(index)
                    self.get_logger().info(f"{fault.type} started at t={t:.1f}s {fault.params}")
                return fault
        return None

    def _on_odom(self, msg: Odometry) -> None:
        if self._t0 is None:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
        fault = self._active("odom_scale")
        scale_v = float(fault.params["v"]) if fault else 1.0
        scale_w = float(fault.params["w"]) if fault else 1.0
        self._odom_integrator.k_v = scale_v
        self._odom_integrator.k_w = scale_w
        q = msg.pose.pose.orientation
        x, y, yaw = self._odom_integrator.update(
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(q.z, q.w))
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        out.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        out.twist = msg.twist
        out.twist.twist.linear.x = msg.twist.twist.linear.x * scale_v
        out.twist.twist.angular.z = msg.twist.twist.angular.z * scale_w
        self._odom_pub.publish(out)

    def _on_navpvt(self, msg: NavPVT) -> None:
        if self._active("gnss_drop"):
            return
        fault = self._active("gnss_bias")
        if fault:
            bias = fault.params["bias"]
            lat, lon = fa.shift_latlon(
                msg.lat * 1e-7, msg.lon * 1e-7, float(bias.get("x", 0.0)), float(bias.get("y", 0.0)))
            msg.lat = int(round(lat * 1e7))
            msg.lon = int(round(lon * 1e7))
            if "carr_soln" in fault.params:
                msg.flags = fa.set_carrier_solution(msg.flags, fault.params["carr_soln"])
        self._navpvt_pub.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        fault = self._active("scan_drop")
        if fault:
            ranges = fa.mask_scan_sector(
                np.asarray(msg.ranges, dtype=float), msg.angle_min, msg.angle_increment,
                tuple(fault.params["sector_deg"]))
            msg.ranges = ranges.astype(np.float32).tolist()
        self._scan_pub.publish(msg)

    def _on_timer(self) -> None:
        t = self._t_rel()
        if t is None:
            return
        for index, fault in enumerate(self._faults):
            if fault.type != "kidnap" or index in self._kidnap_done or t < fault.start:
                continue
            try:
                transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
            except tf2_ros.TransformException:
                return
            offset = fault.params["offset"]
            tr = transform.transform.translation
            yaw = yaw_from_quaternion(transform.transform.rotation.z, transform.transform.rotation.w)
            new_yaw = yaw + float(offset.get("yaw", 0.0))
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.pose.pose.position.x = tr.x + float(offset.get("x", 0.0))
            msg.pose.pose.position.y = tr.y + float(offset.get("y", 0.0))
            msg.pose.pose.orientation.z = float(np.sin(new_yaw / 2.0))
            msg.pose.pose.orientation.w = float(np.cos(new_yaw / 2.0))
            msg.pose.covariance[0] = KIDNAP_POSITION_VARIANCE
            msg.pose.covariance[7] = KIDNAP_POSITION_VARIANCE
            msg.pose.covariance[35] = KIDNAP_YAW_VARIANCE
            self._initialpose_pub.publish(msg)
            self._kidnap_done.add(index)
            self.get_logger().info(f"kidnap at t={t:.1f}s offset={offset}")


def main() -> None:
    parser = argparse.ArgumentParser(description="自己位置推定の評価のために、センサ入力に故障を注入する")
    parser.add_argument("faults", help="故障定義 YAML")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args()

    rclpy.init()
    node = FaultInjector(load_faults(args.faults), args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
