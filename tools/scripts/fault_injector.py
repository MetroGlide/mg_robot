#!/usr/bin/env python3
"""
fault_injector.py

自己位置推定の再生評価 (run_localization_variant.sh --faults) で、センサ入力に故障を注入する。
故障の定義 (YAML) は tools/common/faults.py を参照。時刻は再生開始 (最初の /fault/odom) からの経過秒。

rosbag の再生では、注入の対象のトピックを /fault/ 以下へ付け替えて配信し、このノードが中継する。
  /fault/odom            -> /odom/raw           odom_scale (ホイールのスリップ)
  /fault/navpvt          -> /navpvt             gnss_bias / gnss_bias_field / gnss_drop / gnss_noise / gnss_accuracy
  /fault/gps/fix         -> /gps/fix            同上 (Gazebo の navsat)
  /fault/scan_top_lidar  -> /scan_top_lidar     scan_drop / scan_keep / scan_thin / scan_range / scan_jitter /
                                              scan_occlude
Gazebo では、ブリッジの出力を /fault/ 以下にして、このノードが中継する (--odom-out /odom)。
kidnap は、その時刻の推定姿勢 (EKF) をずらした姿勢を /initialpose に送り、AMCL だけをずらす。
amcl_drift は、区間の中で interval 秒ごとに、AMCL の推定 (/amcl_pose_origin) を rate * interval だけずらした
姿勢を /initialpose に送り、AMCL を少しずつずらす (送るたびに粒子は送った姿勢の周りに撒き直される)。
--allow-triggers のとき、trigger: <名前> の故障は、サービス /fault_injector/trigger/<名前> (std_srvs/SetBool)
で入れ切りする (Gazebo のシナリオの call_set_bool から呼ぶ)。
"""

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, NavSatFix
from std_srvs.srv import SetBool
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
        self._abs_w = 0.0
        self._rng = np.random.default_rng()
        self._gnss_noise = fa.GnssNoise(self._rng)
        self._scan_occluder = fa.ScanOccluder(self._rng)
        self._kidnap_done = set()
        self._amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._drift_last: Dict[int, float] = {}
        # 注入用のホイールオドメトリの積算。スリップで生じたずれは、その後も残る
        self._odom_integrator = OdomCorrector(1.0, 1.0, 0.0, reset_jump_m=1e9)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._map_frame = args.map_frame
        self._base_frame = args.base_frame

        self._odom_pub = self.create_publisher(Odometry, args.odom_out, 10)
        self._gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self._navpvt_pub = self.create_publisher(NavPVT, "/navpvt", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan_top_lidar", qos_profile_sensor_data)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.create_subscription(Odometry, "/fault/odom", self._on_odom, 10)
        self.create_subscription(NavPVT, "/fault/navpvt", self._on_navpvt, 10)
        self.create_subscription(NavSatFix, "/fault/gps/fix", self._on_navsat, 10)
        self.create_subscription(LaserScan, "/fault/scan_top_lidar", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose_origin", self._on_amcl, 10)
        self.create_timer(0.1, self._on_timer)
        # 外からの指示で入れ切りする故障の、いまの入/切
        self._triggers: Dict[str, bool] = {}
        for name in sorted({f.trigger for f in faults if f.trigger is not None}):
            self._triggers[name] = False
            self.create_service(SetBool, f"~/trigger/{name}", self._trigger_callback(name))
        self.get_logger().info(
            "fault_injector started: " + ", ".join(
                f"{f.type}@{f.trigger}" if f.trigger else f"{f.type}@{f.start:g}" for f in faults))

    def _trigger_callback(self, name: str):
        def callback(request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            self._triggers[name] = bool(request.data)
            self.get_logger().info(f"trigger {name} -> {'on' if request.data else 'off'} at t={self._t_rel()}")
            response.success = True
            return response
        return callback

    def _t_rel(self) -> Optional[float]:
        if self._t0 is None:
            return None
        return self.get_clock().now().nanoseconds * 1e-9 - self._t0

    def _active(self, fault_type: str) -> Optional[Fault]:
        t = self._t_rel()
        if t is None:
            return None
        for index, fault in enumerate(self._faults):
            if fault.trigger is not None and not self._triggers[fault.trigger]:
                continue
            if fault.type == fault_type and fa.is_active(fault, t) and fa.is_gated_on(fault, self._abs_w):
                if index not in self._logged:
                    self._logged.add(index)
                    self.get_logger().info(f"{fault.type} started at t={t:.1f}s {fault.params}")
                return fault
        return None

    def _on_odom(self, msg: Odometry) -> None:
        # /clock を受け取る前の時計は 0 を返すので、それを起点にすると故障の時刻がずれる
        now_ns = self.get_clock().now().nanoseconds
        if self._t0 is None and now_ns > 0:
            self._t0 = now_ns * 1e-9
        self._abs_w = abs(msg.twist.twist.angular.z)
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

    def _gnss_offset(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        """いま作用している GNSS のバイアスとノイズの合計 (東, 北) [m]。lat_deg・lon_deg はずらす前の位置。"""
        dx, dy = 0.0, 0.0
        fault = self._active("gnss_bias")
        if fault:
            dx += float(fault.params["bias"].get("x", 0.0))
            dy += float(fault.params["bias"].get("y", 0.0))
        fault = self._active("gnss_bias_field")
        if fault:
            anchor = fault.params["anchor"]
            x, y = fa.local_east_north(lat_deg, lon_deg, float(anchor["latitude"]), float(anchor["longitude"]))
            bx, by = fa.bias_field(fault.params["points"], x, y, float(fault.params.get("length_m", 10.0)))
            dx += bx
            dy += by
        fault = self._active("gnss_noise")
        if fault:
            noise = self._gnss_noise.sample(self._t_rel(), float(fault.params["sigma"]),
                                            float(fault.params.get("tau", 0.0)))
            dx += float(noise[0])
            dy += float(noise[1])
        return dx, dy

    def _on_navpvt(self, msg: NavPVT) -> None:
        if self._active("gnss_drop"):
            return
        dx, dy = self._gnss_offset(msg.lat * 1e-7, msg.lon * 1e-7)
        if dx != 0.0 or dy != 0.0:
            lat, lon = fa.shift_latlon(msg.lat * 1e-7, msg.lon * 1e-7, dx, dy)
            msg.lat = int(round(lat * 1e7))
            msg.lon = int(round(lon * 1e7))
        fault = self._active("gnss_bias")
        if fault and "carr_soln" in fault.params:
            msg.flags = fa.set_carrier_solution(msg.flags, fault.params["carr_soln"])
        fault = self._active("gnss_accuracy")
        if fault:
            # h_acc は mm
            msg.h_acc = int(round(float(fault.params["std_m"]) * 1000.0))
        self._navpvt_pub.publish(msg)

    def _on_navsat(self, msg: NavSatFix) -> None:
        if self._active("gnss_drop"):
            return
        dx, dy = self._gnss_offset(msg.latitude, msg.longitude)
        if dx != 0.0 or dy != 0.0:
            msg.latitude, msg.longitude = fa.shift_latlon(msg.latitude, msg.longitude, dx, dy)
        fault = self._active("gnss_accuracy")
        if fault:
            msg.position_covariance = fa.horizontal_covariance(
                list(msg.position_covariance), float(fault.params["std_m"]))
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        changed = False
        fault = self._active("scan_drop")
        if fault:
            ranges = fa.mask_scan_sector(ranges, msg.angle_min, msg.angle_increment,
                                         tuple(fault.params["sector_deg"]))
            changed = True
        fault = self._active("scan_keep")
        if fault:
            ranges = fa.keep_scan_sectors(ranges, msg.angle_min, msg.angle_increment, fault.params["sectors_deg"])
            changed = True
        fault = self._active("scan_range")
        if fault:
            ranges = fa.limit_scan_range(ranges, float(fault.params["max_range"]))
            changed = True
        fault = self._active("scan_occlude")
        if fault:
            p = fault.params
            ranges = self._scan_occluder.apply(
                ranges, msg.angle_min, msg.angle_increment, self._t_rel(), int(p["count"]), float(p["width_m"]),
                p["range_m"], p.get("sectors_deg", [[-180.0, 180.0]]), float(p.get("hold_sec", 2.0)))
            changed = True
        fault = self._active("scan_jitter")
        if fault:
            ranges = fa.jitter_scan(ranges, float(fault.params["ratio"]), float(fault.params["depth_m"]), self._rng)
            changed = True
        fault = self._active("scan_thin")
        if fault:
            ranges = fa.thin_scan(ranges, int(fault.params["keep_every"]))
            changed = True
        if changed:
            msg.ranges = ranges.astype(np.float32).tolist()
        self._scan_pub.publish(msg)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg

    def _publish_initialpose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        msg.pose.covariance[0] = KIDNAP_POSITION_VARIANCE
        msg.pose.covariance[7] = KIDNAP_POSITION_VARIANCE
        msg.pose.covariance[35] = KIDNAP_YAW_VARIANCE
        self._initialpose_pub.publish(msg)

    def _drift(self, t: float) -> None:
        for index, fault in enumerate(self._faults):
            if fault.type != "amcl_drift" or not fa.is_active(fault, t) or self._amcl_pose is None:
                continue
            interval = float(fault.params.get("interval", 1.0))
            if t - self._drift_last.get(index, fault.start - interval) < interval:
                continue
            self._drift_last[index] = t
            rate = fault.params["rate"]
            pose = self._amcl_pose.pose.pose
            yaw = yaw_from_quaternion(pose.orientation.z, pose.orientation.w)
            self._publish_initialpose(
                pose.position.x + float(rate.get("x", 0.0)) * interval,
                pose.position.y + float(rate.get("y", 0.0)) * interval,
                yaw + float(rate.get("yaw", 0.0)) * interval)

    def _on_timer(self) -> None:
        t = self._t_rel()
        if t is None:
            return
        self._drift(t)
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
            self._publish_initialpose(
                tr.x + float(offset.get("x", 0.0)), tr.y + float(offset.get("y", 0.0)),
                yaw + float(offset.get("yaw", 0.0)))
            self._kidnap_done.add(index)
            self.get_logger().info(f"kidnap at t={t:.1f}s offset={offset}")


def main() -> None:
    parser = argparse.ArgumentParser(description="自己位置推定の評価のために、センサ入力に故障を注入する")
    parser.add_argument("faults", help="故障定義 YAML")
    parser.add_argument("--odom-out", default="/odom/raw", help="中継したオドメトリの出力先 (Gazebo では /odom)")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--allow-triggers", action="store_true",
                        help="trigger の故障を許す (Gazebo のシナリオ。再生評価では時刻でしか入れられない)")
    args = parser.parse_args()

    rclpy.init()
    node = FaultInjector(load_faults(args.faults, args.allow_triggers), args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
