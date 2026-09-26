#!/usr/bin/env python3
"""ホイールオドメトリの補正ノード。

/odom/raw (ドライバが出す生のオドメトリ) を購読し、スケールとバイアスを補正して積算し直した
オドメトリを /odom に出す。あわせて EKF が使う速度の共分散を設定する。
補正の計算は wheel_odom_correction.py にある。値は tools/scripts/calib_wheel_odom.py で
走行ログから推定する。

enabled が false のときは補正せず、生の値をそのまま通す (共分散は設定する)。
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wheel_odom_correction import OdomCorrector, TwistEstimator, shift_stamp, yaw_from_quaternion


class WheelOdomCorrectorNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odom_corrector_node')
        self._enabled = self.declare_parameter('enabled', True).value
        k_v = self.declare_parameter('k_v', 1.0).value
        k_w = self.declare_parameter('k_w', 1.0).value
        yaw_bias = self.declare_parameter('yaw_bias_per_meter', 0.0).value
        reset_jump = self.declare_parameter('reset_jump_m', 2.0).value
        self._time_offset = self.declare_parameter('time_offset', 0.0).value
        # 速度の出どころ。raw: ドライバの速度をスケール補正して使う / pose_diff: 補正した姿勢の差分から求める
        self._twist_source = self.declare_parameter('twist_source', 'raw').value
        if self._twist_source not in ('raw', 'pose_diff'):
            raise ValueError(f"twist_source must be 'raw' or 'pose_diff': {self._twist_source}")
        self._twist_estimator = TwistEstimator(self.declare_parameter('twist_window', 2).value)
        self._pose_cov = [
            self.declare_parameter('covariance_x', 10.0).value,
            self.declare_parameter('covariance_y', 10.0).value,
            self.declare_parameter('covariance_yaw', 0.4).value,
        ]
        self._twist_cov = [
            self.declare_parameter('covariance_vx', 0.01).value,
            self.declare_parameter('covariance_vy', 0.0001).value,
            self.declare_parameter('covariance_vyaw', 0.02).value,
        ]
        if self._enabled:
            self._corrector = OdomCorrector(k_v, k_w, yaw_bias, reset_jump)
        else:
            self._corrector = OdomCorrector(1.0, 1.0, 0.0, reset_jump)
            self._time_offset = 0.0

        self._pub = self.create_publisher(Odometry, 'odom', 10)
        self._sub = self.create_subscription(Odometry, 'odom/raw', self._on_odom, 10)
        self.get_logger().info(
            f'wheel_odom_corrector_node started: enabled={self._enabled} k_v={k_v} k_w={k_w} '
            f'yaw_bias_per_meter={yaw_bias} time_offset={self._time_offset} '
            f'twist_source={self._twist_source}')

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        x, y, yaw = self._corrector.update(
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quaternion(q.z, q.w))
        vx, wz = self._corrector.correct_twist(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self._twist_source == 'pose_diff':
            if self._corrector.was_reset:
                self._twist_estimator.reset()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            estimated = self._twist_estimator.update(stamp, x, y, yaw)
            # 窓がそろうまでの最初の数メッセージは、ドライバの速度を補正したものを使う
            if estimated is not None:
                vx, wz = estimated

        out = Odometry()
        out.header = msg.header
        out.header.stamp.sec, out.header.stamp.nanosec = shift_stamp(
            msg.header.stamp.sec, msg.header.stamp.nanosec, self._time_offset)
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        out.pose.covariance[0] = self._pose_cov[0]
        out.pose.covariance[7] = self._pose_cov[1]
        out.pose.covariance[35] = self._pose_cov[2]
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = wz
        out.twist.covariance[0] = self._twist_cov[0]
        out.twist.covariance[7] = self._twist_cov[1]
        out.twist.covariance[35] = self._twist_cov[2]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomCorrectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
