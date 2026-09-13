#!/usr/bin/env python3
"""デプス画像（およびカラー画像）から PointCloud2 を復元・生成するノード。

rosbag 再生時にデプス画像とカメラ情報から RealSense 互換の PointCloud2 を生成し、
障害物検知ノード (obstacle_detection_3d_node) や RViz に供給します。
"""

import struct
from typing import Optional

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        # パラメータ宣言
        self.declare_parameter('use_color', True)
        self.declare_parameter('depth_scale', 0.001)  # 16UC1 (mm) -> m
        self.declare_parameter('min_depth', 0.1)      # m
        self.declare_parameter('max_depth', 10.0)     # m
        self.declare_parameter('stride', 1)           # ダウンサンプリング幅 (1=全画素)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.05)          # 同期許容秒数

        self._use_color: bool = self.get_parameter('use_color').value
        self._depth_scale: float = self.get_parameter('depth_scale').value
        self._min_depth: float = self.get_parameter('min_depth').value
        self._max_depth: float = self.get_parameter('max_depth').value
        self._stride: int = max(1, self.get_parameter('stride').value)
        queue_size: int = self.get_parameter('queue_size').value
        slop: float = self.get_parameter('slop').value

        self._bridge = CvBridge()
        self._cached_shape: Optional[tuple[int, int]] = None
        self._u_grid: Optional[np.ndarray] = None
        self._v_grid: Optional[np.ndarray] = None

        # QoS設定
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # パブリッシャー
        self._pub_points = self.create_publisher(PointCloud2, 'points', 10)

        # サブスクリプション & 同期
        self._sub_depth = message_filters.Subscriber(
            self, Image, 'image_rect', qos_profile=qos)
        self._sub_depth_info = message_filters.Subscriber(
            self, CameraInfo, 'depth_camera_info', qos_profile=qos)

        if self._use_color:
            self._sub_color = message_filters.Subscriber(
                self, Image, 'image_color', qos_profile=qos)
            self._sub_color_info = message_filters.Subscriber(
                self, CameraInfo, 'color_camera_info', qos_profile=qos)

            self._sync = message_filters.ApproximateTimeSynchronizer(
                [self._sub_depth, self._sub_depth_info, self._sub_color, self._sub_color_info],
                queue_size=queue_size,
                slop=slop,
            )
            self._sync.registerCallback(self._on_synced_color_depth)
            self.get_logger().info('Initialized depth_to_pointcloud_node with XYZRGB mode')
        else:
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [self._sub_depth, self._sub_depth_info],
                queue_size=queue_size,
                slop=slop,
            )
            self._sync.registerCallback(self._on_synced_depth_only)
            self.get_logger().info('Initialized depth_to_pointcloud_node with XYZ only mode')

    def _init_grid(self, h: int, w: int, fx: float, fy: float, cx: float, cy: float):
        if self._cached_shape == (h, w):
            return
        u = np.arange(0, w, self._stride, dtype=np.float32)
        v = np.arange(0, h, self._stride, dtype=np.float32)
        u_grid, v_grid = np.meshgrid(u, v)
        self._u_norm = (u_grid - cx) / fx
        self._v_norm = (v_grid - cy) / fy
        self._cached_shape = (h, w)

    def _on_synced_color_depth(
        self,
        depth_msg: Image,
        depth_info: CameraInfo,
        color_msg: Image,
        color_info: CameraInfo,
    ):
        try:
            depth_img = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            color_img = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert images: {e}')
            return

        h, w = depth_img.shape[:2]
        fx = depth_info.k[0]
        cx = depth_info.k[2]
        fy = depth_info.k[4]
        cy = depth_info.k[5]

        if fx == 0 or fy == 0:
            return

        self._init_grid(h, w, fx, fy, cx, cy)

        # デプス値の単位変換 (m)
        depth_m = depth_img[::self._stride, ::self._stride].astype(np.float32) * self._depth_scale

        valid = (depth_m > self._min_depth) & (depth_m < self._max_depth) & np.isfinite(depth_m)
        if not np.any(valid):
            return

        z = depth_m[valid]
        x = self._u_norm[valid] * z
        y = self._v_norm[valid] * z

        # カラー画像から RGB 取得
        # RealSense aligned_depth_to_color の場合、デプスとカラーは同一直線・同一解像度
        color_sub = color_img[::self._stride, ::self._stride]
        bgr = color_sub[valid]
        r = bgr[:, 2].astype(np.uint32)
        g = bgr[:, 1].astype(np.uint32)
        b = bgr[:, 0].astype(np.uint32)

        # uint32 に pack された rgb 値
        rgb_packed = (r << 16) | (g << 8) | b
        rgb_float = rgb_packed.view(np.float32)

        # (x, y, z, rgb) の構造化配列
        cloud_data = np.zeros(
            len(z),
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
        )
        cloud_data['x'] = x
        cloud_data['y'] = y
        cloud_data['z'] = z
        cloud_data['rgb'] = rgb_float

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = depth_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id

        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self._pub_points.publish(cloud_msg)

    def _on_synced_depth_only(self, depth_msg: Image, depth_info: CameraInfo):
        try:
            depth_img = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
            return

        h, w = depth_img.shape[:2]
        fx = depth_info.k[0]
        cx = depth_info.k[2]
        fy = depth_info.k[4]
        cy = depth_info.k[5]

        if fx == 0 or fy == 0:
            return

        self._init_grid(h, w, fx, fy, cx, cy)

        depth_m = depth_img[::self._stride, ::self._stride].astype(np.float32) * self._depth_scale
        valid = (depth_m > self._min_depth) & (depth_m < self._max_depth) & np.isfinite(depth_m)
        if not np.any(valid):
            return

        z = depth_m[valid]
        x = self._u_norm[valid] * z
        y = self._v_norm[valid] * z

        cloud_data = np.zeros(
            len(z),
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)]
        )
        cloud_data['x'] = x
        cloud_data['y'] = y
        cloud_data['z'] = z

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = depth_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id

        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self._pub_points.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
