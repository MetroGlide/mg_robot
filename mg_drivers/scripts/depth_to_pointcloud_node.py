#!/usr/bin/env python3
"""デプス画像（およびカラー画像）から PointCloud2 を復元・生成するノード。

rosbag 再生時にデプス画像とカメラ情報から RealSense 互換の PointCloud2 を生成し、
障害物検知ノード (obstacle_detection_3d_node) や RViz に供給します。
"""

from typing import Optional

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

from mg_utils.point_cloud import build_point_array


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
            color_img = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert images: {e}')
            return
        # RealSense aligned_depth_to_color の場合、デプスとカラーは同一解像度
        self._publish_points(depth_msg, depth_info, color_img)

    def _on_synced_depth_only(self, depth_msg: Image, depth_info: CameraInfo):
        self._publish_points(depth_msg, depth_info, None)

    def _publish_points(
        self,
        depth_msg: Image,
        depth_info: CameraInfo,
        color_img: Optional[np.ndarray],
    ):
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

        # デプス値の単位変換 (m)
        depth_m = depth_img[::self._stride, ::self._stride].astype(np.float32) * self._depth_scale
        bgr = None if color_img is None else color_img[::self._stride, ::self._stride]

        points = build_point_array(
            depth_m, self._u_norm, self._v_norm, self._min_depth, self._max_depth, bgr)
        if points is None:
            return

        fields = [
            PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z', 'rgb')[:points.shape[1]])
        ]

        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.stamp = depth_msg.header.stamp
        cloud_msg.header.frame_id = depth_msg.header.frame_id
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 4 * points.shape[1]
        cloud_msg.row_step = cloud_msg.point_step * points.shape[0]
        cloud_msg.is_dense = True
        # 点ごとの Python ループを避け、配列のバイト列をそのまま格納する
        cloud_msg.data = points.tobytes()
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
