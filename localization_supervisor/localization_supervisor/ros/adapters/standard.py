"""標準の入力のアダプタ。設定の sources.<名前>.input.type に書く登録名と対応する。

  pose_with_covariance : geometry_msgs/PoseWithCovarianceStamped -> pose
  odometry             : nav_msgs/Odometry -> pose (地図座標の推定器の出力) または motion (オドメトリ)
  map_scan             : nav_msgs/OccupancyGrid と sensor_msgs/LaserScan -> reference
"""
import math
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from ...core.geometry import Pose2
from ...core.grid import OccupancyMap
from ...core.interfaces import MotionSample, ParamSpec, PoseSample, ScanSample, SourceKind
from .base import SourceAdapter, Subscription

# geometry_msgs の 6x6 の共分散 (行優先) のうち、x, y, yaw の添字
_XYYAW = (0, 1, 5)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_from_msg(pose) -> Pose2:
    return (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))


def covariance_xyyaw(covariance) -> np.ndarray:
    full = np.asarray(covariance, dtype=float).reshape(6, 6)
    return full[np.ix_(_XYYAW, _XYYAW)]


def occupancy_from_msg(msg: OccupancyGrid, occupied_threshold: int = 65) -> OccupancyMap:
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return OccupancyMap(data, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                        occupied_threshold)


class PoseWithCovarianceAdapter(SourceAdapter):
    KINDS = (SourceKind.POSE,)
    PARAMS = {'topic': ParamSpec(str, None)}

    def subscriptions(self) -> List[Subscription]:
        return [Subscription(self.params['topic'], PoseWithCovarianceStamped, self._on_pose)]

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.store.add_pose(self.source, PoseSample(
            stamp_to_sec(msg.header.stamp), pose_from_msg(msg.pose.pose), covariance_xyyaw(msg.pose.covariance)))


class OdometryAdapter(SourceAdapter):
    KINDS = (SourceKind.POSE, SourceKind.MOTION)
    PARAMS = {'topic': ParamSpec(str, None)}

    def subscriptions(self) -> List[Subscription]:
        return [Subscription(self.params['topic'], Odometry, self._on_odometry)]

    def _on_odometry(self, msg: Odometry) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        pose = pose_from_msg(msg.pose.pose)
        if self.kind == SourceKind.MOTION:
            self.store.add_motion(self.source, MotionSample(stamp, pose))
        else:
            self.store.add_pose(self.source, PoseSample(stamp, pose, covariance_xyyaw(msg.pose.covariance)))


class MapScanAdapter(SourceAdapter):
    """地図とスキャン。センサの取り付け位置は sensor_pose ([x, y, yaw]) か、TF (base_frame -> スキャンの frame)。"""
    KINDS = (SourceKind.REFERENCE,)
    PARAMS = {
        'map_topic': ParamSpec(str, '/map'),
        'scan_topic': ParamSpec(str, None),
        'base_frame': ParamSpec(str, 'base_link'),
        'sensor_pose': ParamSpec(list, [], '空なら TF から引く'),
        'occupied_threshold': ParamSpec(int, 65),
    }

    def __init__(self, source, kind, params, store, context) -> None:
        super().__init__(source, kind, params, store, context)
        fixed = self.params['sensor_pose']
        if fixed and len(fixed) != 3:
            raise ValueError(f'{source}: sensor_pose must be [x, y, yaw]')
        self._sensor_pose: Optional[Pose2] = tuple(float(v) for v in fixed) if fixed else None

    def subscriptions(self) -> List[Subscription]:
        return [Subscription(self.params['map_topic'], OccupancyGrid, self._on_map, qos='latched'),
                Subscription(self.params['scan_topic'], LaserScan, self._on_scan, qos='sensor')]

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.store.set_map(self.source, occupancy_from_msg(msg, self.params['occupied_threshold']))

    def _on_scan(self, msg: LaserScan) -> None:
        if self._sensor_pose is None:
            self._sensor_pose = self.context.lookup(self.params['base_frame'], msg.header.frame_id)
        self.store.set_scan(self.source, ScanSample(
            stamp_to_sec(msg.header.stamp), np.asarray(msg.ranges, dtype=float), msg.angle_min,
            msg.angle_increment, msg.range_min, msg.range_max, self._sensor_pose))


STANDARD_ADAPTERS = {
    'pose_with_covariance': PoseWithCovarianceAdapter,
    'odometry': OdometryAdapter,
    'map_scan': MapScanAdapter,
}
