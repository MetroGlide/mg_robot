"""標準の入力のアダプタ。設定の sources.<名前>.input.type に書く登録名と対応する。

  pose_with_covariance : geometry_msgs/PoseWithCovarianceStamped -> pose
  odometry             : nav_msgs/Odometry -> pose (地図座標の推定器の出力) または motion (オドメトリ)
  map_scan             : nav_msgs/OccupancyGrid と sensor_msgs/LaserScan -> reference
  occupancy_grid       : nav_msgs/OccupancyGrid だけ -> reference (計画用地図など)
  particle_cloud       : nav2_msgs/ParticleCloud (AMCL の粒子) -> hypotheses (粒子を分けた峰)
"""
import math
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from ...core.geometry import Pose2
from ...core.grid import OccupancyMap
from ...core.interfaces import (
    Hypothesis, HypothesisSample, MotionSample, ParamSpec, PoseSample, ScanSample, SourceKind)
from ...core.particle_modes import cluster_particles
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


class OccupancyGridAdapter(SourceAdapter):
    """地図だけ (スキャンなし)。計画用地図のように、安全の信号にだけ使う地図。"""
    KINDS = (SourceKind.REFERENCE,)
    PARAMS = {
        'topic': ParamSpec(str, None),
        'occupied_threshold': ParamSpec(int, 65, 'これ以上の値のセルを占有 (致死) とする'),
    }

    def subscriptions(self) -> List[Subscription]:
        return [Subscription(self.params['topic'], OccupancyGrid, self._on_map, qos='latched')]

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.store.set_map(self.source, occupancy_from_msg(msg, self.params['occupied_threshold']))


class ParticleCloudAdapter(SourceAdapter):
    """粒子を、格子でつながる群れと向きの分布の谷で峰に分け、仮説の集合にする (core/particle_modes.py)。

    分けるのは min_interval_sec ごとに 1 回 (粒子の配信ごとには分けない。間の配信は捨てる)。
    AMCL はリサンプリングの後に粒子を出すので、仮説の重みは粒子の数の割合 (尤度の比ではない)。
    """
    KINDS = (SourceKind.HYPOTHESES,)
    PARAMS = {
        'topic': ParamSpec(str, None),
        'min_interval_sec': ParamSpec(float, 1.0, '粒子を分ける最短の間隔'),
        'cell_m': ParamSpec(float, 0.5, 'これより離れた粒子の群れは別の仮説'),
        'cell_yaw_rad': ParamSpec(float, 0.26),
        'yaw_bin_rad': ParamSpec(float, 0.17, '群れの中の向きの分布を見る区間の幅'),
        'valley_ratio': ParamSpec(float, 0.5, '向きの分布の谷が、低い方の山のこの割合未満なら分ける (0 で分けない)'),
    }

    def __init__(self, source, kind, params, store, context) -> None:
        super().__init__(source, kind, params, store, context)
        self._last_stamp: Optional[float] = None

    def subscriptions(self) -> List[Subscription]:
        return [Subscription(self.params['topic'], ParticleCloud, self._on_cloud, qos='sensor')]

    def _on_cloud(self, msg: ParticleCloud) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        p = self.params
        if self._last_stamp is not None and 0.0 <= stamp - self._last_stamp < p['min_interval_sec']:
            return
        if not msg.particles:
            return
        self._last_stamp = stamp
        poses = np.array([pose_from_msg(particle.pose) for particle in msg.particles], dtype=float)
        weights = np.array([particle.weight for particle in msg.particles], dtype=float)
        modes = cluster_particles(poses, weights, p['cell_m'], p['cell_yaw_rad'], p['yaw_bin_rad'], p['valley_ratio'])
        self.store.add_hypotheses(self.source, HypothesisSample(stamp, [
            Hypothesis(mode['weight'], tuple(float(v) for v in mode['mean']), mode['cov']) for mode in modes]))


STANDARD_ADAPTERS = {
    'pose_with_covariance': PoseWithCovarianceAdapter,
    'odometry': OdometryAdapter,
    'map_scan': MapScanAdapter,
    'occupancy_grid': OccupancyGridAdapter,
    'particle_cloud': ParticleCloudAdapter,
}
