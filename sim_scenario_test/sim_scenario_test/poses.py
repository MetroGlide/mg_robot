from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import rclpy.duration
import rclpy.time
import tf2_ros

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose, PoseSpec, yaw_from_quaternion

if TYPE_CHECKING:
    import rclpy.node


class RobotPoseSource(ABC):
    """ロボットの現在姿勢 (map 座標) の取得元。"""

    @abstractmethod
    def wait_available(self, timeout_sec: float) -> bool:
        ...

    @abstractmethod
    def get(self) -> Pose:
        ...


class TfRobotPoseSource(RobotPoseSource):
    """TF (map -> base) からロボット姿勢を取得する。"""

    def __init__(self, node: "rclpy.node.Node", map_frame: str, base_frame: str):
        self._map = map_frame
        self._base = base_frame
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, node)

    def wait_available(self, timeout_sec: float) -> bool:
        return self._buffer.can_transform(
            self._map, self._base, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout_sec))

    def get(self) -> Pose:
        try:
            tf = self._buffer.lookup_transform(
                self._map, self._base, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            raise ScenarioError(
                f"TF lookup {self._map} -> {self._base} failed: {e}") from e
        t = tf.transform.translation
        r = tf.transform.rotation
        return Pose(t.x, t.y, t.z, yaw_from_quaternion(r.x, r.y, r.z, r.w))


class PoseResolver:
    """PoseSpec を map / world 座標の Pose に解決する。"""

    def __init__(self, map_in_world: Pose, robot: RobotPoseSource):
        self._map_in_world = map_in_world
        self._world_in_map = map_in_world.inverse()
        self._robot = robot

    @property
    def robot(self) -> RobotPoseSource:
        return self._robot

    def to_map(self, spec: PoseSpec) -> Pose:
        if spec.frame == "map":
            return spec.as_pose()
        if spec.frame == "world":
            return self._world_in_map.compose(spec.as_pose())
        return self._robot.get().compose(spec.as_pose())

    def to_world(self, spec: PoseSpec) -> Pose:
        if spec.frame == "world":
            return spec.as_pose()
        return self._map_in_world.compose(self.to_map(spec))
