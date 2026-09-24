"""Nav2 の共通操作 (初期位置設定・コストマップクリア・lifecycle 確認)。"""
from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Trigger

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose, quaternion_from_yaw

if TYPE_CHECKING:
    import rclpy.node
    from sim_scenario_test.profile import FramesSpec, Nav2Spec


def call_service(node: "rclpy.node.Node", client, request, timeout_sec: float):
    """サービスを同期的に呼び出す。別スレッドで executor が spin している前提。"""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise ScenarioError(f"service {client.srv_name} not available")
    future = client.call_async(request)
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout=timeout_sec) or future.result() is None:
        raise ScenarioError(f"service {client.srv_name} call timed out")
    return future.result()


class Nav2Interface:
    def __init__(self, node: "rclpy.node.Node", spec: "Nav2Spec", frames: "FramesSpec"):
        self._node = node
        self._spec = spec
        self._frames = frames
        self._initialpose_pub = node.create_publisher(
            PoseWithCovarianceStamped, spec.initialpose_topic, 10)
        self._clear_clients = [
            node.create_client(ClearEntireCostmap, name)
            for name in spec.clear_costmap_services
        ]

    def publish_initial_pose(self, pose_map: Pose) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frames.map
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_map.x
        msg.pose.pose.position.y = pose_map.y
        qx, qy, qz, qw = quaternion_from_yaw(pose_map.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = self._spec.initialpose_cov_xy
        msg.pose.covariance[7] = self._spec.initialpose_cov_xy
        msg.pose.covariance[35] = self._spec.initialpose_cov_yaw
        self._initialpose_pub.publish(msg)

    def clear_costmaps(self, timeout_sec: float = 5.0) -> None:
        for client in self._clear_clients:
            call_service(self._node, client, ClearEntireCostmap.Request(), timeout_sec)

    def lifecycle_active(self, manager: str, timeout_sec: float) -> bool:
        client = self._node.create_client(Trigger, f"/{manager}/is_active")
        try:
            return call_service(
                self._node, client, Trigger.Request(), timeout_sec).success
        except ScenarioError:
            return False
        finally:
            self._node.destroy_client(client)
