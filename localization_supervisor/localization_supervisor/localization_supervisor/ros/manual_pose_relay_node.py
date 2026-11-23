"""手動の初期姿勢の中継: 専用のトピックの姿勢を、そのまま推定器の初期姿勢のトピックへ送る。

門のノードを動かさないときに、手動の初期姿勢を従来どおり推定器に届ける (doc/motion_gate.md)。

  パラメータ
    input_topic  : 手動の初期姿勢の専用のトピック
    output_topic : 推定器の初期姿勢のトピック
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ManualPoseRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_pose_relay')
        input_topic = self.declare_parameter('input_topic', '/localization/manual_initial_pose').value
        output_topic = self.declare_parameter('output_topic', '/initialpose').value
        self._pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, input_topic, self._pub.publish, 10)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualPoseRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
