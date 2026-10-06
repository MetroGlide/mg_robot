#!/usr/bin/env python3
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rosidl_runtime_py.utilities import get_message


class TopicRateMonitor:
    """トピックの配信レートを滑動平均で計測する。"""

    def __init__(self, window: int = 10):
        self._timestamps: deque = deque(maxlen=window)
        self._lock = threading.Lock()

    def tick(self) -> None:
        with self._lock:
            self._timestamps.append(time.monotonic())

    def hz(self) -> float:
        with self._lock:
            if len(self._timestamps) < 2:
                return 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            if span <= 0.0:
                return 0.0
            return (len(self._timestamps) - 1) / span


_BEST_EFFORT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# トピックの QoS。診断ごとに params の qos で選ぶ
_QOS_PROFILES = {
    "best_effort": _BEST_EFFORT_QOS,
    "reliable": 10,
}

# 診断の values に入れる group の値。UI はこれでセンサとその他のトピックを分ける
TOPIC_GROUPS = ("sensor", "topic")


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        self._declare_parameters()

        self._rate_monitors: dict[str, TopicRateMonitor] = {}
        for topic in self._monitored_topics:
            name = topic["name"]
            monitor = TopicRateMonitor()
            self._rate_monitors[name] = monitor
            self.create_subscription(
                get_message(topic["type"]),
                name,
                lambda msg, m=monitor: m.tick(),
                _QOS_PROFILES[topic["qos"]],
            )

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish)

    def _declare_parameters(self) -> None:
        self.declare_parameter('monitored_nodes', [
            'waypoint_sequencer_node',
            'amcl',
            'bt_navigator',
            'controller_server',
            'planner_server',
            'collision_monitor',
            'motor_driver_node',
        ])
        self.declare_parameter('hz_warn_ratio', 0.5)

        self._monitored_nodes: list[str] = list(
            self.get_parameter(
                'monitored_nodes').get_parameter_value().string_array_value
        )
        self._hz_warn_ratio: float = (
            self.get_parameter(
                'hz_warn_ratio').get_parameter_value().double_value
        )

        # 監視するトピックは、ID の一覧 (monitored_topics) と、ID ごとの設定で指定する
        self.declare_parameter('monitored_topics', Parameter.Type.STRING_ARRAY)
        topic_ids = list(
            self.get_parameter(
                'monitored_topics').get_parameter_value().string_array_value
        )
        self._monitored_topics: list[dict] = [
            self._declare_topic(topic_id) for topic_id in topic_ids
        ]

    def _declare_topic(self, topic_id: str) -> dict:
        """monitored_topics の 1 件分の設定を読む。設定が欠けていればそのまま例外にする。"""
        self.declare_parameter(f'{topic_id}.topic', Parameter.Type.STRING)
        self.declare_parameter(f'{topic_id}.type', Parameter.Type.STRING)
        self.declare_parameter(f'{topic_id}.expected_hz', Parameter.Type.DOUBLE)
        self.declare_parameter(f'{topic_id}.qos', 'reliable')
        self.declare_parameter(f'{topic_id}.group', 'topic')

        def get(key: str):
            return self.get_parameter(f'{topic_id}.{key}').value

        topic = {
            "name": get('topic'),
            "type": get('type'),
            "expected_hz": get('expected_hz'),
            "qos": get('qos'),
            "group": get('group'),
        }
        if topic["qos"] not in _QOS_PROFILES:
            raise ValueError(
                f'{topic_id}.qos は {sorted(_QOS_PROFILES)} のいずれか: {topic["qos"]}')
        if topic["group"] not in TOPIC_GROUPS:
            raise ValueError(
                f'{topic_id}.group は {list(TOPIC_GROUPS)} のいずれか: {topic["group"]}')
        return topic

    def _make_hz_status(
            self, name: str, group: str, hz: float, expected: float) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'topic/{name}'
        status.hardware_id = 'ros2_topic'
        status.values.append(KeyValue(key='group', value=group))
        status.values.append(KeyValue(key='hz', value=f'{hz:.2f}'))
        status.values.append(
            KeyValue(key='expected_hz', value=f'{expected:.2f}'))

        if hz >= expected * self._hz_warn_ratio:
            status.level = DiagnosticStatus.OK
            status.message = f'{hz:.1f} Hz'
        elif hz > 0.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'low rate: {hz:.1f} Hz (expected {expected:.1f})'
        else:
            status.level = DiagnosticStatus.WARN
            status.message = 'no data'

        return status

    def _check_nodes(self) -> list[DiagnosticStatus]:
        alive_names = {name for name,
                       _ in self.get_node_names_and_namespaces()}
        statuses = []
        for node_name in self._monitored_nodes:
            status = DiagnosticStatus()
            status.name = f'node/{node_name}'
            status.hardware_id = 'ros2_node'
            if node_name in alive_names:
                status.level = DiagnosticStatus.OK
                status.message = 'alive'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = 'not found'
            statuses.append(status)
        return statuses

    def _check_topics(self) -> list[DiagnosticStatus]:
        return [
            self._make_hz_status(
                topic["name"],
                topic["group"],
                self._rate_monitors[topic["name"]].hz(),
                topic["expected_hz"],
            )
            for topic in self._monitored_topics
        ]

    def _publish(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.extend(self._check_nodes())
        msg.status.extend(self._check_topics())
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
