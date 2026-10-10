#!/usr/bin/env python3
"""第三者のノードのトピックの受信頻度と、ノードの生存を、診断として出す。

自分で診断を出せないノード (Nav2、LiDAR のドライバなど) のために、外から見て調べる。
自作のノードは、自分の状態を自分の診断として出すので、ここでは調べない
(判定は、それを最もよく知るノードが 1 回だけ行う。doc/monitoring.md)。

- トピック: 設定した受信頻度を下回ったら WARN、途絶したら fault_level (既定 ERROR)
- ノード: ROS のグラフにいなければ node_fault_level (既定 ERROR)

トピックの設定の書き方は mg_utils.monitoring.topic_source_config にある。
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from mg_utils.monitoring.diagnostic_task import TopicHealthTask
from mg_utils.monitoring.health import FAULT_LEVELS
from mg_utils.monitoring.topic_source import TopicSource, declare_source_config

HARDWARE_ID = 'mg-robot'

_LEVELS = {
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class NodeAliveTask(DiagnosticTask):
    """ノードが ROS のグラフにいるかを調べる。ノード名は名前空間を含めた形 (/ns/name)。

    grace_until (time.monotonic の時刻) までは、いなくても異常にしない (スタックの起動を待つ)。
    """

    def __init__(
        self, node: Node, full_name: str, fault_level: bytes, grace_until: float,
    ) -> None:
        super().__init__(f'node {full_name}')
        self._node = node
        self._full_name = full_name
        self._fault_level = fault_level
        self._grace_until = grace_until

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        alive = {
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in self._node.get_node_names_and_namespaces()
        }
        if self._full_name in alive:
            stat.summary(DiagnosticStatus.OK, 'alive')
        elif time.monotonic() < self._grace_until:
            stat.summary(DiagnosticStatus.OK, 'waiting for node')
        else:
            stat.summary(self._fault_level, 'not found')
        return stat


class TopicMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('topic_monitor_node')
        stale_timeout = self.declare_parameter('stale_timeout_sec', 3.0).value
        recover_hold = self.declare_parameter('recover_hold_sec', 2.0).value
        startup_grace = self.declare_parameter('startup_grace_sec', 30.0).value
        hz_warn_ratio = self.declare_parameter('hz_warn_ratio', 0.5).value
        node_fault_level = self.declare_parameter('node_fault_level', 'error').value
        if node_fault_level not in FAULT_LEVELS:
            raise ValueError(f'node_fault_level は {list(FAULT_LEVELS)} のいずれか: {node_fault_level}')
        monitored_nodes = self.declare_parameter(
            'monitored_nodes', Parameter.Type.STRING_ARRAY).value or []
        monitored_sources = self.declare_parameter(
            'monitored_sources', Parameter.Type.STRING_ARRAY).value or []

        self._updater = Updater(self)
        self._updater.setHardwareID(HARDWARE_ID)

        for full_name in monitored_nodes:
            if not full_name.startswith('/'):
                raise ValueError(f'monitored_nodes は名前空間を含めた形 (/ns/name): {full_name}')
            self._updater.add(NodeAliveTask(
                self, full_name, _LEVELS[node_fault_level], time.monotonic() + startup_grace))

        self._sources = []
        for source_id in monitored_sources:
            config = declare_source_config(self, source_id, stale_timeout)
            if not config.enabled:
                continue
            source = TopicSource(self, config, recover_hold, startup_grace)
            self._sources.append(source)
            self._updater.add(TopicHealthTask(
                f'topic {config.topic}', source.monitor, time.monotonic,
                expected_hz=config.expected_hz, hz_warn_ratio=hz_warn_ratio,
                fault_level=config.fault_level))

        self.get_logger().info(
            f'topic_monitor_node started: nodes={list(monitored_nodes)} '
            f'topics={[s.config.topic for s in self._sources]}')


def main() -> None:
    rclpy.init()
    node = TopicMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
