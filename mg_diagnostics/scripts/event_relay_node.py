#!/usr/bin/env python3
"""イベントログを /events に中継し、診断のレベルの変化をイベントにする。

- /rosout のうち、本文が `evt=` で始まるもの (イベントログ。doc/monitoring.md) を、そのまま
  /events (rcl_interfaces/Log、transient_local) に流す。Web UI が履歴ごと受け取れる。
- 集約された診断 (/diagnostics_agg) の項目のレベルが変わったら、`evt=health.level_changed` を
  ログに出す。それが /rosout を経由して /events にも流れる。第三者のノード (Nav2 など) の変化も、
  ここでイベントになる。

この中継が止まっても、ログそのもの (標準出力・/rosout・ファイル) は残る。
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mg_utils.event_log import EventLog, parse_event
from mg_utils.monitoring.level_tracker import LevelChangeTracker

# /rosout は reliable・transient_local だが、過去の分は要らないので volatile で受ける
_ROSOUT_QOS = QoSProfile(
    depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class EventRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('event_relay_node')
        startup_grace = self.declare_parameter('startup_grace_sec', 30.0).value
        history_depth = self.declare_parameter('history_depth', 500).value
        diagnostics_topic = self.declare_parameter('diagnostics_topic', '/diagnostics_agg').value

        self._tracker = LevelChangeTracker(time.monotonic(), startup_grace)
        self._events = EventLog(self.get_logger())
        self._pub = self.create_publisher(
            Log, '/events',
            QoSProfile(
                depth=history_depth, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Log, '/rosout', self._on_rosout, _ROSOUT_QOS)
        self.create_subscription(DiagnosticArray, diagnostics_topic, self._on_diagnostics, 10)
        self.get_logger().info(
            f'event_relay_node started: diagnostics={diagnostics_topic} '
            f'startup_grace_sec={startup_grace} history_depth={history_depth}')

    def _on_rosout(self, msg: Log) -> None:
        if parse_event(msg.msg) is not None:
            self._pub.publish(msg)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        now = time.monotonic()
        for status in msg.status:
            # 集約のグループ (`/Robot/Drivers` など) ではなく、元の診断 (`<ノード名>: <タスク名>`) を見る
            if ': ' not in status.name:
                continue
            level = int.from_bytes(status.level, 'little')
            change = self._tracker.update(now, status.name, level, status.message)
            if change is None:
                continue
            fields = dict(
                path=change.name, from_=change.from_level, to=change.to_level,
                message=change.message)
            if level == 0:
                self._events.info('health.level_changed', **fields)
            elif level == 1:
                self._events.warn('health.level_changed', **fields)
            else:
                self._events.error('health.level_changed', **fields)


def main() -> None:
    rclpy.init()
    node = EventRelayNode()
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
