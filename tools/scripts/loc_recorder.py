#!/usr/bin/env python3
"""
loc_recorder.py

指定したトピックを、シミュレーション時刻 (/clock) の時刻印で rosbag2 (MCAP) に記録する。
再生評価 (localization_replay.sh) で使う。`ros2 bag record` は ros2bag と rosbag2_py のバージョンが
食い違う環境で動かないため、rosbag2_py で直接書き込む。

まだ配信されていないトピックは、現れた時点で購読を始める。SIGINT / SIGTERM で終了して bag を閉じる。
"""

import argparse
import os
import signal
import sys
from typing import Dict, List

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import open_writer  # noqa: E402

# どの QoS の配信元とも接続できるよう best-effort で購読し、取りこぼさないよう深いキューを持つ
QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)


class Recorder(Node):
    def __init__(self, output: str, topics: List[str], use_sim_time: bool) -> None:
        super().__init__(
            "loc_recorder",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        self._writer = open_writer(output, storage_id="mcap")
        self._pending = list(topics)
        self._subscriptions_by_topic: Dict[str, object] = {}
        self._count = 0
        self.create_timer(1.0, self._discover)

    def _discover(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        for topic in list(self._pending):
            types = graph.get(topic)
            if not types:
                continue
            type_name = types[0]
            self._writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_name, serialization_format="cdr"))
            self._subscriptions_by_topic[topic] = self.create_subscription(
                get_message(type_name), topic,
                lambda data, name=topic: self._on_message(name, data),
                QOS, raw=True)
            self._pending.remove(topic)
            self.get_logger().info(f"記録を開始します: {topic} ({type_name})")

    def _on_message(self, topic: str, data: bytes) -> None:
        self._writer.write(topic, data, self.get_clock().now().nanoseconds)
        self._count += 1

    def close(self) -> None:
        self.get_logger().info(f"記録を終了します: {self._count} 件")
        del self._writer


def main() -> None:
    parser = argparse.ArgumentParser(description="トピックをシミュレーション時刻で rosbag2 に記録する")
    parser.add_argument("-o", "--output", required=True, help="出力先 bag ディレクトリ (存在しないこと)")
    parser.add_argument("--wall-time", action="store_true", help="シミュレーション時刻ではなく実時間の時刻印にする")
    parser.add_argument("topics", nargs="+", help="記録するトピック")
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.output, args.topics, use_sim_time=not args.wall_time)
    signal.signal(signal.SIGINT, lambda *_: rclpy.try_shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
