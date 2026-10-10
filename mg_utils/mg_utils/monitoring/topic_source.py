"""監視するトピックの購読と、ActivityMonitor の組み立て。ノードから使う。"""
import time
from typing import Any, Callable

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

from mg_utils.monitoring.activity import ActivityMonitor
from mg_utils.monitoring.topic_source_config import TopicSourceConfig, parse_source_config

_QOS_PROFILES = {
    'best_effort': QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE),
    'reliable': QoSProfile(depth=10),
}


def declare_source_config(
    node: Node, source_id: str, default_stale_timeout_sec: float,
) -> TopicSourceConfig:
    """ノードのパラメータ `<source_id>.*` を宣言して読み、TopicSourceConfig にする。"""
    def get(key: str, default: Any) -> Any:
        name = f'{source_id}.{key}'
        if default is None:
            return node.declare_parameter(name, Parameter.Type.STRING).value
        return node.declare_parameter(name, default).value
    return parse_source_config(source_id, get, default_stale_timeout_sec)


class TopicSource:
    """設定に従ってトピックを購読し、受信を ActivityMonitor に記録する。"""

    def __init__(
        self,
        node: Node,
        config: TopicSourceConfig,
        recover_hold_sec: float,
        startup_grace_sec: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.monitor = ActivityMonitor(
            config.stale_timeout_sec, recover_hold_sec, clock(), startup_grace_sec,
            config.armed_on_first_message)
        self._clock = clock
        msg_class = get_message(config.msg_type)
        qos = _QOS_PROFILES[config.qos]
        if config.raw:
            node.create_subscription(msg_class, config.topic, self._on_raw, qos, raw=True)
        else:
            node.create_subscription(msg_class, config.topic, self._on_message, qos)

    def _on_raw(self, _data: bytes) -> None:
        self.monitor.on_message(self._clock())

    def _on_message(self, msg: Any) -> None:
        self.monitor.on_message(self._clock(), ok=bool(getattr(msg, self.config.ok_field)))
