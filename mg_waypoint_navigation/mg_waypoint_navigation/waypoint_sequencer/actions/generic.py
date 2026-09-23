"""importlib による汎用サービスコール / トピックパブリッシュアクション"""
from __future__ import annotations

import importlib

import rclpy.node

from mg_waypoint_navigation.waypoint import ActionConfig
from mg_waypoint_navigation.waypoint_sequencer.actions.base import (
    BaseAction,
    EndpointCache,
)


class GenericServiceAction(BaseAction):
    """YAML で指定したサービスを呼び出す"""

    def __init__(
        self,
        config: ActionConfig,
        node: rclpy.node.Node,
        endpoints: EndpointCache,
    ):
        super().__init__(config, node, endpoints)
        module = importlib.import_module(config.srv_module)
        self._srv_type = getattr(module, config.srv_class)

    def execute(self) -> None:
        request = self._srv_type.Request()
        for key, value in self._config.request.items():
            setattr(request, key, value)
        self._call_service(self._srv_type, self._config.service, request)


class GenericPublishAction(BaseAction):
    """YAML で指定したトピックにメッセージを1回パブリッシュする"""

    def __init__(
        self,
        config: ActionConfig,
        node: rclpy.node.Node,
        endpoints: EndpointCache,
    ):
        super().__init__(config, node, endpoints)
        module = importlib.import_module(config.msg_module)
        self._msg_type = getattr(module, config.msg_class)

    def execute(self) -> None:
        publisher = self._endpoints.get_publisher(
            self._msg_type, self._config.topic)
        msg = self._msg_type()
        for key, value in self._config.data.items():
            setattr(msg, key, value)
        publisher.publish(msg)
        self._node.get_logger().debug(
            f"Published to {self._config.topic}: {self._config.data}"
        )
