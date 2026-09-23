"""アクション基底クラスと ROS エンドポイントのキャッシュ"""
from __future__ import annotations

import abc
import threading
from typing import Any, Dict, Optional, Tuple

import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup

from mg_waypoint_navigation.waypoint import ActionConfig


class EndpointCache:
    """アクション間で service client / publisher を使い回すためのキャッシュ"""

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._callback_group = ReentrantCallbackGroup()
        self._clients: Dict[Tuple[Any, str], Any] = {}
        self._publishers: Dict[Tuple[Any, str], Any] = {}
        self._lock = threading.Lock()

    def get_client(self, srv_type: Any, name: str):
        with self._lock:
            key = (srv_type, name)
            if key not in self._clients:
                self._clients[key] = self._node.create_client(
                    srv_type, name, callback_group=self._callback_group)
            return self._clients[key]

    def get_publisher(self, msg_type: Any, topic: str):
        with self._lock:
            key = (msg_type, topic)
            if key not in self._publishers:
                self._publishers[key] = self._node.create_publisher(
                    msg_type, topic, 1)
            return self._publishers[key]


class BaseAction(abc.ABC):
    """on_reached_actions の各アクションが実装するインターフェース"""

    def __init__(
        self,
        config: ActionConfig,
        node: rclpy.node.Node,
        endpoints: EndpointCache,
    ):
        self._config = config
        self._node = node
        self._endpoints = endpoints

    @abc.abstractmethod
    def execute(self) -> None:
        """アクションを実行する。同期的に完了すること。"""
        ...

    def _call_service(
        self,
        srv_type: Any,
        name: str,
        request: Any,
        timeout_sec: float = 5.0,
    ) -> Optional[Any]:
        """サービスを呼び出し、応答を待つ。

        アクションはノードの executor 外のスレッドで動くため、ここでは spin せず、
        executor が応答を処理するのを Event で待つ。
        """
        client = self._endpoints.get_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                f"Service {name} not available after {timeout_sec}s")
            return None

        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_sec):
            client.remove_pending_request(future)
            self._node.get_logger().error(
                f"Service call to {name} timed out")
            return None
        return future.result()

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(type={self._config.type})"
