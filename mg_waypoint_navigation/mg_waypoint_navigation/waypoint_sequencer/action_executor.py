"""on_reached_actions を別スレッドで順次実行するエグゼキュータ"""
from __future__ import annotations

import threading
from typing import Callable, List

import rclpy.node

from mg_waypoint_navigation.waypoint import ActionConfig
from mg_waypoint_navigation.waypoint_sequencer.actions import (
    EndpointCache,
    build_action,
)


class ActionExecutor:
    """アクションリストを別スレッドで順次実行し、完了時にコールバックを呼ぶ。

    個々のアクションの生成・実行で例外が出てもログに残して次へ進み、
    done_callback は必ず呼ぶ。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._endpoints = EndpointCache(node)
        self._thread: threading.Thread | None = None

    def execute(
        self,
        actions: List[ActionConfig],
        done_callback: Callable[[], None],
    ) -> None:
        def _run():
            for config in actions:
                try:
                    build_action(config, self._node, self._endpoints).execute()
                except Exception as e:
                    self._node.get_logger().error(
                        f"Action {config.type} raised exception: {e}"
                    )
            done_callback()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
