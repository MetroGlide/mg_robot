"""on_reached_actions を別スレッドで順次実行するエグゼキュータ"""
from __future__ import annotations

import threading
import time
from typing import Callable, List

import rclpy.node

from mg_utils.event_log import EventLog
from mg_waypoint_navigation.waypoint import ActionConfig
from mg_waypoint_navigation.waypoint_sequencer.actions import (
    EndpointCache,
    build_action,
)
from mg_waypoint_navigation.waypoint_sequencer.costmap_mode import (
    CostmapModeSwitcher,
)
from mg_waypoint_navigation.waypoint_sequencer.map_loader import MapLoader


class ActionExecutor:
    """アクションリストを別スレッドで順次実行し、完了時にコールバックを呼ぶ。

    個々のアクションの生成・実行で例外が出ても (失敗は ActionFailed)、イベントに残して次へ進み、
    done_callback は必ず呼ぶ。イベント: action.started / action.succeeded / action.failed。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._events = EventLog(node.get_logger())
        self._endpoints = EndpointCache(node)
        self._map_loader = MapLoader(node, self._endpoints)
        self._costmap_switcher = CostmapModeSwitcher(node, self._endpoints)
        self._thread: threading.Thread | None = None
        self._current_action: str = ""

    @property
    def current_action(self) -> str:
        """実行中のアクションの type。実行していなければ空文字。"""
        return self._current_action

    @property
    def map_loader(self) -> MapLoader:
        return self._map_loader

    @property
    def costmap_switcher(self) -> CostmapModeSwitcher:
        return self._costmap_switcher

    def execute(
        self,
        actions: List[ActionConfig],
        done_callback: Callable[[], None],
        waypoint_index: int = -1,
    ) -> None:
        def _run():
            for config in actions:
                fields = dict(type=config.type, index=waypoint_index)
                self._events.info('action.started', **fields)
                self._current_action = config.type
                started = time.monotonic()
                try:
                    build_action(
                        config, self._node, self._endpoints,
                        self._map_loader, self._costmap_switcher,
                    ).execute()
                except Exception as e:
                    self._events.error(
                        'action.failed', **fields,
                        error=f"{type(e).__name__}: {e}")
                else:
                    self._events.info(
                        'action.succeeded', **fields,
                        duration_sec=round(time.monotonic() - started, 2))
            self._current_action = ""
            done_callback()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
