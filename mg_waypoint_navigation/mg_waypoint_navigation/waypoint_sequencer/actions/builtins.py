"""組み込みアクション: load_map, amcl_reset, wait, wait_trigger, set_navigation_mode"""
from __future__ import annotations

import time

from std_srvs.srv import Empty

from mg_waypoint_navigation.waypoint_sequencer.actions.base import BaseAction


class LoadMapAction(BaseAction):
    """測位マップ / 計画マップを map_server にロードする"""

    def execute(self) -> None:
        ok, message = self._map_loader.load(
            self._config.localization, self._config.planning)
        if not ok:
            self._node.get_logger().error(f"LoadMapAction: {message}")


class AmclResetAction(BaseAction):
    """AMCL のパーティクルフィルタをリセットする"""

    def execute(self) -> None:
        self._call_service(
            Empty, "/reinitialize_global_localization", Empty.Request())


class WaitAction(BaseAction):
    """countdown_ms ミリ秒待機する"""

    def execute(self) -> None:
        ms = self._config.countdown_ms
        if ms > 0:
            self._node.get_logger().info(f"WaitAction: waiting {ms} ms")
            time.sleep(ms / 1000.0)


class WaitTriggerAction(BaseAction):
    """外部トリガー（start()）待ちに移行するアクション。execute() 自体は何もしない。
    FSM が on_reached_actions に wait_trigger を検出した時点で IDLE へ遷移し、
    次の start() 呼び出しまで待機する。"""

    def execute(self) -> None:
        pass


class SetNavigationModeAction(BaseAction):
    """ナビゲーションモード（normal / queue_wait等）を切り替え、
    必要なNav2パラメータ（global_costmapの障害物レイヤー等）を動的に変更する。"""

    def execute(self) -> None:
        mode = self._config.mode
        # queue_waitの場合はグローバルコストマップの動的障害物を無視してパスを引かせる
        enabled = (mode != "queue_wait")

        ok, _ = self._costmap_switcher.set_global_obstacle_layers(enabled)
        if ok:
            self._node.get_logger().info(
                f"Set navigation mode to '{mode}' "
                f"(global obstacles enabled: {enabled})")
        else:
            self._node.get_logger().error(
                f"Failed to set navigation mode to '{mode}'")
