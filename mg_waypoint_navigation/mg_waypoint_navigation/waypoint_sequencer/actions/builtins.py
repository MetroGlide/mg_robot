"""組み込みアクション: load_map, amcl_reset, wait, wait_trigger, set_navigation_mode"""
from __future__ import annotations

import time

from nav2_msgs.srv import LoadMap
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Empty

from mg_waypoint_navigation.waypoint_sequencer.actions.base import BaseAction


class LoadMapAction(BaseAction):
    """測位マップ / 計画マップを map_server にロードする"""

    def execute(self) -> None:
        if self._config.localization:
            req = LoadMap.Request()
            req.map_url = self._config.localization
            self._call_service(
                LoadMap, "/map_server/load_map", req, timeout_sec=10.0)

        if self._config.planning:
            req = LoadMap.Request()
            req.map_url = self._config.planning
            self._call_service(
                LoadMap, "/planning_map_server/load_map", req, timeout_sec=10.0)


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

        req = SetParametersAtomically.Request()
        req.parameters = [
            self._bool_param("top_obstacle_layer.enabled", enabled),
            self._bool_param("obstacle_stvl_layer.enabled", enabled),
        ]
        response = self._call_service(
            SetParametersAtomically,
            "/global_costmap/global_costmap/set_parameters_atomically",
            req,
        )
        if response is not None and response.result.successful:
            self._node.get_logger().info(
                f"Set navigation mode to '{mode}' "
                f"(global obstacles enabled: {enabled})")
        else:
            self._node.get_logger().error(
                f"Failed to set navigation mode to '{mode}'")

    @staticmethod
    def _bool_param(name: str, value: bool) -> Parameter:
        param = Parameter()
        param.name = name
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = value
        return param
