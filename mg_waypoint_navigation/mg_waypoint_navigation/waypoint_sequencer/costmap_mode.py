"""global_costmap のセンサ障害物層の有効・無効を切り替える"""
from __future__ import annotations

from typing import Tuple

import rclpy.node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParametersAtomically

from mg_waypoint_navigation.waypoint_sequencer.actions.base import EndpointCache

GLOBAL_COSTMAP_SET_PARAMETERS = (
    "/global_costmap/global_costmap/set_parameters_atomically")

# queue_wait では、動的障害物を無視してパスを引かせるために無効にするセンサ由来の層
SENSOR_OBSTACLE_LAYERS = ("top_obstacle_layer", "obstacle_stvl_layer")


def _bool_param(name: str, value: bool) -> Parameter:
    param = Parameter()
    param.name = name
    param.value.type = ParameterType.PARAMETER_BOOL
    param.value.bool_value = value
    return param


class CostmapModeSwitcher:
    """set_navigation_mode アクションと、手動ゴールのモード切替の共通の経路。"""

    def __init__(self, node: rclpy.node.Node, endpoints: EndpointCache):
        self._node = node
        self._endpoints = endpoints

    def set_global_obstacle_layers(self, enabled: bool) -> Tuple[bool, str]:
        """global_costmap のセンサ障害物層を有効 / 無効にする。反映できたときだけ True を返す。"""
        request = SetParametersAtomically.Request()
        request.parameters = [
            _bool_param(f"{layer}.enabled", enabled)
            for layer in SENSOR_OBSTACLE_LAYERS
        ]
        response = self._endpoints.call_service(
            SetParametersAtomically, GLOBAL_COSTMAP_SET_PARAMETERS, request)
        if response is not None and response.result.successful:
            return True, f"global obstacle layers enabled: {enabled}"
        return False, (
            f"failed to set global obstacle layers (enabled: {enabled}); "
            f"is {GLOBAL_COSTMAP_SET_PARAMETERS} available?")
