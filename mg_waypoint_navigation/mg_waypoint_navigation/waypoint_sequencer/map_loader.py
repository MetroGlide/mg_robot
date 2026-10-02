"""測位用・計画用の地図を map_server に読み込ませ、読み込み済みの地図を記録して配信する"""
from __future__ import annotations

import threading
from typing import Tuple

import rclpy.node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav2_msgs.srv import LoadMap

from mg_msgs.msg import LoadedMaps
from mg_waypoint_navigation.waypoint_sequencer.actions.base import EndpointCache

LOCALIZATION_MAP_SERVER = "/map_server/load_map"
PLANNING_MAP_SERVER = "/planning_map_server/load_map"
_LOAD_TIMEOUT_SEC = 10.0


class MapLoader:
    """load_map アクションと ~/load_map サービスの共通の経路。

    読み込みに成功した地図だけを記録し、~/loaded_maps (transient_local) で配信する。
    起動時の地図は launch から渡されるパラメータで初期化する。
    """

    def __init__(self, node: rclpy.node.Node, endpoints: EndpointCache):
        self._node = node
        self._endpoints = endpoints
        self._lock = threading.Lock()
        self._localization = node.declare_parameter(
            "initial_localization_map", "").value
        self._planning = node.declare_parameter(
            "initial_planning_map", "").value
        self._pub = node.create_publisher(
            LoadedMaps, "~/loaded_maps",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._publish()

    @property
    def loaded(self) -> Tuple[str, str]:
        with self._lock:
            return self._localization, self._planning

    def load(self, localization: str = "", planning: str = "") -> Tuple[bool, str]:
        """空文字でない側の地図を読み込む。両方成功したときだけ True を返す。"""
        with self._lock:
            messages = []
            ok = True
            if localization:
                done, message = self._load_one(LOCALIZATION_MAP_SERVER, localization)
                if done:
                    self._localization = localization
                ok = ok and done
                messages.append(f"localization: {message}")
            if planning:
                done, message = self._load_one(PLANNING_MAP_SERVER, planning)
                if done:
                    self._planning = planning
                ok = ok and done
                messages.append(f"planning: {message}")
            if not messages:
                return False, "no map specified"
            self._publish()
            return ok, ", ".join(messages)

    def _load_one(self, service: str, map_url: str) -> Tuple[bool, str]:
        request = LoadMap.Request()
        request.map_url = map_url
        response = self._endpoints.call_service(
            LoadMap, service, request, timeout_sec=_LOAD_TIMEOUT_SEC)
        if response is None:
            return False, f"no response from {service}"
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, f"failed to load {map_url} (result={response.result})"
        return True, f"loaded {map_url}"

    def _publish(self) -> None:
        msg = LoadedMaps()
        msg.localization = self._localization
        msg.planning = self._planning
        self._pub.publish(msg)
