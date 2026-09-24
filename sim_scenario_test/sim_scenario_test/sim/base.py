from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from sim_scenario_test.geometry import Pose
from sim_scenario_test.sim.model import ModelSpec

if TYPE_CHECKING:
    import rclpy.node


class SimulationBackend(ABC):
    """シミュレータ操作の抽象。姿勢はすべてシミュレータのワールド座標で受け取る。

    操作に失敗した場合は ScenarioError を送出する。
    """

    def __init__(self, node: "rclpy.node.Node", world_name: str):
        self._node = node
        self._world = world_name

    @abstractmethod
    def is_ready(self) -> bool:
        """対象ワールドが起動し操作可能かを返す。"""

    @abstractmethod
    def entity_exists(self, name: str) -> bool:
        """ワールド上に name のエンティティが存在するかを返す。"""

    @abstractmethod
    def set_entity_pose(self, name: str, pose: Pose) -> None:
        ...

    @abstractmethod
    def spawn_entity(self, name: str, model: ModelSpec, pose: Pose) -> None:
        ...

    @abstractmethod
    def remove_entity(self, name: str) -> None:
        ...
