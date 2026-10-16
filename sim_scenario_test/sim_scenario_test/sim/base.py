from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Optional, Tuple

from sim_scenario_test.errors import ScenarioError
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

    def entity_pose(self, name: str) -> Optional[Pose]:
        """シミュレータ上の真の姿勢 (ワールド座標) を返す。まだ届いていなければ None。

        取れないバックエンドは ScenarioError を送出する (既定)。
        """
        raise ScenarioError(f"backend {type(self).__name__} does not provide entity poses")

    @staticmethod
    def companion_nodes(world_name: str) -> List[NodeSpec]:
        """シミュレータと一緒に起動するノード (entity_pose のための中継など)。"""
        return []


@dataclass(frozen=True)
class NodeSpec:
    """launch で起動するノード (バックエンドを launch に依存させないための記述)。"""
    package: str
    executable: str
    name: str
    arguments: Tuple[str, ...] = ()
    remappings: Tuple[Tuple[str, str], ...] = ()
