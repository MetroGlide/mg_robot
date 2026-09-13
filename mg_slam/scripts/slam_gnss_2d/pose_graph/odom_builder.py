from __future__ import annotations

import math
from typing import Optional

import numpy as np

from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.core.data_types import OdomData, PoseEdge, PoseNode, ScanData
from slam_gnss_2d.core.geometry import (
    angle_diff,
    local_delta_to_world,
    normalize_angle,
    world_delta_to_local,
)

_ODOM_INFORMATION = np.diag([100.0, 100.0, 50.0])


class OdomOnlyBuilder(PoseGraphBuilderBase):
    """オドメトリのみを使用してポーズグラフを構築する Phase 1 実装。

    スキャンマッチングなしの最もシンプルな構成。
    OdomSource の topic 引数を '/odom/gnss' に変更するだけで
    GNSS補正オドメトリベースの構築にも対応できる。
    """

    def __init__(
        self,
        min_translation: float,
        min_rotation: float,
    ) -> None:
        """
        Args:
            min_translation: ノード追加の最小移動距離 [m]
            min_rotation: ノード追加の最小回転量 [rad]
        """
        self._min_translation = min_translation
        self._min_rotation = min_rotation
        self._nodes: list[PoseNode] = []
        self._edges: list[PoseEdge] = []
        self._last_odom: Optional[OdomData] = None

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        if not self._nodes:
            node = PoseNode(
                index=0,
                timestamp=scan.timestamp,
                x=odom.x,
                y=odom.y,
                yaw=odom.yaw,
                scan=scan,
            )
            self._nodes.append(node)
            self._last_odom = odom
            return node

        if self._last_odom is None:
            return None

        dx_w = odom.x - self._last_odom.x
        dy_w = odom.y - self._last_odom.y
        dist = math.hypot(dx_w, dy_w)
        dyaw = abs(angle_diff(odom.yaw, self._last_odom.yaw))
        if dist < self._min_translation and dyaw < self._min_rotation:
            return None

        prev_node = self._nodes[-1]

        dx_local, dy_local = world_delta_to_local(
            dx_w, dy_w, self._last_odom.yaw)
        dyaw_delta = angle_diff(odom.yaw, self._last_odom.yaw)

        dx_world, dy_world = local_delta_to_world(
            dx_local, dy_local, prev_node.yaw)
        new_x = prev_node.x + dx_world
        new_y = prev_node.y + dy_world
        new_yaw = normalize_angle(prev_node.yaw + dyaw_delta)

        node = PoseNode(
            index=len(self._nodes),
            timestamp=scan.timestamp,
            x=new_x,
            y=new_y,
            yaw=new_yaw,
            scan=scan,
        )
        self._nodes.append(node)
        self._edges.append(PoseEdge(
            from_index=prev_node.index,
            to_index=node.index,
            dx=dx_local,
            dy=dy_local,
            dyaw=dyaw_delta,
            information=_ODOM_INFORMATION.copy(),
        ))
        self._last_odom = odom
        return node

    def get_nodes(self) -> list[PoseNode]:
        return list(self._nodes)

    def get_edges(self) -> list[PoseEdge]:
        return list(self._edges)

    def reset(self) -> None:
        self._nodes.clear()
        self._edges.clear()
        self._last_odom = None

    @property
    def loop_just_closed(self) -> bool:
        return False

    def replace_nodes(self, nodes: list[PoseNode]) -> None:
        self._nodes = list(nodes)
