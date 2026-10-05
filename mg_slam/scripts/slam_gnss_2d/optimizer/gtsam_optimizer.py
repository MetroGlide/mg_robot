from __future__ import annotations

import logging
from typing import Sequence

import numpy as np
from gtsam import (
    BetweenFactorPose2,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    Pose2,
    PoseTranslationPrior2D,
    PriorFactorPose2,
    Values,
    noiseModel,
)


from slam_gnss_2d.core.data_types import GnssPrior, PoseEdge, PoseNode

_logger = logging.getLogger(__name__)

# 最初のノードを固定するアンカー拘束の分散値 (x, y, yaw)
# 非常に小さい値で最初のノードを強く固定する
_ANCHOR_VARIANCES = np.array([1e-6, 1e-6, 1e-8])


class GTSAMOptimizer:
    """GTSAM LevenbergMarquardt による 2D ポーズグラフ最適化。"""

    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        gnss_priors: Sequence[GnssPrior] = (),
    ) -> list[PoseNode]:
        if len(nodes) < 2:
            return list(nodes)
        if not edges and not gnss_priors:
            return list(nodes)

        graph = NonlinearFactorGraph()
        initial = Values()

        for node in nodes:
            initial.insert(node.index, Pose2(node.x, node.y, node.yaw))

        # 最初のノードをアンカー固定（グラフのゲージ自由度を除去する）
        anchor = nodes[0]
        prior_noise = noiseModel.Diagonal.Variances(_ANCHOR_VARIANCES)
        graph.add(PriorFactorPose2(
            anchor.index,
            Pose2(anchor.x, anchor.y, anchor.yaw),
            prior_noise,
        ))

        for edge in edges:
            noise = noiseModel.Gaussian.Information(edge.information)
            graph.add(BetweenFactorPose2(
                edge.from_index,
                edge.to_index,
                Pose2(edge.dx, edge.dy, edge.dyaw),
                noise,
            ))

        # GNSS絶対位置拘束を PoseTranslationPrior2D として投入する（xy平面のみ直接拘束）
        for gnss_prior in gnss_priors:
            if not initial.exists(gnss_prior.node_index):
                _logger.warning(
                    f'GNSS prior skipped: node_index={gnss_prior.node_index} not in graph'
                )
                continue
            gnss_noise = noiseModel.Gaussian.Information(gnss_prior.information)
            graph.add(PoseTranslationPrior2D(
                gnss_prior.node_index,
                Pose2(gnss_prior.x, gnss_prior.y, 0.0),
                gnss_noise,
            ))

        params = LevenbergMarquardtParams()
        params.setVerbosity('SILENT')
        result = LevenbergMarquardtOptimizer(graph, initial, params).optimize()

        updated: list[PoseNode] = []
        for node in nodes:
            pose = result.atPose2(node.index)
            updated.append(PoseNode(
                index=node.index,
                timestamp=node.timestamp,
                x=pose.x(),
                y=pose.y(),
                yaw=pose.theta(),
                scan=node.scan,
            ))

        _logger.info(
            f'GTSAMOptimizer: {len(nodes)} nodes, {len(edges)} edges, '
            f'{len(list(gnss_priors))} gnss_priors optimized'
        )
        return updated
