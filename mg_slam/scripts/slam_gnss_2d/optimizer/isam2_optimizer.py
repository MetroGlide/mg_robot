from __future__ import annotations

import threading

import numpy as np
from gtsam import (
    BetweenFactorPose2,
    ISAM2,
    ISAM2Params,
    NonlinearFactorGraph,
    Pose2,
    PoseTranslationPrior2D,
    PriorFactorPose2,
    Values,
    noiseModel,
)



class ISAM2Optimizer:
    """iSAM2 を用いた 2D ポーズグラフのインクリメンタル最適化器。"""

    def __init__(self, relinearize_threshold: float = 0.1) -> None:
        params = ISAM2Params()
        params.setRelinearizeThreshold(relinearize_threshold)
        self._params = params
        self._isam2 = ISAM2(self._params)
        self._pending_graph = NonlinearFactorGraph()
        self._pending_values = Values()
        self._latest_estimate = Values()
        self._initialized = False
        self._lock = threading.Lock()

    def initialize(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
        pos_sigma: float,
        yaw_sigma: float,
    ) -> None:
        with self._lock:
            self._isam2 = ISAM2(self._params)
            self._pending_graph = NonlinearFactorGraph()
            self._pending_values = Values()
            self._latest_estimate = Values()

            self._pending_values.insert(node_index, Pose2(x, y, theta))
            prior_noise = noiseModel.Diagonal.Sigmas(
                np.array([pos_sigma, pos_sigma, yaw_sigma], dtype=np.float64)
            )
            self._pending_graph.add(
                PriorFactorPose2(node_index, Pose2(x, y, theta), prior_noise)
            )
            self._initialized = True

    def add_between_factor(
        self,
        from_index: int,
        to_index: int,
        dx: float,
        dy: float,
        dyaw: float,
        information,
    ) -> None:
        with self._lock:
            if not self._initialized:
                return
            noise = noiseModel.Gaussian.Information(
                np.asarray(information, dtype=np.float64))
            self._pending_graph.add(
                BetweenFactorPose2(from_index, to_index,
                                   Pose2(dx, dy, dyaw), noise)
            )

    def add_gnss_prior(
        self,
        node_index: int,
        x: float,
        y: float,
        sigma_xy: float,
        yaw_variance: float = 0.0,
    ) -> None:
        with self._lock:
            if not self._initialized:
                return
            noise_2d = noiseModel.Diagonal.Sigmas(
                np.array([sigma_xy, sigma_xy], dtype=np.float64)
            )
            self._pending_graph.add(
                PoseTranslationPrior2D(
                    node_index,
                    Pose2(x, y, 0.0),
                    noise_2d,
                )
            )

    def add_initial_estimate(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
    ) -> None:
        with self._lock:
            if not self._initialized:
                return
            if self._pending_values.exists(node_index):
                return
            self._pending_values.insert(node_index, Pose2(x, y, theta))

    def update(self) -> None:
        with self._lock:
            if not self._initialized:
                return
            if self._pending_graph.size() == 0 and self._pending_values.size() == 0:
                return
            self._isam2.update(self._pending_graph, self._pending_values)
            self._latest_estimate = self._isam2.calculateEstimate()
            self._pending_graph = NonlinearFactorGraph()
            self._pending_values = Values()

    def get_pose(self, node_index: int) -> tuple[float, float, float] | None:
        with self._lock:
            if not self._initialized:
                return None
            try:
                pose = self._latest_estimate.atPose2(node_index)
            except RuntimeError:
                return None
            return (pose.x(), pose.y(), pose.theta())

    def get_all_poses(self) -> dict[int, tuple[float, float, float]]:
        with self._lock:
            if not self._initialized:
                return {}
            out: dict[int, tuple[float, float, float]] = {}
            for key in self._latest_estimate.keys():
                pose = self._latest_estimate.atPose2(key)
                out[int(key)] = (pose.x(), pose.y(), pose.theta())
            return out
