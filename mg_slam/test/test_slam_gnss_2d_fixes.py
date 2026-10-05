"""slam_gnss_2d のバグ修正と改良項目の単体テスト。"""
from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from slam_gnss_2d.core.data_types import GnssData, GnssPrior, OdomData, PoseEdge, PoseNode, ScanData, SensorFrame
from slam_gnss_2d.core.geometry import scan_to_points
from slam_gnss_2d.core.graph_orchestrator import GraphOrchestrator
from slam_gnss_2d.input.time_series import interpolate_gnss, nearest_by_timestamp
from slam_gnss_2d.map_manager.grid_utils import scan_hits_to_pixels
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.optimizer.isam2_optimizer import ISAM2Optimizer
from slam_gnss_2d.pose_graph.loop_closure_builder import LoopClosureBuilder
from slam_gnss_2d.pose_graph.scan_matching_builder import ScanMatchingBuilder
from slam_gnss_2d.scan_matching.icp_matcher import ICPMatcher
from slam_gnss_2d.scan_matching.ndt_matcher import NDTMatcher


def _make_dummy_scan(lidar_x: float = 0.0, lidar_y: float = 0.0) -> ScanData:
    """前方1.0mに反射点があるダミースキャンデータを生成する。"""
    ranges = np.full(360, 1.0, dtype=np.float32)
    return ScanData(
        timestamp=100.0,
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360,
        range_min=0.1,
        range_max=30.0,
        lidar_x=lidar_x,
        lidar_y=lidar_y,
    )


class TestLoopClosureBuilderFixes:
    """1.1 LoopClosureBuilder の replace_nodes と委譲プロパティのテスト。"""

    def test_replace_nodes_updates_inner_and_cache(self):
        inner = MagicMock(spec=ScanMatchingBuilder)
        loop_matcher = MagicMock()
        builder = LoopClosureBuilder(
            inner=inner,
            loop_matcher=loop_matcher,
            loop_closure_search_radius=5.0,
            loop_closure_min_node_gap=3,
            loop_closure_max_failure_streak=3,
            max_loop_dyaw_deg=30.0,
            loop_closure_crossing_reject_deg=0.0,
            loop_closure_submap_radius=3.0,
            loop_closure_max_score=0.1,
        )

        node0 = PoseNode(index=0, timestamp=1.0, x=0.0, y=0.0, yaw=0.0)
        node1 = PoseNode(index=1, timestamp=2.0, x=1.0, y=0.0, yaw=0.0)
        builder._all_nodes_cache = [node0, node1]

        # 最適化後のノード
        updated_node0 = PoseNode(index=0, timestamp=1.0, x=0.5, y=0.2, yaw=0.1)
        updated_node1 = PoseNode(index=1, timestamp=2.0, x=1.5, y=0.3, yaw=0.2)
        new_nodes = [updated_node0, updated_node1]

        builder.replace_nodes(new_nodes)

        # inner.replace_nodes が呼ばれたこと
        inner.replace_nodes.assert_called_once_with(new_nodes)

        # _all_nodes_cache の座標が更新されたこと
        assert builder._all_nodes_cache[0].x == pytest.approx(0.5)
        assert builder._all_nodes_cache[0].y == pytest.approx(0.2)
        assert builder._all_nodes_cache[0].yaw == pytest.approx(0.1)
        assert builder._all_nodes_cache[1].x == pytest.approx(1.5)
        assert builder._all_nodes_cache[1].y == pytest.approx(0.3)
        assert builder._all_nodes_cache[1].yaw == pytest.approx(0.2)

    def test_stats_delegation(self):
        inner = MagicMock(spec=ScanMatchingBuilder)
        inner.icp_attempt_count = 42
        inner.icp_success_count = 40
        inner.odom_fallback_count = 2

        builder = LoopClosureBuilder(
            inner=inner,
            loop_matcher=MagicMock(),
            loop_closure_search_radius=5.0,
            loop_closure_min_node_gap=3,
            loop_closure_max_failure_streak=3,
            max_loop_dyaw_deg=30.0,
            loop_closure_crossing_reject_deg=0.0,
            loop_closure_submap_radius=3.0,
            loop_closure_max_score=0.1,
        )

        assert builder.icp_attempt_count == 42
        assert builder.icp_success_count == 40
        assert builder.odom_fallback_count == 2


class TestTimeSeriesMaxDt:
    """1.2 GNSS 時間同期の max_dt テスト。"""

    def test_nearest_by_timestamp_with_max_dt(self):
        items = ["a", "b", "c"]
        timestamps = [10.0, 20.0, 30.0]

        # max_dt 内
        assert nearest_by_timestamp(items, timestamps, 20.5, max_dt=1.0) == "b"
        assert nearest_by_timestamp(items, timestamps, 19.5, max_dt=1.0) == "b"

        # max_dt 超過（古いデータや未来データのドロップ）
        assert nearest_by_timestamp(items, timestamps, 25.0, max_dt=2.0) is None
        assert nearest_by_timestamp(items, timestamps, 50.0, max_dt=5.0) is None
        assert nearest_by_timestamp(items, timestamps, 0.0, max_dt=5.0) is None

    def test_interpolate_gnss_with_max_dt(self):
        gnss0 = GnssData(timestamp=10.0, x=0.0, y=0.0, covariance=np.eye(2), fix_status=2)
        gnss1 = GnssData(timestamp=20.0, x=10.0, y=10.0, covariance=np.eye(2), fix_status=2)
        gnss_list = [gnss0, gnss1]
        timestamps = [10.0, 20.0]

        # 正常補間
        interp = interpolate_gnss(gnss_list, timestamps, 15.0, max_dt=10.0)
        assert interp is not None
        assert interp.x == pytest.approx(5.0)
        assert interp.y == pytest.approx(5.0)

        # 許容時間差超過
        assert interpolate_gnss(gnss_list, timestamps, 30.0, max_dt=5.0) is None

    def test_interpolate_gnss_large_gap_rejected(self):
        # 途絶区間（20秒ギャップ）において、片方の観測時刻に近い場合でも拒絶されること
        gnss0 = GnssData(timestamp=0.0, x=0.0, y=0.0, covariance=np.eye(2), fix_status=2)
        gnss1 = GnssData(timestamp=20.0, x=20.0, y=20.0, covariance=np.eye(2), fix_status=2)
        gnss_list = [gnss0, gnss1]
        timestamps = [0.0, 20.0]

        # timestamp=3.0 は prev(0.0) からは3秒差（<=5.0s）だが、区間長20秒のため拒絶されるべき
        assert interpolate_gnss(gnss_list, timestamps, 3.0, max_dt=5.0) is None


class TestDuplicateGnssPriorPrevention:
    """1.2 同一 GNSS 観測値の二重投入防止テスト。"""

    def test_duplicate_gnss_not_added(self):
        logger = MagicMock()
        pose_graph = MagicMock()
        orchestrator = GraphOrchestrator(
            logger=logger,
            pose_graph=pose_graph,
            use_gnss=True,
        )
        orchestrator._anchor_manager = MagicMock()
        orchestrator._anchor_manager.is_initialized = True
        orchestrator._anchor_manager.to_local.return_value = (10.0, 20.0)
        orchestrator._optimizer = MagicMock()

        gnss = GnssData(timestamp=100.0, x=10.0, y=20.0, covariance=np.eye(2) * 0.04, fix_status=2)
        node0 = PoseNode(index=0, timestamp=100.0, x=0.0, y=0.0, yaw=0.0)
        node1 = PoseNode(index=1, timestamp=100.5, x=0.2, y=0.0, yaw=0.0)

        frame0 = SensorFrame(scan=_make_dummy_scan(), odom=OdomData(100.0, 0, 0, 0), gnss=gnss)
        frame1 = SensorFrame(scan=_make_dummy_scan(), odom=OdomData(100.5, 0.2, 0, 0), gnss=gnss)

        # 1回目の投入: 成功
        prior0 = orchestrator._add_gnss_prior(frame0, node0)
        assert prior0 is not None
        assert orchestrator._optimizer.add_gnss_prior.call_count == 1

        # 2回目の投入（同一 GNSS timestamp）: 重複のためスキップ
        prior1 = orchestrator._add_gnss_prior(frame1, node1)
        assert prior1 is None
        assert orchestrator._optimizer.add_gnss_prior.call_count == 1


class TestInformationMatrixScaling:
    """2.1 スキャンマッチング情報行列のスケーリングテスト。"""

    def test_icp_information_scale(self):
        matcher = ICPMatcher(
            max_iterations=5,
            tolerance=1e-3,
            max_correspondence_dist=1.0,
            robust_kernel="none",
            robust_kernel_scale=0.1,
            yaw_information_multiplier=5.0,
        )
        # 点群設定（直交する壁面）
        pts = np.array([
            [1.0, y] for y in np.linspace(-1.0, 1.0, 20)
        ] + [
            [x, 1.0] for x in np.linspace(0.0, 1.0, 10)
        ])
        matcher.set_target_cloud(np.array(pts))

        scan = _make_dummy_scan()
        result = matcher.match(scan, OdomData(timestamp=100.0, x=0.0, y=0.0, yaw=0.0))

        # 情報行列の並進成分が 0.5 程度ではなく、400 オーダー（50以上 2000以下）になっていること
        info = result.information
        assert info.shape == (3, 3)
        assert 50.0 <= np.linalg.norm(info[:2, :2]) <= 2000.0


class TestLidar2DOffset:
    """2.2 LiDAR 2D並進オフセット（23cm）のテスト。"""

    def test_scan_to_points_offset(self):
        # 前方 0.23m に設置された LiDAR
        scan = _make_dummy_scan(lidar_x=0.23, lidar_y=0.0)
        pts = scan_to_points(scan)

        # angle=0 の点（前方）: 距離 1.0m + lidar_x 0.23m = 1.23m
        # ranges の中で angle がほぼ 0 のインデックスを探す
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        idx_zero = np.argmin(np.abs(angles))
        assert pts[idx_zero, 0] == pytest.approx(1.23, abs=0.01)
        assert pts[idx_zero, 1] == pytest.approx(0.0, abs=0.01)

    def test_scan_hits_to_pixels_origin(self):
        scan = _make_dummy_scan(lidar_x=0.23, lidar_y=0.0)
        node = PoseNode(index=0, timestamp=100.0, x=0.0, y=0.0, yaw=0.0, scan=scan)

        resolution = 0.05
        origin_x = -5.0
        origin_y = -5.0
        map_size = 200

        robot_px, robot_py, hit_px, hit_py, _ = scan_hits_to_pixels(
            node, origin_x, origin_y, resolution, map_size
        )

        # 光線始点は (node.x + lidar_x, node.y + lidar_y) = (0.23, 0.0)
        expected_px = int((0.23 - origin_x) / resolution)
        expected_py = int((0.0 - origin_y) / resolution)
        assert robot_px == expected_px
        assert robot_py == expected_py

    def test_bag_resolve_lidar_tf_composition(self):
        from slam_gnss_2d.input.ros2.bag_reader import BagScanSource

        source = BagScanSource("dummy.bag", "/scan")
        # 2つのジョイント:
        # base_link -> top_frame_link: tx=0.23, ty=0.0, yaw=0.0
        # top_frame_link -> top_lidar: tx=0.0, ty=0.1, yaw=π/2
        # 合成後: top_lidar は base_link から見て tx=0.23, ty=0.1, yaw=π/2
        tf_transforms = [
            MagicMock(
                header=MagicMock(frame_id="base_link"),
                child_frame_id="top_frame_link",
                transform=MagicMock(
                    translation=MagicMock(x=0.23, y=0.0, z=0.45),
                    rotation=MagicMock(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            MagicMock(
                header=MagicMock(frame_id="top_frame_link"),
                child_frame_id="top_lidar",
                transform=MagicMock(
                    translation=MagicMock(x=0.0, y=0.1, z=0.03),
                    rotation=MagicMock(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)),  # yaw = π/2
                ),
            ),
        ]
        msg = MagicMock(transforms=tf_transforms)

        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(
                "slam_gnss_2d.input.ros2.bag_reader._open_reader",
                lambda path, topics: MagicMock(
                    has_next=MagicMock(side_effect=[True, True, False]),
                    read_next=MagicMock(
                        side_effect=[
                            ("/tf_static", b"dummy_tf", 0),
                            ("/scan", b"dummy_scan", 0),
                        ]
                    ),
                ),
            )
            from tf2_msgs.msg import TFMessage

            mp.setattr(
                "slam_gnss_2d.input.ros2.bag_reader.deserialize_message",
                lambda data, cls: msg if cls is TFMessage else MagicMock(header=MagicMock(frame_id="top_lidar")),
            )
            yaw, lx, ly = source._resolve_lidar_tf()

        assert yaw == pytest.approx(math.pi / 2, abs=1e-4)
        assert lx == pytest.approx(0.23, abs=1e-4)
        assert ly == pytest.approx(0.1, abs=1e-4)


class TestPoseTranslationPrior2D:
    """2.3 PoseTranslationPrior2D のテスト。"""

    def test_isam2_add_gnss_prior(self):
        optimizer = ISAM2Optimizer()
        optimizer.initialize(0, 0.0, 0.0, 0.0, 0.1, 0.1)

        # PoseTranslationPrior2D で GNSS prior を追加
        optimizer.add_gnss_prior(0, 1.0, 2.0, sigma_xy=0.2)
        optimizer.update()

        pose = optimizer.get_pose(0)
        assert pose is not None
        # x, y が GNSS 観測値の方向へ補正されていること
        assert pose[0] > 0.0
        assert pose[1] > 0.0

    def test_gtsam_optimizer_with_gnss_prior(self):
        optimizer = GTSAMOptimizer()
        nodes = [
            PoseNode(index=0, timestamp=1.0, x=0.0, y=0.0, yaw=0.0),
            PoseNode(index=1, timestamp=2.0, x=1.0, y=0.0, yaw=0.0),
        ]
        edges = [
            PoseEdge(
                from_index=0,
                to_index=1,
                dx=1.0,
                dy=0.0,
                dyaw=0.0,
                information=np.diag([100.0, 100.0, 50.0]),
            )
        ]
        priors = [
            GnssPrior(
                node_index=1,
                x=1.2,
                y=0.1,
                information=np.diag([25.0, 25.0]),
            )
        ]

        result = optimizer.optimize(nodes, edges, priors)
        assert len(result) == 2
        # node 1 が GNSS 側に引き寄せられていること
        assert result[1].x > 1.0
