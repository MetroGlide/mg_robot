"""GNSS による初期姿勢を、スキャンと地図の照合で確かめる (initializer/verify.py) のテスト。"""
import math

import numpy as np

from localization_supervisor.core.grid import OccupancyMap
from mg_localization_supervisor.initializer.verify import VerifyConfig, covariance_6x6, verify_pose

TRUTH = (1.0, -1.5, 0.4)


def _room(with_feature=True):
    """20 m 四方の部屋 (0.1 m 解像度、原点 (-10, -10))。中に L 字の壁を置き、形を非対称にする。"""
    data = np.zeros((200, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    if with_feature:
        data[120:122, 60:110] = 100
        data[80:122, 108:110] = 100
    return OccupancyMap(data, 0.1, -10.0, -10.0)


def _scan_points(occupancy, pose, n=360, max_range=15.0):
    """pose から地図に光線を飛ばした、base 座標の点。"""
    points = []
    for a in -math.pi + 2.0 * math.pi * np.arange(n) / n:
        world = pose[2] + a
        for r in np.arange(0.1, max_range, 0.05):
            x, y = pose[0] + r * math.cos(world), pose[1] + r * math.sin(world)
            _, _, inside = occupancy.cells(np.array([[x, y]]))
            if inside[0] and occupancy.occupied_at(np.array([[x, y]]))[0]:
                points.append((r * math.cos(a), r * math.sin(a)))
                break
    return np.array(points)


def test_a_shifted_gnss_pose_is_corrected_to_the_scan_match():
    room = _room()
    gnss = (TRUTH[0] + 2.0, TRUTH[1] - 1.5, TRUTH[2] + 0.2)
    result = verify_pose(room, _scan_points(room, TRUTH), gnss, VerifyConfig())
    assert result.reason == 'matched'
    assert math.hypot(result.pose[0] - TRUTH[0], result.pose[1] - TRUTH[1]) < 0.2
    assert abs(result.pose[2] - TRUTH[2]) < 0.06
    assert result.std_m == VerifyConfig().matched_std_m


def test_an_ambiguous_scan_falls_back_to_the_gnss_pose_with_a_wide_covariance():
    # 何もない四角い部屋の中央付近: 向きを 90 度変えても、位置を少しずらしても似て見える
    room = _room(with_feature=False)
    truth = (0.0, 0.0, 0.0)
    gnss = (0.5, 0.5, 0.0)
    cfg = VerifyConfig(search_yaw_rad=math.pi, step_yaw_rad=0.1)
    result = verify_pose(room, _scan_points(room, truth), gnss, cfg)
    assert result.reason == 'ambiguous'
    assert result.pose == gnss and result.std_m == cfg.fallback_std_m


def test_missing_inputs_fall_back():
    room = _room()
    gnss = (0.0, 0.0, 0.0)
    assert verify_pose(None, _scan_points(room, TRUTH), gnss, VerifyConfig()).reason == 'no_map'
    assert verify_pose(room, None, gnss, VerifyConfig()).reason == 'too_few_points'
    assert verify_pose(room, np.zeros((3, 2)), gnss, VerifyConfig()).reason == 'too_few_points'


def test_a_scan_that_does_not_fit_the_map_falls_back():
    room = _room()
    points = np.random.default_rng(0).uniform(-3.0, 3.0, size=(200, 2))
    result = verify_pose(room, points, (0.0, 0.0, 0.0), VerifyConfig())
    assert result.reason == 'low_ratio' and result.std_m == VerifyConfig().fallback_std_m


def test_covariance_fills_x_y_and_yaw():
    cov = covariance_6x6(2.0, 0.5)
    assert cov[0] == cov[7] == 4.0 and cov[35] == 0.25
    assert sum(cov) == 4.0 + 4.0 + 0.25
