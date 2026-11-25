"""GNSS による初期姿勢を、スキャンと地図の照合で確かめる (initializer/verify.py) のテスト。"""
import math

import numpy as np

from localization_supervisor.core.grid import OccupancyMap
from localization_supervisor.core.initial_search import InitialSearchConfig, covariance_6x6, search_initial_pose

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
    result = search_initial_pose(room, _scan_points(room, TRUTH), gnss, InitialSearchConfig())
    assert result.reason == 'matched'
    assert math.hypot(result.pose[0] - TRUTH[0], result.pose[1] - TRUTH[1]) < 0.2
    assert abs(result.pose[2] - TRUTH[2]) < 0.06
    assert result.std_m == InitialSearchConfig().matched_std_m


def test_an_ambiguous_scan_falls_back_to_the_gnss_pose_with_a_wide_covariance():
    # 何もない四角い部屋の中央付近: 向きを 90 度変えても、位置を少しずらしても似て見える
    room = _room(with_feature=False)
    truth = (0.0, 0.0, 0.0)
    gnss = (0.5, 0.5, 0.0)
    cfg = InitialSearchConfig(search_yaw_rad=math.pi, step_yaw_rad=0.1)
    result = search_initial_pose(room, _scan_points(room, truth), gnss, cfg)
    assert result.reason == 'ambiguous'
    assert result.pose == gnss and result.std_m == cfg.fallback_std_m


def test_missing_inputs_fall_back():
    room = _room()
    gnss = (0.0, 0.0, 0.0)
    assert search_initial_pose(None, _scan_points(room, TRUTH), gnss, InitialSearchConfig()).reason == 'no_map'
    assert search_initial_pose(room, None, gnss, InitialSearchConfig()).reason == 'too_few_points'
    assert search_initial_pose(room, np.zeros((3, 2)), gnss, InitialSearchConfig()).reason == 'too_few_points'


def test_a_scan_that_does_not_fit_the_map_falls_back():
    room = _room()
    points = np.random.default_rng(0).uniform(-3.0, 3.0, size=(200, 2))
    result = search_initial_pose(room, points, (0.0, 0.0, 0.0), InitialSearchConfig())
    assert result.reason == 'low_ratio' and result.std_m == InitialSearchConfig().fallback_std_m


def test_covariance_fills_x_y_and_yaw():
    cov = covariance_6x6(2.0, 0.5)
    assert cov[0] == cov[7] == 4.0 and cov[35] == 0.25
    assert sum(cov) == 4.0 + 4.0 + 0.25


def _garden(seed=0):
    """30 m 四方 (0.05 m 解像度、原点 (-15, -15))。はっきりした壁と、植え込みのような点の散らばる帯を置く。

    植え込みは、地図では幅 1〜2 m の帯に点が散らばり、走行中のスキャンでも点はまばらに返る。
    """
    rng = np.random.default_rng(seed)
    n = 600
    data = np.zeros((n, n), dtype=np.int8)
    band = np.zeros((n, n), dtype=bool)
    data[0:3, :] = data[-3:, :] = 100
    data[:, 0:3] = 100
    data[300:304, 100:260] = 100
    data[150:304, 256:260] = 100
    data[420:520, 380:384] = 100
    band[40:560, 560:600] = True
    band[440:480, 60:300] = True
    band[100:400, 420:450] = True
    data[band & (rng.random((n, n)) < 0.25)] = 100
    return OccupancyMap(data, 0.05, -15.0, -15.0), band


def _sparse_scan(occupancy, band, pose, rng, hit_in_band=0.3, n=360, max_range=20.0):
    """pose から光線を飛ばした base 座標の点。植え込みの帯の中では、点に当たっても抜けることがある。"""
    angles = -math.pi + 2.0 * math.pi * np.arange(n) / n
    r = np.arange(0.15, max_range, 0.025)
    wx = pose[0] + np.cos(pose[2] + angles)[:, None] * r[None]
    wy = pose[1] + np.sin(pose[2] + angles)[:, None] * r[None]
    cols, rows, inside = occupancy.cells(np.stack([wx.ravel(), wy.ravel()], axis=1))
    hit = np.zeros(wx.size, dtype=bool)
    hit[inside] = occupancy.data[rows[inside], cols[inside]] >= 50
    in_band = np.zeros(wx.size, dtype=bool)
    in_band[inside] = band[rows[inside], cols[inside]]
    hit &= ~in_band | (rng.random(wx.size) < hit_in_band)
    hit = hit.reshape(wx.shape)
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    ok = first >= 0
    d = r[first[ok]] + rng.normal(0.0, 0.02, int(ok.sum()))
    return np.stack([d * np.cos(angles[ok]), d * np.sin(angles[ok])], axis=1)


def test_vegetation_bands_never_lead_to_a_wrong_pose():
    garden, band = _garden()
    rng = np.random.default_rng(1)
    truths = [(-8.0, -4.0, 0.3), (6.0, 0.0, 2.0), (10.0, 8.0, -1.0), (-5.0, 9.0, 1.2), (12.0, -10.0, 3.0),
              (2.0, -8.0, -2.5), (-10.0, 3.0, 0.0), (5.0, 11.0, -0.5)]
    matched = 0
    for truth in truths:
        offset = rng.uniform(-math.pi, math.pi)
        gnss = (truth[0] + 2.0 * math.cos(offset), truth[1] + 2.0 * math.sin(offset), rng.uniform(-math.pi, math.pi))
        result = search_initial_pose(garden, _sparse_scan(garden, band, truth, rng), gnss, InitialSearchConfig())
        if result.reason != 'matched':
            assert result.pose == gnss
            continue
        matched += 1
        assert math.hypot(result.pose[0] - truth[0], result.pose[1] - truth[1]) < 0.5, truth
        assert abs(math.remainder(result.pose[2] - truth[2], 2.0 * math.pi)) < 0.1, truth
    assert matched >= len(truths) // 2


def test_points_beyond_the_scan_radius_are_not_used():
    room = _room()
    far = _scan_points(room, TRUTH) * 0.0 + np.array([30.0, 0.0])
    assert search_initial_pose(room, far, TRUTH, InitialSearchConfig()).reason == 'too_few_points'
