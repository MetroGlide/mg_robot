"""総当たりの照合のテスト。"""
import math

import numpy as np
import pytest

from localization_supervisor.core.grid import build_local_distance_field, scan_points_in_map
from localization_supervisor.core.interfaces import SearchWindow
from localization_supervisor.core.matcher import STANDARD_MATCHERS
from localization_supervisor.core.matcher.correlative import window_size
from localization_supervisor.core.plugins import resolve_params

from supervisor_test_helpers import room_scan, wall_map

TRUTH = (1.0, 2.0, 0.3)


def _matcher(**params):
    cls = STANDARD_MATCHERS['correlative']
    return cls('matcher', resolve_params(cls, params, 'matcher'))


def _points(pose=TRUTH, max_points=200):
    scan = room_scan(0.0, pose)
    return scan_points_in_map(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                              scan.range_max, (0.0, 0.0, 0.0), scan.sensor_pose, max_points)


def _field():
    return build_local_distance_field(wall_map(), 0.0, 0.0, 15.0)


def test_finds_the_true_pose_from_an_offset_center():
    peaks = _matcher().match(_field(), _points(), (1.4, 1.7, 0.2), SearchWindow(0.8, 0.2, 0.2, 0.1))
    assert peaks
    best = peaks[0]
    assert math.hypot(best.pose[0] - TRUTH[0], best.pose[1] - TRUTH[1]) < 0.15
    assert abs(best.pose[2] - TRUTH[2]) < 0.05
    assert best.ratio > 0.9


def test_returns_distinct_peaks_in_order_of_fit():
    peaks = _matcher().match(_field(), _points(), (1.0, 2.0, 0.3), SearchWindow(3.0, 0.5, 0.0), max_peaks=3)
    assert len(peaks) == 3
    ratios = [p.ratio for p in peaks]
    assert ratios == sorted(ratios, reverse=True)
    for i, a in enumerate(peaks):
        for b in peaks[i + 1:]:
            # 粗い格子で離れていたもの (separation_m) が、絞り込みで半刻みずつ寄っても、その分より近くはならない
            assert math.hypot(a.pose[0] - b.pose[0], a.pose[1] - b.pose[1]) >= 1.0 - 0.5


def test_too_few_points_cannot_be_matched():
    assert _matcher().match(_field(), _points()[:5], TRUTH, SearchWindow(0.5, 0.25)) == []


def test_points_outside_the_field_are_not_counted():
    far = _points() + np.array([100.0, 100.0])
    assert _matcher().match(_field(), far, (0.0, 0.0, 0.0), SearchWindow(0.5, 0.25)) == []


def test_window_size_follows_covariance_within_limits():
    cov = np.diag([0.5 ** 2, 0.2 ** 2, 0.1 ** 2])
    assert window_size(cov, 3.0, 0.5, 10.0) == pytest.approx((1.5, 0.3))
    assert window_size(cov, 3.0, 2.0, 10.0)[0] == 2.0
    assert window_size(cov, 30.0, 0.5, 10.0)[0] == 10.0
