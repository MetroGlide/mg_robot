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


def test_points_outside_the_field_count_as_misses_when_asked():
    points = np.vstack([_points(), _points()[:50] + np.array([100.0, 100.0])])
    window = SearchWindow(0.0, 0.0)
    kept = _matcher().match(_field(), points, TRUTH, window)[0]
    missed = _matcher(outside_is_miss=True).match(_field(), points, TRUTH, window)[0]
    assert kept.ratio > 0.95
    assert missed.ratio == pytest.approx(kept.ratio * (points.shape[0] - 50) / points.shape[0])


def test_a_wide_coarse_tolerance_keeps_a_narrow_peak_and_scores_with_the_fine_tolerance():
    # 粗い向きの刻みが大きく、細かい許容距離では本当の姿勢の近くの粗い格子も一致率が低い
    window = SearchWindow(1.0, 0.5, math.pi, 0.3)
    center = (1.3, 1.8, 0.0)
    missed = _matcher(tolerance_m=0.05).match(_field(), _points(), center, window)[0]
    assert math.hypot(missed.pose[0] - TRUTH[0], missed.pose[1] - TRUTH[1]) > 0.3
    params = dict(tolerance_m=0.05, coarse_tolerance_m=0.4, coarse_candidates=8, separation_m=1.0)
    peaks = _matcher(**params).match(_field(), _points(), center, window, max_peaks=2)
    best = peaks[0]
    assert math.hypot(best.pose[0] - TRUTH[0], best.pose[1] - TRUTH[1]) < 0.15
    assert abs(math.remainder(best.pose[2] - TRUTH[2], 2 * math.pi)) < 0.06
    fine = _matcher(tolerance_m=0.05).match(_field(), _points(), best.pose, SearchWindow(0.0, 0.0))[0]
    assert best.ratio == pytest.approx(fine.ratio)
    assert len(peaks) == 2
    a, b = peaks
    assert (math.hypot(a.pose[0] - b.pose[0], a.pose[1] - b.pose[1]) >= 1.0
            or abs(math.remainder(a.pose[2] - b.pose[2], 2 * math.pi)) >= 0.5)
