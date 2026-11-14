import math

import numpy as np
import pytest

from tools.common import fault_apply as fa
from tools.common.faults import parse_faults


def _fault(**raw):
    return parse_faults({"faults": [raw]})[0]


def test_is_active_window_and_instant():
    drop = _fault(type="gnss_drop", start=10, end=20)
    assert not fa.is_active(drop, 9.99)
    assert fa.is_active(drop, 10.0)
    assert fa.is_active(drop, 19.99)
    assert not fa.is_active(drop, 20.0)
    kidnap = _fault(type="kidnap", at=5, offset={"x": 1.0, "y": 0.0, "yaw": 0.0})
    assert not fa.is_active(kidnap, 5.0)


def test_mask_scan_sector_masks_only_the_sector():
    n = 360
    ranges = np.full(n, 5.0)
    masked = fa.mask_scan_sector(ranges, -math.pi, 2 * math.pi / n, (-90.0, 90.0))
    angles = -180.0 + np.arange(n)
    assert np.all(np.isinf(masked[(angles >= -90) & (angles <= 90)]))
    assert np.all(masked[(angles < -90) | (angles > 90)] == 5.0)
    # 元の配列は変えない
    assert np.all(ranges == 5.0)


def test_mask_scan_sector_normalizes_angles():
    ranges = np.full(4, 3.0)
    # 角度 0, 90, 180, 270 度 (270 度は -90 度と同じ)
    masked = fa.mask_scan_sector(ranges, 0.0, math.pi / 2, (-100.0, -80.0))
    assert np.isinf(masked[3]) and np.all(masked[:3] == 3.0)


def test_shift_latlon_moves_by_meters():
    lat, lon = 36.0, 140.0
    new_lat, new_lon = fa.shift_latlon(lat, lon, 10.0, 20.0)
    north = (new_lat - lat) * fa.METERS_PER_DEGREE_LAT
    east = (new_lon - lon) * fa.METERS_PER_DEGREE_LAT * math.cos(math.radians(lat))
    assert north == pytest.approx(20.0, abs=1e-6)
    assert east == pytest.approx(10.0, abs=1e-6)


def test_set_carrier_solution_keeps_other_bits():
    flags = 0x01 | (2 << 6)
    assert fa.set_carrier_solution(flags, "float") == 0x01 | (1 << 6)
    assert fa.set_carrier_solution(flags, "none") == 0x01
    assert fa.set_carrier_solution(0x01, "fixed") == 0x01 | (2 << 6)


def test_is_gated_on_uses_min_abs_w():
    gated = _fault(type="odom_scale", start=0.0, end=10.0, v=1.0, w=1.1, min_abs_w=0.2)
    plain = _fault(type="odom_scale", start=0.0, end=10.0, v=1.0, w=1.1)
    assert not fa.is_gated_on(gated, 0.1)
    assert fa.is_gated_on(gated, 0.2)
    assert fa.is_gated_on(plain, 0.0)


def test_thin_scan_keeps_every_nth():
    result = fa.thin_scan(np.array([1.0, 2.0, 3.0, 4.0, 5.0]), 2)
    assert result[0] == 1.0 and result[2] == 3.0 and result[4] == 5.0
    assert np.isinf(result[1]) and np.isinf(result[3])


def test_limit_scan_range_drops_far_and_nan():
    result = fa.limit_scan_range(np.array([1.0, 9.0, np.nan]), 5.0)
    assert result[0] == 1.0 and np.isinf(result[1]) and np.isinf(result[2])


def test_jitter_scan_drops_or_stretches_only_chosen_beams():
    ranges = np.full(10000, 2.0)
    result = fa.jitter_scan(ranges, 0.3, 0.8, np.random.default_rng(0))
    changed = result != 2.0
    assert 0.25 < changed.mean() < 0.35
    finite = result[changed & np.isfinite(result)]
    assert np.all((finite > 2.0) & (finite <= 2.8))
    assert 0.4 < np.isinf(result[changed]).mean() < 0.6


def test_gnss_noise_same_fix_and_correlation():
    noise = fa.GnssNoise(np.random.default_rng(0))
    first = noise.sample(0.0, 1.0, 30.0).copy()
    # 同じ測位の 2 つのメッセージには同じ値
    assert np.array_equal(noise.sample(0.05, 1.0, 30.0), first)
    values = np.array([noise.sample(float(t), 1.0, 30.0).copy() for t in range(1, 20001)])
    assert 0.8 < values.std() < 1.2
    # 1 s 離れた値の相関は exp(-1/30) に近く、白色 (tau = 0) ではほぼ 0
    rho = np.corrcoef(values[:-1, 0], values[1:, 0])[0, 1]
    assert abs(rho - math.exp(-1.0 / 30.0)) < 0.03
    white = np.array([noise.sample(float(t), 1.0, 0.0).copy() for t in range(30000, 32000)])
    assert abs(np.corrcoef(white[:-1, 0], white[1:, 0])[0, 1]) < 0.1


def test_horizontal_covariance_keeps_vertical():
    cov = [4.0, 0.5, 0.0, 0.5, 4.0, 0.0, 0.0, 0.0, 9.0]
    out = fa.horizontal_covariance(cov, 0.5)
    assert out[0] == out[4] == 0.25
    assert out[1] == out[3] == 0.0
    assert out[8] == 9.0 and cov[0] == 4.0
