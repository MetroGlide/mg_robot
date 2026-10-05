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
