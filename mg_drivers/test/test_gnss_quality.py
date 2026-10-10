import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from gnss_quality import classify  # noqa: E402

FIX_OK = 1


def _flags(carrier, fix_ok=True):
    return (carrier << 6) | (FIX_OK if fix_ok else 0)


def test_rtk_fixed_is_ok():
    assert classify(3, _flags(2)) == ('RTK Fixed', 'ok')


def test_rtk_fixed_is_ok_even_without_fix_ok_flag():
    assert classify(3, _flags(2, fix_ok=False)) == ('RTK Fixed', 'ok')


def test_rtk_float_is_warn():
    assert classify(3, _flags(1)) == ('RTK Float', 'warn')


@pytest.mark.parametrize('fix_type, expected', [
    (0, ('No Fix', 'error')),
    (1, ('DR only', 'warn')),
    (2, ('2D Fix', 'warn')),
    (3, ('3D Fix', 'warn')),
    (4, ('GNSS+DR', 'warn')),
    (5, ('Time only', 'error')),
])
def test_without_carrier_solution_uses_fix_type(fix_type, expected):
    assert classify(fix_type, _flags(0)) == expected


def test_unknown_fix_type_is_warn():
    assert classify(9, _flags(0)) == ('fix 9', 'warn')


def test_other_flag_bits_do_not_change_carrier_phase():
    # bit 5 (heading valid) と bit 2-4 (power save mode)
    assert classify(3, 0b00111100) == ('3D Fix', 'warn')
