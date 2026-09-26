import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from wheel_odom_correction import OdomCorrector, TwistEstimator, shift_stamp  # noqa: E402


def _drive_straight(corrector, distance=10.0, step=0.05):
    pose = None
    for i in range(int(distance / step) + 1):
        pose = corrector.update(i * step, 0.0, 0.0)
    return pose


def test_identity_passes_raw_pose_through():
    corrector = OdomCorrector(1.0, 1.0, 0.0, 2.0)
    x, y, yaw = _drive_straight(corrector)
    assert (x, y, yaw) == pytest.approx((10.0, 0.0, 0.0), abs=1e-9)


def test_scale_k_v_scales_distance():
    corrector = OdomCorrector(1.02, 1.0, 0.0, 2.0)
    x, y, yaw = _drive_straight(corrector)
    assert x == pytest.approx(10.2, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_yaw_bias_per_meter_curves_straight_path():
    corrector = OdomCorrector(1.0, 1.0, 0.01, 2.0)
    x, y, yaw = _drive_straight(corrector)
    # 10 m 進んで 0.1 rad 曲がる
    assert yaw == pytest.approx(0.1, abs=1e-9)
    assert y > 0.0


def test_scale_k_w_scales_rotation_in_place():
    corrector = OdomCorrector(1.0, 0.9, 0.0, 2.0)
    yaw = 0.0
    for i in range(101):
        _, _, yaw = corrector.update(0.0, 0.0, i * 0.02)
    assert yaw == pytest.approx(0.9 * 2.0, abs=1e-9)


def test_rotation_across_pi_is_wrapped():
    corrector = OdomCorrector(1.0, 1.0, 0.0, 2.0)
    corrector.update(0.0, 0.0, math.pi - 0.05)
    _, _, yaw = corrector.update(0.0, 0.0, -math.pi + 0.05)
    # π - 0.05 から +0.1 rad 回転した向き
    assert math.cos(yaw) == pytest.approx(math.cos(math.pi + 0.05), abs=1e-9)
    assert math.sin(yaw) == pytest.approx(math.sin(math.pi + 0.05), abs=1e-9)


def test_turning_arc_is_corrected_in_body_frame():
    # 半径 1 m の円弧を 1/4 周。k_v を掛けると半径が k_v 倍になる
    corrector = OdomCorrector(2.0, 1.0, 0.0, 5.0)
    n = 200
    x = y = yaw = 0.0
    for i in range(n + 1):
        angle = (math.pi / 2) * i / n
        x, y, yaw = corrector.update(math.sin(angle), 1.0 - math.cos(angle), angle)
    assert (x, y) == pytest.approx((2.0, 2.0), abs=1e-3)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-9)


def test_pose_jump_resyncs_to_raw_pose():
    corrector = OdomCorrector(1.05, 1.0, 0.0, 2.0)
    corrector.update(0.0, 0.0, 0.0)
    corrector.update(0.5, 0.0, 0.0)
    # ドライバの再起動で姿勢が (100, 50) に飛ぶ
    assert corrector.update(100.0, 50.0, 1.0) == (100.0, 50.0, 1.0)
    x, y, yaw = corrector.update(100.0 + 0.1 * math.cos(1.0), 50.0 + 0.1 * math.sin(1.0), 1.0)
    # 飛んだ後は、そこからの増分に k_v が掛かる
    assert math.hypot(x - 100.0, y - 50.0) == pytest.approx(0.105, abs=1e-9)


def test_correct_twist():
    corrector = OdomCorrector(1.1, 0.9, 0.01, 2.0)
    vx, wz = corrector.correct_twist(1.0, 0.5)
    assert vx == pytest.approx(1.1)
    assert wz == pytest.approx(0.9 * 0.5 + 0.01 * 1.1)


def test_shift_stamp_moves_to_past_with_borrow():
    assert shift_stamp(10, 100_000_000, 0.25) == (9, 850_000_000)
    assert shift_stamp(10, 100_000_000, 0.0) == (10, 100_000_000)


def test_twist_estimator_straight_and_rotation():
    estimator = TwistEstimator(window=2)
    result = None
    x = y = yaw = 0.0
    for i in range(20):
        # 1.0 m/s で前進しながら 0.5 rad/s で旋回する (20 Hz)
        result = estimator.update(i * 0.05, x, y, yaw)
        mid = yaw + 0.5 * 0.5 * 0.05
        x += 1.0 * 0.05 * math.cos(mid)
        y += 1.0 * 0.05 * math.sin(mid)
        yaw += 0.5 * 0.05
    vx, wz = result
    assert wz == pytest.approx(0.5, abs=1e-9)
    # 窓の間の弦の長さは弧の長さよりわずかに短い (中心角 0.05 rad で約 0.01%)
    assert vx == pytest.approx(1.0, abs=1e-3)


def test_twist_estimator_needs_window_and_handles_wrap_and_reset():
    estimator = TwistEstimator(window=2)
    assert estimator.update(0.0, 0.0, 0.0, math.pi - 0.05) is None
    assert estimator.update(0.05, 0.0, 0.0, -math.pi + 0.0) is None
    vx, wz = estimator.update(0.10, 0.0, 0.0, -math.pi + 0.05)
    assert wz == pytest.approx(0.1 / 0.1, abs=1e-9)
    assert vx == pytest.approx(0.0, abs=1e-9)
    estimator.reset()
    assert estimator.update(1.0, 5.0, 5.0, 0.0) is None


def test_twist_estimator_recovers_speed_when_driver_reports_zero():
    # ドライバの速度が 0 のまま姿勢だけ更新される期間があっても、姿勢の差分からは動いていると分かる
    estimator = TwistEstimator(window=2)
    result = None
    for i in range(20):
        result = estimator.update(i * 0.05, 0.8 * i * 0.05, 0.0, 0.0)
    assert result[0] == pytest.approx(0.8, abs=1e-9)
    assert result[1] == pytest.approx(0.0, abs=1e-12)


def test_corrector_reports_reset_on_first_message_and_jump():
    corrector = OdomCorrector(1.0, 1.0, 0.0, 2.0)
    corrector.update(0.0, 0.0, 0.0)
    assert corrector.was_reset
    corrector.update(0.1, 0.0, 0.0)
    assert not corrector.was_reset
    corrector.update(100.0, 0.0, 0.0)
    assert corrector.was_reset
