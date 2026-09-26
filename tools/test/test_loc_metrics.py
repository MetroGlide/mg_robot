import math

import numpy as np
import pytest

from tools.common import loc_metrics as lm


def test_wrap_angle():
    assert lm.wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert lm.wrap_angle(-0.5) == pytest.approx(-0.5)


def test_summarize_empty_and_values():
    assert lm.summarize(np.zeros(0)) == {"n": 0}
    s = lm.summarize(np.array([1.0, 2.0, 3.0]))
    assert s["n"] == 3
    assert s["median"] == pytest.approx(2.0)
    assert s["max"] == pytest.approx(3.0)


def test_compose_identity_and_translation():
    a = np.array([[1.0, 2.0, math.pi / 2]])
    b = np.array([[1.0, 0.0, 0.0]])
    c = lm.compose(a, b)
    # 90 度回った a の前方 1 m は、a の +y 方向
    assert c[0, 0] == pytest.approx(1.0)
    assert c[0, 1] == pytest.approx(3.0)
    assert c[0, 2] == pytest.approx(math.pi / 2)


def test_hold_index_before_first_is_minus_one():
    idx = lm.hold_index(np.array([1.0, 2.0, 3.0]), np.array([0.5, 1.0, 2.5, 9.0]))
    assert list(idx) == [-1, 0, 1, 2]


def test_pose_errors_wrap_yaw():
    est = np.array([[0.0, 0.0, math.pi - 0.1]])
    gt = np.array([[3.0, 4.0, -math.pi + 0.1]])
    pos, yaw = lm.pose_errors(est, gt)
    assert pos[0] == pytest.approx(5.0)
    assert yaw[0] == pytest.approx(-0.2)


def test_correction_jumps_yaw_correction_moves_far_base():
    # odom 原点から遠い (50, 0) にいるとき、map->odom の yaw が 0.1 rad 変わると、
    # ロボットは約 5 m 飛ぶ (yaw の補正が遠くでは位置の飛びになる)
    map_odom = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]])
    base = np.array([[50.0, 0.0, 0.0]])
    pos, yaw = lm.correction_jumps(map_odom, base)
    assert yaw[0] == pytest.approx(0.1)
    assert pos[0] == pytest.approx(math.hypot(50 * (math.cos(0.1) - 1), 50 * math.sin(0.1)))


def test_correction_jumps_pure_translation_and_short_input():
    map_odom = np.array([[0.0, 0.0, 0.0], [0.3, 0.4, 0.0]])
    pos, yaw = lm.correction_jumps(map_odom, np.array([[5.0, 5.0, 1.0]]))
    assert pos[0] == pytest.approx(0.5)
    assert yaw[0] == pytest.approx(0.0)
    assert lm.correction_jumps(map_odom[:1], np.zeros((0, 3)))[0].size == 0


def test_nees_of_consistent_error_is_dof():
    rng = np.random.default_rng(0)
    sigma = np.array([0.1, 0.2, 0.05])
    n = 4000
    errors = rng.normal(size=(n, 3)) * sigma
    cov6 = np.zeros((n, 36))
    cov6[:, 0] = sigma[0] ** 2
    cov6[:, 7] = sigma[1] ** 2
    cov6[:, 35] = sigma[2] ** 2
    result = lm.nees(errors, cov6)
    assert np.nanmean(result) == pytest.approx(3.0, abs=0.2)


def test_nees_singular_covariance_is_nan():
    result = lm.nees(np.ones((1, 3)), np.zeros((1, 36)))
    assert np.isnan(result[0])


def test_best_lag_recovers_known_delay():
    t = np.arange(0.0, 20.0, 0.05)
    ref = np.sin(t) + 0.5 * np.sin(3.1 * t)
    sig = np.interp(t - 0.4, t, ref)
    lag, rmse = lm.best_lag(t, ref, t, sig, max_lag=1.0, step=0.05)
    assert lag == pytest.approx(0.4, abs=0.051)
    assert rmse < 0.05


def test_recovery_metrics_recovers_after_kidnap():
    t = np.arange(0.0, 60.0, 0.5)
    err = np.where((t >= 10.0) & (t < 25.0), 3.0, 0.05)
    result = lm.recovery_metrics(t, err, 10.0, 60.0, ok_threshold=0.5, hold_sec=2.0)
    assert result["deviated"] is True
    assert result["max_error_m"] == pytest.approx(3.0)
    assert result["recovery_sec"] == pytest.approx(15.0)


def test_recovery_metrics_never_recovers():
    t = np.arange(0.0, 30.0, 0.5)
    err = np.where(t >= 10.0, 3.0, 0.05)
    result = lm.recovery_metrics(t, err, 10.0, 30.0, ok_threshold=0.5, hold_sec=2.0)
    assert result["deviated"] is True
    assert result["recovery_sec"] is None


def test_recovery_metrics_not_deviated():
    t = np.arange(0.0, 30.0, 0.5)
    err = np.full_like(t, 0.1)
    result = lm.recovery_metrics(t, err, 10.0, 30.0, ok_threshold=0.5, hold_sec=2.0)
    assert result["deviated"] is False
    assert result["recovery_sec"] == 0.0


def test_count_episodes_separates_false_detections():
    t = np.arange(0.0, 100.0, 1.0)
    s = np.zeros_like(t, dtype=int)
    s[20:25] = 2   # 故障区間内 (15-45)
    s[70:72] = 1   # 故障区間外
    result = lm.count_episodes(t, s, [(15.0, 45.0)])
    assert result == {"episodes_in_fault_windows": 1, "false_detections": 1}


def test_summarize_status_values_skips_unavailable_and_counts_states():
    #             t   state d2    jump  diff  ratio  gain
    status = np.array([
        [0.0, 0, -1.0, -1.0, -1.0, 0.30, 0.00],
        [1.0, 0, 0.5, -1.0, 0.1, 0.34, 0.05],
        [2.0, 1, 30.0, 4.0, 3.0, 0.10, 0.40],
        [3.0, 0, 0.7, 0.1, 0.2, 0.32, 0.00],
    ])
    result = lm.summarize_status_values(status)
    assert result["values"]["gnss_d2"]["n"] == 3
    assert result["values"]["gnss_d2"]["min"] == pytest.approx(0.5)
    assert result["values"]["scan_ratio"]["n"] == 4
    assert result["values"]["jump_m"]["max"] == pytest.approx(4.0)
    assert result["state_fractions"] == {0: 0.75, 1: 0.25}


def test_first_detection_delay():
    t = np.arange(0.0, 50.0, 1.0)
    s = np.zeros_like(t, dtype=int)
    s[23:] = 2
    assert lm.first_detection_delay(t, s, 20.0, 50.0) == pytest.approx(3.0)
    assert lm.first_detection_delay(t, s, 30.0, 50.0) == pytest.approx(0.0)
    assert lm.first_detection_delay(t, np.zeros_like(s), 20.0, 50.0) is None
