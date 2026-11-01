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


def test_summarize_status_values_per_check_and_counts_states():
    #             t   state level
    status = np.array([
        [0.0, 0, 0.0],
        [1.0, 0, 0.0],
        [2.0, 1, 2.5],
        [3.0, 0, 0.5],
    ])
    result = lm.summarize_status_values(status, {"amcl_gnss": np.array([0.5, 3.0, 0.7]),
                                                 "scan_ekf": np.zeros(0)})
    assert result["values"]["fusion_level"]["max"] == pytest.approx(2.5)
    assert result["values"]["amcl_gnss"]["n"] == 3
    assert result["values"]["amcl_gnss"]["min"] == pytest.approx(0.5)
    assert result["values"]["scan_ekf"]["n"] == 0
    assert result["state_fractions"] == {0: 0.75, 1: 0.25}


def test_first_detection_delay():
    t = np.arange(0.0, 50.0, 1.0)
    s = np.zeros_like(t, dtype=int)
    s[23:] = 2
    assert lm.first_detection_delay(t, s, 20.0, 50.0) == pytest.approx(3.0)
    assert lm.first_detection_delay(t, s, 30.0, 50.0) == pytest.approx(0.0)
    assert lm.first_detection_delay(t, np.zeros_like(s), 20.0, 50.0) is None


def _straight_then_turn():
    # 0-10 s は +x へ直進、10-14 s で左に 90 度曲がり、その後 +y へ直進する
    rows = [[t, t, 0.0, 0.0] for t in np.arange(0.0, 10.0, 0.5)]
    rows += [[10.0 + k, 10.0, 0.0, math.pi / 2 * k / 4] for k in range(1, 5)]
    rows += [[14.0 + t, 10.0, t, math.pi / 2] for t in np.arange(0.5, 10.0, 0.5)]
    return np.array(rows)


def test_detect_turns_finds_one_left_turn():
    turns = lm.detect_turns(_straight_then_turn(), min_turn_rad=math.radians(45), window_sec=5.0)
    assert len(turns) == 1
    start, end, dyaw = turns[0]
    # 区間は、窓の中で向きが変わる範囲なので、実際の曲がり (10-14 s) より窓の幅だけ前後に広がりうる
    assert 9.0 - 5.0 <= start <= 10.0
    assert 14.0 <= end <= 14.0 + 5.0
    assert dyaw == pytest.approx(math.pi / 2, abs=0.4)


def test_detect_turns_ignores_straight_and_short_input():
    straight = np.array([[t, t, 0.0, 0.01 * t] for t in np.arange(0.0, 20.0, 0.5)])
    assert lm.detect_turns(straight, math.radians(45), 5.0) == []
    assert lm.detect_turns(straight[:1], math.radians(45), 5.0) == []


def test_deviation_episodes_uses_median_merges_gaps_and_marks_until_end():
    times = np.arange(0.0, 60.0, 0.1)
    pos = np.full(times.shape, 0.2)
    yaw = np.zeros(times.shape)
    # 一瞬の飛び (1 s の中の少数の標本) は、1 s ごとの中央値では数えない
    pos[(times >= 5.0) & (times < 5.2)] = 50.0
    # yaw のずれ 2 つ (間 5 s) は 1 つの事象にまとめる
    yaw[(times >= 10.0) & (times < 15.0)] = 1.0
    yaw[(times >= 20.0) & (times < 25.0)] = 1.0
    # 最後までずれたまま
    pos[times >= 50.0] = 8.0
    episodes = lm.deviation_episodes(times, pos, yaw, 6.0, math.radians(30), merge_sec=10.0)
    assert [(e["start_s"], e["end_s"]) for e in episodes] == [
        pytest.approx((10.0, 25.0)), pytest.approx((50.0, 60.0))]
    assert episodes[0]["max_yaw_rad"] == pytest.approx(1.0)
    assert episodes[0]["until_end"] is False
    assert episodes[1]["until_end"] is True
    assert episodes[1]["max_position_m"] == pytest.approx(8.0)
    # 間が merge_sec を超えれば分ける
    assert len(lm.deviation_episodes(times, pos, yaw, 6.0, math.radians(30), merge_sec=2.0)) == 3
    assert lm.deviation_episodes(times[:0], pos[:0], yaw[:0], 6.0, 0.5) == []


def test_turn_errors_marks_mispick_only_when_deviation_persists():
    times = np.arange(0.0, 40.0, 0.1)
    turns = [(10.0, 14.0, math.pi / 2)]
    pos = np.full(times.shape, 0.2)
    yaw = np.zeros(times.shape)
    # 角の中だけずれても、抜けた後に戻れば取り違えではない
    pos[(times >= 11.0) & (times < 13.0)] = 7.0
    episodes = lm.deviation_episodes(times, pos, yaw, 6.0, 0.5)
    ok = lm.turn_errors(times, pos, yaw, turns, 10.0, episodes)
    assert ok[0]["mispicked"] is False
    assert ok[0]["deviated"] is True
    assert ok[0]["max_position_m"] == pytest.approx(7.0)
    # 抜けた後も yaw が 90 度ずれたまま
    yaw[times >= 12.0] = math.pi / 2
    episodes = lm.deviation_episodes(times, pos, yaw, 6.0, 0.5)
    bad = lm.turn_errors(times, pos, yaw, turns, 10.0, episodes)
    assert bad[0]["mispicked"] is True
    # 角から離れた事象は、その角のずれにしない
    far = lm.turn_errors(times, pos, yaw, [(2.0, 4.0, 1.0)], 3.0, episodes)
    assert far[0]["deviated"] is False
    # 抜けた後の記録が無ければ判定しない
    none = lm.turn_errors(times, pos, yaw, [(50.0, 51.0, 1.0)], 10.0, episodes)
    assert none[0]["mispicked"] is None
    assert none[0]["deviated"] is None


def test_gnss_weak_mask_by_sigma_and_staleness():
    gps_t = np.array([1.0, 2.0, 3.0])
    gps_sigma = np.array([0.3, 2.0, 0.3])
    times = np.array([0.5, 1.5, 2.5, 3.5, 10.0])
    weak = lm.gnss_weak_mask(times, gps_t, gps_sigma, sigma_threshold=1.0, stale_sec=3.0)
    # 届く前・σ が大きい・古すぎるときは弱い
    assert list(weak) == [True, False, True, False, True]
