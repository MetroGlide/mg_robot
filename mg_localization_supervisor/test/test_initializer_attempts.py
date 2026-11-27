"""初期姿勢を決めるまでの照合の段取り (initializer/attempts.py) のテスト。"""
import math

import pytest

from mg_localization_supervisor.initializer.attempts import InitAttempts, compose, propagate, relative


def test_the_first_miss_publishes_the_gnss_pose_once_and_retries_after_moving():
    attempts = InitAttempts(retry_distance_m=2.0)
    assert attempts.should_attempt((0.0, 0.0))
    assert attempts.record((0.0, 0.0), matched=False)
    assert not attempts.should_attempt((1.0, 1.0))
    assert attempts.should_attempt((2.0, 0.0))
    assert not attempts.record((2.0, 0.0), matched=False)
    assert attempts.should_attempt((4.0, 0.0))
    assert not attempts.record((4.0, 0.0), matched=True)
    assert attempts.decided and attempts.attempts == 3
    assert not attempts.should_attempt((10.0, 0.0))


def test_an_external_initial_pose_stops_the_gnss_initialization():
    attempts = InitAttempts(retry_distance_m=2.0)
    attempts.record((0.0, 0.0), matched=False)
    attempts.take_external()
    assert attempts.decided and attempts.external
    assert not attempts.should_attempt((10.0, 0.0))


def test_reset_starts_over():
    attempts = InitAttempts(retry_distance_m=2.0)
    attempts.record((0.0, 0.0), matched=True)
    attempts.reset()
    assert not attempts.decided and attempts.should_attempt((0.0, 0.0))
    assert attempts.record((0.0, 0.0), matched=False)


def test_relative_and_compose_are_inverse():
    a, b = (1.0, 2.0, 0.7), (-3.0, 0.5, -2.0)
    c = compose(a, relative(a, b))
    assert c == pytest.approx(b)


def test_propagate_moves_the_matched_pose_by_the_odometry_since_the_scan():
    # map では北向き (yaw = pi/2)、odom では東向きの姿勢で照合した後、odom で前へ 1 m 進んだ
    matched = (5.0, 5.0, math.pi / 2)
    odom_at_scan, odom_now = (10.0, 0.0, 0.0), (11.0, 0.0, 0.0)
    assert propagate(matched, odom_at_scan, odom_now) == pytest.approx((5.0, 6.0, math.pi / 2))
    # 進まなければそのまま
    assert propagate(matched, odom_at_scan, odom_at_scan) == pytest.approx(matched)
