import math
import os
import sys

import numpy as np
import pytest

from tools.common import odom_calib as oc

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "mg_drivers", "scripts"))

from wheel_odom_correction import OdomCorrector  # noqa: E402

K_V, K_W, C_BIAS = 1.03, 0.95, 0.004


def _synthetic_run(step=0.05, length=120.0, node_every=10, time_offset=0.0):
    """S 字に曲がる経路を、真値のノード列と、誤差を含む生のオドメトリで作る。

    生のオドメトリは、補正 (K_V, K_W, C_BIAS) をかけると真値に戻るように、逆算して作る。
    """
    n = int(length / step)
    curvature = 0.25 * np.sin(np.arange(n) * step / 6.0)
    dyaw_gt = curvature * step
    x = y = yaw = 0.0
    gt = [(0.0, x, y, yaw)]
    raw_x = raw_y = raw_yaw = 0.0
    raw = [(0.0, 0.0, 0.0, 0.0)]
    for k in range(n):
        # 真値: 車体の前方に step 進み、dyaw 回る
        mid = yaw + 0.5 * dyaw_gt[k]
        x += step * math.cos(mid)
        y += step * math.sin(mid)
        yaw += dyaw_gt[k]
        # 生のオドメトリ: 補正で真値に戻る量
        raw_dx = step / K_V
        raw_dyaw = (dyaw_gt[k] - C_BIAS * step) / K_W
        raw_mid = raw_yaw + 0.5 * raw_dyaw
        raw_x += raw_dx * math.cos(raw_mid)
        raw_y += raw_dx * math.sin(raw_mid)
        raw_yaw += raw_dyaw
        t = (k + 1) * 0.05
        raw.append((t + time_offset, raw_x, raw_y, raw_yaw))
        if (k + 1) % node_every == 0:
            gt.append((t, x, y, yaw))
    return np.array(gt), np.array(raw)


def _windows_data(gt, raw, window_m=5.0, time_offset=0.0):
    windows = oc.build_windows(gt, window_m, max_gap_sec=2.0)
    steps_list, gt_rel, keep = [], [], []
    for w in windows:
        steps = oc.steps_between(raw, w.t_a, w.t_b, time_offset)
        if steps is None:
            continue
        steps_list.append(steps)
        gt_rel.append(oc.gt_relative(gt, w))
        keep.append(w)
    return keep, steps_list, np.array(gt_rel)


def test_build_windows_length_and_gaps():
    nodes = np.array([[i * 1.0, i * 0.5, 0.0, 0.0] for i in range(40)])
    windows = oc.build_windows(nodes, 5.0, 2.0)
    assert len(windows) >= 3
    for w in windows:
        length = (nodes[w.j, 1] - nodes[w.i, 1])
        assert 5.0 <= length < 5.6
    # 大きな時間の空きをまたぐ窓は作らない
    nodes[20:, 0] += 100.0
    for w in oc.build_windows(nodes, 5.0, 2.0):
        assert not (w.i < 20 <= w.j)


def test_integrate_matches_odom_corrector():
    rng = np.random.default_rng(3)
    yaw = np.cumsum(rng.normal(0, 0.02, 200))
    x = np.cumsum(0.05 * np.cos(yaw))
    y = np.cumsum(0.05 * np.sin(yaw))
    odom = np.stack([np.arange(200) * 0.05, x, y, yaw], axis=1)
    steps = oc.steps_between(odom, odom[0, 0], odom[-1, 0])
    corrector = OdomCorrector(K_V, K_W, C_BIAS, 5.0)
    pose = None
    for row in odom:
        pose = corrector.update(*_relative(odom[0], row))
    expected = np.array(pose)
    actual = oc.integrate(steps, K_V, K_W, C_BIAS)
    assert actual == pytest.approx(expected, abs=1e-9)


def _relative(start, row):
    """始点の姿勢を原点にした座標での、生の姿勢。"""
    c, s = math.cos(start[3]), math.sin(start[3])
    dx, dy = row[1] - start[1], row[2] - start[2]
    return c * dx + s * dy, -s * dx + c * dy, row[3] - start[3]


def test_fit_recovers_known_parameters():
    gt, raw = _synthetic_run()
    _, steps_list, gt_rel = _windows_data(gt, raw)
    assert len(steps_list) > 10
    theta, cost = oc.fit_parameters(steps_list, gt_rel)
    assert theta == pytest.approx([K_V, K_W, C_BIAS], abs=2e-3)
    assert cost < 1e-6


def test_fit_is_robust_to_outlier_windows():
    gt, raw = _synthetic_run()
    _, steps_list, gt_rel = _windows_data(gt, raw)
    gt_rel = gt_rel.copy()
    gt_rel[2] += [1.5, -1.0, 0.3]   # 真値の誤った窓
    gt_rel[7] += [-2.0, 0.5, -0.2]
    theta, _ = oc.fit_parameters(steps_list, gt_rel)
    assert theta == pytest.approx([K_V, K_W, C_BIAS], abs=1e-2)


def test_time_offset_scan_finds_the_delay():
    delay = 0.2
    gt, raw = _synthetic_run(time_offset=delay)
    best = None
    for offset in np.arange(0.0, 0.41, 0.05):
        _, steps_list, gt_rel = _windows_data(gt, raw, time_offset=offset)
        _, cost = oc.fit_parameters(steps_list, gt_rel)
        if best is None or cost < best[1]:
            best = (offset, cost)
    assert best[0] == pytest.approx(delay, abs=0.051)


def test_velocity_errors_are_small_after_fit_and_large_without():
    gt, raw = _synthetic_run()
    windows, steps_list, gt_rel = _windows_data(gt, raw)
    durations = np.array([w.t_b - w.t_a for w in windows])
    theta, _ = oc.fit_parameters(steps_list, gt_rel)
    v_fit, w_fit = oc.window_velocity_errors(steps_list, gt_rel, durations, theta)
    v_raw, w_raw = oc.window_velocity_errors(steps_list, gt_rel, durations, np.array([1.0, 1.0, 0.0]))
    assert np.max(np.abs(v_fit)) < 1e-3
    assert np.max(np.abs(w_fit)) < 1e-3
    assert np.median(np.abs(v_raw)) > 0.02
    assert np.median(np.abs(w_raw)) > 0.005


def test_steps_between_out_of_range_is_none():
    odom = np.array([[0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0]])
    assert oc.steps_between(odom, -0.5, 0.5) is None
    assert oc.steps_between(odom, 0.5, 2.0) is None
    assert oc.steps_between(odom, 0.0, 1.0, time_offset=0.5) is None
