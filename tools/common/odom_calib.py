"""ホイールオドメトリの校正 (スケール・バイアス・時刻ずれの推定) の計算モジュール (numpy のみに依存)。

走行ログのオドメトリ (/odom の姿勢) と、真値 (SLAM の pose_graph のキーフレーム姿勢) を比べる。
全体を一度に積算すると誤差が累積して崩れるため、真値の経路で一定の長さごとに区切った「窓」ごとに、
窓の始点から終点への相対移動を比べる。

補正のモデルは mg_drivers/scripts/wheel_odom_correction.py の OdomCorrector と同じ:
  並進 dx' = k_v * dx (車体座標系)、旋回 dyaw' = k_w * dyaw + c * dx'
  (c は走行距離あたりに曲がる量 [rad/m])
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Window:
    """真値のノード i から j までの区間。"""
    i: int
    j: int
    t_a: float
    t_b: float


def build_windows(
    nodes: np.ndarray, window_m: float, max_gap_sec: float, min_move_m: float = 1.0,
) -> List[Window]:
    """真値のノード列 [t, x, y, yaw] から、経路長が window_m 以上になる窓を、重ならないように作る。

    ノード間の時間が max_gap_sec を超える (停止や欠損) 窓は捨てる。
    """
    windows: List[Window] = []
    n = nodes.shape[0]
    i = 0
    while i < n - 1:
        length = 0.0
        j = i
        broken = False
        while j < n - 1 and length < window_m:
            if nodes[j + 1, 0] - nodes[j, 0] > max_gap_sec:
                broken = True
                break
            length += float(np.hypot(nodes[j + 1, 1] - nodes[j, 1], nodes[j + 1, 2] - nodes[j, 2]))
            j += 1
        if broken:
            i = j + 1
            continue
        if length >= window_m and j > i:
            move = float(np.hypot(nodes[j, 1] - nodes[i, 1], nodes[j, 2] - nodes[i, 2]))
            if move >= min_move_m:
                windows.append(Window(i, j, float(nodes[i, 0]), float(nodes[j, 0])))
        i = j if j > i else i + 1
    return windows


def gt_relative(nodes: np.ndarray, window: Window) -> np.ndarray:
    """真値の始点の車体座標系での、終点への相対移動 [dx, dy, dyaw]。"""
    a, b = nodes[window.i], nodes[window.j]
    dx, dy = b[1] - a[1], b[2] - a[2]
    c, s = np.cos(a[3]), np.sin(a[3])
    return np.array([c * dx + s * dy, -s * dx + c * dy, float(wrap_angle(b[3] - a[3]))])


def steps_between(
    odom: np.ndarray, t_a: float, t_b: float, time_offset: float = 0.0,
) -> Optional[np.ndarray]:
    """オドメトリ姿勢 odom (N, 4) [t, x, y, yaw(unwrap 済み)] の t_a から t_b までを、
    中間の向きの車体座標系での増分 [dx, dy, dyaw] の列 (K, 3) にする。

    time_offset: オドメトリの時刻を time_offset 秒だけ過去へずらして参照する (オドメトリの遅れ)。
    区間が範囲外なら None。
    """
    t = odom[:, 0] - time_offset
    if t_a < t[0] or t_b > t[-1]:
        return None
    inside = (t > t_a) & (t < t_b)
    times = np.concatenate([[t_a], t[inside], [t_b]])
    x = np.interp(times, t, odom[:, 1])
    y = np.interp(times, t, odom[:, 2])
    yaw = np.interp(times, t, odom[:, 3])
    dyaw = np.diff(yaw)
    mid = yaw[:-1] + 0.5 * dyaw
    dx, dy = np.diff(x), np.diff(y)
    c, s = np.cos(mid), np.sin(mid)
    return np.stack([c * dx + s * dy, -s * dx + c * dy, dyaw], axis=1)


def integrate(steps: np.ndarray, k_v: float, k_w: float, c: float) -> np.ndarray:
    """増分の列を補正して積算し、始点の座標系での終点 [x, y, yaw] を返す。"""
    dxb = k_v * steps[:, 0]
    dyb = k_v * steps[:, 1]
    dyaw = k_w * steps[:, 2] + c * dxb
    yaw_after = np.cumsum(dyaw)
    mid = yaw_after - dyaw + 0.5 * dyaw
    cm, sm = np.cos(mid), np.sin(mid)
    return np.array([
        np.sum(cm * dxb - sm * dyb),
        np.sum(sm * dxb + cm * dyb),
        yaw_after[-1],
    ])


def residual_vector(
    theta: np.ndarray, steps_list: List[np.ndarray], gt_list: np.ndarray, yaw_weight: float,
) -> np.ndarray:
    """全窓の残差 (真値 - 補正後のオドメトリ) を並べたベクトル。yaw の残差は yaw_weight [m/rad] で長さに換算する。"""
    k_v, k_w, c = theta
    res = np.empty(3 * len(steps_list))
    for n, steps in enumerate(steps_list):
        pred = integrate(steps, k_v, k_w, c)
        res[3 * n] = gt_list[n, 0] - pred[0]
        res[3 * n + 1] = gt_list[n, 1] - pred[1]
        res[3 * n + 2] = yaw_weight * float(wrap_angle(gt_list[n, 2] - pred[2]))
    return res


def huber_cost(res: np.ndarray, delta: float) -> float:
    a = np.abs(res)
    return float(np.sum(np.where(a <= delta, 0.5 * a ** 2, delta * (a - 0.5 * delta))))


def fit_parameters(
    steps_list: List[np.ndarray],
    gt_list: np.ndarray,
    yaw_weight: float = 5.0,
    huber_delta: float = 0.1,
    iterations: int = 30,
    initial: Tuple[float, float, float] = (1.0, 1.0, 0.0),
) -> Tuple[np.ndarray, float]:
    """k_v, k_w, c を Huber 損失で推定する (再重み付き最小二乗 + 数値ヤコビアン)。

    Returns: (theta = [k_v, k_w, c], Huber 損失)
    """
    theta = np.array(initial, dtype=float)
    scale = np.array([1e-4, 1e-4, 1e-4])
    for _ in range(iterations):
        res = residual_vector(theta, steps_list, gt_list, yaw_weight)
        jac = np.empty((res.size, 3))
        for k in range(3):
            step = np.zeros(3)
            step[k] = scale[k]
            jac[:, k] = (residual_vector(theta + step, steps_list, gt_list, yaw_weight)
                         - residual_vector(theta - step, steps_list, gt_list, yaw_weight)) / (2 * scale[k])
        # Huber の重み (外れ値の窓の影響を抑える)
        weights = np.where(np.abs(res) <= huber_delta, 1.0, huber_delta / np.maximum(np.abs(res), 1e-12))
        jw = jac * weights[:, None]
        # 残差 r(θ) = gt - pred(θ) なので、r(θ + d) ≈ r + J d を最小にする d = -(JᵀWJ)⁻¹ JᵀW r
        lhs = jac.T @ jw + 1e-9 * np.eye(3)
        delta = -np.linalg.solve(lhs, jw.T @ res)
        theta = theta + delta
        if np.max(np.abs(delta)) < 1e-9:
            break
    res = residual_vector(theta, steps_list, gt_list, yaw_weight)
    return theta, huber_cost(res, huber_delta)


def window_velocity_errors(
    steps_list: List[np.ndarray], gt_list: np.ndarray, durations: np.ndarray, theta: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """窓ごとの、平均の速度誤差 (真値 - 補正後) を返す。

    Returns:
      v_err: 進行方向の速度誤差 [m/s]
      w_err: 角速度誤差 [rad/s]
    EKF は速度を観測として使うので、この低周波の (窓の間続く) 誤差が共分散の目安になる。
    """
    v_err = np.empty(len(steps_list))
    w_err = np.empty(len(steps_list))
    for n, steps in enumerate(steps_list):
        pred = integrate(steps, *theta)
        v_err[n] = (gt_list[n, 0] - pred[0]) / durations[n]
        w_err[n] = float(wrap_angle(gt_list[n, 2] - pred[2])) / durations[n]
    return v_err, w_err
