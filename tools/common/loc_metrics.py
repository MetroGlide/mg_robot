"""自己位置推定の評価指標を計算する共通モジュール (numpy のみに依存)。

姿勢は SE(2) の [x, y, yaw] で表す。時刻は秒 (昇順) とする。
"""

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 自己位置推定の状態 (mg_msgs/LocalizationStatus.state と対応)
STATE_NORMAL = 0


def wrap_angle(angle):
    """角度を [-pi, pi) に正規化する。"""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def summarize(values: np.ndarray) -> Dict[str, Any]:
    """1 次元の値 (絶対値を想定) の統計量を返す。"""
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"n": 0}
    return {
        "n": int(values.size),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "rms": float(np.sqrt(np.mean(values ** 2))),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """SE(2) の合成 a * b を要素ごとに求める。shape は (N, 3) 同士。"""
    c = np.cos(a[:, 2])
    s = np.sin(a[:, 2])
    x = a[:, 0] + c * b[:, 0] - s * b[:, 1]
    y = a[:, 1] + s * b[:, 0] + c * b[:, 1]
    yaw = wrap_angle(a[:, 2] + b[:, 2])
    return np.stack([x, y, yaw], axis=1)


def hold_index(src_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """query_times 時点で直前 (以前) にある src_times の添字を返す。

    Nav2 が TF を参照するときと同じく、最新値を保持する (ゼロ次ホールド) 扱い。
    src_times より前の query は -1 を返す。
    """
    return np.searchsorted(src_times, query_times, side="right") - 1


def pose_errors(est: np.ndarray, gt: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """推定と真値の姿勢誤差を返す。

    Returns:
      pos_err: 位置誤差の大きさ [m] (N,)
      yaw_err: yaw 誤差 [rad] (符号付き、真値からの差) (N,)
    """
    pos_err = np.hypot(est[:, 0] - gt[:, 0], est[:, 1] - gt[:, 1])
    yaw_err = wrap_angle(est[:, 2] - gt[:, 2])
    return pos_err, yaw_err


def correction_jumps(
    map_odom: np.ndarray, base_in_odom: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """map->odom が更新されたときに、ロボットが map 上で受ける姿勢の飛びを返す。

    map->odom の並進は odom 原点から離れるほど yaw の変化で大きく動くため、その差は使わない。
    同じ odom->base に更新前後の map->odom を適用して比べる。

    Args:
      map_odom: 更新ごとの map->odom (N, 3)
      base_in_odom: 各更新 (2 番目以降) の時刻の odom->base (N-1, 3)
    Returns:
      (位置の飛び [m]、|yaw の飛び| [rad]) それぞれ (N-1,)
    """
    if map_odom.shape[0] < 2:
        return np.zeros(0), np.zeros(0)
    before = compose(map_odom[:-1], base_in_odom)
    after = compose(map_odom[1:], base_in_odom)
    pos = np.hypot(after[:, 0] - before[:, 0], after[:, 1] - before[:, 1])
    yaw = np.abs(wrap_angle(after[:, 2] - before[:, 2]))
    return pos, yaw


def nees(errors: np.ndarray, cov6: np.ndarray) -> np.ndarray:
    """NEES (誤差を共分散で正規化した二乗マハラノビス距離) を求める。

    Args:
      errors: (N, 3) [ex, ey, eyaw]
      cov6: (N, 36) nav_msgs の 6x6 共分散 (行優先)。x, y, yaw の 3x3 を取り出して使う。
    一貫した推定なら平均は自由度 (3) に近づく。逆行列が求まらない標本は nan にする。
    """
    idx = [0, 1, 5]
    result = np.full(errors.shape[0], np.nan)
    for i in range(errors.shape[0]):
        cov = cov6[i].reshape(6, 6)[np.ix_(idx, idx)]
        try:
            result[i] = float(errors[i] @ np.linalg.solve(cov, errors[i]))
        except np.linalg.LinAlgError:
            continue
    return result


def best_lag(
    ref_times: np.ndarray,
    ref_values: np.ndarray,
    sig_times: np.ndarray,
    sig_values: np.ndarray,
    max_lag: float = 2.0,
    step: float = 0.05,
) -> Tuple[float, float]:
    """sig が ref に対して何秒遅れているかを、二乗誤差が最小になる遅れとして求める。

    sig(t) ~ ref(t - lag) を探す。Returns: (lag [s], そのときの RMSE)
    """
    best = (0.0, float("inf"))
    for lag in np.arange(0.0, max_lag + 1e-9, step):
        shifted = np.interp(sig_times - lag, ref_times, ref_values)
        mask = (sig_times - lag >= ref_times[0]) & (sig_times - lag <= ref_times[-1])
        if not np.any(mask):
            continue
        rmse = float(np.sqrt(np.mean((shifted[mask] - sig_values[mask]) ** 2)))
        if rmse < best[1]:
            best = (float(lag), rmse)
    return best


def recovery_metrics(
    times: np.ndarray,
    pos_err: np.ndarray,
    fault_time: float,
    end_time: float,
    ok_threshold: float,
    hold_sec: float,
) -> Dict[str, Any]:
    """故障を注入した時刻からの、誤差の悪化と復旧の指標を返す。

    Args:
      times, pos_err: 位置誤差の時系列
      fault_time: 故障の注入時刻
      end_time: この故障の評価区間の終わり (次の故障の時刻か bag の終端)
      ok_threshold: 復旧したとみなす位置誤差 [m]
      hold_sec: その誤差以下が連続していなければならない時間 [s]
    Returns:
      max_error_m: 区間内の最大誤差
      deviated: 閾値を超えて悪化したか
      recovery_sec: 悪化してから復旧を確認できるまでの、故障時刻からの経過時間 (未復旧は None)
    """
    mask = (times >= fault_time) & (times < end_time)
    t = times[mask]
    e = pos_err[mask]
    if t.size == 0:
        return {"max_error_m": None, "deviated": False, "recovery_sec": None}
    result: Dict[str, Any] = {
        "max_error_m": float(np.max(e)),
        "deviated": bool(np.any(e > ok_threshold)),
        "recovery_sec": None,
    }
    over = np.where(e > ok_threshold)[0]
    if over.size == 0:
        result["recovery_sec"] = 0.0
        return result
    # 最初に悪化した後で、誤差が hold_sec 以上ずっと閾値以下になる最初の時刻を探す
    ok_start: Optional[float] = None
    for i in range(over[0], t.size):
        if e[i] <= ok_threshold:
            if ok_start is None:
                ok_start = t[i]
            if t[i] - ok_start >= hold_sec:
                result["recovery_sec"] = float(ok_start - fault_time)
                return result
        else:
            ok_start = None
    return result


def count_episodes(
    times: np.ndarray,
    states: np.ndarray,
    allowed_windows: List[Tuple[float, float]],
) -> Dict[str, Any]:
    """状態が NORMAL 以外に入った回数 (エピソード) を、許容区間の内外で数える。

    許容区間 (故障を注入した時刻の前後) の外で起きたものは誤検知として数える。
    """
    inside = 0
    outside = 0
    previous = STATE_NORMAL
    for t, s in zip(times, states):
        if previous == STATE_NORMAL and s != STATE_NORMAL:
            if any(a <= t <= b for a, b in allowed_windows):
                inside += 1
            else:
                outside += 1
        previous = s
    return {"episodes_in_fault_windows": inside, "false_detections": outside}


def first_detection_delay(
    times: np.ndarray, states: np.ndarray, fault_time: float, end_time: float
) -> Optional[float]:
    """故障を注入してから、状態が NORMAL 以外になるまでの時間 [s] (無ければ None)。"""
    mask = (times >= fault_time) & (times < end_time) & (states != STATE_NORMAL)
    if not np.any(mask):
        return None
    return float(times[mask][0] - fault_time)
