"""自己位置推定の評価指標を計算する共通モジュール (numpy のみに依存)。

姿勢は SE(2) の [x, y, yaw] で表す。時刻は秒 (昇順) とする。
"""

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 自己位置推定の状態 (localization_supervisor_msgs/LocalizationStatus.state と対応)
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
        "min": float(np.min(values)),
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


def detect_turns(nodes: np.ndarray, min_turn_rad: float, window_sec: float) -> List[Tuple[float, float, float]]:
    """真値のノード列から、window_sec 以内に向きが min_turn_rad 以上変わる区間 (角) を見つける。

    Args:
      nodes: shape (N, 4) -> [t, x, y, yaw] (t 昇順)
    Returns:
      角ごとの (開始時刻, 終了時刻, 向きの変化 [rad]。左回りが正)。重なる区間はまとめる
    """
    if nodes.shape[0] < 2:
        return []
    t = nodes[:, 0]
    yaw = np.unwrap(nodes[:, 3])
    intervals: List[List[float]] = []
    j = 0
    for i in range(t.size):
        j = max(j, i)
        while j + 1 < t.size and t[j + 1] - t[i] <= window_sec:
            j += 1
        if j == i or abs(yaw[j] - yaw[i]) < min_turn_rad:
            continue
        if intervals and t[i] <= intervals[-1][1]:
            intervals[-1][1] = max(intervals[-1][1], t[j])
        else:
            intervals.append([t[i], t[j]])
    turns = []
    for start, end in intervals:
        i0, i1 = np.searchsorted(t, start), np.searchsorted(t, end)
        turns.append((float(start), float(end), float(yaw[i1] - yaw[i0])))
    return turns


def deviation_episodes(
    times: np.ndarray,
    pos_err: np.ndarray,
    yaw_err: np.ndarray,
    pos_threshold: float,
    yaw_threshold: float,
    bin_sec: float = 1.0,
    merge_sec: float = 10.0,
) -> List[Dict[str, Any]]:
    """推定が真値から大きくずれた区間 (事象) を、時刻に沿って見つける。

    bin_sec ごとの誤差の中央値が、位置 pos_threshold [m] か yaw yaw_threshold [rad] を超えた区間を
    ずれとし、間が merge_sec 以下の区間はつなぐ。中央値にするのは、一瞬の飛びを数えないため。
    真値の無い時間は判定しない (前後のずれた区間は、間が merge_sec 以下ならつながる)。

    Returns:
      事象ごとの start_s, end_s, max_position_m, max_yaw_rad (区間の中の中央値の最大)、
      until_end (真値と比べられた最後の時刻までずれたまま)
    """
    if times.size == 0:
        return []
    bins = np.floor((times - times[0]) / bin_sec).astype(int)
    keys, inverse = np.unique(bins, return_inverse=True)
    bin_t = times[0] + keys * bin_sec
    bin_pos = np.array([np.median(pos_err[inverse == i]) for i in range(keys.size)])
    bin_yaw = np.array([np.median(np.abs(yaw_err[inverse == i])) for i in range(keys.size)])
    bad = np.flatnonzero((bin_pos > pos_threshold) | (bin_yaw > yaw_threshold))
    episodes: List[Dict[str, Any]] = []
    if bad.size == 0:
        return episodes
    groups = np.split(bad, np.flatnonzero(np.diff(bin_t[bad]) > merge_sec + bin_sec) + 1)
    for group in groups:
        episodes.append({
            "start_s": float(bin_t[group[0]]),
            "end_s": float(bin_t[group[-1]] + bin_sec),
            "max_position_m": float(bin_pos[group].max()),
            "max_yaw_rad": float(bin_yaw[group].max()),
            "until_end": bool(group[-1] == keys.size - 1),
        })
    return episodes


def turn_errors(
    times: np.ndarray,
    pos_err: np.ndarray,
    yaw_err: np.ndarray,
    turns: List[Tuple[float, float, float]],
    after_sec: float,
    episodes: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """角ごとに、角の中と抜けた後の誤差と、ずれたか、取り違えたまま残ったかを返す。

    判定は deviation_episodes の事象に従う (角は事象が起きた場所を示すだけ)。
    deviated: 角の始まりから抜けた後 after_sec までに、事象が重なった (後で戻ったものも含む)
    mispicked: その事象が、抜けた後 after_sec を過ぎても続いた (または最後までずれたまま)
    抜けた後 after_sec の間に真値が無ければ、判定できない (None)。
    """
    results = []
    for start, end, dyaw in turns:
        entry: Dict[str, Any] = {"start_s": start, "end_s": end, "turn_rad": dyaw}
        during = (times >= start) & (times < end + after_sec)
        after = (times >= end) & (times < end + after_sec)
        if not np.any(after):
            entry["deviated"] = None
            entry["mispicked"] = None
            results.append(entry)
            continue
        overlapping = [e for e in episodes if e["start_s"] < end + after_sec and e["end_s"] > start]
        entry.update({
            "max_position_m": float(np.max(pos_err[during])),
            "max_yaw_rad": float(np.max(np.abs(yaw_err[during]))),
            "after_position_m": float(np.median(pos_err[after])),
            "after_yaw_rad": float(np.median(np.abs(yaw_err[after]))),
            "deviated": bool(overlapping),
            "mispicked": any(e["until_end"] or e["end_s"] > end + after_sec for e in overlapping),
        })
        results.append(entry)
    return results


def gnss_weak_mask(times: np.ndarray, gps_times: np.ndarray, gps_sigma: np.ndarray,
                   sigma_threshold: float, stale_sec: float) -> np.ndarray:
    """times の各時点で GNSS が弱いか。直前の測位の σ が sigma_threshold を超えるか、stale_sec より古い
    (または 1 つも無い) とき弱いとする。"""
    idx = hold_index(gps_times, times)
    weak = np.ones(times.shape, dtype=bool)
    has = idx >= 0
    fresh = times[has] - gps_times[idx[has]] <= stale_sec
    weak[has] = ~fresh | (gps_sigma[idx[has]] > sigma_threshold)
    return weak


# 監督ノードの状態の記録 (行: [t, state, gnss_d2, jump_m, ekf_diff_m, scan_ratio, scan_gain]) の値の列
# localization_supervisor_msgs/LocalizationCheck.status の「判定できない」
CHECK_UNKNOWN = 2


def summarize_status_values(status: np.ndarray, check_scores: Dict[str, np.ndarray]) -> Dict[str, Any]:
    """監督の総合の強さと、判定のインスタンスごとの score (判定できた周期だけ) の分布、状態ごとの時間の割合。

    status は [t, state, fusion_level] の行。
    """
    values = {"fusion_level": summarize(status[:, 2])}
    for name, scores in sorted(check_scores.items()):
        values[name] = summarize(scores)
    states = status[:, 1].astype(int)
    fractions = {int(s): float(np.mean(states == s)) for s in np.unique(states)}
    return {"values": values, "state_fractions": fractions}


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
