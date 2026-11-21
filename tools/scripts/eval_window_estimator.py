#!/usr/bin/env python3
"""
eval_window_estimator.py

窓つきの推定器 (直近の窓の姿勢をまとめて解き直す因子グラフ) の試作を、再生評価
(run_localization_variant.sh) の出力 bag の入力に対してオフラインで回し、走行に使った EKF の姿勢と
真値との誤差で比べる。推定器は、各時刻でそれまでに届いた観測だけを使う (先の観測は使わない)。

比べる推定:
  ekf          : 再生で走行に使った姿勢 (/tf の map->odom と odom->base の合成)
  amcl         : AMCL の最新の姿勢に、その後のオドメトリの動きを足したもの
  window_gnss  : 窓つきの推定器 (オドメトリ + GNSS)
  window_full  : 窓つきの推定器 (オドメトリ + GNSS + AMCL)
  window_full_lidar : 上に LiDAR オドメトリ (--lidar-odom) を足したもの。ホイールと LiDAR オドメトリの動きが
                 食い違ったら、絶対の情報源と合わない方を弱め、合う方を基準にする

推定器の作り:
  - キーフレームは絶対の観測 (GNSS、AMCL) の時刻ごとに置き、連続するキーフレームをオドメトリの因子でつなぐ。
  - 窓 (走った距離と時間の上限) を超えた古い姿勢は、今の解の周辺共分散を事前分布にして落とす。
  - 絶対の観測は外れ値に強い損失 (Cauchy) で入れる。AMCL は間引いて、共分散を割り引いて入れる
    (連続する出力は互いに独立ではなく、ホイールのオドメトリと地図を内部で使うため)。
  - 情報源の切り離し (--gate、動きの合意): 直近の区間で、一方の絶対の情報源だけが動いた量でオドメトリと
    合わないとき、その情報源を一時的に外す。動きが合い、切り離した時点の GNSS の偏りで位置も合えば戻す。
    AMCL も GNSS もオドメトリと合わないときは、直近のオドメトリを弱める (スリップ)。位置が食い違うのに
    原因を決められないときは「割れ」として数える。
  - GNSS の偏り (--gnss-bias): GNSS の位置 = 姿勢 + 偏り として、偏りをゆっくり変わる量として推定する
    (地図と GNSS の変換の誤差を、走りながら吸収する)。

真値は 2 通り:
  gnss : 評価する走行の SLAM 出力 (GNSS を拘束に使っている) を UTM 経由で地図の座標へ移したもの。
         GNSS に寄る推定ほど良く見える。地図と GNSS の整合の後の状態に当たる。
  map  : gnss の真値に、故障のない回 (--map-reference) の AMCL と真値の差のゆっくり変わる成分を足したもの。
         今の地図の座標での真値の近似。参照の回と時刻の範囲が重ならない変種では出さない。

入力のトピック: /odom (ホイール、故障の注入の後)、/odom/gps (地図座標の GNSS)、/amcl_pose。

例:
  python3 tools/scripts/eval_window_estimator.py \\
      --map-reference .../eval_loc/map043837_eval051635_short/baseline \\
      .../eval_loc/map043837_eval051635_short/base_none_kidnap --gate --gnss-bias --out /tmp/window.md
"""

import argparse
import glob
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import gtsam
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader, read_map_base_poses  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.pose_graph import interpolate_nodes, load_slam_output, shift_nodes_to_anchor  # noqa: E402
from tools.scripts.loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

# 評価の時刻の間隔 [s]
EVAL_STEP_SEC = 0.2
# SLAM のキーフレームの間隔 (0.5 m) より少し大きい。止まっている間の真値の補間を許す移動量 [m]
STATIONARY_DIST_M = 0.6
# 「ずれたまま走った」とみなす誤差 [m] (不変条件 1 の 1 m)
DEVIATION_M = 1.0
# 初期姿勢を与えた後の AMCL とみなす、走行に使った姿勢との距離 [m]
INIT_MATCH_M = 5.0
# 地図の座標の真値を作るときの、ゆっくり変わる成分の窓 [s]
MAP_OFFSET_WINDOW_SEC = 60.0
# オドメトリの欠けとみなす、隣の時刻との間隔 [s]
ODOM_GAP_SEC = 0.5
# どのオドメトリも欠けている区間をつなぐ拘束の σ (x [m], y [m], yaw [rad])
UNCOVERED_SIGMA = (5.0, 5.0, 1.0)
# 切り離しを判断するのに要る、直近の観測の数
GATE_MIN_SAMPLES = 3


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def X(i: int) -> int:
    return gtsam.symbol("x", i)


def B(j: int) -> int:
    return gtsam.symbol("b", j)


class Inputs:
    """推定器の入力。すべて header.stamp の時刻順。"""

    def __init__(self) -> None:
        self.odom: List[Tuple[float, float, float, float]] = []   # t, x, y, yaw (odom 座標)
        self.gps: List[Tuple[float, float, float, float]] = []    # t, x, y, σ
        self.amcl: List[Tuple[float, np.ndarray, np.ndarray]] = []  # t, [x, y, yaw], cov3


def read_inputs(bag: str) -> Inputs:
    reader = open_reader(bag, topics=["/odom", "/odom/gps", "/amcl_pose"])
    deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    data = Inputs()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserializer.deserialize(topic, raw)
        t = stamp_sec(msg.header.stamp)
        p = msg.pose.pose
        yaw = quaternion_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        if topic == "/odom":
            data.odom.append((t, p.position.x, p.position.y, yaw))
        elif topic == "/odom/gps":
            data.gps.append((t, p.position.x, p.position.y, math.sqrt(max(cov[0, 0], cov[1, 1], 0.0))))
        else:
            cov3 = cov[np.ix_([0, 1, 5], [0, 1, 5])]
            data.amcl.append((t, np.array([p.position.x, p.position.y, yaw]), cov3))
    data.odom.sort()
    data.gps.sort()
    data.amcl.sort(key=lambda r: r[0])
    return data


class Odometry:
    """odom 座標の姿勢列から、任意の時刻の姿勢 (線形補間) を返す。"""

    def __init__(self, rows: List[Tuple[float, float, float, float]]) -> None:
        arr = np.array(rows, dtype=float)
        self.t = arr[:, 0]
        self.xy = arr[:, 1:3]
        self.c = np.cos(arr[:, 3])
        self.s = np.sin(arr[:, 3])

    def covers(self, t0: float, t1: float) -> bool:
        """t0〜t1 がこのオドメトリの範囲にあり、間に GAP_SEC を超える欠けがないか。"""
        if t0 < self.t[0] or t1 > self.t[-1]:
            return False
        i, j = np.searchsorted(self.t, [t0, t1])
        seg = self.t[max(i - 1, 0):j + 1]
        return bool(len(seg) >= 2 and np.max(np.diff(seg)) <= ODOM_GAP_SEC)

    def at(self, t: float) -> gtsam.Pose2:
        i = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        w = float(np.clip((t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9), 0.0, 1.0))
        xy = self.xy[i - 1] * (1 - w) + self.xy[i] * w
        yaw = math.atan2(self.s[i - 1] * (1 - w) + self.s[i] * w, self.c[i - 1] * (1 - w) + self.c[i] * w)
        return gtsam.Pose2(xy[0], xy[1], yaw)


def robust(kernel: float, base: gtsam.noiseModel.Base, huber: bool = False) -> gtsam.noiseModel.Base:
    m = gtsam.noiseModel.mEstimator.Huber.Create(kernel) if huber else gtsam.noiseModel.mEstimator.Cauchy.Create(kernel)
    return gtsam.noiseModel.Robust.Create(m, base)


def gnss_factor(pose_key: int, bias_key: Optional[int], z: np.ndarray, noise) -> gtsam.CustomFactor:
    """GNSS の位置 = 姿勢の位置 (+ 偏り)。向きは拘束しない。"""

    def error(this: gtsam.CustomFactor, values: gtsam.Values, jacobians: Optional[List[np.ndarray]]) -> np.ndarray:
        pose = values.atPose2(this.keys()[0])
        e = np.array([pose.x(), pose.y()]) - z
        if bias_key is not None:
            e = e + values.atPoint2(this.keys()[1])
        if jacobians is not None:
            c, s = math.cos(pose.theta()), math.sin(pose.theta())
            jacobians[0] = np.array([[c, -s, 0.0], [s, c, 0.0]])
            if bias_key is not None:
                jacobians[1] = np.eye(2)
        return e

    keys = [pose_key] if bias_key is None else [pose_key, bias_key]
    return gtsam.CustomFactor(noise, keys, error)


def point2_between(k1: int, k2: int, sigma: float) -> gtsam.CustomFactor:
    def error(this: gtsam.CustomFactor, values: gtsam.Values, jacobians: Optional[List[np.ndarray]]) -> np.ndarray:
        if jacobians is not None:
            jacobians[0] = -np.eye(2)
            jacobians[1] = np.eye(2)
        return values.atPoint2(this.keys()[1]) - values.atPoint2(this.keys()[0])

    return gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(2, sigma), [k1, k2], error)


def point2_prior(key: int, mean: np.ndarray, cov: np.ndarray) -> gtsam.CustomFactor:
    def error(this: gtsam.CustomFactor, values: gtsam.Values, jacobians: Optional[List[np.ndarray]]) -> np.ndarray:
        if jacobians is not None:
            jacobians[0] = np.eye(2)
        return values.atPoint2(this.keys()[0]) - mean

    return gtsam.CustomFactor(gtsam.noiseModel.Gaussian.Covariance(cov), [key], error)


class Measurement:
    """絶対の観測 1 件。whiten は共分散の逆の平方根 (白色化の行列)。"""

    def __init__(self, source: str, t: float, pose_idx: int, z: np.ndarray, cov: np.ndarray,
                 bias_idx: Optional[int], factor: gtsam.NonlinearFactor, active: bool) -> None:
        self.source = source
        self.t = t
        self.pose_idx = pose_idx
        self.z = z
        self.whiten = np.linalg.cholesky(np.linalg.inv(cov)).T
        self.bias_idx = bias_idx
        self.factor = factor
        self.active = active


class WindowEstimator:
    def __init__(self, args: argparse.Namespace, motion: Dict[str, Odometry], use_amcl: bool) -> None:
        self.args = args
        self.motion = motion
        # 絶対の情報源の動きを比べ、解き直しの間の姿勢を進めるオドメトリ。ホイールが外れたら LiDAR に替える
        self.ref = "wheel"
        self.lidar_args = lidar_args(args)
        self.use_amcl = use_amcl
        self.keys: List[int] = []
        self.key_time: Dict[int, float] = {}
        self.key_dist: Dict[int, float] = {}
        self.edges: Dict[str, Dict[int, Tuple[int, gtsam.Pose2, np.ndarray]]] = {m: {} for m in motion}
        self.scale: Dict[str, Dict[int, float]] = {m: {} for m in motion}
        self.estimate: Dict[int, gtsam.Pose2] = {}
        self.meas: List[Measurement] = []
        self.bias_keys: List[int] = []
        self.bias_time: Dict[int, float] = {}
        self.bias_factor: Dict[int, gtsam.NonlinearFactor] = {}
        self.bias: Dict[int, np.ndarray] = {}
        self.pose_prior: Optional[Tuple[int, gtsam.Pose2, np.ndarray]] = None
        self.bias_prior: Optional[Tuple[int, np.ndarray, np.ndarray]] = None
        self.trusted = {"gps": True, "amcl": True}
        self.next_key = 0
        self.travelled = 0.0
        self.last_amcl_t = -1e9
        self.solve_ms: List[float] = []
        self.split_solves = 0
        self.gated_solves = {"gps": 0, "amcl": 0}
        self.odom_inflated_solves = 0
        self.lidar_ref_solves = 0
        self.frozen_bias: Optional[np.ndarray] = None

    def _add_key(self, t: float) -> int:
        key = self.next_key
        self.next_key += 1
        if self.keys:
            prev = self.keys[-1]
            for name, odom in self.motion.items():
                if not odom.covers(self.key_time[prev], t):
                    continue
                delta = odom.at(self.key_time[prev]).between(odom.at(t))
                d = math.hypot(delta.x(), delta.y())
                a = self.args if name == "wheel" else self.lidar_args
                sig = [a.odom_sigma_xy + a.odom_sigma_per_m * d] * 2
                sig.append(a.odom_sigma_yaw + a.odom_sigma_yaw_per_rad * abs(delta.theta()) + a.odom_sigma_yaw_per_m * d)
                self.edges[name][key] = (prev, delta, np.array(sig))
                self.scale[name][key] = 1.0
            delta = self.motion[self.ref].at(self.key_time[prev]).between(self.motion[self.ref].at(t))
            if not any(key in e for e in self.edges.values()):
                # どのオドメトリも欠けている区間は、弱い拘束でつなぐ (つながらない姿勢は解けない)
                self.edges[self.ref][key] = (prev, delta, np.array(UNCOVERED_SIGMA))
                self.scale[self.ref][key] = 1.0
            self.travelled += math.hypot(delta.x(), delta.y())
            self.estimate[key] = self.estimate[prev].compose(delta)
        self.keys.append(key)
        self.key_time[key] = t
        self.key_dist[key] = self.travelled
        return key

    def initialize(self, t: float, pose: np.ndarray, cov3: np.ndarray) -> None:
        key = self._add_key(t)
        self.estimate[key] = gtsam.Pose2(*pose)
        self.pose_prior = (key, gtsam.Pose2(*pose), np.diag(np.maximum(np.diag(cov3), [0.25, 0.25, 0.01])))

    def _add_bias(self, t: float) -> int:
        j = len(self.bias_time)
        if self.bias_keys:
            prev = self.bias_keys[-1]
            dt = max(t - self.bias_time[prev], 1e-3)
            self.bias_factor[j] = point2_between(B(prev), B(j), self.args.gnss_bias_rw * math.sqrt(dt) + 1e-3)
            self.bias[j] = self.bias[prev].copy()
        else:
            self.bias[j] = np.zeros(2)
            self.bias_prior = (j, np.zeros(2), np.eye(2) * self.args.gnss_bias_sigma0 ** 2)
        self.bias_keys.append(j)
        self.bias_time[j] = t
        return j

    def add_gps(self, t: float, x: float, y: float, sigma: float) -> None:
        key = self._add_key(t)
        sigma = max(sigma, self.args.gps_sigma_floor) * self.args.gps_sigma_scale
        cov = np.eye(2) * sigma ** 2
        bias_idx = self._add_bias(t) if self.args.gnss_bias else None
        z = np.array([x, y])
        factor = gnss_factor(X(key), None if bias_idx is None else B(bias_idx), z,
                             robust(self.args.gps_kernel, gtsam.noiseModel.Gaussian.Covariance(cov)))
        self.meas.append(Measurement("gps", t, key, z, cov, bias_idx, factor, self.trusted["gps"]))

    def add_amcl(self, t: float, pose: np.ndarray, cov3: np.ndarray) -> None:
        if not self.use_amcl or t - self.last_amcl_t < self.args.amcl_period:
            return
        self.last_amcl_t = t
        key = self._add_key(t)
        floor = np.array([self.args.amcl_sigma_floor_xy ** 2] * 2 + [self.args.amcl_sigma_floor_yaw ** 2])
        cov = cov3.copy()
        cov[np.diag_indices(3)] = np.maximum(np.diag(cov), floor)
        cov *= self.args.amcl_cov_scale
        factor = gtsam.PriorFactorPose2(X(key), gtsam.Pose2(*pose),
                                        robust(self.args.amcl_kernel, gtsam.noiseModel.Gaussian.Covariance(cov)))
        self.meas.append(Measurement("amcl", t, key, pose, cov, None, factor, self.trusted["amcl"]))

    def _graph(self) -> Tuple[gtsam.NonlinearFactorGraph, gtsam.Values]:
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        key, pose, cov = self.pose_prior
        graph.add(gtsam.PriorFactorPose2(X(key), pose, gtsam.noiseModel.Gaussian.Covariance(cov)))
        for k in self.keys:
            for name in self.motion:
                if k in self.edges[name]:
                    prev, delta, sig = self.edges[name][k]
                    noise = robust(self.args.odom_kernel,
                                   gtsam.noiseModel.Diagonal.Sigmas(sig * self.scale[name][k]), huber=True)
                    graph.add(gtsam.BetweenFactorPose2(X(prev), X(k), delta, noise))
            values.insert(X(k), self.estimate[k])
        for m in self.meas:
            if m.active:
                graph.add(m.factor)
        if self.bias_keys:
            j, mean, bcov = self.bias_prior
            graph.add(point2_prior(B(j), mean, bcov))
            for j in self.bias_keys:
                if j in self.bias_factor:
                    graph.add(self.bias_factor[j])
                values.insert(B(j), self.bias[j])
        return graph, values

    def _trim(self) -> None:
        """窓を超えた古い姿勢と、それにつながる観測・偏りを落とし、残る先頭に周辺共分散の事前分布を置く。"""
        newest = self.keys[-1]
        keep = [k for k in self.keys
                if self.key_dist[newest] - self.key_dist[k] <= self.args.window_m
                and self.key_time[newest] - self.key_time[k] <= self.args.window_sec]
        if len(keep) == len(self.keys) or len(keep) < 2:
            return
        head_time = self.key_time[keep[0]]
        keep_bias = [j for j in self.bias_keys if self.bias_time[j] >= head_time]
        graph, values = self._graph()
        marginals = gtsam.Marginals(graph, values)
        self.pose_prior = (keep[0], self.estimate[keep[0]], marginals.marginalCovariance(X(keep[0])))
        if keep_bias and keep_bias[0] != self.bias_keys[0]:
            j = keep_bias[0]
            self.bias_prior = (j, self.bias[j].copy(), marginals.marginalCovariance(B(j)))
            for d in self.bias_keys[:self.bias_keys.index(j)]:
                self.bias.pop(d)
                self.bias_factor.pop(d, None)
            self.bias_factor.pop(j, None)
            self.bias_keys = keep_bias
        dropped = set(self.keys) - set(keep)
        for k in dropped | {keep[0]}:
            self.estimate.pop(k, None) if k in dropped else None
            for name in self.motion:
                self.edges[name].pop(k, None)
                self.scale[name].pop(k, None)
        self.meas = [m for m in self.meas if m.pose_idx not in dropped]
        self.keys = keep

    def _optimize(self) -> None:
        graph, values = self._graph()
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(self.args.max_iterations)
        result = gtsam.LevenbergMarquardtOptimizer(graph, values, params).optimize()
        for k in self.keys:
            self.estimate[k] = result.atPose2(X(k))
        for j in self.bias_keys:
            self.bias[j] = np.asarray(result.atPoint2(B(j)))

    def _recent(self, source: str) -> List[Measurement]:
        since = self.key_time[self.keys[-1]] - self.args.gate_sec
        return [m for m in self.meas if m.source == source and m.t >= since]

    def _xy(self, m: Measurement, t: float, bias: Optional[np.ndarray] = None) -> np.ndarray:
        """観測 m を、時刻 t の位置に、オドメトリの動きで移したもの (GNSS は偏りを差し引く。
        bias を渡すとその値を、省略するとその観測の偏りの推定を使う)。"""
        heading = self.estimate[m.pose_idx].theta() if m.source == "gps" else m.z[2]
        odom = self.motion[self.ref]
        moved = odom.at(m.t).between(odom.at(t))
        c, s = math.cos(heading), math.sin(heading)
        xy = m.z[:2] + np.array([c * moved.x() - s * moved.y(), s * moved.x() + c * moved.y()])
        if m.source == "gps":
            if bias is not None:
                xy = xy - bias
            elif m.bias_idx is not None:
                xy = xy - self.bias[m.bias_idx]
        return xy

    def _pair_disagreement(self, gps: List[Measurement], amcl: List[Measurement],
                           bias: Optional[np.ndarray] = None) -> Optional[float]:
        """AMCL と GNSS の位置の直接の食い違い (白色化した距離の 2 乗の中央値)。"""
        if not amcl:
            return None
        at = np.array([m.t for m in amcl])
        vals = []
        for g in gps:
            a = amcl[int(np.argmin(np.abs(at - g.t)))]
            d = self._xy(a, g.t) - self._xy(g, g.t, bias)
            var = self.args.gps_sigma_floor ** 2 + np.linalg.inv(g.whiten.T @ g.whiten)[0, 0] \
                + np.linalg.inv(a.whiten.T @ a.whiten)[0, 0]
            vals.append(float(d @ d) / var)
        return float(np.median(vals)) if len(vals) >= GATE_MIN_SAMPLES else None

    def _motion_mismatch(self, ms: List[Measurement]) -> Optional[float]:
        """直近の区間で、情報源の動いた量とオドメトリの動いた量の差を、情報源の精度で割ったもの。"""
        if len(ms) < GATE_MIN_SAMPLES:
            return None
        first, last = ms[0], ms[-1]
        d = self._xy(last, last.t) - self._xy(first, last.t)
        sigma = math.sqrt(np.linalg.inv(first.whiten.T @ first.whiten)[0, 0]) * math.sqrt(2.0)
        return float(np.linalg.norm(d)) / sigma

    def _weaken(self, name: str) -> bool:
        """直近の区間の、そのオドメトリの因子を弱める。弱めたら True。"""
        since = self.key_time[self.keys[-1]] - self.args.gate_sec
        weakened = False
        for k in self.keys:
            if self.key_time[k] >= since and self.scale[name].get(k) == 1.0:
                self.scale[name][k] = self.args.odom_inflate
                weakened = True
        if weakened:
            self.odom_inflated_solves += 1
        return weakened

    def _odom_motion(self, name: str, t0: float, t1: float) -> Optional[np.ndarray]:
        odom = self.motion[name]
        if not odom.covers(t0, t1):
            return None
        d = odom.at(t0).between(odom.at(t1))
        return np.array([d.x(), d.y()])

    def _check_motion_sources(self, absolute: List[Measurement]) -> bool:
        """ホイールと LiDAR オドメトリの動きが食い違ったら、絶対の情報源と合わない方を弱め、
        合う方を基準にする。弱めたら True。"""
        if "lidar" not in self.motion or len(self.keys) < 2:
            return False
        t1 = self.key_time[self.keys[-1]]
        t0 = max(t1 - self.args.gate_sec, self.key_time[self.keys[0]])
        w, l = self._odom_motion("wheel", t0, t1), self._odom_motion("lidar", t0, t1)
        if w is None or l is None:
            self.ref = "wheel" if l is None else self.ref
            return False
        dist = max(np.linalg.norm(w), np.linalg.norm(l), 1.0)
        if np.linalg.norm(w - l) / (self.args.odom_sigma_per_m * dist) <= self.args.motion_min:
            self.ref = "wheel"
            return False
        errors = {}
        for name in ("wheel", "lidar"):
            self.ref = name
            vals = [self._motion_mismatch([m for m in absolute if m.source == s and m.active])
                    for s in ("gps", "amcl")]
            vals = [v for v in vals if v is not None]
            errors[name] = min(vals) if vals else None
        if None in errors.values():
            self.ref = "wheel"
            return False
        bad = "wheel" if errors["wheel"] > errors["lidar"] else "lidar"
        self.ref = "lidar" if bad == "wheel" else "wheel"
        if self.ref == "lidar":
            self.lidar_ref_solves += 1
        return self._weaken(bad)

    def _set_trust(self, source: str, trusted: bool) -> None:
        self.trusted[source] = trusted
        for m in self._recent(source):
            m.active = trusted

    def _gate(self) -> bool:
        """動きの合意で、外れた情報源を切り離す・戻す、またはオドメトリを弱める。切り替えたら True。

        切り離す: 直近の区間で、一方の情報源だけが動いた量でオドメトリと合わない。
        オドメトリを弱める: AMCL も GNSS もオドメトリと合わない。
        戻す: 動きがオドメトリと合い、切り離した時点の GNSS の偏りで位置も合う。
        位置の食い違いだけでは切り離さない (GNSS の偏りが、もう一方のゆっくりしたずれを吸収するため)。
        """
        gps, amcl = self._recent("gps"), self._recent("amcl")
        changed = self._check_motion_sources(gps + amcl)
        mm = {"gps": self._motion_mismatch(gps), "amcl": self._motion_mismatch(amcl)}
        if None in mm.values():
            return changed
        if all(self.trusted.values()):
            for s, o in (("amcl", "gps"), ("gps", "amcl")):
                if mm[s] > self.args.motion_min and mm[s] > self.args.motion_ratio * mm[o]:
                    self._set_trust(s, False)
                    self.frozen_bias = self.bias[self.bias_keys[-1]].copy() if self.bias_keys else None
                    return True
            if min(mm.values()) > self.args.motion_min and self._weaken(self.ref):
                changed = True
            else:
                disagree = self._pair_disagreement(gps, amcl)
                if disagree is not None and disagree > self.args.gate_out:
                    self.split_solves += 1
            return changed
        s = "amcl" if not self.trusted["amcl"] else "gps"
        disagree = self._pair_disagreement(gps, amcl, self.frozen_bias)
        if mm[s] < self.args.motion_min and disagree is not None and disagree < self.args.gate_in:
            self._set_trust(s, True)
            changed = True
        return changed

    def solve(self) -> None:
        start = time.perf_counter()
        self._trim()
        self._optimize()
        if self.args.gate and self._gate():
            self._optimize()
        for s in ("gps", "amcl"):
            if not self.trusted[s]:
                self.gated_solves[s] += 1
        self.solve_ms.append((time.perf_counter() - start) * 1e3)

    def pose_at(self, t: float) -> np.ndarray:
        """t までに置いた最新の姿勢に、その後のオドメトリの動きを足した姿勢。"""
        key = self.keys[-1]
        odom = self.motion[self.ref]
        p = self.estimate[key].compose(odom.at(self.key_time[key]).between(odom.at(t)))
        return np.array([p.x(), p.y(), p.theta()])


def first_initialized_amcl(data: Inputs, ekf_t: np.ndarray, ekf_p: np.ndarray) -> int:
    """初期姿勢を与えた後の最初の AMCL の添字。与える前の AMCL は地図の原点の付近を出すので、
    走行に使った姿勢 (EKF) と INIT_MATCH_M 以内で合う最初のものを選ぶ。"""
    for i, (t, p, _) in enumerate(data.amcl):
        j = int(np.searchsorted(ekf_t, t)) - 1
        if j >= 0 and math.hypot(p[0] - ekf_p[j, 0], p[1] - ekf_p[j, 1]) < INIT_MATCH_M:
            return i
    raise SystemExit("エラー: 走行に使った姿勢と合う AMCL の姿勢がありません。")


def run_window(args: argparse.Namespace, data: Inputs, motion: Dict[str, Odometry], use_amcl: bool,
               eval_times: np.ndarray, init_index: int) -> Tuple[np.ndarray, Dict[str, float]]:
    """観測を時刻順に入れ、solve_period ごとに解き直し、評価の時刻の姿勢と推定器の統計を返す。"""
    est = WindowEstimator(args, motion, use_amcl)
    t_init, pose0, cov0 = data.amcl[init_index]
    est.initialize(t_init, pose0, cov0)
    events = [(t, "gps", (x, y, s)) for t, x, y, s in data.gps if t > t_init]
    events += [(t, "amcl", (p, c)) for t, p, c in data.amcl if t > t_init]
    events.sort(key=lambda e: e[0])
    out = np.full((len(eval_times), 3), np.nan)
    jumps: List[float] = []
    next_solve = t_init + args.solve_period
    ei = 0
    qi = int(np.searchsorted(eval_times, t_init))
    while qi < len(eval_times):
        t_q = eval_times[qi]
        if t_q >= next_solve:
            while ei < len(events) and events[ei][0] <= next_solve:
                t, kind, payload = events[ei]
                if kind == "gps":
                    est.add_gps(t, *payload)
                else:
                    est.add_amcl(t, *payload)
                ei += 1
            before = est.pose_at(next_solve)
            est.solve()
            after = est.pose_at(next_solve)
            jumps.append(float(math.hypot(after[0] - before[0], after[1] - before[1])))
            next_solve += args.solve_period
            continue
        out[qi] = est.pose_at(t_q)
        qi += 1
    n = max(len(est.solve_ms), 1)
    stats = {
        "jump_p99": float(np.percentile(jumps, 99)) if jumps else 0.0,
        "solve_ms_p95": float(np.percentile(est.solve_ms, 95)) if est.solve_ms else 0.0,
        "split_pct": 100.0 * est.split_solves / n,
        "gate_gps_pct": 100.0 * est.gated_solves["gps"] / n,
        "gate_amcl_pct": 100.0 * est.gated_solves["amcl"] / n,
        "odom_weak_pct": 100.0 * est.odom_inflated_solves / n,
        "lidar_ref_pct": 100.0 * est.lidar_ref_solves / n,
    }
    return out, stats


def lidar_args(args: argparse.Namespace) -> argparse.Namespace:
    """LiDAR オドメトリの因子の σ を、ホイールと同じ名前で引けるようにしたもの。"""
    return argparse.Namespace(
        odom_sigma_xy=args.lidar_sigma_xy, odom_sigma_per_m=args.lidar_sigma_per_m,
        odom_sigma_yaw=args.lidar_sigma_yaw, odom_sigma_yaw_per_rad=args.lidar_sigma_yaw_per_rad,
        odom_sigma_yaw_per_m=args.lidar_sigma_yaw_per_m)


def read_lidar_odom(bags: List[str]) -> Optional[Odometry]:
    """LiDAR オドメトリ (nav_msgs/Odometry、base_footprint の姿勢) の bag を読む。"""
    rows = []
    for bag in bags:
        reader = open_reader(bag)
        deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            msg = deserializer.deserialize(topic, raw)
            p = msg.pose.pose
            rows.append((stamp_sec(msg.header.stamp), p.position.x, p.position.y,
                         quaternion_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)))
    return Odometry(sorted(rows)) if rows else None


def amcl_hold(data: Inputs, odom: Odometry, eval_times: np.ndarray) -> np.ndarray:
    times = np.array([r[0] for r in data.amcl])
    out = np.full((len(eval_times), 3), np.nan)
    idx = lm.hold_index(times, eval_times)
    for i, j in enumerate(idx):
        if j < 0:
            continue
        t, p, _ = data.amcl[j]
        q = gtsam.Pose2(*p).compose(odom.at(t).between(odom.at(eval_times[i])))
        out[i] = [q.x(), q.y(), q.theta()]
    return out


def metrics(est: np.ndarray, gt: np.ndarray, valid: np.ndarray, suffix: str) -> Dict[str, float]:
    ok = valid & ~np.isnan(est[:, 0]) & ~np.isnan(gt[:, 0])
    if not np.any(ok):
        return {}
    pos, yaw = lm.pose_errors(est[ok], gt[ok])
    step = np.r_[0.0, np.hypot(np.diff(gt[ok, 0]), np.diff(gt[ok, 1]))]
    step[step > 2.0] = 0.0
    return {
        f"p50_{suffix}": float(np.percentile(pos, 50)), f"p95_{suffix}": float(np.percentile(pos, 95)),
        f"max_{suffix}": float(pos.max()), f"dev_{suffix}": float(step[pos > DEVIATION_M].sum()),
    }


def truth_nodes(variant: str) -> np.ndarray:
    dataset = load_dataset(resolve_dataset_path(os.path.basename(os.path.dirname(variant.rstrip("/")))))
    nodes, transform, _ = load_slam_output(dataset["GT_DIR"])
    if dataset["MAP_GT_DIR"]:
        _, map_transform, _ = load_slam_output(dataset["MAP_GT_DIR"])
        nodes = shift_nodes_to_anchor(nodes, transform, map_transform)
    return nodes


def run_bags(variant: str) -> List[str]:
    return [os.path.join(r, "output") for r in sorted(glob.glob(os.path.join(variant, "run_*")))
            if os.path.isdir(os.path.join(r, "output"))]


def map_offset(reference: str) -> Tuple[np.ndarray, np.ndarray]:
    """故障のない回の AMCL と真値の差 (xy) の、ゆっくり変わる成分。時刻と (N, 2) を返す。"""
    nodes = truth_nodes(reference)
    diffs = []
    times = None
    for bag in run_bags(reference):
        data = read_inputs(bag)
        odom = Odometry(data.odom)
        ekf_t, ekf_p = read_map_base_poses(bag, "map", "odom", "base_footprint")
        t0 = data.amcl[first_initialized_amcl(data, ekf_t, ekf_p)][0]
        ts = np.arange(math.ceil(t0), data.odom[-1][0], EVAL_STEP_SEC)
        times = ts if times is None else times
        gt, valid = interpolate_nodes(nodes, times, stationary_dist=STATIONARY_DIST_M)
        d = amcl_hold(data, odom, times)[:, :2] - gt[:, :2]
        d[~valid] = np.nan
        diffs.append(d)
    d = np.nanmedian(np.stack(diffs), axis=0)
    half = int(MAP_OFFSET_WINDOW_SEC / EVAL_STEP_SEC / 2)
    smooth = np.array([np.nanmedian(d[max(0, i - half):i + half + 1], axis=0) for i in range(len(d))])
    return times, smooth


def evaluate_run(args: argparse.Namespace, bag: str, nodes: np.ndarray,
                 offset: Optional[Tuple[np.ndarray, np.ndarray]],
                 lidar: Optional[Odometry]) -> Dict[str, Dict[str, float]]:
    data = read_inputs(bag)
    odom = Odometry(data.odom)
    ekf_t, ekf_p = read_map_base_poses(bag, "map", "odom", "base_footprint")
    init_index = first_initialized_amcl(data, ekf_t, ekf_p)
    t0 = max(data.odom[0][0], data.amcl[init_index][0])
    eval_times = np.arange(t0, data.odom[-1][0], EVAL_STEP_SEC)
    gt, valid = interpolate_nodes(nodes, eval_times, stationary_dist=STATIONARY_DIST_M)
    gt_map = np.full_like(gt, np.nan)
    if offset is not None:
        ot, ov = offset
        inside = (eval_times >= ot[0]) & (eval_times <= ot[-1])
        if np.any(inside):
            gt_map[inside] = gt[inside]
            gt_map[inside, 0] += np.interp(eval_times[inside], ot, ov[:, 0])
            gt_map[inside, 1] += np.interp(eval_times[inside], ot, ov[:, 1])
    idx = lm.hold_index(ekf_t, eval_times)
    estimates = {"ekf": np.where((idx >= 0)[:, None], ekf_p[np.maximum(idx, 0)], np.nan),
                 "amcl": amcl_hold(data, odom, eval_times)}
    stats = {}
    runs = [("window_gnss", False, {"wheel": odom}), ("window_full", True, {"wheel": odom})]
    if lidar is not None:
        runs.append(("window_full_lidar", True, {"wheel": odom, "lidar": lidar}))
    for name, use_amcl, motion in runs:
        estimates[name], stats[name] = run_window(args, data, motion, use_amcl, eval_times, init_index)
    rows = {}
    for name, est in estimates.items():
        rows[name] = {**metrics(est, gt, valid, "g"), **metrics(est, gt_map, valid, "m"), **stats.get(name, {})}
    return rows


COLUMNS = ["p50_m", "p95_m", "max_m", "dev_m", "p50_g", "p95_g", "max_g", "dev_g",
           "jump_p99", "solve_ms_p95", "split_pct", "gate_gps_pct", "gate_amcl_pct", "odom_weak_pct", "lidar_ref_pct"]


def format_table(results: List[Tuple[str, Dict[str, Dict[str, float]]]]) -> str:
    lines = ["列: _m は地図の座標の真値、_g は GNSS の座標の真値。dev は誤差 1 m 超で走った距離 [m]。",
             "", "| 変種/回 | 推定 | " + " | ".join(COLUMNS) + " |", "|" + "---|" * (len(COLUMNS) + 2)]
    for name, rows in results:
        for method, m in rows.items():
            lines.append(f"| {name} | {method} | " + " | ".join(f"{m[c]:.2f}" if c in m else "" for c in COLUMNS) + " |")
    groups: Dict[str, Dict[str, List[Dict[str, float]]]] = {}
    for name, rows in results:
        for method, m in rows.items():
            groups.setdefault(name.split("/")[0], {}).setdefault(method, []).append(m)
    agg = ["p95_m", "max_m", "dev_m", "p95_g", "max_g", "dev_g"]
    lines += ["", "## 変種ごと (p95・max は回の平均、dev は回の平均)", "",
              "| 変種 | 推定 | " + " | ".join(agg) + " |", "|" + "---|" * (len(agg) + 2)]
    for variant, methods in groups.items():
        for method, ms in methods.items():
            vals = []
            for c in agg:
                xs = [m[c] for m in ms if c in m]
                vals.append(f"{np.mean(xs):.2f}" if xs else "")
            lines.append(f"| {variant} | {method} | " + " | ".join(vals) + " |")
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[1], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("variants", nargs="+", help="再生評価の変種ディレクトリ (run_N/output を含む)")
    p.add_argument("--map-reference", help="地図の座標の真値を作る、故障のない変種のディレクトリ")
    p.add_argument("--out", help="結果の Markdown の出力先 (省略で標準出力だけ)")
    p.add_argument("--window-m", type=float, default=50.0, help="窓に残す走った距離 [m]")
    p.add_argument("--window-sec", type=float, default=120.0, help="窓に残す時間 [s] (止まっている間の上限)")
    p.add_argument("--solve-period", type=float, default=1.0, help="解き直しの周期 [s]")
    p.add_argument("--max-iterations", type=int, default=20)
    p.add_argument("--odom-sigma-xy", type=float, default=0.02)
    p.add_argument("--odom-sigma-per-m", type=float, default=0.05)
    p.add_argument("--odom-sigma-yaw", type=float, default=0.005)
    p.add_argument("--odom-sigma-yaw-per-rad", type=float, default=0.05)
    p.add_argument("--odom-sigma-yaw-per-m", type=float, default=0.01)
    p.add_argument("--odom-kernel", type=float, default=2.0, help="オドメトリの Huber の幅 (白色化した単位)")
    p.add_argument("--gps-sigma-floor", type=float, default=0.1)
    p.add_argument("--gps-sigma-scale", type=float, default=1.0)
    p.add_argument("--gps-kernel", type=float, default=1.0, help="GNSS の Cauchy の幅")
    p.add_argument("--amcl-period", type=float, default=0.0, help="AMCL を入れる最短の間隔 [s] (間引き)")
    p.add_argument("--amcl-sigma-floor-xy", type=float, default=0.2)
    p.add_argument("--amcl-sigma-floor-yaw", type=float, default=0.05)
    p.add_argument("--amcl-cov-scale", type=float, default=4.0,
                   help="AMCL の共分散に掛ける倍率 (オドメトリと地図を共有する分の割り引き)")
    p.add_argument("--amcl-kernel", type=float, default=1.0, help="AMCL の Cauchy の幅")
    p.add_argument("--gate", action="store_true", help="情報源の切り離しを使う")
    p.add_argument("--gate-sec", type=float, default=10.0, help="切り離しの判断に使う直近の時間 [s]")
    p.add_argument("--gate-out", type=float, default=9.0,
                   help="AMCL と GNSS が食い違う (割れ) とみなす、白色化した距離の 2 乗の中央値")
    p.add_argument("--gate-in", type=float, default=4.0, help="切り離した情報源を戻す、同じ値")
    p.add_argument("--motion-ratio", type=float, default=2.0,
                   help="原因とみなす、動きの差 (オドメトリとの) の、もう一方に対する比")
    p.add_argument("--motion-min", type=float, default=3.0, help="原因とみなす、動きの差の下限 (精度で割った値)")
    p.add_argument("--odom-inflate", type=float, default=10.0,
                   help="AMCL と GNSS が合っていて、どちらもオドメトリと合わないとき、直近のオドメトリの σ に掛ける倍率")
    p.add_argument("--lidar-odom", nargs="*", default=[],
                   help="LiDAR オドメトリの bag (指定すると window_full_lidar も比べる)")
    p.add_argument("--lidar-sigma-xy", type=float, default=0.02)
    p.add_argument("--lidar-sigma-per-m", type=float, default=0.03)
    p.add_argument("--lidar-sigma-yaw", type=float, default=0.005)
    p.add_argument("--lidar-sigma-yaw-per-rad", type=float, default=0.03)
    p.add_argument("--lidar-sigma-yaw-per-m", type=float, default=0.005)
    p.add_argument("--gnss-bias", action="store_true", help="GNSS の偏りを推定する")
    p.add_argument("--gnss-bias-sigma0", type=float, default=1.0, help="偏りの最初の不確かさ [m]")
    p.add_argument("--gnss-bias-rw", type=float, default=0.01, help="偏りの変わる速さ [m/√s]")
    return p


def main() -> None:
    args = build_parser().parse_args()
    offset = map_offset(args.map_reference) if args.map_reference else None
    lidar = read_lidar_odom(args.lidar_odom)
    results = []
    for variant in args.variants:
        nodes = truth_nodes(variant)
        for bag in run_bags(variant):
            name = f"{os.path.basename(variant.rstrip('/'))}/{os.path.basename(os.path.dirname(bag))}"
            rows = evaluate_run(args, bag, nodes, offset, lidar)
            results.append((name, rows))
            print(name, " ".join(f"{m}:{r.get('p95_m', float('nan')):.2f}/{r.get('dev_m', float('nan')):.0f}"
                                 f"|{r['p95_g']:.2f}/{r['dev_g']:.0f}" for m, r in rows.items()), flush=True)
    text = "# 窓つきの推定器の試作と EKF の比較\n\n" + format_table(results)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
