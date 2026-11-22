#!/usr/bin/env python3
"""
eval_gate_arbitration.py

AMCL の門 (AMCL の姿勢を走らせる推定器へ渡すかを決める中継) と、地図との照合 (L0) による判定を、
再生評価 (run_localization_variant.sh) の記録の入力でオフラインに回し、走らせる推定器の候補と見張り役の
方式を真値との誤差で比べる。各時刻で、それまでに届いた観測だけを使う。

門の規則:
  1. AMCL の姿勢が、走らせる推定器の今の姿勢 (前に受け入れた AMCL からのオドメトリの推測) から
     --jump-m / --jump-yaw を超えて離れたら、門を閉じて L0 を求める。
  2. L0 の候補は、走らせる推定器の姿勢 (飛ぶ前からの続き)、AMCL、AMCL の生の値、見張り役の姿勢、GNSS。
     今のスキャン (動きで重ねた直近のもの) を各候補の周りで照合し、一致率が最も高い候補を選ぶ。
     最良が --l0-min-score 未満、または離れた次点との差が --l0-margin 未満なら決めない。
  3. 門を開けるのは L0 が決めたときだけ。決めた姿勢へ、走らせる推定器・見張り役・AMCL を置き直す
     (AMCL の置き直しは、記録の AMCL に座標の補正を掛けて模擬する)。
  4. 起動の直後 (L0 が一度確かめるまで) は門の判定を使わず、L0 で AMCL を確かめる。

比べるもの:
  ekf_rec       : 再生で走行に使った EKF (今の構成)
  ekf_emu       : 記録の入力で回した EKF の模擬 (門なし)。ekf_rec との差で模擬の確かさを見る
  ekf_gate      : EKF の模擬 + 門 + L0 (見張り役は門の判定だけ = 簡単な方式)
  ekf_gate_watch: 上に、窓つきの推定器の見張り役を足したもの。EKF と食い違いが続いたら L0 を求める
  win_gate      : 窓つきの推定器で走り、門 + L0。推定器の中の AMCL の切り離しも L0 に渡し、L0 でだけ戻す

例:
  python3 tools/scripts/eval_gate_arbitration.py \\
      --map-reference .../eval_loc/map043837_eval051635_short/baseline \\
      --lidar-odom /path/to/lidar_odom_bag \\
      .../eval_loc/map043837_eval051635_short/base_none_kidnap --out tools/data/gate.md
"""

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import gtsam
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for path in (REPO_ROOT, os.path.join(REPO_ROOT, "localization_supervisor", "localization_supervisor")):
    if path not in sys.path:
        sys.path.insert(0, path)

from localization_supervisor.core.grid import build_local_distance_field  # noqa: E402
from localization_supervisor.core.interfaces import SearchWindow  # noqa: E402
from localization_supervisor.core.matcher.correlative import CorrelativeMatcher  # noqa: E402
from localization_supervisor.ros.map_io import load_map_yaml  # noqa: E402
from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader, read_map_base_poses  # noqa: E402
from tools.common.geo import quaternion_to_yaw  # noqa: E402
from tools.common.pose_graph import interpolate_nodes  # noqa: E402
from tools.scripts import eval_window_estimator as w  # noqa: E402
from tools.scripts.loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

SCAN_TOPIC = "/scan_top_lidar"
BASE_FRAME = "base_footprint"
# 窓つきの推定器の設定 (段階 P の p0)
WINDOW_CONFIG = ["x", "--amcl-period", "1", "--gate", "--gnss-bias", "--odom-sigma-yaw-per-rad", "0.2",
                 "--odom-sigma-per-m", "0.1", "--gnss-bias-rw", "0.03", "--gate-sec", "30", "--motion-min", "2"]
# 置き直した姿勢の共分散 (x, y, yaw の σ)
RESET_SIGMA = (0.1, 0.1, 0.05)
# L0 の候補に GNSS を入れる、最新の GNSS の古さの上限 [s]
GNSS_FRESH_SEC = 2.0


def pose2(p) -> gtsam.Pose2:
    return gtsam.Pose2(float(p[0]), float(p[1]), float(p[2]))


def arr(p: gtsam.Pose2) -> np.ndarray:
    return np.array([p.x(), p.y(), p.theta()])


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- スキャンと地図

class Scans:
    """元の走行の bag のスキャン (base 座標の点に直したもの) と、base->LiDAR の静的 TF。"""

    def __init__(self, bag: str, t0: float, t1: float, points_per_scan: int) -> None:
        reader = open_reader(bag, topics=[SCAN_TOPIC, "/tf_static"])
        deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
        static: Dict[str, Tuple[str, np.ndarray]] = {}
        self.t: List[float] = []
        self.points: List[np.ndarray] = []
        raw: List[Tuple[float, str, np.ndarray, float, float, float, float]] = []
        while reader.has_next():
            topic, data, received_ns = reader.read_next()
            if topic == SCAN_TOPIC and not t0 - 5.0 <= received_ns * 1e-9 <= t1 + 5.0:
                continue
            msg = deserializer.deserialize(topic, data)
            if topic == "/tf_static":
                for tf in msg.transforms:
                    q, p = tf.transform.rotation, tf.transform.translation
                    static[tf.child_frame_id] = (tf.header.frame_id, np.array(
                        [p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)]))
                continue
            t = w.stamp_sec(msg.header.stamp)
            if t < t0 or t > t1:
                continue
            raw.append((t, msg.header.frame_id, np.asarray(msg.ranges, dtype=float), msg.angle_min,
                        msg.angle_increment, msg.range_min, msg.range_max))
        for t, frame, ranges, a0, da, rmin, rmax in raw:
            base_lidar = self._chain(static, frame)
            angles = a0 + da * np.arange(ranges.size)
            ok = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
            r, a = ranges[ok], angles[ok]
            if r.size > points_per_scan:
                keep = np.linspace(0, r.size - 1, points_per_scan).astype(int)
                r, a = r[keep], a[keep]
            c, s = math.cos(base_lidar[2]), math.sin(base_lidar[2])
            px, py = r * np.cos(a), r * np.sin(a)
            self.t.append(t)
            self.points.append(np.stack([base_lidar[0] + c * px - s * py, base_lidar[1] + s * px + c * py], axis=1))
        if not self.t:
            raise SystemExit(f"エラー: {bag} の {SCAN_TOPIC} に評価の時間のスキャンがありません。")
        self.t_arr = np.array(self.t)

    @staticmethod
    def _chain(static: Dict[str, Tuple[str, np.ndarray]], frame: str) -> np.ndarray:
        pose = gtsam.Pose2()
        while frame != BASE_FRAME:
            if frame not in static:
                raise SystemExit(f"エラー: /tf_static に {frame} から {BASE_FRAME} へのつながりがありません。")
            parent, p = static[frame]
            pose = pose2(p).compose(pose)
            frame = parent
        return arr(pose)

    def stacked(self, t: float, odom: w.Odometry, min_distance: float, max_scans: int,
                max_points: int) -> Optional[np.ndarray]:
        """t 以前のスキャンを、オドメトリの動きで t の base 座標へ重ねた点 (N, 2)。"""
        j = int(np.searchsorted(self.t_arr, t, side="right")) - 1
        if j < 0:
            return None
        now = odom.at(t)
        chunks = []
        last = None
        travelled = 0.0
        for k in range(j, max(j - max_scans * 10, -1), -1):
            then = odom.at(self.t[k])
            if last is not None:
                step = math.hypot(then.x() - last.x(), then.y() - last.y())
                if step < 0.05 and len(chunks) > 0:
                    continue
                travelled += step
            last = then
            rel = now.between(then)
            c, s = math.cos(rel.theta()), math.sin(rel.theta())
            p = self.points[k]
            chunks.append(np.stack([rel.x() + c * p[:, 0] - s * p[:, 1], rel.y() + s * p[:, 0] + c * p[:, 1]], axis=1))
            if travelled >= min_distance or len(chunks) >= max_scans:
                break
        pts = np.concatenate(chunks, axis=0)
        if pts.shape[0] > max_points:
            pts = pts[np.linspace(0, pts.shape[0] - 1, max_points).astype(int)]
        return pts


class L0:
    """今のスキャンで候補を地図に照合し、1 つに決める (決まらなければ None)。"""

    def __init__(self, args: argparse.Namespace, map_yaml: str, scans: Scans, odom: w.Odometry) -> None:
        self.args = args
        self.occupancy = load_map_yaml(map_yaml)
        self.scans = scans
        self.odom = odom
        params = {k: spec.default for k, spec in CorrelativeMatcher.PARAMS.items()}
        params["tolerance_m"] = args.l0_tolerance
        self.matcher = CorrelativeMatcher("l0", params)
        self.window = SearchWindow(args.l0_range_xy, args.l0_step_xy, args.l0_range_yaw, args.l0_step_yaw)
        self.calls = 0
        self.decided = 0
        self.log: List[Dict] = []

    def decide(self, t: float, candidates: Dict[str, np.ndarray], reason: str) -> Optional[Tuple[str, np.ndarray]]:
        self.calls += 1
        pts = self.scans.stacked(t, self.odom, self.args.l0_stack_m, self.args.l0_stack_scans, self.args.l0_points)
        scored = []
        if pts is not None:
            for name, pose in candidates.items():
                field = build_local_distance_field(self.occupancy, pose[0], pose[1], self.args.l0_field_radius)
                if field is None:
                    continue
                peaks = self.matcher.match(field, pts, (float(pose[0]), float(pose[1]), float(pose[2])), self.window)
                if peaks:
                    scored.append((peaks[0].ratio, name, np.array(peaks[0].pose)))
        scored.sort(key=lambda s: -s[0])
        result = None
        margin = None
        if scored and scored[0][0] >= self.args.l0_min_score:
            best = scored[0]
            # 位置か向きが離れた候補は、別の仮説として次点に数える
            rival = next((s for s in scored[1:] if math.hypot(*(s[2][:2] - best[2][:2])) >= self.args.l0_separation
                          or abs(wrap(s[2][2] - best[2][2])) >= self.args.l0_separation_yaw), None)
            margin = best[0] - rival[0] if rival is not None else None
            if rival is None or margin >= self.args.l0_margin:
                result = (best[1], best[2])
                self.decided += 1
        self.log.append({
            "t": t, "reason": reason, "result": result[0] if result else None, "margin": margin,
            "scores": {s[1]: round(s[0], 3) for s in scored},
            "poses": {s[1]: [round(float(v), 3) for v in s[2]] for s in scored},
            "inputs": {k: [round(float(v), 3) for v in p] for k, p in candidates.items()}})
        return result


# ---------------------------------------------------------------- 走らせる推定器

class EkfEmu:
    """今の EKF (robot_localization、ekf_global.yaml) の模擬。状態は地図座標の (x, y, yaw)。

    予測はホイールのオドメトリの動き、更新は AMCL の姿勢 (棄却なし) と GNSS の位置 (マハラノビス距離で棄却)。
    """

    def __init__(self, args: argparse.Namespace, odom: w.Odometry, t: float, pose: np.ndarray,
                 cov: np.ndarray) -> None:
        self.args = args
        self.odom = odom
        self.reset(t, pose, cov)

    def reset(self, t: float, pose: np.ndarray, cov: np.ndarray) -> None:
        self.t = t
        self.x = np.array(pose, dtype=float)
        self.P = np.array(cov, dtype=float)

    def _predict(self, t: float) -> None:
        if t <= self.t:
            return
        rel = self.odom.at(self.t).between(self.odom.at(t))
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        F = np.array([[1, 0, -s * rel.x() - c * rel.y()], [0, 1, c * rel.x() - s * rel.y()], [0, 0, 1]])
        self.x = np.array([self.x[0] + c * rel.x() - s * rel.y(), self.x[1] + s * rel.x() + c * rel.y(),
                           wrap(self.x[2] + rel.theta())])
        dt = t - self.t
        q = self.args.ekf_q_scale * dt * np.diag([0.05, 0.05, 0.06])
        self.P = F @ self.P @ F.T + q
        self.t = t

    def add_amcl(self, t: float, pose: np.ndarray, cov: np.ndarray) -> None:
        self._predict(t)
        nu = np.array(pose, dtype=float) - self.x
        nu[2] = wrap(nu[2])
        self._update(nu, np.eye(3), cov)

    def add_gps(self, t: float, x: float, y: float, sigma: float) -> None:
        self._predict(t)
        H = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        nu = np.array([x, y]) - self.x[:2]
        R = np.eye(2) * max(sigma, 1e-3) ** 2
        S = H @ self.P @ H.T + R
        if float(nu @ np.linalg.solve(S, nu)) > self.args.ekf_gps_rejection ** 2:
            return
        self._update(nu, H, R)

    def _update(self, nu: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    def step(self, t: float) -> None:
        self._predict(t)

    def pose(self, t: float) -> np.ndarray:
        rel = self.odom.at(self.t).between(self.odom.at(max(t, self.t)))
        return arr(pose2(self.x).compose(rel))


class ArbitratedWindow(w.WindowEstimator):
    """窓つきの推定器。中の動きの合意で AMCL を切り離したら L0 を求め、戻すのは L0 (置き直し) だけにする。"""

    def __init__(self, args, motion, use_amcl) -> None:
        super().__init__(args, motion, use_amcl)
        self.allow_amcl_cut = False
        self.suspect = False

    def _set_trust(self, source: str, trusted: bool) -> None:
        if source == "amcl":
            if trusted:
                return
            if not self.allow_amcl_cut:
                return
            self.suspect = True
        super()._set_trust(source, trusted)


class WindowDriver:
    """窓つきの推定器を、走らせる推定器・見張り役として使う包み。置き直しは作り直しで行う。"""

    def __init__(self, args: argparse.Namespace, motion: Dict[str, w.Odometry], t: float, pose: np.ndarray,
                 cov: np.ndarray) -> None:
        self.args = args
        self.motion = motion
        self.est: Optional[ArbitratedWindow] = None
        self.reset(t, pose, cov)

    def reset(self, t: float, pose: np.ndarray, cov: np.ndarray) -> None:
        allow = self.est.allow_amcl_cut if self.est is not None else False
        self.est = ArbitratedWindow(self.args, self.motion, True)
        self.est.initialize(t, np.array(pose, dtype=float), np.array(cov, dtype=float))
        self.est.allow_amcl_cut = allow
        self.next_solve = t + self.args.solve_period

    def add_amcl(self, t: float, pose: np.ndarray, cov: np.ndarray) -> None:
        if t > self.est.key_time[self.est.keys[-1]]:
            self.est.add_amcl(t, pose, cov)

    def add_gps(self, t: float, x: float, y: float, sigma: float) -> None:
        if t > self.est.key_time[self.est.keys[-1]]:
            self.est.add_gps(t, x, y, sigma)

    def step(self, t: float) -> None:
        while t >= self.next_solve:
            self.est.solve()
            self.next_solve += self.args.solve_period

    def pose(self, t: float) -> np.ndarray:
        return self.est.pose_at(t)


# ---------------------------------------------------------------- 門と判定

class System:
    """走らせる推定器 + 門 + L0 (+ 見張り役) を時刻順に回す。"""

    def __init__(self, args: argparse.Namespace, data: w.Inputs, odom: w.Odometry, l0: Optional[L0],
                 driver, watch: Optional[WindowDriver], use_gate: bool) -> None:
        self.args = args
        self.data = data
        self.odom = odom
        self.l0 = l0
        self.driver = driver
        self.watch = watch
        self.use_gate = use_gate
        self.correction = gtsam.Pose2()
        self.gate_open = True
        self.confirmed = not use_gate
        self.pending: Optional[str] = None
        self.next_l0 = -1e9
        self.disagree_since: Optional[float] = None
        self.quiet_until = -1e9
        self.last_amcl: Optional[Tuple[float, np.ndarray]] = None
        self.last_gps: Optional[Tuple[float, float, float]] = None
        # 門を通した直近の AMCL (時刻, 姿勢)。動きの合意で、直近の区間の動きをオドメトリと比べる
        self.accepted: List[Tuple[float, np.ndarray]] = []
        self.last_amcl_t = -1e9
        self.closures = 0

    def _amcl_at(self, t: float, corrected: bool) -> Optional[np.ndarray]:
        if self.last_amcl is None:
            return None
        ta, raw = self.last_amcl
        p = pose2(raw)
        if corrected:
            p = self.correction.compose(p)
        return arr(p.compose(self.odom.at(ta).between(self.odom.at(t))))

    def on_amcl(self, t: float, raw: np.ndarray, cov: np.ndarray) -> None:
        self.last_amcl = (t, raw)
        pose = arr(self.correction.compose(pose2(raw)))
        if self.watch is not None:
            self.watch.add_amcl(t, pose, cov)
        if not self.use_gate:
            self.driver.add_amcl(t, pose, cov)
            return
        if not self.confirmed:
            self.driver.add_amcl(t, pose, cov)
            if self.pending is None:
                self.pending = "startup"
            return
        if not self.gate_open:
            return
        cont = self.driver.pose(t)
        if not self._apart(raw, pose, self.args.amcl_basin):
            # 置き直した AMCL は、収束の範囲にあるスキャンに合う姿勢 (記録の AMCL) へ数秒で寄る
            keep = 1.0 - min(1.0, (t - self.last_amcl_t) / self.args.amcl_converge_sec)
            local = pose2(raw).between(pose2(pose))
            moved = pose2(raw).compose(gtsam.Pose2(local.x() * keep, local.y() * keep, local.theta() * keep))
            self.correction = moved.compose(pose2(raw).inverse())
            pose = arr(moved)
        self.last_amcl_t = t
        while self.accepted and t - self.accepted[0][0] > self.args.gate_window_sec:
            self.accepted.pop(0)
        refs = [cont]
        if self.accepted:
            ta, pa = self.accepted[0]
            refs.append(arr(pose2(pa).compose(self.odom.at(ta).between(self.odom.at(t)))))
        if any(self._apart(pose, ref, 1.0) for ref in refs):
            self.gate_open = False
            self.closures += 1
            self.pending = "gate"
            self.accepted = []
            self.next_l0 = min(self.next_l0, t)
            return
        self.accepted.append((t, pose))
        self.driver.add_amcl(t, pose, cov)

    def _apart(self, a: np.ndarray, b: np.ndarray, scale: float) -> bool:
        """位置か向きが、門の閾値 (の scale 倍) より離れているか。"""
        return (math.hypot(a[0] - b[0], a[1] - b[1]) > scale * self.args.jump_m
                or abs(wrap(a[2] - b[2])) > scale * self.args.jump_yaw)

    def _separated(self, a: np.ndarray, b: np.ndarray) -> bool:
        """L0 で別の仮説とみなすほど離れているか。"""
        return (math.hypot(a[0] - b[0], a[1] - b[1]) >= self.args.l0_separation
                or abs(wrap(a[2] - b[2])) >= self.args.l0_separation_yaw)

    def on_gps(self, t: float, x: float, y: float, sigma: float) -> None:
        self.last_gps = (t, x, y)
        self.driver.add_gps(t, x, y, sigma)
        if self.watch is not None:
            self.watch.add_gps(t, x, y, sigma)

    def on_tick(self, t: float) -> None:
        self.driver.step(t)
        if self.watch is not None:
            self.watch.step(t)
        if not self.use_gate:
            return
        if isinstance(self.driver, WindowDriver) and self.driver.est.suspect and self.pending is None:
            self.pending = "window_cut"
        if self.watch is not None and self.confirmed and self.pending is None and t >= self.quiet_until:
            if self.watch.est.suspect:
                self.pending = "watch_cut"
            else:
                d = np.hypot(*(self.watch.pose(t)[:2] - self.driver.pose(t)[:2]))
                if d > self.args.disagree_m:
                    self.disagree_since = t if self.disagree_since is None else self.disagree_since
                    if t - self.disagree_since >= self.args.disagree_sec:
                        self.pending = "disagree"
                else:
                    self.disagree_since = None
        if self.pending is not None and t >= self.next_l0:
            self._run_l0(t)

    def _run_l0(self, t: float) -> None:
        cands: Dict[str, np.ndarray] = {"driver": self.driver.pose(t)}
        a = self._amcl_at(t, True)
        if a is not None:
            cands["amcl"] = a
            raw = self._amcl_at(t, False)
            if np.hypot(*(raw[:2] - a[:2])) > 0.1:
                cands["amcl_raw"] = raw
        if self.watch is not None:
            cands["watch"] = self.watch.pose(t)
        if self.last_gps is not None and t - self.last_gps[0] < GNSS_FRESH_SEC:
            cands["gnss"] = np.array([self.last_gps[1], self.last_gps[2], cands["driver"][2]])
        if self.pending == "startup":
            cands = {k: v for k, v in cands.items() if k in ("amcl", "driver")}
        result = self.l0.decide(t, cands, self.pending)
        if result is None:
            self.next_l0 = t + self.args.l0_retry_sec
            return
        name, pose = result
        cov = np.diag(np.square(RESET_SIGMA))
        # L0 は離れた仮説を見分けるもので、細かい位置合わせには使わない。決めた姿勢が AMCL の候補と
        # 同じ仮説 (l0_separation 以内) なら、照合の姿勢ではなく AMCL の姿勢を採る
        for key in ("amcl_raw", "amcl"):
            if key in cands and not self._separated(cands[key], pose):
                name, pose = key, cands[key]
                break
        # 起動の直後は、AMCL と同じ仮説なら確かめるだけ (AMCL の収束を待つ)。それ以外は、決めた姿勢へ全部を置き直す
        if self.pending != "startup" or name not in ("amcl", "amcl_raw"):
            if a is not None:
                # AMCL の置き直しの模擬。AMCL を採ったなら補正はそのまま (生の値なら外す)。別の仮説に
                # 決めたなら、記録の AMCL がその姿勢に来るよう補正を掛ける
                if name == "amcl_raw":
                    self.correction = gtsam.Pose2()
                elif name != "amcl":
                    raw_np = self._amcl_at(t, False)
                    self.correction = pose2(pose).compose(pose2(raw_np).inverse())
            self.driver.reset(t, pose, cov)
            if self.watch is not None:
                self.watch.reset(t, pose, cov)
        self.confirmed = True
        self.gate_open = True
        self.accepted = []
        self.pending = None
        self.disagree_since = None
        self.quiet_until = t + self.args.disagree_quiet_sec
        if isinstance(self.driver, WindowDriver):
            self.driver.est.allow_amcl_cut = True
            self.driver.est.suspect = False
        if self.watch is not None:
            self.watch.est.allow_amcl_cut = True
            self.watch.est.suspect = False


def run_system(args, data, odom, eval_times, init_index, l0, kind: str, motion) -> Tuple[np.ndarray, Dict]:
    t_init, pose0, cov0 = data.amcl[init_index]
    driver = EkfEmu(args, odom, t_init, pose0, cov0) if kind.startswith("ekf") else WindowDriver(
        args.window, motion, t_init, pose0, cov0)
    watch = WindowDriver(args.window, motion, t_init, pose0, cov0) if kind == "ekf_gate_watch" else None
    sys_ = System(args, data, odom, l0, driver, watch, kind != "ekf_emu")
    events = [(t, "gps", (x, y, s)) for t, x, y, s in data.gps if t > t_init]
    events += [(t, "amcl", (p, c)) for t, p, c in data.amcl if t > t_init]
    events.sort(key=lambda e: e[0])
    out = np.full((len(eval_times), 3), np.nan)
    gate = np.zeros(len(eval_times), dtype=bool)
    ei = 0
    calls0 = l0.calls if l0 else 0
    decided0 = l0.decided if l0 else 0
    log0 = len(l0.log) if l0 else 0
    for qi, t in enumerate(eval_times):
        if t < t_init:
            continue
        while ei < len(events) and events[ei][0] <= t:
            te, k, payload = events[ei]
            (sys_.on_gps if k == "gps" else sys_.on_amcl)(te, *payload)
            ei += 1
        sys_.on_tick(t)
        out[qi] = driver.pose(t)
        gate[qi] = (not sys_.gate_open) or (sys_.pending is not None and sys_.pending != "startup")
    stats = {"closures": sys_.closures}
    if l0 is not None:
        stats["l0_calls"] = l0.calls - calls0
        stats["l0_decided"] = l0.decided - decided0
        stats["l0_log"] = l0.log[log0:]
    return out, {**stats, "_gate": gate}


# ---------------------------------------------------------------- 評価

def output_jumps(est: np.ndarray, odom: w.Odometry, times: np.ndarray) -> np.ndarray:
    """評価の時刻の間で、出力の動きがオドメトリの動きから外れた量 [m]。"""
    out = []
    for i in range(1, len(times)):
        if np.isnan(est[i - 1, 0]) or np.isnan(est[i, 0]):
            continue
        pred = pose2(est[i - 1]).compose(odom.at(times[i - 1]).between(odom.at(times[i])))
        out.append(math.hypot(est[i, 0] - pred.x(), est[i, 1] - pred.y()))
    return np.array(out) if out else np.zeros(1)


def evaluate(args: argparse.Namespace, variant: str, bag: str, offset, lidar, scans_cache) -> Dict:
    data = w.read_inputs(bag)
    odom = w.Odometry(data.odom)
    ekf_t, ekf_p = read_map_base_poses(bag, "map", "odom", "base_footprint")
    i0 = w.first_initialized_amcl(data, ekf_t, ekf_p)
    t0 = max(data.odom[0][0], data.amcl[i0][0])
    times = np.arange(t0, data.odom[-1][0], w.EVAL_STEP_SEC)
    gt, valid = interpolate_nodes(w.truth_nodes(variant), times, stationary_dist=w.STATIONARY_DIST_M)
    gt_map = np.full_like(gt, np.nan)
    if offset is not None:
        ot, ov = offset
        inside = (times >= ot[0]) & (times <= ot[-1])
        gt_map[inside] = gt[inside]
        gt_map[inside, 0] += np.interp(times[inside], ot, ov[:, 0])
        gt_map[inside, 1] += np.interp(times[inside], ot, ov[:, 1])
    dataset = load_dataset(resolve_dataset_path(os.path.basename(os.path.dirname(variant.rstrip("/")))))
    src_bag = dataset["BAG"]
    cache_key = (src_bag, round(t0), round(times[-1]))
    if cache_key not in scans_cache:
        scans_cache.clear()
        scans_cache[cache_key] = Scans(src_bag, t0 - 30.0, times[-1] + 1.0, args.l0_points_per_scan)
    scans = scans_cache[cache_key]
    map_yaml = dataset["MAP_YAML"]
    motion = {"wheel": odom}
    if lidar is not None:
        motion["lidar"] = lidar
    idx = lm.hold_index(ekf_t, times)
    estimates = {"ekf_rec": np.where((idx >= 0)[:, None], ekf_p[np.maximum(idx, 0)], np.nan)}
    extra: Dict[str, Dict] = {}
    amcl = w.amcl_hold(data, odom, times)
    for kind in args.methods:
        l0 = None if kind == "ekf_emu" else L0(args, map_yaml, scans, odom)
        estimates[kind], extra[kind] = run_system(args, data, odom, times, i0, l0, kind, motion)
    rows = {}
    for name, est in estimates.items():
        r = {**w.metrics(est, gt, valid, "g"), **w.metrics(est, gt_map, valid, "m")}
        j = output_jumps(est, odom, times)
        r["jump_p99"] = float(np.percentile(j, 99))
        r["jump_max"] = float(j.max())
        st = extra.get(name, {})
        if "l0_calls" in st:
            r["l0_calls"] = st["l0_calls"]
            r["l0_decided_pct"] = 100.0 * st["l0_decided"] / max(st["l0_calls"], 1)
            r["closures"] = st["closures"]
            g = st["_gate"]
            ref = gt_map if offset is not None and not np.all(np.isnan(gt_map[:, 0])) else gt
            ok = valid & ~np.isnan(ref[:, 0])
            e_drv = np.full(len(times), np.nan)
            e_amcl = np.full(len(times), np.nan)
            e_drv[ok] = lm.pose_errors(est[ok], ref[ok])[0]
            e_amcl[ok] = lm.pose_errors(amcl[ok], ref[ok])[0]
            # 門を閉じている (判定待ちの) 間に、AMCL の方が走らせる推定器より明らかに真値に近かった時間
            lock = g & ok & (e_amcl + 0.5 < e_drv)
            r["lockout_s"] = float(lock.sum() * w.EVAL_STEP_SEC)
            r["pending_s"] = float(g.sum() * w.EVAL_STEP_SEC)
            r["l0_log"] = st["l0_log"]
        rows[name] = r
    ref = gt_map if offset is not None and not np.all(np.isnan(gt_map[:, 0])) else gt
    series = {"t": (times - times[0]).tolist(), "valid": (valid & ~np.isnan(ref[:, 0])).tolist()}
    ok = valid & ~np.isnan(ref[:, 0])
    for name, est in list(estimates.items()) + [("amcl", amcl)]:
        e = np.full(len(times), np.nan)
        e[ok] = lm.pose_errors(est[ok], ref[ok])[0]
        series[name] = [None if np.isnan(x) else round(float(x), 3) for x in e]
    for name, st in extra.items():
        if "_gate" in st:
            series[f"gate_{name}"] = st["_gate"].tolist()
    return {"rows": rows, "series": series, "t0": float(times[0])}


COLUMNS = ["p95_m", "max_m", "dev_m", "p95_g", "dev_g", "jump_p99", "jump_max", "closures", "l0_calls",
           "l0_decided_pct", "pending_s", "lockout_s"]


def format_table(results: List[Tuple[str, Dict]]) -> str:
    lines = ["列: _m は地図の座標の真値、_g は GNSS の座標の真値。dev は誤差 1 m 超で走った距離 [m]。",
             "jump は出力の動きがオドメトリから外れた量 [m] (0.2 s ごと)。closures は門を閉じた回数。",
             "pending_s は門を閉じている・判定待ちの時間、lockout_s はその間に AMCL の方が 0.5 m 以上真値に近かった時間。",
             "", "| 変種/回 | 推定 | " + " | ".join(COLUMNS) + " |", "|" + "---|" * (len(COLUMNS) + 2)]
    for name, res in results:
        for method, m in res["rows"].items():
            lines.append(f"| {name} | {method} | " + " | ".join(
                f"{m[c]:.2f}" if isinstance(m.get(c), float) else (str(m[c]) if c in m else "") for c in COLUMNS) + " |")
    groups: Dict[str, Dict[str, List[Dict]]] = {}
    for name, res in results:
        for method, m in res["rows"].items():
            groups.setdefault(name.split("/")[0], {}).setdefault(method, []).append(m)
    agg = ["p95_m", "max_m", "dev_m", "p95_g", "dev_g", "jump_max", "l0_calls", "l0_decided_pct", "pending_s",
           "lockout_s"]
    lines += ["", "## 変種ごと (回の平均)", "", "| 変種 | 推定 | " + " | ".join(agg) + " |",
              "|" + "---|" * (len(agg) + 2)]
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
    p.add_argument("--lidar-odom", nargs="*", default=[], help="LiDAR オドメトリの bag (窓つきの推定器に入れる)")
    p.add_argument("--methods", nargs="+", default=["ekf_emu", "ekf_gate", "ekf_gate_watch", "win_gate"])
    p.add_argument("--out", help="結果の Markdown の出力先")
    p.add_argument("--json", help="時系列と L0 の記録の JSON の出力先")
    p.add_argument("--jump-m", type=float, default=1.0, help="門を閉じる、AMCL と推測の位置の差 [m]")
    p.add_argument("--jump-yaw", type=float, default=0.3, help="門を閉じる、AMCL と推測の向きの差 [rad]")
    p.add_argument("--gate-window-sec", type=float, default=10.0,
                   help="動きの合意で、AMCL の動きをオドメトリと比べる直近の区間 [s]")
    p.add_argument("--amcl-basin", type=float, default=0.8,
                   help="置き直した AMCL が記録の AMCL へ収束するとみなす範囲 (門の閾値に対する倍率)")
    p.add_argument("--amcl-converge-sec", type=float, default=3.0, help="その収束にかかる時間 [s]")
    p.add_argument("--disagree-m", type=float, default=1.0, help="見張り役との食い違いで L0 を求める距離 [m]")
    p.add_argument("--disagree-sec", type=float, default=2.0, help="その食い違いが続く時間 [s]")
    p.add_argument("--disagree-quiet-sec", type=float, default=5.0, help="L0 が決めた後、食い違いで求めない時間 [s]")
    p.add_argument("--l0-min-score", type=float, default=0.35)
    p.add_argument("--l0-margin", type=float, default=0.1)
    p.add_argument("--l0-separation", type=float, default=1.0)
    p.add_argument("--l0-separation-yaw", type=float, default=0.2)
    p.add_argument("--l0-tolerance", type=float, default=0.3)
    p.add_argument("--l0-range-xy", type=float, default=0.5)
    p.add_argument("--l0-step-xy", type=float, default=0.1)
    p.add_argument("--l0-range-yaw", type=float, default=0.15)
    p.add_argument("--l0-step-yaw", type=float, default=0.05)
    p.add_argument("--l0-stack-m", type=float, default=3.0, help="重ねるスキャンの走った距離 [m]")
    p.add_argument("--l0-stack-scans", type=int, default=10)
    p.add_argument("--l0-points", type=int, default=400)
    p.add_argument("--l0-points-per-scan", type=int, default=180)
    p.add_argument("--l0-field-radius", type=float, default=25.0)
    p.add_argument("--l0-retry-sec", type=float, default=2.0)
    p.add_argument("--ekf-q-scale", type=float, default=1.0, help="EKF の模擬の過程雑音の倍率")
    p.add_argument("--ekf-gps-rejection", type=float, default=5.0, help="EKF の模擬の GNSS の棄却 (マハラノビス距離)")
    return p


def main() -> None:
    args = build_parser().parse_args()
    args.window = w.build_parser().parse_args(WINDOW_CONFIG)
    offset = w.map_offset(args.map_reference) if args.map_reference else None
    lidar = w.read_lidar_odom(args.lidar_odom)
    results = []
    scans_cache: Dict = {}
    for variant in args.variants:
        for bag in w.run_bags(variant):
            name = f"{os.path.basename(variant.rstrip('/'))}/{os.path.basename(os.path.dirname(bag))}"
            res = evaluate(args, variant, bag, offset, lidar, scans_cache)
            results.append((name, res))
            print(name, " ".join(f"{m}:{r.get('p95_m', float('nan')):.2f}/{r.get('dev_m', float('nan')):.0f}"
                                 f"|{r.get('p95_g', float('nan')):.2f}" for m, r in res["rows"].items()), flush=True)
    text = "# AMCL の門と L0 による判定の比較\n\n" + format_table(results)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump([{"name": n, "t0": r["t0"], "series": r["series"], "l0": {m: r["rows"][m].get("l0_log", []) for m in r["rows"]}}
                       for n, r in results], f)
    print(text)


if __name__ == "__main__":
    main()
