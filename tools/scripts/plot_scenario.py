#!/usr/bin/env python3
"""
plot_scenario.py

シナリオテスト (sim_scenario_test) の結果を図にする。

  run     <結果>...           1 回分の結果 (result.json と bag/ のあるディレクトリ) ごとに、地図の上の軌跡
                              (真値・EKF・AMCL・GNSS) と、時系列 (真値との差、GNSS の申告の精度、速さ、速度の指令の
                              回転、自己位置監視の状態) を <結果>/scenario_plot.png に出す。日時のディレクトリを渡すと、中の
                              結果をすべて描く。自己位置の姿勢補正 (evt=localization.pose_reset) ごとに、その直後の
                              速度の指令の回転の急な変化を、姿勢補正の前と比べて表示する。
  compare <日時の dir>=<名前>...  構成ごとの結果 (scenario-test-all の日時のディレクトリ) を、シナリオごとに並べる
                              (真値との最大の差、ずれたまま走った距離、最も長い停止。点 1 つが 1 回)。

記録するトピック (シナリオの record_topics): /sim_scenario_test/entity_poses (真値)、/ekf_global_odom、
/amcl_pose_origin、/odom/gps、/localization/status、/initialpose、/cmd_vel、/rosout。時刻はヘッダの時刻 (シミュレーション時刻) で、
走行の始まり (シーケンサの最初の evt=nav.goal_sent) を 0 にする。
"""

import argparse
import glob
import json
import math
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import open_reader  # noqa: E402
from tools.common.map import load_map_info  # noqa: E402

TRUTH_TOPIC = "/sim_scenario_test/entity_poses"
STATE_NAMES = ["NORMAL", "SUSPECT", "ISOLATED", "RECOVERING", "DEGRADED"]
COLOR = {"truth": "#2b2b29", "ekf": "#2a78d6", "amcl": "#eb6834", "gnss": "#1baf7a", "event": "#eda100"}
CONFIG_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6"]
CONFIG_MARKERS = ["o", "s", "^", "D"]
GOAL_SENT = re.compile(r"evt=nav\.goal_sent index=(\d+)")
TRIGGER = re.compile(r"trigger (\w+) -> (on|off)")
NUM = r"([-0-9.]+)"


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag_dir: str, entity: str) -> Dict[str, list]:
    """記録から、時刻つきの値の並びを読む。"""
    reader = open_reader(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out: Dict[str, list] = {
        k: [] for k in ("truth", "ekf", "amcl", "gnss", "status", "initialpose", "cmd_vel", "rosout")}
    classes = {name: get_message(kind) for name, kind in types.items()}
    # 真値のメッセージはヘッダの時刻が空なので、記録の受信時刻を、/odom のヘッダの時刻 (シミュレーション時刻) との
    # 対応で読み替える (シミュレータの実時間比が 1 でなくても合うように、対応を補間する)
    clock_pairs: List[Tuple[float, float]] = []
    while reader.has_next():
        topic, data, received_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == TRUTH_TOPIC:
            for tf in msg.transforms:
                if tf.child_frame_id == entity:
                    tr = tf.transform.translation
                    out["truth"].append((received_ns * 1e-9, tr.x, tr.y))
        elif topic == "/odom":
            clock_pairs.append((received_ns * 1e-9, stamp_sec(msg.header.stamp)))
        elif topic == "/ekf_global_odom":
            p = msg.pose.pose.position
            out["ekf"].append((stamp_sec(msg.header.stamp), p.x, p.y))
        elif topic == "/amcl_pose_origin":
            p = msg.pose.pose.position
            out["amcl"].append((stamp_sec(msg.header.stamp), p.x, p.y))
        elif topic == "/odom/gps":
            p = msg.pose.pose.position
            out["gnss"].append((stamp_sec(msg.header.stamp), p.x, p.y, math.sqrt(max(msg.pose.covariance[0], 0.0))))
        elif topic == "/localization/status":
            out["status"].append((stamp_sec(msg.header.stamp), int(msg.state), bool(msg.hold_requested)))
        elif topic == "/initialpose":
            p = msg.pose.pose.position
            out["initialpose"].append((stamp_sec(msg.header.stamp), p.x, p.y))
        elif topic == "/cmd_vel":
            # ヘッダがないので、真値と同じく受信時刻から読み替える
            out["cmd_vel"].append((received_ns * 1e-9, msg.linear.x, msg.angular.z))
        elif topic == "/rosout":
            # ログの時刻は壁時計なので、真値と同じく受信時刻から読み替える
            out["rosout"].append((received_ns * 1e-9, msg.name, msg.msg))
    if len(clock_pairs) < 2:
        out["truth"] = []
        out["rosout"] = []
        out["cmd_vel"] = []
        return out
    pairs = np.asarray(sorted(clock_pairs))
    to_sim = lambda t: float(np.interp(t, pairs[:, 0], pairs[:, 1]))  # noqa: E731
    out["truth"] = [(to_sim(t), x, y) for t, x, y in out["truth"]]
    out["rosout"] = [(to_sim(t), name, text) for t, name, text in out["rosout"]]
    out["cmd_vel"] = [(to_sim(t), v, w) for t, v, w in out["cmd_vel"]]
    return out


def run_start(rosout: list) -> Optional[float]:
    for t, _, text in sorted(rosout):
        m = GOAL_SENT.search(text)
        if m and m.group(1) == "0":
            return t
    return None


def pose_resets(rosout: list) -> List[Tuple[float, str]]:
    """自己位置の姿勢補正のイベントの (時刻, イベントの文)。"""
    return sorted((t, text[text.index("evt="):]) for t, _, text in rosout if "evt=localization.pose_reset" in text)


def turn_change(cmd_vel: list, start: float, end: float) -> Optional[float]:
    """区間の中で、速度の指令の回転が続く 2 つの指令の間に変わった量の最大 [rad/s]。"""
    w = [row[2] for row in sorted(cmd_vel) if start <= row[0] <= end]
    return max((abs(b - a) for a, b in zip(w, w[1:])), default=None)


def reset_reactions(data: Dict[str, list], window_sec: float = 5.0) -> List[Tuple[float, str, Optional[float],
                                                                                   Optional[float]]]:
    """姿勢補正ごとの (時刻, イベントの文, 直後の回転の変化の最大, 直前の同じ長さの区間の回転の変化の最大)。"""
    return [(t, text, turn_change(data["cmd_vel"], t, t + window_sec), turn_change(data["cmd_vel"], t - window_sec, t))
            for t, text in pose_resets(data["rosout"])]


def error_series(truth: np.ndarray, est: list) -> Tuple[np.ndarray, np.ndarray]:
    """推定 (時刻, x, y) を真値の時刻に補間して、真値との距離を返す (推定の時刻の範囲の中だけ)。"""
    if len(est) < 2 or len(truth) < 2:
        return np.zeros(0), np.zeros(0)
    e = np.asarray(sorted(est))[:, :3]
    t = truth[:, 0]
    inside = (t >= e[0, 0]) & (t <= e[-1, 0])
    ex = np.interp(t[inside], e[:, 0], e[:, 1])
    ey = np.interp(t[inside], e[:, 0], e[:, 2])
    return t[inside], np.hypot(ex - truth[inside, 1], ey - truth[inside, 2])


def plot_run(run_dir: str, map_yaml: str, entity: str) -> Optional[str]:
    bag_dir = os.path.join(run_dir, "bag")
    if not os.path.isdir(bag_dir):
        print(f"記録がないので飛ばします: {run_dir}")
        return None
    result = json.load(open(os.path.join(run_dir, "result.json")))
    data = read_bag(bag_dir, entity)
    if len(data["truth"]) < 2:
        print(f"真値 ({TRUTH_TOPIC} の {entity}) がないので飛ばします: {run_dir}")
        return None
    t0 = run_start(data["rosout"]) or min(t for t, _, _ in data["truth"])
    truth = np.asarray(sorted(data["truth"]))
    truth = truth[truth[:, 0] >= t0 - 5.0]

    fig = plt.figure(figsize=(16, 9))
    grid = fig.add_gridspec(5, 2, width_ratios=[1.0, 1.4], hspace=0.35, wspace=0.15)
    ax_map = fig.add_subplot(grid[:, 0])
    img, origin, res, _ = load_map_info(map_yaml)
    h, w = img.shape[:2]
    ax_map.imshow(img, cmap="gray", extent=(origin[0], origin[0] + w * res, origin[1], origin[1] + h * res))
    ax_map.plot(truth[:, 1], truth[:, 2], color=COLOR["truth"], lw=2.0, label="truth")
    for key in ("ekf", "amcl"):
        if data[key]:
            arr = np.asarray(sorted(data[key]))
            arr = arr[arr[:, 0] >= t0 - 5.0]
            ax_map.plot(arr[:, 1], arr[:, 2], color=COLOR[key], lw=1.2, label=key)
    if data["gnss"]:
        arr = np.asarray(sorted(data["gnss"]))
        arr = arr[arr[:, 0] >= t0 - 5.0]
        ax_map.scatter(arr[:, 1], arr[:, 2], s=6, color=COLOR["gnss"], label="gnss", zorder=3)
    if data["initialpose"]:
        arr = np.asarray(data["initialpose"])
        ax_map.scatter(arr[:, 1], arr[:, 2], marker="x", s=60, color="#e34948", label="initialpose", zorder=4)
    triggers = [(t, m.group(1), m.group(2)) for t, name, text in data["rosout"]
                if name == "fault_injector" and (m := TRIGGER.search(text))]
    for t, name, state in triggers:
        i = int(np.clip(np.searchsorted(truth[:, 0], t), 0, len(truth) - 1))
        ax_map.scatter(truth[i, 1], truth[i, 2], marker="o" if state == "on" else "s", s=60,
                       facecolor="white", edgecolor=COLOR["event"], linewidth=2, zorder=5)
        ax_map.annotate(f"{name} {state}", (truth[i, 1], truth[i, 2]), xytext=(5, 5), textcoords="offset points",
                        fontsize=8)
    pad = 3.0
    ax_map.set_xlim(truth[:, 1].min() - pad, truth[:, 1].max() + pad)
    ax_map.set_ylim(truth[:, 2].min() - pad, truth[:, 2].max() + pad)
    ax_map.set_aspect("equal")
    ax_map.legend(loc="upper right", fontsize=8)
    ax_map.set_title(f"{result['scenario_name']}  {result['status']}  "
                     f"reached {result['goals']['reached']}/{result['goals']['total']}", fontsize=10)

    axes = [fig.add_subplot(grid[i, 1]) for i in range(5)]
    for ax in axes[1:]:
        ax.sharex(axes[0])
    rel = lambda t: np.asarray(t) - t0  # noqa: E731
    for key in ("ekf", "amcl", "gnss"):
        t, err = error_series(truth, [row[:3] for row in data[key]])
        if t.size:
            axes[0].plot(rel(t), err, color=COLOR[key], lw=1.0, label=key)
    axes[0].axhline(1.0, color="#6b6a63", ls="--", lw=0.8)
    axes[0].set_ylabel("error vs truth [m]")
    axes[0].legend(loc="upper left", fontsize=8, ncol=3)
    if data["gnss"]:
        arr = np.asarray(sorted(data["gnss"]))
        axes[1].plot(rel(arr[:, 0]), arr[:, 3], color=COLOR["gnss"], lw=1.0)
    axes[1].set_ylabel("GNSS declared std [m]")
    # 真値は高い頻度で届くので、0.5 s ごとに間引いてから速さを求める
    grid_t = np.arange(truth[0, 0], truth[-1, 0], 0.5)
    gx = np.interp(grid_t, truth[:, 0], truth[:, 1])
    gy = np.interp(grid_t, truth[:, 0], truth[:, 2])
    speed = np.hypot(np.diff(gx), np.diff(gy)) / 0.5
    axes[2].plot(rel(grid_t[1:]), speed, color=COLOR["truth"], lw=0.8)
    axes[2].set_ylabel("speed [m/s]")
    if data["cmd_vel"]:
        arr = np.asarray(sorted(data["cmd_vel"]))
        axes[3].plot(rel(arr[:, 0]), arr[:, 2], color=COLOR["truth"], lw=0.8)
    axes[3].set_ylabel("cmd_vel angular [rad/s]")
    resets = reset_reactions(data)
    for t, text, after, before in resets:
        for ax in axes:
            ax.axvline(t - t0, color="#6250d6", lw=1.0, ls="-.")
        print(f"  姿勢補正 t={t - t0:.1f} s  回転の指令の変化の最大: 直後 {after} / 直前 {before} [rad/s]  {text}")
    if data["status"]:
        arr = np.asarray(sorted(data["status"]), dtype=float)
        axes[4].step(rel(arr[:, 0]), arr[:, 1], where="post", color=COLOR["amcl"], lw=1.5)
        hold = arr[:, 2] > 0.5
        axes[4].fill_between(rel(arr[:, 0]), -0.4, 4.4, where=hold, step="post", color="#e34948", alpha=0.12,
                             label="hold")
        axes[4].legend(loc="upper left", fontsize=8)
    axes[4].set_yticks(range(len(STATE_NAMES)), STATE_NAMES, fontsize=7)
    axes[4].set_ylim(-0.4, 4.4)
    axes[4].set_xlabel("time from run start [s]")
    for t, name, text in data["rosout"]:
        if GOAL_SENT.search(text):
            for ax in axes:
                ax.axvline(t - t0, color="#b0afa8", lw=0.6, ls=":")
    for t, name, state in triggers:
        for ax in axes:
            ax.axvline(t - t0, color=COLOR["event"], lw=1.0, ls="-" if state == "on" else "--")
    for t, _, _ in data["initialpose"]:
        axes[0].axvline(t - t0, color="#e34948", lw=0.8)
    axes[0].set_xlim(-5, rel(truth[-1, 0]))
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.text(0.99, 0.005, "dotted: goal sent / orange: fault trigger (solid on, dashed off) / red: /initialpose / "
             "purple: pose reset",
             ha="right", fontsize=8, color="#6b6a63")
    out = os.path.join(run_dir, "scenario_plot.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"保存しました: {out}")
    return out


def run_dirs(paths: List[str]) -> List[str]:
    dirs = []
    for path in paths:
        if os.path.isfile(os.path.join(path, "result.json")):
            dirs.append(path)
        else:
            dirs.extend(sorted(os.path.dirname(p) for p in glob.glob(os.path.join(path, "*", "result.json"))))
    return dirs


def check_message(checks: list, name: str) -> str:
    return next((c["message"] for c in checks if c["name"] == name), "")


def run_metrics(result: dict) -> Optional[Tuple[float, float, float, bool]]:
    """真値との最大の差 [m]、ずれたまま走った距離 [m]、最も長い停止 [s]、全通過点に着いたか。"""
    pe = re.search(r"max error " + NUM + r".*travelled " + NUM, check_message(result["checks"], "monitor:pose_error"))
    st = re.search(r"longest stop " + NUM, check_message(result["checks"], "monitor:max_stop_duration"))
    if not pe or not st:
        return None
    goals = result.get("goals") or {}
    return float(pe.group(1)), float(pe.group(2)), float(st.group(1)), goals.get("reached") == goals.get("total")


def plot_compare(pairs: List[Tuple[str, str]], out: str) -> None:
    table: Dict[str, Dict[str, list]] = {}
    for path, label in pairs:
        for run in run_dirs([path]):
            result = json.load(open(os.path.join(run, "result.json")))
            metrics = run_metrics(result)
            if metrics is not None:
                table.setdefault(result["scenario_name"], {}).setdefault(label, []).append(metrics)
    scenarios = sorted(table)
    labels = [label for _, label in pairs]
    panels = [("max error vs truth [m]", 0), ("distance driven while deviated >1 m [m]", 1), ("longest stop [s]", 2)]
    fig, axes = plt.subplots(len(panels), 1, figsize=(max(8, 1.6 * len(scenarios)), 9), sharex=True)
    width = 0.7 / max(len(labels), 1)
    for ci, label in enumerate(labels):
        for si, scenario in enumerate(scenarios):
            x = si + (ci - (len(labels) - 1) / 2) * width
            for metrics in table[scenario].get(label, []):
                for ax, (_, k) in zip(axes, panels):
                    color = CONFIG_COLORS[ci % len(CONFIG_COLORS)]
                    ax.scatter(x, metrics[k], s=40, marker=CONFIG_MARKERS[ci % len(CONFIG_MARKERS)],
                               color=color if metrics[3] else "white", edgecolors=color, linewidths=1.6, zorder=3)
    for ax, (title, _) in zip(axes, panels):
        ax.set_title(title, loc="left", fontsize=10)
        ax.grid(axis="y", alpha=0.3)
        for si in range(len(scenarios) - 1):
            ax.axvline(si + 0.5, color="#e4e3de", lw=0.8)
    axes[0].axhline(1.0, color="#6b6a63", ls="--", lw=0.8)
    axes[-1].set_xticks(range(len(scenarios)), [s.replace("anomaly_", "").replace("_", "\n") for s in scenarios],
                        fontsize=8)
    handles = [plt.Line2D([], [], marker=CONFIG_MARKERS[i % 4], ls="", color=CONFIG_COLORS[i % 4], label=label)
               for i, label in enumerate(labels)]
    handles.append(plt.Line2D([], [], marker="o", ls="", markerfacecolor="white", markeredgecolor="#6b6a63",
                              label="hollow: not all goals reached"))
    fig.legend(handles=handles, loc="upper center", ncol=len(handles), frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"保存しました: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="シナリオテストの結果を図にする")
    sub = parser.add_subparsers(dest="command", required=True)
    p_run = sub.add_parser("run", help="1 回分の軌跡と時系列")
    p_run.add_argument("results", nargs="+", help="結果のディレクトリ (1 回分、または日時のディレクトリ)")
    p_run.add_argument("--map", default=os.path.join(REPO_ROOT, "mg_simulation", "maps", "warehouse", "planning.yaml"),
                       help="背景に描く地図 (map.yaml)")
    p_run.add_argument("--entity", default="mg", help="真値の姿勢を取るモデル名 (プロファイルの sim.robot_entity)")
    p_cmp = sub.add_parser("compare", help="構成ごとの結果をシナリオごとに並べる")
    p_cmp.add_argument("pairs", nargs="+", help="<日時のディレクトリ>=<構成の名前>")
    p_cmp.add_argument("-o", "--output", help="出力先 (省略で最初のディレクトリの compare.png)")
    args = parser.parse_args()

    if args.command == "run":
        for run in run_dirs(args.results):
            plot_run(run, args.map, args.entity)
    else:
        pairs = [tuple(p.rsplit("=", 1)) for p in args.pairs]
        plot_compare(pairs, args.output or os.path.join(pairs[0][0], "compare.png"))


if __name__ == "__main__":
    main()
