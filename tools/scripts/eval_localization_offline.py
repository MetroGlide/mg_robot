#!/usr/bin/env python3
"""
eval_localization_offline.py

自己位置監視 (localization_supervisor) を、再生評価の記録 (run_localization_variant.sh の run_N/output) で
オフライン評価する。実時間を待たずに、判定・総合・状態遷移だけを回す。

- 入力: 記録した推定器の出力 (AMCL・EKF・GNSS・オドメトリ、記録していれば AMCL の粒子) と、データセットの
  評価 bag のスキャンと地図
- 故障: --faults で、再生評価と同じ書式の故障を、記録した標本に注入する (tools/common/offline_faults.py)。
  AMCL をずらす故障では、AMCL の粒子 (設定の仮説の集合の情報源) も同じだけずらす。
  --labels は、記録したときにすでに注入されていた故障 (例: kidnap の変種の記録) を正解としてだけ使う
- 真値: --gt で、再初期化の指示の候補を、データセットの SLAM 出力 (真値) と比べる。故障を注入しない記録で
  実際にずれた事象 (角での誤収束など) の復旧の候補を確かめるのに使う
- プロファイル: --profile を並べると、同じ入力で比べる (既定の設定は '')

使い方:
  eval_localization_offline.py <変種または run のディレクトリ>... [--dataset 名前] [--faults yaml] [--labels yaml]
                               [--profile 名前 ...] [--config yaml] [--followers ekf=0.5] [--gt] [--out 名前]
例:
  eval_localization_offline.py <bag>/eval_loc/map043837_eval051635_short/adopted \
      --faults tools/datasets/localization/faults/kidnap.yaml --profile '' --profile consecutive

出力: <run>/offline/<出力名>/<プロファイル>/{timeline.jsonl, events.jsonl, report.md, (--gt) reinit_truth.json} と、
      標準出力の比較表
"""

import argparse
import glob
import json
import os
import statistics
import sys
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for path in (REPO_ROOT, os.path.join(REPO_ROOT, "localization_supervisor", "localization_supervisor")):
    if path not in sys.path:
        sys.path.insert(0, path)

from localization_supervisor.core.config import load_config  # noqa: E402
from localization_supervisor.core.faults import Fault, parse_faults  # noqa: E402
from localization_supervisor.core.interfaces import SourceKind  # noqa: E402
from localization_supervisor.ros.offline_runner import (  # noqa: E402
    BagInput, OfflineOptions, OfflineResult, run_offline, write_outputs)
from tools.common import loc_metrics as lm  # noqa: E402
from tools.common.faults import load_faults  # noqa: E402
from tools.common.offline_faults import to_offline  # noqa: E402
from tools.common.pose_graph import interpolate_nodes, load_slam_output, shift_nodes_to_anchor  # noqa: E402
from tools.scripts.loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

DEFAULT_CONFIG = os.path.join(REPO_ROOT, "mg_localization_supervisor", "config", "supervision.yaml")
# 記録した推定器の出力のうち、自己位置監視の入力にするトピック (supervision.yaml の sources と合わせる)
OUTPUT_TOPICS = ["/amcl_pose_origin", "/ekf_global_odom", "/odom", "/odom/gps", "/particle_cloud"]
# 評価 bag から読むトピック (スキャンと、センサの取り付け位置)
SOURCE_TOPICS = ["/scan_top_lidar", "/tf_static"]
# 故障ごとに、正しい blame
EXPECTED_BLAME = {("pose_offset", "amcl"): "amcl", ("pose_drift", "amcl"): "amcl", ("pose_offset", "gnss"): "gnss"}
# 真値の補間で、止まっているとみなす隣接ノード間の移動 [m] (eval_localization.py と同じ)
STATIONARY_DIST_M = 0.6
# 再初期化の候補が誤りとみなす向きの差 [deg] (位置は --ok-threshold)
WRONG_YAW_DEG = 25.0


def find_runs(paths: Sequence[str]) -> List[str]:
    runs: List[str] = []
    for path in paths:
        if os.path.isdir(os.path.join(path, "output")):
            runs.append(path)
        else:
            runs += sorted(p for p in glob.glob(os.path.join(path, "run_*")) if os.path.isdir(os.path.join(p, "output")))
    if not runs:
        raise SystemExit(f"エラー: run_N/output が見つかりません: {list(paths)}")
    return runs


def offline_faults(path: Optional[str], followers: Dict[str, float]) -> List[Fault]:
    if not path:
        return []
    return parse_faults(to_offline(load_faults(path), followers))


def load_truth(dataset: Dict[str, str]) -> np.ndarray:
    """評価の走行の真値を、地図の座標で (地図が別の走行なら UTM 経由でそろえる)。"""
    nodes, transform, _ = load_slam_output(dataset["GT_DIR"])
    if dataset["MAP_GT_DIR"]:
        _, map_transform, _ = load_slam_output(dataset["MAP_GT_DIR"])
        nodes = shift_nodes_to_anchor(nodes, transform, map_transform)
    return nodes


def reinit_truth(result: OfflineResult, nodes: np.ndarray) -> List[Dict[str, Any]]:
    """再初期化の指示ごとの、候補と真値の差 (真値を補間できない時刻のものは除く)。"""
    rows = []
    for step in result.steps:
        reinit = step.get("reinit")
        if not reinit:
            continue
        gt, valid = interpolate_nodes(nodes, np.array([result.t0 + step["t"]]), stationary_dist=STATIONARY_DIST_M)
        if not valid[0]:
            continue
        pos_err, yaw_err = lm.pose_errors(np.array([reinit["pose"]]), gt)
        rows.append({"t": step["t"], "origin": reinit["origin"], "error_m": round(float(pos_err[0]), 3),
                     "error_yaw_deg": round(float(np.degrees(abs(yaw_err[0]))), 1)})
    return rows


def summarize_truth(truths: Dict[str, List[List[Dict[str, Any]]]], ok_threshold_m: float) -> str:
    """プロファイルごとの、再初期化の候補と真値の差。"""
    lines = [f"再初期化の候補と真値の差。誤りは、位置が {ok_threshold_m:g} m か向きが {WRONG_YAW_DEG:g} deg を超えたもの。", "",
             "| プロファイル | 試行 | 再初期化[回] | 誤り | 位置の差 中央値[m] | 向きの差 中央値[deg] | 候補を作ったもの |",
             "|---|---|---|---|---|---|---|"]
    for profile, runs in truths.items():
        rows = [row for run in runs for row in run]
        wrong = sum(1 for r in rows if r["error_m"] > ok_threshold_m or r["error_yaw_deg"] > WRONG_YAW_DEG)
        origins: Dict[str, int] = {}
        for r in rows:
            origins[r["origin"]] = origins.get(r["origin"], 0) + 1
        lines.append(
            f"| {profile or '(既定)'} | {len(runs)} | {len(rows)} | {wrong} | "
            f"{statistics.median([r['error_m'] for r in rows]) if rows else '-'} | "
            f"{statistics.median([r['error_yaw_deg'] for r in rows]) if rows else '-'} | "
            f"{', '.join(f'{k} {v}' for k, v in sorted(origins.items())) or '-'} |")
    return "\n".join(lines) + "\n"


def summarize(results: Dict[str, List[OfflineResult]]) -> str:
    """プロファイルごとに、試行をまとめた表。"""
    lines = ["検知の対象は姿勢をずらす故障だけ。誤検知は、その区間の外 (標本が欠けるだけの故障の区間を含む) で確認に"
             "移った回数。再初期化は、自己位置監視が指示した回数 (誤りは、候補が真の姿勢から ok_threshold を超えて離れたもの。"
             "区間外は、故障の区間の外での指示)。復旧は、指示の後に NORMAL に戻るまでの中央値。", "",
             "| プロファイル | 試行 | 確認した故障 | 疑義まで 中央値[s] | 確認まで 中央値[s] | blame の正解 | 誤検知[回] |"
             " 故障の外で NORMAL 以外の割合 | 再初期化[回] | 誤り | 区間外 | 見送り | 復旧 中央値[s] |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for profile, runs in results.items():
        faults = [f for r in runs for f in r.report.expected]
        confirmed = [f for f in faults if f.confirm_after is not None]
        suspects = [f.suspect_after for f in faults if f.suspect_after is not None]
        judged = [f for f in confirmed if (f.type, f.source) in EXPECTED_BLAME]
        correct = sum(1 for f in judged if f.blame == EXPECTED_BLAME[(f.type, f.source)])
        clean = sum(r.report.clean_ticks for r in runs)
        not_normal = sum(r.report.clean_not_normal_ticks for r in runs)
        recovered = [f.recovered_after - f.reinit_after for f in faults
                     if f.recovered_after is not None and f.reinit_after is not None]
        skipped = sum(sum(r.report.skipped.values()) for r in runs)
        lines.append(
            f"| {profile or '(既定)'} | {len(runs)} | {len(confirmed)}/{len(faults)} | "
            f"{statistics.median(suspects) if suspects else '-'} | "
            f"{statistics.median([f.confirm_after for f in confirmed]) if confirmed else '-'} | "
            f"{correct}/{len(judged)} | {sum(r.report.false_alarms for r in runs)} | "
            f"{not_normal / clean if clean else 0.0:.3f} | "
            f"{sum(len(r.report.reinits) for r in runs)} | {sum(r.report.wrong_reinits for r in runs)} | "
            f"{sum(r.report.spurious_reinits for r in runs)} | {skipped} | "
            f"{statistics.median(recovered) if recovered else '-'} |")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="自己位置監視を再生評価の記録でオフライン評価する")
    parser.add_argument("paths", nargs="+", help="変種ディレクトリ (run_N を含む) または run_N ディレクトリ")
    parser.add_argument("--dataset", default="map043837_eval051635_short", help="データセット (名前またはパス)")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="自己位置監視の設定ファイル")
    parser.add_argument("--profile", action="append", help="比べるプロファイル ('' は既定の設定)")
    parser.add_argument("--faults", help="記録した標本に注入する故障 (再生評価と同じ書式)")
    parser.add_argument("--labels", help="記録したときに注入されていた故障 (正解としてだけ使う)")
    parser.add_argument("--followers", action="append", default=[],
                        help="AMCL のずれに EKF も引きずられる割合 (例: ekf=0.5)")
    parser.add_argument("--apply-reset", default="amcl,ekf",
                        help="再初期化の指示の後、選んだ姿勢に合わせる姿勢の情報源 (記録は初期化し直されないので、"
                             "同じ効果を真似る。空なら、指示を記録するだけ。空でなければ AMCL の粒子も合わせる)")
    parser.add_argument("--truth", default="", help="候補の誤差を測る真の姿勢の情報源 (故障を注入する前の記録。"
                                                    "既定は設定の anchors.pose)")
    parser.add_argument("--ok-threshold", type=float, default=1.0, help="候補の誤差がこれを超えたら誤った復旧 [m]")
    parser.add_argument("--period", type=float, default=1.0, help="周期 [s]")
    parser.add_argument("--gt", action="store_true", help="再初期化の候補を、データセットの真値と比べる")
    parser.add_argument("--out", help="出力の名前 (既定は故障の定義のファイル名)")
    args = parser.parse_args()

    dataset = load_dataset(resolve_dataset_path(args.dataset))
    followers = {k: float(v) for k, v in (item.split("=", 1) for item in args.followers)}
    # AMCL の粒子 (仮説の集合の情報源) は、AMCL と一緒にずらし、再初期化の後も AMCL に合わせる
    hypotheses = [name for name, kind in load_config(args.config).kinds().items() if kind == SourceKind.HYPOTHESES]
    for name in hypotheses:
        followers.setdefault(name, 1.0)
    apply_reset = [n for n in args.apply_reset.split(",") if n]
    if apply_reset:
        apply_reset += [n for n in hypotheses if n not in apply_reset]
    faults = offline_faults(args.faults, followers)
    labels = offline_faults(args.labels, {})
    profiles = args.profile if args.profile else [""]
    out_name = args.out or os.path.splitext(os.path.basename(args.faults or args.labels or "nofault"))[0]

    nodes = load_truth(dataset) if args.gt else None
    results: Dict[str, List[OfflineResult]] = {}
    truths: Dict[str, List[List[Dict[str, Any]]]] = {}
    for profile in profiles:
        config = load_config(args.config, profile)
        for run in find_runs(args.paths):
            bags = [BagInput(os.path.join(run, "output"), OUTPUT_TOPICS),
                    BagInput(dataset["BAG"], SOURCE_TOPICS)]
            result = run_offline(config, bags, OfflineOptions(
                period_sec=args.period, maps={"map_scan": dataset["MAP_YAML"]}, faults=faults, labels=labels,
                apply_reset=apply_reset, truth=args.truth,
                ok_threshold_m=args.ok_threshold))
            out_dir = os.path.join(run, "offline", out_name, profile or "default")
            write_outputs(result, out_dir, f"{os.path.basename(run)} {profile or '(既定)'}")
            results.setdefault(profile, []).append(result)
            if nodes is not None:
                truth = reinit_truth(result, nodes)
                with open(os.path.join(out_dir, "reinit_truth.json"), "w", encoding="utf-8") as f:
                    json.dump(truth, f, ensure_ascii=False, indent=1)
                truths.setdefault(profile, []).append(truth)
            print(f"{run} [{profile or '(既定)'}]: 確認 {result.report.detected}/{len(result.report.faults)}、"
                  f"誤検知 {result.report.false_alarms} -> {out_dir}", file=sys.stderr)
    print(summarize(results))
    if nodes is not None:
        print(summarize_truth(truths, args.ok_threshold))


if __name__ == "__main__":
    main()
