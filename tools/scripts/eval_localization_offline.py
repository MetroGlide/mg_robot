#!/usr/bin/env python3
"""
eval_localization_offline.py

自己位置の監督 (localization_supervisor) を、再生評価の記録 (run_localization_variant.sh の run_N/output) で
オフライン評価する。実時間を待たずに、判定・総合・状態遷移だけを回す。

- 入力: 記録した推定器の出力 (AMCL・EKF・GNSS・オドメトリ) と、データセットの評価 bag のスキャンと地図
- 故障: --faults で、再生評価と同じ書式の故障を、記録した標本に注入する (tools/common/offline_faults.py)。
  --labels は、記録したときにすでに注入されていた故障 (例: kidnap の変種の記録) を正解としてだけ使う
- プロファイル: --profile を並べると、同じ入力で比べる (既定の設定は '')

使い方:
  eval_localization_offline.py <変種または run のディレクトリ>... [--dataset 名前] [--faults yaml] [--labels yaml]
                               [--profile 名前 ...] [--config yaml] [--followers ekf=0.5] [--out 名前]
例:
  eval_localization_offline.py <bag>/eval_loc/map043837_eval051635_short/adopted \
      --faults tools/datasets/localization/faults/kidnap.yaml --profile '' --profile consecutive

出力: <run>/offline/<出力名>/<プロファイル>/{timeline.jsonl, events.jsonl, report.md} と、標準出力の比較表
"""

import argparse
import glob
import os
import statistics
import sys
from typing import Dict, List, Optional, Sequence

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for path in (REPO_ROOT, os.path.join(REPO_ROOT, "localization_supervisor", "localization_supervisor")):
    if path not in sys.path:
        sys.path.insert(0, path)

from localization_supervisor.core.config import load_config  # noqa: E402
from localization_supervisor.core.faults import Fault, parse_faults  # noqa: E402
from localization_supervisor.ros.offline_runner import (  # noqa: E402
    BagInput, OfflineOptions, OfflineResult, run_offline, write_outputs)
from tools.common.faults import load_faults  # noqa: E402
from tools.common.offline_faults import to_offline  # noqa: E402
from tools.scripts.loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

DEFAULT_CONFIG = os.path.join(REPO_ROOT, "mg_localization_supervisor", "config", "supervision.yaml")
# 記録した推定器の出力のうち、監督の入力にするトピック (supervision.yaml の sources と合わせる)
OUTPUT_TOPICS = ["/amcl_pose_origin", "/ekf_global_odom", "/odom", "/odom/gps"]
# 評価 bag から読むトピック (スキャンと、センサの取り付け位置)
SOURCE_TOPICS = ["/scan_top_lidar", "/tf_static"]
# 故障ごとに、正しい blame
EXPECTED_BLAME = {("pose_offset", "amcl"): "amcl", ("pose_drift", "amcl"): "amcl", ("pose_offset", "gnss"): "gnss"}


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


def summarize(results: Dict[str, List[OfflineResult]]) -> str:
    """プロファイルごとに、試行をまとめた表。"""
    lines = ["検知の対象は姿勢をずらす故障だけ。誤検知は、その区間の外 (標本が欠けるだけの故障の区間を含む) で確認に"
             "移った回数。再初期化は、監督が指示した回数 (誤りは、候補が真の姿勢から ok_threshold を超えて離れたもの。"
             "区間外は、故障の区間の外での指示)。復旧は、指示の後に NORMAL に戻るまでの中央値。", "",
             "| プロファイル | 試行 | 確認した故障 | 疑いまで 中央値[s] | 確認まで 中央値[s] | blame の正解 | 誤検知[回] |"
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
    parser = argparse.ArgumentParser(description="自己位置の監督を再生評価の記録でオフライン評価する")
    parser.add_argument("paths", nargs="+", help="変種ディレクトリ (run_N を含む) または run_N ディレクトリ")
    parser.add_argument("--dataset", default="map043837_eval051635_short", help="データセット (名前またはパス)")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="監督の設定ファイル")
    parser.add_argument("--profile", action="append", help="比べるプロファイル ('' は既定の設定)")
    parser.add_argument("--faults", help="記録した標本に注入する故障 (再生評価と同じ書式)")
    parser.add_argument("--labels", help="記録したときに注入されていた故障 (正解としてだけ使う)")
    parser.add_argument("--followers", action="append", default=[],
                        help="AMCL のずれに EKF も引きずられる割合 (例: ekf=0.5)")
    parser.add_argument("--apply-reset", default="amcl,ekf",
                        help="再初期化の指示の後、選んだ姿勢に合わせる姿勢の情報源 (記録は初期化し直されないので、"
                             "同じ効果を真似る。空なら、指示を記録するだけ)")
    parser.add_argument("--truth", default="", help="候補の誤差を測る真の姿勢の情報源 (故障を注入する前の記録。"
                                                    "既定は設定の anchors.pose)")
    parser.add_argument("--ok-threshold", type=float, default=1.0, help="候補の誤差がこれを超えたら誤った復旧 [m]")
    parser.add_argument("--period", type=float, default=1.0, help="周期 [s]")
    parser.add_argument("--out", help="出力の名前 (既定は故障の定義のファイル名)")
    args = parser.parse_args()

    dataset = load_dataset(resolve_dataset_path(args.dataset))
    followers = {k: float(v) for k, v in (item.split("=", 1) for item in args.followers)}
    faults = offline_faults(args.faults, followers)
    labels = offline_faults(args.labels, {})
    profiles = args.profile if args.profile else [""]
    out_name = args.out or os.path.splitext(os.path.basename(args.faults or args.labels or "nofault"))[0]

    results: Dict[str, List[OfflineResult]] = {}
    for profile in profiles:
        config = load_config(args.config, profile)
        for run in find_runs(args.paths):
            bags = [BagInput(os.path.join(run, "output"), OUTPUT_TOPICS),
                    BagInput(dataset["BAG"], SOURCE_TOPICS)]
            result = run_offline(config, bags, OfflineOptions(
                period_sec=args.period, maps={"map_scan": dataset["MAP_YAML"]}, faults=faults, labels=labels,
                apply_reset=[n for n in args.apply_reset.split(",") if n], truth=args.truth,
                ok_threshold_m=args.ok_threshold))
            out_dir = os.path.join(run, "offline", out_name, profile or "default")
            write_outputs(result, out_dir, f"{os.path.basename(run)} {profile or '(既定)'}")
            results.setdefault(profile, []).append(result)
            print(f"{run} [{profile or '(既定)'}]: 確認 {result.report.detected}/{len(result.report.faults)}、"
                  f"誤検知 {result.report.false_alarms} -> {out_dir}", file=sys.stderr)
    print(summarize(results))


if __name__ == "__main__":
    main()
