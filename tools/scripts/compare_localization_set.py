#!/usr/bin/env python3
"""
compare_localization_set.py

評価セット (run_localization_set.sh) の結果を、条件 (行) × 名前 (列) の表にまとめる。
指標ごとに 1 つの表を出し、最後の行に全条件をまとめた値 (割合は試行を合わせた割合、誤差は条件の平均) を置く。
各条件の結果は <評価 bag>/eval_loc/<データセット>/<名前>_<ラベル>/run_*/eval_localization.json から読む。

使い方:
  compare_localization_set.py [--set <名前|パス>] <名前> [<名前> ...] [-o summary.md]
"""

import argparse
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from compare_localization import (  # noqa: E402
    DEVIATED_LABEL, FAILURE_LABEL, FAILURE_P95_M, MISPICK_LABEL, format_cell, get_path, load_runs, summarize_variant,
)
from loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

SET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "datasets", "localization", "sets")
CONTAINER_DATA = "/root/ros2_data"

# (表の見出し, JSON 内のパス, 小数点以下の桁数)。割合の指標は compare_localization.py の集計を使う
RATE_LABELS = [FAILURE_LABEL, DEVIATED_LABEL, MISPICK_LABEL]
ERROR_METRICS: List[Tuple[str, Tuple[str, ...], int]] = [
    ("yaw 誤差 p95[rad]", ("accuracy", "yaw_rad", "p95"), 3),
    ("位置誤差 p95[m]", ("accuracy", "position_m", "p95"), 3),
    ("整合後の位置誤差 p95[m]", ("accuracy", "aligned", "position_m", "p95"), 3),
    ("GNSS が弱い時間の位置誤差 中央値[m]", ("gnss_weak", "weak_position_m", "median"), 3),
    ("NEES 平均 (理想 3)", ("nees", "mean"), 2),
]


def read_set(name_or_path: str) -> List[Tuple[str, str]]:
    """評価セットの (ラベル, データセット) の並び。"""
    path = name_or_path if os.path.isfile(name_or_path) else os.path.join(SET_DIR, name_or_path + ".txt")
    if not os.path.isfile(path):
        raise SystemExit(f"エラー: 評価セットが見つかりません: {name_or_path}")
    entries = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            fields = line.split()
            if len(fields) >= 2 and not fields[0].startswith("#"):
                entries.append((fields[0], fields[1]))
    return entries


def host_path(path: str) -> str:
    """コンテナ内のデータのパスを、ホストで読めるパスに直す (ホストで実行したとき)。"""
    if not os.path.exists(path) and path.startswith(CONTAINER_DATA):
        return os.path.join(os.path.expanduser("~/ros2_data"), os.path.relpath(path, CONTAINER_DATA))
    return path


def variant_dir(dataset: str, name: str, label: str) -> str:
    values = load_dataset(resolve_dataset_path(dataset))
    return host_path(os.path.join(values["BAG"], "eval_loc", values["DATASET_NAME"], f"{name}_{label}"))


def collect(entries: List[Tuple[str, str]], names: List[str]) -> Dict[Tuple[str, str], List[Dict[str, Any]]]:
    return {(label, name): load_runs(variant_dir(dataset, name, label))
            for label, dataset in entries for name in names}


def pooled_rate(runs_list: List[List[Dict[str, Any]]], label: str) -> Optional[float]:
    """条件をまたいで試行を合わせた割合。"""
    hits, total = 0.0, 0
    for runs in runs_list:
        agg = summarize_variant(runs)[label] if runs else None
        if agg is None:
            continue
        judged = len(runs)
        hits += agg[0] * judged
        total += judged
    return hits / total if total else None


def render(
    entries: List[Tuple[str, str]], names: List[str], results: Dict[Tuple[str, str], List[Dict[str, Any]]],
) -> str:
    header = "| 条件 | " + " | ".join(names) + " |"
    rule = "|---|" + "---|" * len(names)
    lines = ["# 自己位置推定 評価セットの比較", "",
             "各セルは 試行の平均 (最小–最大)。括弧の前の値が小さいほど良い (NEES は 3 に近いほど良い)。",
             f"破綻した試行 = 位置誤差の p95 が {FAILURE_P95_M:g} m を超えた試行。試行の無い条件は -。", "",
             "| 条件 | " + " | ".join(f"{n} 試行数" for n in names) + " |", rule]
    for label, _ in entries:
        lines.append(f"| {label} | " + " | ".join(str(len(results[(label, n)])) for n in names) + " |")
    for title in RATE_LABELS:
        lines += ["", f"## {title}", "", header, rule]
        for label, _ in entries:
            cells = [format_cell(summarize_variant(results[(label, n)])[title], 2) if results[(label, n)] else "-"
                     for n in names]
            lines.append(f"| {label} | " + " | ".join(cells) + " |")
        totals = [pooled_rate([results[(label, n)] for label, _ in entries], title) for n in names]
        lines.append("| **全条件** | " + " | ".join("-" if v is None else f"{v:.2f}" for v in totals) + " |")
    for title, path, digits in ERROR_METRICS:
        lines += ["", f"## {title}", "", header, rule]
        means: Dict[str, List[float]] = {n: [] for n in names}
        for label, _ in entries:
            cells = []
            for n in names:
                agg = summarize_values([get_path(run, path) for run in results[(label, n)]])
                cells.append(format_cell(agg, digits))
                if agg is not None:
                    means[n].append(agg[0])
            lines.append(f"| {label} | " + " | ".join(cells) + " |")
        lines.append("| **全条件の平均** | " + " | ".join(
            f"{sum(means[n]) / len(means[n]):.{digits}f}" if means[n] else "-" for n in names) + " |")
    return "\n".join(lines) + "\n"


def summarize_values(values: List[Optional[float]]) -> Optional[Tuple[float, float, float]]:
    present = [v for v in values if v is not None]
    if not present:
        return None
    return sum(present) / len(present), min(present), max(present)


def main() -> None:
    parser = argparse.ArgumentParser(description="評価セットの結果を名前ごとに比べる")
    parser.add_argument("names", nargs="+", help="run_localization_set.sh に渡した名前")
    parser.add_argument("--set", default="robustness", help="評価セット (sets/<名前>.txt またはパス)")
    parser.add_argument("-o", "--output", default=None, help="Markdown の出力先 (省略時は標準出力のみ)")
    args = parser.parse_args()

    entries = read_set(args.set)
    results = collect(entries, args.names)
    text = render(entries, args.names, results)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
