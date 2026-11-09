#!/usr/bin/env python3
"""
compare_localization_set.py

評価セット (run_localization_set.sh) の結果を、条件 (行) × 名前 (列) の表にまとめる。
指標ごとに 1 つの表を出し、最後の行に全条件をまとめた値 (割合は試行を合わせた割合、誤差は条件の平均) を置く。
各条件の結果は <評価 bag>/eval_loc/<データセット>/<名前>_<ラベル>/run_*/eval_localization.json から読む。
--baseline を渡すと、その名前と比べて許容差を超えて悪くなったセルに印を付け、一覧にする。

使い方:
  compare_localization_set.py [--set <名前|パス>] [--baseline <名前>] <名前> [<名前> ...] [-o summary.md]
"""

import argparse
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from compare_localization import (  # noqa: E402
    DEVIATED_LABEL, UNRECOVERED_LABEL, aggregate, format_cell, get_path, load_runs, summarize_variant,
)
from loc_dataset import load_dataset, resolve_dataset_path  # noqa: E402

SET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "datasets", "localization", "sets")
CONTAINER_DATA = "/root/ros2_data"

# (表の見出し, JSON 内のパス, 小数点以下の桁数)。割合の指標は compare_localization.py の集計を使う
RATE_LABELS = [DEVIATED_LABEL, UNRECOVERED_LABEL]
ERROR_METRICS: List[Tuple[str, Tuple[str, ...], int]] = [
    ("yaw 誤差 p95[rad]", ("accuracy", "yaw_rad", "p95"), 3),
    ("位置誤差 p95[m]", ("accuracy", "position_m", "p95"), 3),
    ("整合後の位置誤差 p95[m]", ("accuracy", "aligned", "position_m", "p95"), 3),
    ("GNSS が弱い時間の位置誤差 中央値[m]", ("gnss_weak", "weak_position_m", "median"), 3),
    ("ずれた時間[s]", ("deviations", "total_sec"), 0),
    ("止める要求[回]", ("hold", "count"), 0),
    ("止める要求の時間[s]", ("hold", "total_sec"), 0),
    ("NEES 平均 (理想 3)", ("nees", "mean"), 2),
    ("偏りを除いた NEES 中央値 (事象の外)", ("nees", "residual_outside_episodes", "median"), 2),
]
# 「悪くならない」の許容差: 指標ごとの (絶対量, 基準に対する割合)。基準の中央値 + max(絶対量, 割合 × 基準) を
# 超えたら悪くなったとする。評価の前に決めておく値で、3 回の試行のばらつき程度より大きくする。
# 割合の指標 (RATE_LABELS) は、基準より増えたら悪くなったとする
TOLERANCES: Dict[str, Tuple[float, float]] = {
    "yaw 誤差 p95[rad]": (0.05, 0.2),
    "整合後の位置誤差 p95[m]": (0.3, 0.2),
    "GNSS が弱い時間の位置誤差 中央値[m]": (0.3, 0.2),
    "止める要求[回]": (0.0, 0.0),
}
WORSE_MARK = " ⚠"


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


def is_worse(title: str, value: Optional[float], base: Optional[float]) -> bool:
    """value が基準 base より、許容差を超えて悪いか。比べられなければ False。"""
    if value is None or base is None:
        return False
    if title in RATE_LABELS:
        return value > base + 1e-9
    if title not in TOLERANCES:
        return False
    absolute, relative = TOLERANCES[title]
    return value > base + max(absolute, relative * abs(base)) + 1e-9


def sensitivity_rate(runs_list: List[List[Dict[str, Any]]], yaw_deg: float, position_m: float) -> Optional[float]:
    """閾値の組 (yaw_deg, position_m) で、ずれた事象が 1 件以上あった試行の割合 (条件をまたいで合わせる)。"""
    hits, total = 0, 0
    for runs in runs_list:
        for run in runs:
            for entry in run.get("deviations", {}).get("sensitivity", []):
                if entry["yaw_deg"] == yaw_deg and entry["position_m"] == position_m:
                    hits += 1 if entry["n"] > 0 else 0
                    total += 1
    return hits / total if total else None


def render(
    entries: List[Tuple[str, str]], names: List[str], results: Dict[Tuple[str, str], List[Dict[str, Any]]],
    baseline: Optional[str] = None,
) -> str:
    header = "| 条件 | " + " | ".join(names) + " |"
    rule = "|---|" + "---|" * len(names)
    lines = ["# 自己位置推定 評価セットの比較", "",
             "各セルは 試行の中央値 (最小–最大)。括弧の前の値が小さいほど良い (NEES は 3 に近いほど良い)。",
             "ずれた試行 = 推定が真値から大きくずれた事象が 1 件以上あった試行 (eval_localization.py の deviations)。"
             "試行の無い条件は -。", ""]
    if baseline:
        lines += [f"{WORSE_MARK.strip()} = 基準 `{baseline}` より許容差を超えて悪い "
                  "(割合は基準より多い。誤差は 基準 + max(絶対量, 割合 × 基準) を超える。"
                  "許容差は compare_localization_set.py の TOLERANCES)。", ""]
    lines += ["| 条件 | " + " | ".join(f"{n} 試行数" for n in names) + " |", rule]
    for label, _ in entries:
        lines.append(f"| {label} | " + " | ".join(str(len(results[(label, n)])) for n in names) + " |")
    worse: List[Tuple[str, str, str]] = []

    def cell(title: str, label: str, n: str, agg: Optional[Tuple[float, float, float]], digits: int) -> str:
        text = format_cell(agg, digits)
        if baseline and n != baseline and agg is not None:
            base = table[(label, baseline)]
            if is_worse(title, agg[0], None if base is None else base[0]):
                worse.append((n, label, title))
                return text + WORSE_MARK
        return text

    for title in RATE_LABELS:
        table = {(label, n): summarize_variant(results[(label, n)])[title] if results[(label, n)] else None
                 for label, _ in entries for n in names}
        lines += ["", f"## {title}", "", header, rule]
        for label, _ in entries:
            lines.append(f"| {label} | " + " | ".join(
                cell(title, label, n, table[(label, n)], 2) for n in names) + " |")
        totals = [pooled_rate([results[(label, n)] for label, _ in entries], title) for n in names]
        lines.append("| **全条件** | " + " | ".join("-" if v is None else f"{v:.2f}" for v in totals) + " |")
    for title, path, digits in ERROR_METRICS:
        table = {(label, n): aggregate([v for v in (get_path(run, path) for run in results[(label, n)])
                                        if v is not None])
                 for label, _ in entries for n in names}
        lines += ["", f"## {title}", "", header, rule]
        means: Dict[str, List[float]] = {n: [] for n in names}
        for label, _ in entries:
            lines.append(f"| {label} | " + " | ".join(
                cell(title, label, n, table[(label, n)], digits) for n in names) + " |")
            for n in names:
                if table[(label, n)] is not None:
                    means[n].append(table[(label, n)][0])
        lines.append("| **全条件の平均** | " + " | ".join(
            f"{sum(means[n]) / len(means[n]):.{digits}f}" if means[n] else "-" for n in names) + " |")

    combos = sorted({(e["yaw_deg"], e["position_m"]) for runs in results.values() for run in runs
                     for e in run.get("deviations", {}).get("sensitivity", [])})
    if combos:
        lines += ["", "## 閾値を変えたときの、ずれた試行の割合 (全条件)", "", "| yaw / 位置 | " + " | ".join(names) + " |",
                  rule]
        for yaw_deg, position_m in combos:
            rates = [sensitivity_rate([results[(label, n)] for label, _ in entries], yaw_deg, position_m)
                     for n in names]
            lines.append(f"| {yaw_deg:g}° / {position_m:g} m | "
                         + " | ".join("-" if v is None else f"{v:.2f}" for v in rates) + " |")
    if baseline:
        lines += ["", f"## 基準 `{baseline}` より許容差を超えて悪い条件", ""]
        lines += [f"- {n}: {label} の {title}" for n, label, title in worse] or ["- なし"]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="評価セットの結果を名前ごとに比べる")
    parser.add_argument("names", nargs="+", help="run_localization_set.sh に渡した名前")
    parser.add_argument("--set", default="robustness", help="評価セット (sets/<名前>.txt またはパス)")
    parser.add_argument("--baseline", default=None, help="許容差の判定で比べる基準の名前 (names の 1 つ)")
    parser.add_argument("-o", "--output", default=None, help="Markdown の出力先 (省略時は標準出力のみ)")
    args = parser.parse_args()

    if args.baseline and args.baseline not in args.names:
        raise SystemExit(f"エラー: --baseline {args.baseline} が名前の並びにありません")
    entries = read_set(args.set)
    results = collect(entries, args.names)
    text = render(entries, args.names, results, args.baseline)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
