#!/usr/bin/env python3
"""
compare_localization.py

再生評価 (run_localization_variant.sh) の出力を集め、変種ごとの主要な指標を比較表にする。
各変種ディレクトリの run_*/eval_localization.json を読み、複数回の試行を 平均 (最小〜最大) にまとめる。
先頭に指定した変種を基準 (ベースライン) として表の先頭に置く。

使い方:
  compare_localization.py <変種ディレクトリ> [<変種ディレクトリ> ...] [-o summary.md]
  例: compare_localization.py <bag>/eval_loc/<データセット>/baseline <bag>/eval_loc/<データセット>/new
"""

import argparse
import glob
import json
import os
from typing import Any, Dict, List, Optional, Tuple

# (表の見出し, JSON 内のパス, 小数点以下の桁数)。値は小さいほど良い指標
METRICS: List[Tuple[str, Tuple[str, ...], int]] = [
    ("位置誤差 中央値[m]", ("accuracy", "position_m", "median"), 3),
    ("位置誤差 p95[m]", ("accuracy", "position_m", "p95"), 3),
    ("位置誤差 最大[m]", ("accuracy", "position_m", "max"), 2),
    ("整合後の位置誤差 p95[m]", ("accuracy", "aligned", "position_m", "p95"), 3),
    ("yaw 誤差 p95[rad]", ("accuracy", "yaw_rad", "p95"), 3),
    ("位置の飛び p99[m]", ("smoothness", "jump_position_m", "p99"), 3),
    ("yaw の飛び p99[rad]", ("smoothness", "jump_yaw_rad", "p99"), 3),
    ("yaw の飛び 最大[rad]", ("smoothness", "jump_yaw_rad", "max"), 3),
    ("AMCL 遅延 中央値[s]", ("amcl", "latency_s", "median"), 3),
    ("誤検知[回]", ("faults", "status", "false_detections"), 0),
]
# NEES は 3 (自由度) に近いほど良いので、別の列として扱う
CONSISTENCY_METRICS: List[Tuple[str, Tuple[str, ...], int]] = [
    ("NEES 平均 (理想 3)", ("nees", "mean"), 2),
    ("NEES 7.8 以下の割合 (理想 0.95)", ("nees", "fraction_within_chi2_95"), 2),
]


def get_path(data: Dict[str, Any], path: Tuple[str, ...]) -> Optional[float]:
    """入れ子の辞書からパスの値を取り出す。無い、または数値でなければ None。"""
    node: Any = data
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    if isinstance(node, bool) or not isinstance(node, (int, float)):
        return None
    return float(node)


def load_runs(variant_dir: str) -> List[Dict[str, Any]]:
    runs = []
    for path in sorted(glob.glob(os.path.join(variant_dir, "run_*", "eval_localization.json"))):
        with open(path, "r", encoding="utf-8") as f:
            runs.append(json.load(f))
    return runs


def aggregate(values: List[float]) -> Optional[Tuple[float, float, float]]:
    """(平均, 最小, 最大)。値が無ければ None。"""
    if not values:
        return None
    return sum(values) / len(values), min(values), max(values)


FAILURE_LABEL = "破綻した試行の割合"
# 位置誤差の p95 がこれを超えた試行を、破綻 (AMCL や EKF が大きくずれた) とみなす [m]
FAILURE_P95_M = 3.0


def failure_rate(runs: List[Dict[str, Any]], threshold: float = FAILURE_P95_M) -> Optional[float]:
    """位置誤差の p95 が threshold を超えた試行の割合。真値が無く判定できなければ None。"""
    p95 = [v for v in (get_path(run, ("accuracy", "position_m", "p95")) for run in runs) if v is not None]
    if not p95:
        return None
    return sum(1 for v in p95 if v > threshold) / len(p95)


def summarize_variant(runs: List[Dict[str, Any]]) -> Dict[str, Optional[Tuple[float, float, float]]]:
    result: Dict[str, Optional[Tuple[float, float, float]]] = {}
    rate = failure_rate(runs)
    result[FAILURE_LABEL] = None if rate is None else (rate, rate, rate)
    for label, path, _ in METRICS + CONSISTENCY_METRICS:
        values = [v for v in (get_path(run, path) for run in runs) if v is not None]
        result[label] = aggregate(values)
    return result


def format_cell(agg: Optional[Tuple[float, float, float]], digits: int) -> str:
    if agg is None:
        return "-"
    mean, low, high = agg
    if low == high:
        return f"{mean:.{digits}f}"
    return f"{mean:.{digits}f} ({low:.{digits}f}–{high:.{digits}f})"


def render(variants: List[Tuple[str, int, Dict[str, Optional[Tuple[float, float, float]]]]]) -> str:
    """変種ごとの集計を Markdown の表にする。"""
    columns = METRICS + CONSISTENCY_METRICS
    lines = ["# 自己位置推定 変種の比較", "",
             "各セルは 試行の平均 (最小–最大)。位置・yaw・飛びは小さいほど良い。",
             f"破綻した試行 = 位置誤差の p95 が {FAILURE_P95_M:g} m を超えた試行。", "",
             f"| 変種 | 試行数 | {FAILURE_LABEL} | " + " | ".join(label for label, _, _ in columns) + " |",
             "|---|---|---|" + "---|" * len(columns)]
    for name, n_runs, summary in variants:
        cells = [format_cell(summary[label], digits) for label, _, digits in columns]
        lines.append(f"| {name} | {n_runs} | {format_cell(summary[FAILURE_LABEL], 2)} | " + " | ".join(cells) + " |")
    lines += ["", f"基準 (先頭): `{variants[0][0]}`" if variants else ""]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="自己位置推定の変種を比較表にする")
    parser.add_argument("variant_dirs", nargs="+", help="run_*/eval_localization.json を含む変種ディレクトリ")
    parser.add_argument("-o", "--output", default=None, help="Markdown の出力先 (省略時は標準出力のみ)")
    args = parser.parse_args()

    variants = []
    for variant_dir in args.variant_dirs:
        runs = load_runs(variant_dir)
        if not runs:
            raise SystemExit(f"エラー: {variant_dir} に run_*/eval_localization.json がありません。")
        variants.append((os.path.basename(os.path.normpath(variant_dir)), len(runs), summarize_variant(runs)))

    text = render(variants)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"保存: {args.output}")


if __name__ == "__main__":
    main()
