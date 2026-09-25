#!/usr/bin/env python3
"""
loc_dataset.py

自己位置推定の再生評価のデータセット定義 (tools/datasets/localization/*.yaml) を読み込み、
run_localization_variant.sh が使うシェル変数の代入文を標準出力に出す。

使い方:
  eval "$(python3 tools/scripts/loc_dataset.py <名前 または YAML パス>)"

出力する変数:
  DATASET_NAME, MAP_YAML, GNSS_TRANSFORM, MAP_GT_DIR, BAG, GT_DIR, START_OFFSET, DURATION
"""

import argparse
import os
import shlex
from typing import Dict

import yaml

DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "datasets", "localization")
REQUIRED = {
    "map": ["localization_yaml", "gnss_transform", "slam_dir"],
    "eval": ["bag", "slam_dir"],
}


def resolve_dataset_path(name_or_path: str) -> str:
    if os.path.isfile(name_or_path):
        return name_or_path
    candidate = os.path.join(DATASET_DIR, name_or_path + ".yaml")
    if os.path.isfile(candidate):
        return candidate
    raise SystemExit(f"エラー: データセットが見つかりません: {name_or_path}")


def load_dataset(path: str) -> Dict[str, str]:
    """データセット YAML を検証し、シェル変数名から値への辞書にして返す。"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    for section, keys in REQUIRED.items():
        for key in keys:
            if key not in (data.get(section) or {}):
                raise SystemExit(f"エラー: {path} に {section}.{key} がありません。")
    return {
        "DATASET_NAME": data["name"],
        "MAP_YAML": data["map"]["localization_yaml"],
        "GNSS_TRANSFORM": data["map"]["gnss_transform"],
        "MAP_GT_DIR": data["map"]["slam_dir"] if data["eval"]["slam_dir"] != data["map"]["slam_dir"] else "",
        "BAG": data["eval"]["bag"],
        "GT_DIR": data["eval"]["slam_dir"],
        "START_OFFSET": str(data["eval"].get("start_offset", 0.0)),
        "DURATION": str(data["eval"].get("duration", 0.0)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="データセット定義をシェル変数として出力する")
    parser.add_argument("dataset", help="tools/datasets/localization/ 配下の名前、または YAML のパス")
    args = parser.parse_args()
    variables = load_dataset(resolve_dataset_path(args.dataset))
    for key, value in variables.items():
        print(f"{key}={shlex.quote(value)}")


if __name__ == "__main__":
    main()
