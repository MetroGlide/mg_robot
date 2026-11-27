#!/usr/bin/env python3
"""
move_wptroj_groups.py

waypoint-tool のプロジェクト (.wptroj) のグループを、地図の原点の周りの剛体で動かし、別名で保存する。
waypoint-tool の画面の「グループの位置調整」と同じ処理 (tools/common/map_align.py の move_wptroj_group)。
動かす量は、measure_gnss_map_offset.py analyze の報告の「直す動き」を使う。
ウェイポイントは動かさない。出力の地図は、保存したプロジェクトを waypoint-tool で開いてエクスポートする。
"""

import argparse
import json
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.map_align import move_wptroj_group  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("src", help="元のプロジェクト (.wptroj)。書き換えない")
    parser.add_argument("dst", help="保存先 (.wptroj)。既にあれば止める")
    parser.add_argument("--move", nargs=4, action="append", required=True,
                        metavar=("GROUP", "DX", "DY", "DYAW_DEG"),
                        help="グループを R(DYAW)·p + (DX, DY) で動かす [m, deg]。繰り返し指定できる")
    args = parser.parse_args()

    if os.path.abspath(args.src) == os.path.abspath(args.dst):
        raise SystemExit("エラー: 保存先は元のプロジェクトと別にしてください。")
    if os.path.exists(args.dst):
        raise SystemExit(f"エラー: 保存先 {args.dst} は既にあります。")
    with open(args.src, "r", encoding="utf-8") as f:
        project = json.load(f)
    for name, dx, dy, dyaw_deg in args.move:
        try:
            moved = move_wptroj_group(project, name, float(dx), float(dy), math.radians(float(dyaw_deg)))
        except ValueError as e:
            raise SystemExit(f"エラー: {e}")
        print(f"{name}: ({float(dx):+.3f}, {float(dy):+.3f}) m, {float(dyaw_deg):+.3f} deg を当てた "
              f"(レイヤー: {', '.join(moved)})")
    with open(args.dst, "x", encoding="utf-8") as f:
        json.dump(project, f, ensure_ascii=False, indent=2)
    print(f"保存: {args.dst}")


if __name__ == "__main__":
    main()
