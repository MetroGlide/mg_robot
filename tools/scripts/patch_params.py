#!/usr/bin/env python3
"""
patch_params.py

コメント付き ROS2 パラメータ YAML の値を、コメントやレイアウトを保ったまま書き換えるツール。
`/**` ノード配下の `ros__parameters` 内のキーを、ドット区切りのパスで指定する。

例:
  patch_params.py in.yaml out.yaml deskew.enabled=true deskew.direction=-1
"""

import argparse
import re
import sys
from typing import Dict, List, Tuple

KEY_LINE = re.compile(r"^(\s*)([A-Za-z0-9_/*]+):(.*)$")
VALUE_AND_COMMENT = re.compile(r"^\s*([^#]*?)(\s+#.*)?\s*$")


def coerce_like(old_value: str, new_value: str) -> str:
    """元の値が小数 (double パラメータ) なら、整数表記の新しい値に ".0" を補って型を合わせる。

    ROS2 のパラメータは型が厳密で、double に整数値を渡すと起動時に例外になるため。
    """
    old_is_float = re.fullmatch(r"[-+]?\d*\.\d+([eE][-+]?\d+)?|[-+]?\d+[eE][-+]?\d+", old_value.strip())
    if old_is_float and re.fullmatch(r"[-+]?\d+", new_value.strip()):
        return new_value.strip() + ".0"
    return new_value


def patch_lines(lines: List[str], updates: Dict[str, str]) -> Tuple[List[str], List[str]]:
    """updates の各キーを最初にマッチした行で書き換え、(新しい行, 未適用キー) を返す。"""
    remaining = dict(updates)
    stack: List[Tuple[int, str]] = []  # (インデント, キー)
    in_params = False
    out: List[str] = []

    for line in lines:
        match = KEY_LINE.match(line.rstrip("\n"))
        if not match:
            out.append(line)
            continue

        indent = len(match.group(1))
        key = match.group(2)
        rest = match.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()

        if key == "ros__parameters":
            in_params = True
            stack.append((indent, key))
            out.append(line)
            continue
        if indent == 0:
            # トップレベルのノード名 (例: /**) の新しいブロックに入る
            in_params = False
            stack = [(indent, key)]
            out.append(line)
            continue

        stack.append((indent, key))
        # ノード名 (トップレベル) と ros__parameters を除いたキーの連なりがパス
        path = ".".join(k for i, (_, k) in enumerate(stack) if i > 0 and k != "ros__parameters")
        if in_params and path in remaining:
            value_match = VALUE_AND_COMMENT.match(rest)
            comment = value_match.group(2) or "" if value_match else ""
            old_value = value_match.group(1) if value_match else ""
            new_value = coerce_like(old_value, remaining.pop(path))
            out.append(f"{match.group(1)}{key}: {new_value}{comment}\n")
        else:
            out.append(line)

    return out, sorted(remaining)


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 パラメータ YAML の値を書き換える")
    parser.add_argument("input", help="入力 YAML")
    parser.add_argument("output", help="出力 YAML")
    parser.add_argument("updates", nargs="*", help="key.path=value (ドット区切り)")
    args = parser.parse_args()

    updates: Dict[str, str] = {}
    for item in args.updates:
        if "=" not in item:
            parser.error(f"key.path=value の形式で指定してください: {item}")
        key, value = item.split("=", 1)
        updates[key] = value

    with open(args.input, "r", encoding="utf-8") as f:
        lines = f.readlines()
    patched, unapplied = patch_lines(lines, updates)
    if unapplied:
        print(f"エラー: 次のキーが見つかりませんでした: {', '.join(unapplied)}", file=sys.stderr)
        sys.exit(1)
    with open(args.output, "w", encoding="utf-8") as f:
        f.writelines(patched)


if __name__ == "__main__":
    main()
