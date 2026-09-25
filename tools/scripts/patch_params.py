#!/usr/bin/env python3
"""
patch_params.py

コメント付き ROS2 パラメータ YAML の値を、コメントやレイアウトを保ったまま書き換えるツール。
ノード配下の `ros__parameters` 内のキーを、ドット区切りのパスで指定する。
`ノード名:パス=値` の形でノードを指定できる (複数ノードを含む YAML で、同じキーが複数のノードにある場合に使う)。
ノード名を省略すると、最初に見つかったノードのキーを書き換える。
書き換えられるのは 1 行に書かれた値 (スカラー) だけで、複数行にまたがるリストは対象外。

例:
  patch_params.py in.yaml out.yaml deskew.enabled=true deskew.direction=-1
  patch_params.py nav2_params.yaml out.yaml amcl:max_particles=2000 amcl:max_beams=240
"""

import argparse
import re
import sys
from typing import Dict, List, Optional, Tuple

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


def split_spec(spec: str) -> Tuple[Optional[str], str]:
    """`ノード名:パス` を (ノード名, パス) に分ける。ノード名が無ければ (None, パス)。"""
    if ":" in spec:
        node, path = spec.split(":", 1)
        return node, path
    return None, spec


def patch_lines(lines: List[str], updates: Dict[str, str]) -> Tuple[List[str], List[str]]:
    """updates の各キーを最初にマッチした行で書き換え、(新しい行, 未適用キー) を返す。

    updates のキーは `パス` または `ノード名:パス`。
    """
    remaining = dict(updates)
    stack: List[Tuple[int, str]] = []  # (インデント, キー)
    in_params = False
    current_node = ""
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
            current_node = key
            stack = [(indent, key)]
            out.append(line)
            continue

        stack.append((indent, key))
        # ノード名 (トップレベル) と ros__parameters を除いたキーの連なりがパス
        path = ".".join(k for i, (_, k) in enumerate(stack) if i > 0 and k != "ros__parameters")
        spec = next(
            (s for s in remaining
             if in_params and split_spec(s)[1] == path and split_spec(s)[0] in (None, current_node)),
            None)
        if spec is not None:
            # VALUE_AND_COMMENT はどんな文字列にもマッチする
            old_value, comment = VALUE_AND_COMMENT.match(rest).groups(default="")
            new_value = coerce_like(old_value, remaining.pop(spec))
            out.append(f"{match.group(1)}{key}: {new_value}{comment}\n")
        else:
            out.append(line)

    return out, sorted(remaining)


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 パラメータ YAML の値を書き換える")
    parser.add_argument("input", help="入力 YAML")
    parser.add_argument("output", help="出力 YAML")
    parser.add_argument(
        "updates", nargs="*", help="key.path=value (ドット区切り)。ノード指定は node:key.path=value")
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
