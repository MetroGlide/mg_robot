"""CLI 引数処理およびパス解決に関する共通ユーティリティモジュール。"""

import argparse
import os
from typing import Optional


def add_output_args(parser: argparse.ArgumentParser, default_filename: str):
    """共通の出力先オプション (--output, --output-to-bag-dir, --output-dir) をパーサーに追加する。"""
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=default_filename,
        help=f"出力ファイルパス (デフォルト: {default_filename})",
    )
    parser.add_argument(
        "--output-to-bag-dir",
        action="store_true",
        help="対象 rosbag と同じディレクトリに出力ファイルを保存",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="指定したディレクトリに出力ファイルを保存",
    )


def resolve_output_path(
    bag_path: str,
    default_filename: str,
    output: Optional[str] = None,
    output_to_bag_dir: bool = False,
    output_dir: Optional[str] = None,
) -> str:
    """出力先ファイルの絶対パスを決定し、親ディレクトリが存在しない場合は作成する。

    優先度:
      1. output_to_bag_dir が真: 対象 bag ディレクトリ配下の default_filename
      2. output_dir が指定されている: output_dir 配下の default_filename (または output のファイル名)
      3. output が指定されている: output で指定されたファイルパス
      4. 未指定: カレントディレクトリ配下の default_filename
    """
    filename = os.path.basename(output) if output else default_filename

    if output_to_bag_dir:
        target_dir = bag_path if os.path.isdir(bag_path) else os.path.dirname(bag_path)
        output_path = os.path.abspath(os.path.join(target_dir, filename))
    elif output_dir:
        output_path = os.path.abspath(os.path.join(output_dir, filename))
    elif output:
        output_path = os.path.abspath(output)
    else:
        output_path = os.path.abspath(default_filename)

    target_parent = os.path.dirname(output_path)
    if target_parent:
        os.makedirs(target_parent, exist_ok=True)

    return output_path
