#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/rosbag_summary.py"

if [ $# -eq 0 ]; then
    echo "使用方法: $(basename "$0") <bag_path_or_dir> [オプション...]"
    echo ""
    echo "rosbag_summary.py のラッパースクリプトです。"
    echo "指定された rosbag またはディレクトリに対して詳細統計サマリーを出力します。"
    echo ""
    python3 "$PYTHON_SCRIPT" -h
    exit 0
fi

python3 "$PYTHON_SCRIPT" "$@"
