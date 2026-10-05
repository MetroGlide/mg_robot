#!/usr/bin/env python3
"""
diff_bag_list.py

記録対象トピック一覧ファイルと、現在実行中の ROS2 システム上のトピック一覧 (ros2 topic list) を比較し、
差分（記録対象のみ、現在のトピックのみ、両方に存在）をターミナルに色付き表示するツール。
"""

import argparse
import subprocess
from typing import List


def get_topic_list() -> List[str]:
    """現在アクティブな ROS2 トピック名一覧を取得する。"""
    try:
        output = subprocess.check_output(["ros2", "topic", "list"]).decode()
        return [line.strip() for line in output.splitlines() if line.strip()]
    except Exception:
        return []


def get_record_topic_list(record_topic_list_file: str) -> List[str]:
    """ファイルから記録対象トピック名一覧を読み込む。"""
    try:
        with open(record_topic_list_file, "r", encoding="utf-8") as f:
            return [line.strip() for line in f.read().splitlines() if line.strip()]
    except Exception:
        return []


def main():
    parser = argparse.ArgumentParser(
        description="記録対象トピック一覧ファイルと現在アクティブなトピックを比較するツール"
    )
    parser.add_argument("record_topic_list_file", help="記録対象トピック一覧ファイルパス")
    args = parser.parse_args()

    record_topic_list = get_record_topic_list(args.record_topic_list_file)
    current_topic_list = get_topic_list()

    record_only_topic_list = [
        t for t in record_topic_list if t not in current_topic_list
    ]
    current_only_topic_list = [
        t for t in current_topic_list if t not in record_topic_list
    ]
    both_topic_list = [
        t for t in record_topic_list if t in current_topic_list
    ]

    for topic in record_only_topic_list:
        print(f"{topic} \033[33m(only RECORD LIST)\033[0m")

    for topic in current_only_topic_list:
        print(f"{topic} \033[33m(only CURRENT TOPIC)\033[0m")

    for topic in both_topic_list:
        print(topic)


if __name__ == "__main__":
    main()
