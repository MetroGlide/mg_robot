#!/usr/bin/env python3
"""
bag_to_json.py

rosbag2 (MCAP / SQLite3) の内容を JSON に変換してダンプするツール。
"""

import argparse
import json
import os
import sys
from collections import defaultdict
from typing import List, Optional

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import MessageDeserializer, message_to_dict, open_reader  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description="rosbag2の内容をjsonに変換")
    parser.add_argument(
        "--bag_path",
        required=True,
        help="rosbag2のディレクトリまたはファイルパス",
    )
    parser.add_argument(
        "--output_dir",
        default=".",
        help="出力ディレクトリパス (デフォルト: カレントディレクトリ)",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=None,
        help="抽出するトピック名のリスト（スペース区切りで指定）",
    )
    return parser.parse_args()


def read_rosbag2(bag_path: str, filter_topics: Optional[List[str]] = None):
    reader = open_reader(bag_path, topics=filter_topics)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    deserializer = MessageDeserializer(type_map)
    topic_msgs = defaultdict(list)
    all_msgs = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if filter_topics is not None and topic not in filter_topics:
            continue
        msg_type = type_map.get(topic)
        if not msg_type:
            continue

        msg = deserializer.deserialize(topic, data)
        msg_dict = message_to_dict(msg)
        entry = {"topic": topic, "timestamp": t, "msg": msg_dict}
        topic_msgs[topic].append(entry)
        all_msgs.append(entry)

    return topic_msgs, all_msgs


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    topic_msgs, all_msgs = read_rosbag2(args.bag_path, filter_topics=args.topics)

    topics_json = {}
    for topic, msgs in topic_msgs.items():
        topics_json[topic] = [
            {"timestamp": m["timestamp"], "msg": m["msg"]} for m in msgs
        ]

    all_msgs_sorted = sorted(all_msgs, key=lambda x: x["timestamp"])
    timeline_json = [
        {"topic": m["topic"], "timestamp": m["timestamp"], "msg": m["msg"]}
        for m in all_msgs_sorted
    ]

    result = {"by_topic": topics_json, "timeline": timeline_json}
    out_path = os.path.join(args.output_dir, "rosbag2_summary.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2, default=str)
    print(f"書き出し完了: {out_path}")


if __name__ == "__main__":
    main()
