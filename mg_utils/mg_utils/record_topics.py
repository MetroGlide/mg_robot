"""rosbag に記録するトピックのリストの読み込みと結合。ROS に依存しない。

記録の入口 (mg_utils/launch/record_bag.launch.py) が、パッケージごとのリストに、
状態監視とイベントのリスト (params/record_topics_monitoring.txt) を加えるのに使う。
tools/scripts/record.sh は、同じファイルを bash で読む。
"""
from typing import Iterable, List


def read_topic_file(path: str) -> List[str]:
    """1 行 1 トピックのファイルを読む。空行と `#` で始まる行は除く。"""
    with open(path, encoding='utf-8') as f:
        lines = [line.strip() for line in f]
    return [line for line in lines if line and not line.startswith('#')]


def merge_topics(*topic_lists: Iterable[str]) -> List[str]:
    """リストを順に結合する。重複は、最初のものだけ残す。"""
    seen = set()
    merged = []
    for topics in topic_lists:
        for topic in topics:
            if topic not in seen:
                seen.add(topic)
                merged.append(topic)
    return merged
