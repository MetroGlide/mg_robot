#!/usr/bin/env python3
"""
event_timeline.py

イベントログ (evt=<domain>.<event> key=value ...) を、コンテナのログと rosbag から集めて、時刻順の表にする。
書式は doc/monitoring.md。

入力:
  --logs <ディレクトリ...>  make log-collector が書き出したログ (<サービス>/<日付>.log) と、
                            make logs-export のログ (<日時>/<サービス>.log)。既定: ~/ros2_data/logs
  --bag <rosbag>            rosbag の /events と /rosout (どちらも rcl_interfaces/Log)
"""
import argparse
import os
import re
import sys
from typing import List

# tools パッケージルートと、mg_utils (イベントの書式を読む) の解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for path in (REPO_ROOT, os.path.join(REPO_ROOT, "mg_utils")):
    if path not in sys.path:
        sys.path.insert(0, path)

from tools.common import event_timeline as et  # noqa: E402
from tools.common.bag import MessageDeserializer, open_reader  # noqa: E402

DEFAULT_LOG_DIR = os.path.expanduser("~/ros2_data/logs")
BAG_TOPICS = ["/events", "/rosout"]


def parse_args():
    parser = argparse.ArgumentParser(description="イベントログを時刻順の表にする")
    parser.add_argument("--logs", nargs="*", default=None,
                        help=f"ログのディレクトリ (既定: {DEFAULT_LOG_DIR})。--bag だけで使うときは省略できる")
    parser.add_argument("--bag", default=None, help="rosbag のパス (/events と /rosout を読む)")
    parser.add_argument("--since", default=None, help="この時刻以降 (ISO 8601。タイムゾーンが無ければ UTC)")
    parser.add_argument("--until", default=None, help="この時刻まで (ISO 8601)")
    parser.add_argument("--level", choices=et.LEVELS, default=None, help="このレベル以上")
    parser.add_argument("--domain", nargs="*", default=None, help="ドメイン (例: driver watchdog)")
    parser.add_argument("--service", nargs="*", default=None, help="ソース (サービス名または bag)")
    parser.add_argument("--tz", default=None, help="表示のタイムゾーン (UTC、+09:00、Asia/Tokyo。既定: 実行環境のローカル)")
    parser.add_argument("--format", choices=["md", "csv"], default="md", help="出力形式")
    parser.add_argument("-o", "--output", default=None, help="出力先のファイル (既定: 標準出力)")
    args = parser.parse_args()
    if args.logs is None and args.bag is None:
        args.logs = [DEFAULT_LOG_DIR]
    return args


_DATE_RE = re.compile(r"\d{4}-\d{2}-\d{2}")


def _source_of(path: str) -> str:
    """ログのファイルから、ソース (サービス名) を決める。

    <サービス>/<YYYY-MM-DD>.log (make log-collector) ならディレクトリ名、
    <日時>/<サービス>.log (make logs-export) ならファイル名。
    """
    stem = os.path.splitext(os.path.basename(path))[0]
    if _DATE_RE.fullmatch(stem):
        return os.path.basename(os.path.dirname(path))
    return stem


def read_logs(directories: List[str]) -> List[et.EventRecord]:
    records = []
    for directory in directories:
        for dirpath, _, filenames in os.walk(directory):
            for filename in sorted(filenames):
                if not filename.endswith(".log"):
                    continue
                path = os.path.join(dirpath, filename)
                with open(path, encoding="utf-8", errors="replace") as f:
                    records.extend(et.parse_lines(f, _source_of(path)))
    return records


def read_bag(bag_path: str) -> List[et.EventRecord]:
    reader = open_reader(bag_path, topics=BAG_TOPICS)
    deserializer = MessageDeserializer({t.name: t.type for t in reader.get_all_topics_and_types()})
    records = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserializer.deserialize(topic, data)
        record = et.parse_ros_log(
            msg.stamp.sec + msg.stamp.nanosec * 1e-9, int(msg.level), msg.name, msg.msg, "bag")
        if record is not None:
            records.append(record)
    return records


def main():
    args = parse_args()
    records = []
    if args.logs:
        records.extend(read_logs(args.logs))
    if args.bag:
        records.extend(read_bag(args.bag))

    selected = et.filter_records(
        et.merge(records),
        since=et.parse_iso(args.since) if args.since else None,
        until=et.parse_iso(args.until) if args.until else None,
        min_level=args.level, domains=args.domain, sources=args.service)
    tz = et.resolve_tz(args.tz)
    text = et.to_csv(selected, tz) if args.format == "csv" else et.to_markdown(selected, tz)

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"{len(selected)} 件のイベントを {args.output} に書き出しました", file=sys.stderr)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
