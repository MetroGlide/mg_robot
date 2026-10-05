#!/usr/bin/env python3
"""
rosbag_modify_base.py

rosbag 内の特定トピックのメッセージ内容（例: Odometry共分散の付与、LaserScan frame_idの変更など）を
書き換えて新しい rosbag に保存する変換ツール。
"""

import argparse
import os
import shutil
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import (  # noqa: E402
    MessageDeserializer,
    detect_storage_id,
    open_reader,
    open_writer,
)


def add_covariance_to_odom(msg: Odometry):
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    msg.pose.covariance = [
        6.5, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 6.5, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.4,
    ]
    msg.twist.covariance = [
        0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.78,
    ]
    return msg


def change_frame_id_in_laser_scan(msg: LaserScan):
    msg.header.frame_id = "front_lrf_link"
    return msg


MODIFY_FUNC_DICT = {
    "/odom": [add_covariance_to_odom],
    "/scan_front_lidar": [change_frame_id_in_laser_scan]
}


def rename_to_backup(path: str):
    output_path = path + "_bak"
    if os.path.exists(output_path):
        rename_to_backup(output_path)
    shutil.move(path, output_path)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        help="input bag file path",
        default="",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="output bag file path",
        default="",
    )
    parser.add_argument(
        "-F",
        "--force",
        action="store_true",
        help="force overwrite output file",
    )

    args = parser.parse_args()
    if args.input == "":
        raise ValueError("input file is not specified")
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"input file {args.input} not found")
    if not os.path.isfile(args.input):
        raise ValueError(f"input file {args.input} is not file")

    print(f"input file: {args.input}")

    if args.output == "":
        if not args.force:
            input_path = os.path.abspath(os.path.join(os.path.dirname(args.input), ".."))
            input_file_name = os.path.splitext(os.path.basename(args.input))[0]
            args.output = os.path.join(input_path, input_file_name + "_modified")
            print(f"output file is not specified. use {args.output}")

    if os.path.exists(args.output):
        print(f"output file {args.output} already exists")
        if args.force:
            p = rename_to_backup(args.output)
            print(f"backup output file {args.output} -> {p}")
        else:
            raise FileExistsError(f"output file {args.output} already exists")

    storage_id = detect_storage_id(args.input)
    reader = open_reader(args.input, storage_id=storage_id)
    topic_meta_list = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_meta_list}

    writer = open_writer(args.output, storage_id=storage_id)
    for topic_meta in topic_meta_list:
        writer.create_topic(topic_meta)

    deserializer = MessageDeserializer(type_map)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserializer.deserialize(topic, data)
        if topic in MODIFY_FUNC_DICT:
            for func in MODIFY_FUNC_DICT[topic]:
                msg = func(msg)
        writer.write(topic, serialize_message(msg), timestamp)

    del reader
    del writer

    print("DONE")
    print(f"output file: {args.output}")


if __name__ == "__main__":
    main()
