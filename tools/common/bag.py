"""rosbag 操作に関する共通ユーティリティモジュール。"""

import os
from typing import Any, Dict, List, Optional, Tuple, Type

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def detect_storage_id(bag_path: str) -> str:
    """rosbagのパスから storage_id ('mcap' または 'sqlite3') を自動判定する。"""
    if os.path.isdir(bag_path):
        meta_path = os.path.join(bag_path, "metadata.yaml")
        if os.path.exists(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    meta = yaml.safe_load(f)
                info = meta.get("rosbag2_bagfile_information", {})
                detected = info.get("storage_identifier")
                if detected:
                    return detected
            except Exception:
                pass
        for root, _, files in os.walk(bag_path):
            for file in files:
                if file.endswith(".mcap"):
                    return "mcap"
                if file.endswith(".db3"):
                    return "sqlite3"
    elif os.path.isfile(bag_path):
        if bag_path.endswith(".mcap"):
            return "mcap"
        if bag_path.endswith(".db3"):
            return "sqlite3"
    return "mcap"


def load_metadata(bag_path: str) -> Optional[Dict[str, Any]]:
    """rosbag の metadata.yaml が存在すればロードして返却する。"""
    meta_path = (
        os.path.join(bag_path, "metadata.yaml")
        if os.path.isdir(bag_path)
        else os.path.join(os.path.dirname(bag_path), "metadata.yaml")
    )
    if os.path.exists(meta_path):
        try:
            with open(meta_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f)
        except Exception:
            return None
    return None


def open_reader(
    bag_path: str,
    storage_id: Optional[str] = None,
    topics: Optional[List[str]] = None,
) -> rosbag2_py.SequentialReader:
    """rosbagリーダーを初期化してオープンし、リーダーを返却する。

    Args:
        bag_path: rosbag のディレクトリまたはファイルパス
        storage_id: ストレージ形式 ('mcap' / 'sqlite3')。未指定時は自動判定
        topics: 読み込み対象のトピック名リスト (フィルタリング)。未指定時は全トピック

    Returns:
        オープン済みの SequentialReader インスタンス
    """
    sid = storage_id or detect_storage_id(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=sid)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    if topics is not None:
        storage_filter = rosbag2_py.StorageFilter(topics=topics)
        reader.set_filter(storage_filter)

    return reader


def open_writer(
    bag_path: str,
    storage_id: str = "mcap",
) -> rosbag2_py.SequentialWriter:
    """rosbagライターを初期化してオープンし、ライターを返却する。

    Args:
        bag_path: 出力先 rosbag のディレクトリまたはファイルパス
        storage_id: ストレージ形式 (デフォルト: 'mcap')

    Returns:
        writer
    """
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    return writer


class MessageDeserializer:
    """トピックのメッセージ型キャッシュおよびデシリアライズを行うクラス。"""

    def __init__(self, topic_types: Optional[Dict[str, str]] = None):
        self._type_map: Dict[str, str] = topic_types or {}
        self._class_cache: Dict[str, Type[Any]] = {}

    def register_topic_type(self, topic: str, msg_type_str: str):
        """トピック名と型文字列のマッピングを登録する。"""
        self._type_map[topic] = msg_type_str

    def get_message_class(self, msg_type_str: str) -> Type[Any]:
        """型文字列から ROS2 メッセージクラスを取得する (キャッシュ付き)。"""
        if msg_type_str not in self._class_cache:
            self._class_cache[msg_type_str] = get_message(msg_type_str)
        return self._class_cache[msg_type_str]

    def deserialize_by_type(self, msg_type_str: str, raw_data: bytes) -> Any:
        """型文字列から直接生データをデシリアライズする。"""
        msg_cls = self.get_message_class(msg_type_str)
        return deserialize_message(raw_data, msg_cls)

    def deserialize(self, topic: str, raw_data: bytes) -> Any:
        """トピック名から型を解決し、生データをデシリアライズする。"""
        msg_type_str = self._type_map.get(topic)
        if not msg_type_str:
            raise KeyError(f"トピック '{topic}' の型情報が登録されていません。")
        return self.deserialize_by_type(msg_type_str, raw_data)


def message_to_dict(msg: Any) -> Any:
    """ROS2 メッセージを辞書に再帰変換し、NumPy 配列やタプルも JSON シリアライズ可能な形式に変換する。"""
    if hasattr(msg, "__slots__"):
        result = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            if isinstance(val, (list, tuple)):
                result[slot] = [message_to_dict(v) for v in val]
            elif hasattr(val, "__slots__"):
                result[slot] = message_to_dict(val)
            elif isinstance(val, np.ndarray):
                result[slot] = val.tolist()
            else:
                result[slot] = val
        return result
    elif isinstance(msg, (list, tuple)):
        return [message_to_dict(v) for v in msg]
    elif isinstance(msg, np.ndarray):
        return msg.tolist()
    else:
        return msg
