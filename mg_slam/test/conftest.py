"""mg_slam テスト用の pytest 設定。sys.path に scripts を追加し、ROS2 依存をモックする。"""
import os
import sys
from unittest.mock import MagicMock

# scripts ディレクトリを sys.path に追加
_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

_ROS2_MODULES = [
    "rclpy",
    "rclpy.node",
    "rclpy.time",
    "rclpy.serialization",
    "sensor_msgs",
    "sensor_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "tf2_ros",
    "tf2_msgs",
    "tf2_msgs.msg",
    "ublox_msgs",
    "ublox_msgs.msg",
    "rosbag2_py",
    "rosidl_runtime_py",
    "rosidl_runtime_py.utilities",
]

import types


class MockModule(types.ModuleType):
    def __getattr__(self, name):
        mock = MagicMock()
        setattr(self, name, mock)
        return mock


for _mod in _ROS2_MODULES:
    if _mod not in sys.modules:
        m = MockModule(_mod)
        m.__path__ = []
        sys.modules[_mod] = m
