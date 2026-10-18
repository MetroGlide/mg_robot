"""ROS2 依存モジュールを sys.modules でモックし、ROS2 なしで pytest を実行可能にする。

入力のアダプタとオフライン評価は、メッセージの属性だけを読むので、テストでは同じ属性を持つ偽のメッセージを渡す。
rosbag の読み込みは、実データでの評価 (tools/scripts/eval_localization_offline.py) で確かめる。
"""
import sys
from unittest.mock import MagicMock

_ROS2_MODULES = [
    "rclpy",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.qos",
    "rclpy.time",
    "rclpy.serialization",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "diagnostic_msgs",
    "diagnostic_msgs.msg",
    "diagnostic_updater",
    "localization_supervisor_msgs",
    "localization_supervisor_msgs.msg",
    "localization_supervisor_msgs.srv",
    "rosbag2_py",
    "rosidl_runtime_py",
    "rosidl_runtime_py.utilities",
    "tf2_ros",
    "tf2_msgs",
    "tf2_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
]

for _mod in _ROS2_MODULES:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()
