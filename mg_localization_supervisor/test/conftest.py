"""ROS2 依存モジュールを sys.modules でモックし、ROS2 なしで pytest を実行可能にする。"""
import sys
from unittest.mock import MagicMock

_ROS2_MODULES = [
    "rclpy",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.qos",
    "rclpy.time",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "diagnostic_msgs",
    "diagnostic_msgs.msg",
    "diagnostic_updater",
    "localization_supervisor_msgs",
    "localization_supervisor_msgs.msg",
    "localization_supervisor_msgs.srv",
    "tf2_ros",
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
