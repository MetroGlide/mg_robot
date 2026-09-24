"""ROS2 依存モジュールを sys.modules でモックし、ROS2 なしで pytest を実行可能にする。"""
import sys
from unittest.mock import MagicMock

_ROS2_MODULES = [
    "rclpy",
    "rclpy.duration",
    "rclpy.time",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.action",
    "rclpy.executors",
    "tf2_ros",
    "geometry_msgs",
    "geometry_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "diagnostic_msgs",
    "diagnostic_msgs.msg",
    "nav2_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
    "nav2_msgs",
    "nav2_msgs.srv",
    "nav2_msgs.action",
    "nav2_simple_commander",
    "nav2_simple_commander.robot_navigator",
    "ament_index_python",
    "ament_index_python.packages",
]

for _mod in _ROS2_MODULES:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()
