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
    "std_srvs",
    "std_srvs.srv",
    "nav_msgs",
    "nav_msgs.msg",
    "visualization_msgs",
    "visualization_msgs.msg",
    "nav2_msgs.srv",
    "nav2_simple_commander",
    "nav2_simple_commander.robot_navigator",
    "ament_index_python",
    "ament_index_python.packages",
    "builtin_interfaces",
    "builtin_interfaces.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.msg",
    "mg_msgs",
    "mg_msgs.msg",
    "mg_msgs.srv",
]

for _mod in _ROS2_MODULES:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()
