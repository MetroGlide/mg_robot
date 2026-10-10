from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    arg = LaunchArgumentCreator()
    use_topic_monitor = arg.create(
        "use_topic_monitor", default="true",
        description="第三者のノードのトピックの受信頻度とノードの生存を診断として出す")
    topic_monitor_params = arg.create(
        "topic_monitor_params",
        default=PathJoinSubstitution([
            FindPackageShare("mg_diagnostics"), "params", "topic_monitor", "navigation.yaml"]),
        description="topic_monitor_node のパラメータファイル (スタックごとに指定する)")

    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        Node(
            package="mg_diagnostics",
            executable="topic_monitor_node.py",
            name="topic_monitor_node",
            parameters=[topic_monitor_params.launch_config],
            output="screen",
            condition=IfCondition(use_topic_monitor.launch_config),
        ),
    ])
