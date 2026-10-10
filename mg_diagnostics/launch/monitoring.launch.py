from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mg_utils.launch_argument import LaunchArgumentCreator


def _params(*parts):
    return PathJoinSubstitution([FindPackageShare("mg_diagnostics"), "params", *parts])


def generate_launch_description():
    arg = LaunchArgumentCreator()
    simulation = arg.create(
        "simulation", default=EnvironmentVariable("SIMULATION", default_value="false"),
        description="true なら、実機のドライバの診断 (drivers.yaml) を集約しない")
    use_aggregator = arg.create(
        "use_aggregator", default="true",
        description="診断を /diagnostics_agg に集約する。止めても /diagnostics の生の値は残る")
    analyzers = arg.create(
        "analyzers", default=_params("analyzers", "navigation.yaml"),
        description="スタックごとの集約の設定 (common.yaml と一緒に渡される)")
    use_topic_monitor = arg.create(
        "use_topic_monitor", default="true",
        description="第三者のノードのトピックの受信頻度とノードの生存を診断として出す")
    topic_monitor_params = arg.create(
        "topic_monitor_params", default=_params("topic_monitor", "navigation.yaml"),
        description="topic_monitor_node のパラメータファイル (スタックごとに指定する)")

    is_hardware = IfCondition(PythonExpression([
        "'", use_aggregator.launch_config, "' == 'true' and '",
        simulation.launch_config, "' != 'true'"]))
    is_simulation = IfCondition(PythonExpression([
        "'", use_aggregator.launch_config, "' == 'true' and '",
        simulation.launch_config, "' == 'true'"]))

    def aggregator(parameters, condition):
        return Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            output="screen",
            parameters=parameters,
            condition=condition,
        )

    common = _params("analyzers", "common.yaml")
    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        aggregator(
            [common, _params("analyzers", "drivers.yaml"), analyzers.launch_config],
            is_hardware),
        aggregator([common, analyzers.launch_config], is_simulation),
        Node(
            package="mg_diagnostics",
            executable="topic_monitor_node.py",
            name="topic_monitor_node",
            parameters=[topic_monitor_params.launch_config],
            output="screen",
            condition=IfCondition(use_topic_monitor.launch_config),
        ),
    ])
