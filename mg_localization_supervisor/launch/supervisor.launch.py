"""自己位置推定の監督・ゲートの調停・GNSS による初期化を、このロボットの設定で起動する。

mg_navigation の localization_launch.py から include する。
GNSS による初期化と監督ノードは、他のノードが立ち上がるのを待って遅れて起動する
(初期化ノードを先に、監督ノードを後に起動する)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator

INITIALIZER_DELAY_SEC = 10.0
SUPERVISOR_DELAY_SEC = 12.0


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mg_localization_supervisor'), 'config')

    arg = LaunchArgumentCreator()
    use_sim_time = arg.create('use_sim_time', default='false')
    use_gnss_amcl_initializer = arg.create(
        'use_gnss_amcl_initializer', default='true',
        description='GNSS から AMCL・EKF の初期姿勢を与えるノードを起動するか')
    use_supervisor = arg.create(
        'use_supervisor', default='false', description='自己位置推定の監督ノードを起動するか')
    supervisor_params_file = arg.create(
        'supervisor_params_file', default=os.path.join(config_dir, 'supervisor.yaml'),
        description='監督ノードのパラメータファイル')

    def create_nodes(context):
        """TimerAction の中の LaunchConfiguration は、タイマーが発火するとき (include のスコープを抜けた後) に
        評価され、use_sim_time などが既定値になってしまう。値をここ (スコープ内) で確定してから渡す。
        """
        def flag(launch_argument):
            return context.perform_substitution(launch_argument.launch_config).lower() == 'true'

        sim_time = flag(use_sim_time)
        actions = [
            Node(
                package='localization_supervisor',
                executable='gate_arbiter_node.py',
                name='amcl_gate_arbiter',
                output='screen',
                parameters=[os.path.join(config_dir, 'gate_arbiter.yaml'), {'use_sim_time': sim_time}],
            ),
        ]
        if flag(use_gnss_amcl_initializer):
            actions.append(TimerAction(period=INITIALIZER_DELAY_SEC, actions=[Node(
                package='mg_localization_supervisor',
                executable='gnss_amcl_initializer_node.py',
                name='gnss_amcl_initializer_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'initializer.yaml'), {'use_sim_time': sim_time}],
            )]))
        if flag(use_supervisor):
            actions.append(TimerAction(period=SUPERVISOR_DELAY_SEC, actions=[Node(
                package='localization_supervisor',
                executable='localization_supervisor_node.py',
                name='localization_supervisor_node',
                output='screen',
                parameters=[
                    context.perform_substitution(supervisor_params_file.launch_config),
                    {'use_sim_time': sim_time}],
            )]))
        return actions

    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        OpaqueFunction(function=create_nodes),
    ])
