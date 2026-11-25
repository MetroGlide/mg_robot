"""自己位置推定の監督・ゲートの調停・GNSS による初期化を、このロボットの設定で起動する。

mg_navigation の localization_launch.py から include する。
GNSS による初期化と監督ノードは、他のノードが立ち上がるのを待って遅れて起動する
(初期化ノードを先に、監督ノードを後に起動する)。
監督ノードの中の門がないときは、手動の初期姿勢をそのまま /initialpose へ送る中継を起動する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator

INITIALIZER_DELAY_SEC = 10.0
SUPERVISOR_DELAY_SEC = 12.0
# 手動の初期姿勢の専用トピック (Web UI が送る。監督の設定 supervision.yaml の gate.manual_pose_topic と同じ)
MANUAL_POSE_TOPIC = '/localization/manual_initial_pose'


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mg_localization_supervisor'), 'config')

    arg = LaunchArgumentCreator()
    use_sim_time = arg.create('use_sim_time', default='false')
    use_gnss_amcl_initializer = arg.create(
        'use_gnss_amcl_initializer', default='true',
        description='GNSS から AMCL・EKF の初期姿勢を与えるノードを起動するか')
    use_supervisor = arg.create(
        'use_supervisor', default='false', description='自己位置推定の監督ノードを起動するか')
    supervisor_config = arg.create(
        'supervisor_config', default=os.path.join(config_dir, 'supervision.yaml'),
        description='監督の設定ファイル (情報源・判定・総合・状態遷移・プロファイル)')
    localization_profile = arg.create(
        'localization_profile', default='', description='監督の設定に重ねるプロファイル (空は基本設定)')
    use_supervisor_gate = arg.create(
        'use_supervisor_gate', default='false',
        description='監督ノードの中で門を使うか (true | false | observe。supervision.yaml の gate 節。use_supervisor と組で'
                    '使う)。observe は判定と照合だけで、門も監督も何もしない (初期化ノードと中継は起動する)')

    def create_nodes(context):
        """TimerAction の中の LaunchConfiguration は、タイマーが発火するとき (include のスコープを抜けた後) に
        評価され、use_sim_time などが既定値になってしまう。値をここ (スコープ内) で確定してから渡す。
        """
        def value(launch_argument):
            return context.perform_substitution(launch_argument.launch_config)

        def flag(launch_argument):
            return value(launch_argument).lower() == 'true'

        sim_time = flag(use_sim_time)
        gate_mode = value(use_supervisor_gate).lower()
        if gate_mode not in ('true', 'false', 'observe'):
            raise RuntimeError(f'use_supervisor_gate は true / false / observe のどれかです: {gate_mode}')
        # 門が置き直しと手動の初期姿勢を受け持つか (observe では受け持たない)
        supervisor_gate = gate_mode == 'true'
        if gate_mode != 'false' and not flag(use_supervisor):
            raise RuntimeError('use_supervisor_gate は use_supervisor と組で使います (門は監督ノードの中で動く)')
        actions = [
            Node(
                package='localization_supervisor',
                executable='gate_arbiter_node.py',
                name='amcl_gate_arbiter',
                output='screen',
                parameters=[os.path.join(config_dir, 'gate_arbiter.yaml'), {'use_sim_time': sim_time}],
            ),
        ]
        # 監督ノードの中の門は、起動の直後と地図の切り替えの後の姿勢も決める (GNSS の周りの全周の照合を含む)ので、
        # 初期化ノードは起動しない (置き直しを出す部品を 1 つにする)
        if flag(use_gnss_amcl_initializer) and not supervisor_gate:
            actions.append(TimerAction(period=INITIALIZER_DELAY_SEC, actions=[Node(
                package='mg_localization_supervisor',
                executable='gnss_amcl_initializer_node.py',
                name='gnss_amcl_initializer_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'initializer.yaml'),
                            {'use_sim_time': sim_time}],
            )]))
        if flag(use_supervisor):
            actions.append(TimerAction(period=SUPERVISOR_DELAY_SEC, actions=[Node(
                package='localization_supervisor',
                executable='localization_supervisor_node.py',
                name='localization_supervisor_node',
                output='screen',
                parameters=[{
                    'config_file': value(supervisor_config),
                    'profile': value(localization_profile),
                    'use_gate': gate_mode != 'false',
                    'gate_observe_only': gate_mode == 'observe',
                    'use_sim_time': sim_time,
                }],
            )]))
        # 門があるときは、門が手動の初期姿勢を受けて照合する
        if not supervisor_gate:
            actions.append(Node(
                package='localization_supervisor',
                executable='manual_pose_relay_node.py',
                name='manual_pose_relay',
                output='screen',
                parameters=[{'input_topic': MANUAL_POSE_TOPIC, 'output_topic': '/initialpose',
                             'use_sim_time': sim_time}],
            ))
        return actions

    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        OpaqueFunction(function=create_nodes),
    ])
