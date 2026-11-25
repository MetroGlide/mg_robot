"""自己位置推定の監督・門のノード・ゲートの調停・GNSS による初期化を、このロボットの設定で起動する。

mg_navigation の localization_launch.py から include する。
GNSS による初期化と監督ノード・門のノードは、他のノードが立ち上がるのを待って遅れて起動する
(初期化ノードを先に、監督ノード・門のノードを後に起動する)。
門 (門のノード、または監督ノードの中の門) がないときは、手動の初期姿勢をそのまま /initialpose へ送る中継を起動する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator

INITIALIZER_DELAY_SEC = 10.0
SUPERVISOR_DELAY_SEC = 12.0
# 手動の初期姿勢の専用トピック (Web UI が送る。門のノードの設定 motion_gate.yaml の manual_pose_topic と同じ)
MANUAL_POSE_TOPIC = '/localization/manual_initial_pose'


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mg_localization_supervisor'), 'config')

    arg = LaunchArgumentCreator()
    use_sim_time = arg.create('use_sim_time', default='false')
    use_gnss_amcl_initializer = arg.create(
        'use_gnss_amcl_initializer', default='true',
        description='GNSS から AMCL・EKF の初期姿勢を与えるノードを起動するか')
    initializer_verify = arg.create(
        'initializer_verify',
        default=EnvironmentVariable('LOCALIZATION_INITIALIZER_VERIFY', default_value='false'),
        description='GNSS による初期姿勢を、スキャンと地図の照合で確かめるか (initializer.yaml の verify_by_scan を上書き)')
    use_supervisor = arg.create(
        'use_supervisor', default='false', description='自己位置推定の監督ノードを起動するか')
    supervisor_config = arg.create(
        'supervisor_config', default=os.path.join(config_dir, 'supervision.yaml'),
        description='監督の設定ファイル (情報源・判定・総合・状態遷移・プロファイル)')
    localization_profile = arg.create(
        'localization_profile', default='', description='監督の設定に重ねるプロファイル (空は基本設定)')
    motion_gate_config = arg.create(
        'motion_gate_config', default=os.path.join(config_dir, 'motion_gate.yaml'),
        description='門のノードの設定ファイル')
    use_motion_gate = arg.create(
        'use_motion_gate', default='false',
        description='門のノード (AMCL を地図との照合で確かめてから EKF に入れる) を起動するか。監督ノードとは同時に使わない')
    use_supervisor_gate = arg.create(
        'use_supervisor_gate', default='false',
        description='監督ノードの中で門を使うか (supervision.yaml の gate 節。use_supervisor と組で使う)')

    def create_nodes(context):
        """TimerAction の中の LaunchConfiguration は、タイマーが発火するとき (include のスコープを抜けた後) に
        評価され、use_sim_time などが既定値になってしまう。値をここ (スコープ内) で確定してから渡す。
        """
        def value(launch_argument):
            return context.perform_substitution(launch_argument.launch_config)

        def flag(launch_argument):
            return value(launch_argument).lower() == 'true'

        sim_time = flag(use_sim_time)
        motion_gate = flag(use_motion_gate)
        supervisor_gate = flag(use_supervisor_gate)
        if motion_gate and flag(use_supervisor):
            raise RuntimeError('use_motion_gate と use_supervisor は同時に使えません (どちらも AMCL と EKF を置き直す)')
        if supervisor_gate and not flag(use_supervisor):
            raise RuntimeError('use_supervisor_gate は use_supervisor と組で使います (門は監督ノードの中で動く)')
        # 門があるときは、門が /initialpose を推定器の飛びとして判定し、手動の初期姿勢を受ける
        gated = motion_gate or supervisor_gate
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
                            {'use_sim_time': sim_time, 'verify_by_scan': flag(initializer_verify),
                             'check_external_initialpose': not gated}],
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
                    'use_gate': supervisor_gate,
                    'use_sim_time': sim_time,
                }],
            )]))
        if motion_gate:
            actions.append(TimerAction(period=SUPERVISOR_DELAY_SEC, actions=[Node(
                package='localization_supervisor',
                executable='motion_gate_node.py',
                name='motion_gate_node',
                output='screen',
                parameters=[{
                    'config_file': value(motion_gate_config),
                    'use_sim_time': sim_time,
                }],
            )]))
        if not gated:
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
