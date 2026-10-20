# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    supervisor_dir = get_package_share_directory('mg_localization_supervisor')

    arg = LaunchArgumentCreator()
    namespace = arg.create('namespace', default='', description='Top-level namespace').launch_config
    map_yaml_file = arg.create('map', description='Full path to map yaml file to load').launch_config
    use_sim_time = arg.create(
        'use_sim_time', default='false', description='Use simulation (Gazebo) clock if true').launch_config
    params_file = arg.create(
        'params_file', default=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes').launch_config
    autostart = arg.create(
        'autostart', default='true', description='Automatically startup the nav2 stack').launch_config
    use_composition = arg.create(
        'use_composition', default='False', description='Use composed bringup if True').launch_config
    container_name = arg.create(
        'container_name', default='nav2_container',
        description='the name of conatiner that nodes will load in if use composition').launch_config
    container_name_full = (namespace, '/', container_name)
    use_respawn = arg.create(
        'use_respawn', default='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.').launch_config
    log_level = arg.create('log_level', default='info', description='log level').launch_config
    use_gnss_amcl_initializer = arg.create(
        'use_gnss_amcl_initializer', default='true',
        description='GNSS から AMCL の初期姿勢を与えるノードを起動するか').launch_config
    localization_monitor = arg.create(
        'localization_monitor', default='supervisor',
        description='自己位置の監視ノード (none | watchdog | supervisor)').launch_config
    localization_supervisor = arg.create(
        'localization_supervisor', default='false',
        description='監視が watchdog のとき、監督ノードを並行して通知だけで動かすか '
                    '(状態 /localization/status は監督ノードが出す)').launch_config
    supervisor_config = arg.create(
        'supervisor_config', default=os.path.join(supervisor_dir, 'config', 'supervision.yaml'),
        description='監督の設定ファイル').launch_config
    localization_profile = arg.create(
        'localization_profile', default='',
        description='監督の設定に重ねるプロファイル (空は基本設定。監視が watchdog のときは、'
                    '二重に再初期化しないよう notify_only)').launch_config
    # watchdog と並行して動かす監督は、通知だけにする (どちらも再初期化すると互いに上書きする)
    supervisor_profile = PythonExpression([
        "'notify_only' if '", localization_monitor, "' == 'watchdog' and '", localization_profile,
        "' == '' else '", localization_profile, "'"])

    lifecycle_nodes = [
        'map_server',
        'amcl'
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('amcl_pose', 'amcl_pose_origin')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes},
                            {'bond_timeout': 0.0},
                            ])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings + [('amcl_pose', 'amcl_pose_origin')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    change_amcl_publish_state_node = Node(
        package='mg_drivers',
        executable='generic_publish_controller_node.py',
        name='amcl_publish_controller_node',
        parameters=[{
            "msg_module": "geometry_msgs.msg",
            "msg_class": "PoseWithCovarianceStamped",
            # "publish": False,
            "publish": True,
            "queue_size": 1,
            "use_sim_time": use_sim_time,
        }],
        output='screen',
        remappings=[("input_topic", "amcl_pose_origin"),
                    ("output_topic", "amcl_pose")]
    )

    # 監督ノード・AMCL ゲートの調停・GNSS による初期化 (mg_localization_supervisor)
    supervisor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(supervisor_dir, 'launch', 'supervisor.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gnss_amcl_initializer': use_gnss_amcl_initializer,
            'use_supervisor': PythonExpression([
                "'", localization_monitor, "' == 'supervisor' or '", localization_supervisor, "'.lower() == 'true'"]),
            'supervisor_config': supervisor_config,
            'localization_profile': supervisor_profile,
        }.items())

    def create_watchdog(context):
        """watchdog を遅れて起動する。

        TimerAction の中の LaunchConfiguration は、タイマーが発火するとき (include のスコープを抜けた後) に
        評価されるため、use_sim_time が既定値 (false) になってしまう。その結果、シミュレーション・rosbag 再生で
        このノードだけが実時間で動いていた。値をここ (スコープ内) で確定してから渡す。
        """
        if context.perform_substitution(localization_monitor) != 'watchdog':
            return []
        sim_time = context.perform_substitution(use_sim_time).lower() == 'true'
        # 監督ノードを並行して動かすときは、状態の配信を監督ノードに任せる
        with_supervisor = context.perform_substitution(localization_supervisor).lower() == 'true'

        # AMCL watchdog node: monitor amcl covariance and trigger reinitialization when needed
        amcl_watchdog_node = Node(
            package='mg_navigation',
            executable='amcl_watchdog_node.py',
            name='amcl_watchdog_node',
            output='screen',
            parameters=[{'use_sim_time': sim_time, 'publish_status': not with_supervisor}],
            remappings=remappings + [('amcl_pose', 'amcl_pose_origin'),
                                     ('request_reinit', '/gnss_amcl_initializer_node/request_reinit')]
        )
        return [TimerAction(period=12.0, actions=[amcl_watchdog_node])]

    return LaunchDescription([
        stdout_linebuf_envvar,
        *arg.get_created_declare_launch_args(),
        load_nodes,
        load_composable_nodes,
        change_amcl_publish_state_node,
        supervisor_launch,
        OpaqueFunction(function=create_watchdog),
    ])
