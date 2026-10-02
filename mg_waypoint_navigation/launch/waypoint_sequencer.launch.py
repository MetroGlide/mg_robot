from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    load_path_arg = launch_argument_creator.create(
        "load_path", default=EnvironmentVariable("WAYPOINT_PATH")
    )
    # 起動時に map_server が読み込んでいる地図 (読み込み済みの地図の記録の初期値)
    initial_localization_map_arg = launch_argument_creator.create(
        "initial_localization_map", default="")
    initial_planning_map_arg = launch_argument_creator.create(
        "initial_planning_map", default="")
    publish_status_arg = launch_argument_creator.create(
        "publish_waypoint_status", default="true"
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            Node(
                package="mg_waypoint_navigation",
                executable="waypoint_sequencer_node.py",
                parameters=[
                    {
                        "use_sim_time": simulation_arg.launch_config,
                        "load_path": load_path_arg.launch_config,
                        "initial_localization_map": ParameterValue(
                            initial_localization_map_arg.launch_config, value_type=str),
                        "initial_planning_map": ParameterValue(
                            initial_planning_map_arg.launch_config, value_type=str),
                        "publish_waypoint_status": publish_status_arg.launch_config,
                    }
                ],
                output="screen",
            ),
        ]
    )
