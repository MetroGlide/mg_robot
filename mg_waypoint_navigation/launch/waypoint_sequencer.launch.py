from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    load_path_arg = launch_argument_creator.create(
        "load_path", default=EnvironmentVariable("WAYPOINT_PATH")
    )
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
                        "publish_waypoint_status": publish_status_arg.launch_config,
                    }
                ],
                output="screen",
            ),
        ]
    )
