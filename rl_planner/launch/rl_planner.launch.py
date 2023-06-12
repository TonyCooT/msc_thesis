import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.join(get_package_share_path("rl_planner"))
    default_config_path = os.path.join(pkg_share, "config", "params.yaml")

    config_arg = DeclareLaunchArgument(name="config", default_value=default_config_path,
                                     description="Absolute path to config file")

    rl_planner_node = Node(
         package="rl_planner",
         executable="rl_planner",
         namespace="/rl_planner",
         remappings=[
                ("/rl_planner/cmd_vel", "/rl_planner/cmd_vel"),
                ("/rl_planner/pose", "/rl_planner/pose"),
                ("/rl_planner/velocity", "/rl_planner/velocity"),
                ("/rl_planner/peds", "/rl_planner/peds"),
                ("/rl_planner/goal", "/rl_planner/goal")
        ],
         output="screen",
         parameters=[LaunchConfiguration("config")]
    )

    return LaunchDescription([
        config_arg,
        rl_planner_node
    ])
