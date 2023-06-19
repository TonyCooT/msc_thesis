import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.join(get_package_share_path("object314_description"))
    rl_planner_pkg_share = os.path.join(get_package_share_path("rl_planner"))
    helper_pkg_share = os.path.join(get_package_share_path("helper"))
    default_world_path = os.path.join(pkg_share, "world", "social_scene_dynamic_one.world")

    world_arg = DeclareLaunchArgument(name="world", default_value=default_world_path,
                                     description="Absolute path to world file")

    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_share, "launch", "sim.launch.py"
                )]), launch_arguments={"world": LaunchConfiguration("world")}.items()
    )

    rl_planner_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    rl_planner_pkg_share, "launch", "rl_planner.launch.py"
                )])
    )

    helper_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    helper_pkg_share, "launch", "helper.launch.py"
                )])
    )

    return LaunchDescription([
        world_arg,
        sim,
        rl_planner_node,
        helper_node
    ])
