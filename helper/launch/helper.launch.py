import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    helper_node = Node(
         package="helper",
         executable="helper",
         remappings=[
                ("/crowd/ped_1", "/crowd/ped_1"),
                ("/crowd/ped_2", "/crowd/ped_2"),
                ("/crowd/ped_3", "/crowd/ped_3"),
                ("/crowd/ped_4", "/crowd/ped_4"),
                ("/crowd/ped_5", "/crowd/ped_5"),
                ("/ground_truth", "/ground_truth"),
                ("/odometry/filtered", "/odometry/filtered"),
                ("/rl_planner/pose", "/rl_planner/pose"),
                ("/rl_planner/velocity", "/rl_planner/velocity"),
                ("/rl_planner/peds", "/rl_planner/peds"),
                ("/rl_planner/goal", "/rl_planner/goal")
        ],
         output="screen",
         parameters=[ {"radius": 0.3} ]
    )

    return LaunchDescription([
        helper_node
    ])
