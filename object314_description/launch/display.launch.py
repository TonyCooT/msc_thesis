import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.join(get_package_share_path("object314_description"))
    default_model_path = os.path.join(pkg_share, "urdf", "object314.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "display.rviz")

    model_arg = DeclareLaunchArgument(name="model", default_value=default_model_path,
                                      description="Absolute path to robot urdf file")
    ros2_control_arg = DeclareLaunchArgument(name="use_ros2_control", default_value="False", choices=["True", "False"],
                                     description="Flag to enable ros2_control")
    sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False", choices=["True", "False"],
                                      description="Flag to enable use_sim_time")
    rviz_arg = DeclareLaunchArgument(name="rviz_config", default_value=default_rviz_config_path,
                                     description="Absolute path to rviz config file")

    # Note, space is required before Xacro parameter (for example, use_ros2_control)
    robot_description = Command(["xacro ", LaunchConfiguration("model"), " use_ros2_control:=", LaunchConfiguration("use_ros2_control")])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_sim_time"))
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    return LaunchDescription([
        model_arg,
        ros2_control_arg,
        sim_time_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        #rviz_node
    ])
