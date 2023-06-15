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
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "sim.rviz")
    default_localization_config_path = os.path.join(pkg_share, "config", "localization.yaml")
    default_world_path = os.path.join(pkg_share, "world", "social_scene_static.world")

    rviz_arg = DeclareLaunchArgument(name="rviz_config", default_value=default_rviz_config_path,
                                     description="Absolute path to rviz config file")
    ros2_control_arg = DeclareLaunchArgument(name="use_ros2_control", default_value="True", choices=["True", "False"],
                                     description="Flag to enable ros2_control")
    localization_arg = DeclareLaunchArgument(name="localization_config", default_value=default_localization_config_path,
                                     description="Absolute path to localization config file")
    world_arg = DeclareLaunchArgument(name="world", default_value=default_world_path,
                                     description="Absolute path to world file")

    model = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_share, "launch", "display.launch.py"
                )]), launch_arguments={"use_sim_time": "True", "rviz_config": LaunchConfiguration("rviz_config"),
                "use_ros2_control": LaunchConfiguration("use_ros2_control")}.items()
    )

    gzserver = ExecuteProcess(
        cmd=["gzserver",
             "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so",
             LaunchConfiguration("world")],
        output="screen"
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen"
    )

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "object314", "-topic", "robot_description"]
    )

    joint_state_broadcaster_spawn_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control"))
    )

    velocity_controller_spawn_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control"))
    )

    delay_velocity_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawn_node,
            on_exit=[velocity_controller_spawn_node],
        )
    )

    localization_node = Node(
         package="robot_localization",
         executable="ekf_node",
         output="screen",
         parameters=[LaunchConfiguration("localization_config")]
    )

    return LaunchDescription([
        rviz_arg,
        ros2_control_arg,
        localization_arg,
        world_arg,
        model,
        gzserver,
        gzclient,
        spawn_node,
        joint_state_broadcaster_spawn_node,
        delay_velocity_controller_spawn_callback,
        localization_node,
    ])
