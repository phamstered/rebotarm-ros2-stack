import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )

    gazebo_pkg = get_package_share_directory("rebotarm_gazebo")

    ld = LaunchDescription()

    # 1. Gazebo + ros2_control (headless ign gazebo)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, "launch", "rebotarm_gazebo.launch.py")
            )
        )
    )

    # 2. Move group
    move_group_params = [
        moveit_config.to_dict(),
        {"publish_robot_description_semantic": True},
        {"allow_trajectory_execution": True},
        {"publish_planning_scene": True},
        {"publish_geometry_updates": True},
        {"publish_state_updates": True},
        {"publish_transforms_updates": True},
        {"monitor_dynamics": False},
        {"moveit_manage_controllers": True},
        {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
        {"trajectory_execution.allowed_goal_duration_margin": 0.5},
        {"trajectory_execution.allowed_start_tolerance": 0.15},
        {"use_sim_time": True},
    ]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
        )
    )

    # 3. RViz with MoveIt plugin
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config" / "moveit.rviz"),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
            ],
        )
    )

    return ld
