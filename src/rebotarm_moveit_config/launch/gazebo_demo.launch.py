"""
gazebo_demo.launch.py — Gazebo 仿真 + MoveIt 完整演示

启动命令：
  ros2 launch rebotarm_moveit_config gazebo_demo.launch.py

用途：
  启动完整的 Gazebo 仿真 + MoveIt 运动规划环境，包括：
  1. Gazebo 仿真（通过 rebotarm_gazebo 包启动 ign gazebo + ros2_control）
  2. move_group 运动规划节点
  3. RViz2 可视化（带 MoveIt 插件）

使用场景：
  在 Gazebo 中测试运动规划和轨迹执行，验证从臂仿真跟随。

关键参数：
  - use_sim_time: True — 使用 Gazebo 仿真时钟
  - allowed_execution_duration_scaling: 1.2 — 允许轨迹执行时间比规划时间长 20%
  - allowed_goal_duration_margin: 0.5s — 目标到达时间容差
  - allowed_start_tolerance: 0.15 rad — 起始位置容差
"""

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
    # 加载 MoveIt 全部配置（URDF/SRDF/kinematics/controllers/limits）
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )

    gazebo_pkg = get_package_share_directory("rebotarm_gazebo")

    ld = LaunchDescription()

    # === 第 1 步：启动 Gazebo 仿真 + ros2_control ===
    # 包含 rebotarm_gazebo 包的启动文件，启动 ign gazebo 和硬件接口
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, "launch", "rebotarm_gazebo.launch.py")
            )
        )
    )

    # === 第 2 步：启动 move_group 运动规划节点 ===
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
        # 轨迹执行参数：允许比规划时间多 20% 的执行时间
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

    # === 第 3 步：启动 RViz2（带 MoveIt 可视化插件） ===
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
