"""
move_group.launch.py — 单独启动 MoveIt move_group 节点

启动命令：
  # 默认启动
  ros2 launch rebotarm_moveit_config move_group.launch.py

  # 启用 GDB 调试
  ros2 launch rebotarm_moveit_config move_group.launch.py debug:=true

  # 禁用轨迹执行（仅规划不执行）
  ros2 launch rebotarm_moveit_config move_group.launch.py allow_trajectory_execution:=false

用途：
  仅启动 move_group 运动规划节点，不启动 Gazebo 或 RViz。
  适用于已单独启动仿真/硬件，只需补充 MoveIt 规划能力的场景。

功能：
  - 加载完整 MoveIt 配置（URDF/SRDF/kinematics/controllers）
  - 支持 debug 模式（通过 GDB 调试 move_group）
  - 可配置的轨迹执行参数和规划场景发布

Launch 参数：
  debug: 是否启用 GDB 调试
  allow_trajectory_execution: 是否允许轨迹执行
  publish_monitored_planning_scene: 是否发布规划场景
  capabilities: 启用的 MoveIt capabilities
  disable_capabilities: 禁用的 MoveIt capabilities
  monitor_dynamics: 是否监控动力学
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 声明 launch 参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # move_group 核心配置
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    # 轨迹执行参数
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        trajectory_execution,
        {"use_sim_time": True},
    ]

    # 使用 add_debuggable_node 支持可选的 GDB 调试
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
    )

    return ld
