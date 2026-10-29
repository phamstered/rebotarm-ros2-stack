"""MoveIt for an already running real arm; never starts or switches controllers."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    config = (MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
              .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
              .to_moveit_configs().to_dict())
    hardware = Path(get_package_share_directory("microros_stm32h723_bridge"))
    description = xacro.process_file(
        str(hardware / "urdf/rebotarm.arm.micro_ros.xacro"),
        mappings={"enable_gripper": "true", "gains_file": str(hardware / "config/arm_mit_gains.yaml")},
    ).toxml()
    semantic = ET.fromstring(config["robot_description_semantic"])
    semantic.set("name", ET.fromstring(description).get("name"))
    config["robot_description"] = description
    config["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")
    # Independent real-hardware timing bounds. Do not inherit simulation's
    # 50/200 rad/s limits or disabled acceleration bounds.
    config["robot_description_planning"] = {
        "joint_limits": {
            f"joint{index}": {
                "has_velocity_limits": True, "max_velocity": 0.5,
                "has_acceleration_limits": True, "max_acceleration": 0.5,
            }
            for index in range(1, 7)
        }
    }
    config.update({
        "use_sim_time": False,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    })
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group", executable="move_group", name="move_group",
            parameters=[config], remappings=[("joint_states", "/joint_states")], output="screen",
        ),
    ])
