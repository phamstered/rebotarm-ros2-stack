"""MoveIt for an already running real arm; never starts or switches controllers."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def move_group_node(enable_gripper=False):
    config = (MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
              .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
              .to_moveit_configs().to_dict())
    hardware = Path(get_package_share_directory("microros_stm32h723_bridge"))
    description = xacro.process_file(
        str(hardware / "urdf/rebotarm.arm.micro_ros.xacro"),
        mappings={"enable_gripper": str(enable_gripper).lower(), "gains_file": str(hardware / "config/arm_mit_gains.yaml")},
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
                "has_velocity_limits": True, "max_velocity": 1.0,
                "has_acceleration_limits": True, "max_acceleration": 1.0,
            }
            for index in range(1, 7)
        }
    }
    if enable_gripper:
        # Read calibration from the expanded hardware description: never maintain
        # a second raw->metre mapping in the desktop or MoveIt launch.
        robot = ET.fromstring(description)
        hardware_params = {p.get("name"): float(p.text) for p in
                           robot.findall("ros2_control/hardware/param")
                           if p.get("name") in ("gripper_position_scale", "gripper_position_offset")}
        joint = robot.find("ros2_control/joint[@name='gripper_finger_left_joint']")
        bounds = {p.get("name"): float(p.text) for p in
                  joint.findall("command_interface[@name='position']/param")}
        for name, value in hardware_params.items():
            config["debug_desk." + name] = value
        config["debug_desk.gripper_lower"] = bounds["min"]
        config["debug_desk.gripper_upper"] = bounds["max"]
        config["robot_description_planning"]["joint_limits"]["gripper_finger_left_joint"] = {
            "has_position_limits": True,
            "min_position": bounds["min"], "max_position": bounds["max"],
            "has_velocity_limits": True, "max_velocity": 0.02,
            "has_acceleration_limits": True, "max_acceleration": 0.04,
        }
        manager = config["moveit_simple_controller_manager"]
        manager["controller_names"] = ["arm_trajectory_controller", "gripper_trajectory_controller"]
        manager["gripper_trajectory_controller"] = {
            "type": "FollowJointTrajectory", "action_ns": "follow_joint_trajectory",
            "joints": ["gripper_finger_left_joint"],
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
    return Node(
        package="moveit_ros_move_group", executable="move_group", name="move_group",
        parameters=[config], remappings=[("joint_states", "/joint_states")], output="screen",
    )


def launch_setup(context):
    enabled = LaunchConfiguration("enable_gripper").perform(context).lower() == "true"
    return [move_group_node(enabled)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_gripper", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])
