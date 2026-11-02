"""Exercise the launch configuration with stub ROS launch/config packages."""
import importlib.util
from pathlib import Path
import sys
import types
import yaml
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1] / "launch/real_move_group.launch.py"

class RealMoveGroupTests(unittest.TestCase):
    def check_config(self, enabled):
        class Builder:
            def __init__(self, *args, **kwargs): pass
            def planning_pipelines(self, **kwargs):
                assert kwargs == {"pipelines": ["ompl"], "default_planning_pipeline": "ompl"}
                return self
            def to_moveit_configs(self): return self
            def to_dict(self):
                return {"robot_description": "mock", "robot_description_semantic": '<robot name="mock"><group name="arm"/></robot>', "moveit_simple_controller_manager": {"controller_names": ["arm_trajectory_controller"]}}
        class Node:
            def __init__(self, **kwargs): self.config = kwargs
        modules = {
            "ament_index_python": types.ModuleType("ament_index_python"),
            "ament_index_python.packages": types.SimpleNamespace(get_package_share_directory=lambda name: "/packages/" + name),
            "launch": types.SimpleNamespace(LaunchDescription=lambda actions: actions),
            "launch.actions": types.SimpleNamespace(DeclareLaunchArgument=lambda *a, **k: None, OpaqueFunction=lambda **k: None),
            "launch.substitutions": types.SimpleNamespace(LaunchConfiguration=lambda *a: None),
            "launch_ros": types.ModuleType("launch_ros"),
            "launch_ros.actions": types.SimpleNamespace(Node=Node),
            "moveit_configs_utils": types.SimpleNamespace(MoveItConfigsBuilder=Builder),
            "xacro": types.SimpleNamespace(process_file=lambda *args, **kwargs: types.SimpleNamespace(toxml=lambda: '<robot name="real_robot"><ros2_control><hardware><param name="gripper_position_scale">-0.008612826752</param><param name="gripper_position_offset">0.004282757541</param></hardware><joint name="gripper_finger_left_joint"><command_interface name="position"><param name="min">0</param><param name="max">0.05</param></command_interface></joint></ros2_control></robot>' )),
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location("real_launch", SOURCE)
            module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            actions = [module.move_group_node(enabled)]
        self.assertEqual(len(actions), 1)
        node = actions[0].config
        self.assertEqual(node["executable"], "move_group")
        self.assertEqual(node["remappings"], [("joint_states", "/joint_states")])
        config = node["parameters"][0]
        self.assertTrue(config["robot_description"].startswith('<robot name="real_robot">'))
        self.assertIn('name="real_robot"', config["robot_description_semantic"])
        self.assertFalse(config["moveit_manage_controllers"])
        self.assertFalse(config["use_sim_time"])
        self.assertEqual(config["trajectory_execution.allowed_start_tolerance"], .01)
        expected_controllers = ["arm_trajectory_controller"] + (["gripper_trajectory_controller"] if enabled else [])
        self.assertEqual(config["moveit_simple_controller_manager"]["controller_names"], expected_controllers)
        self.assertEqual(config["robot_description_planning"]["default_velocity_scaling_factor"], 1.0)
        self.assertEqual(config["robot_description_planning"]["default_acceleration_scaling_factor"], 1.0)
        limits = config["robot_description_planning"]["joint_limits"]
        self.assertEqual(set(limits), {f"joint{i}" for i in range(1, 7)} | ({"gripper_finger_left_joint"} if enabled else set()))
        if enabled:
            gripper = limits.pop("gripper_finger_left_joint")
            self.assertEqual(gripper["max_velocity"], 0.02)
            self.assertEqual(gripper["max_acceleration"], 0.04)
            self.assertEqual((gripper["min_position"], gripper["max_position"]), (0, 0.05))
            self.assertEqual(config["debug_desk.gripper_position_scale"], -0.008612826752)
            self.assertEqual(config["debug_desk.gripper_position_offset"], 0.004282757541)
            self.assertEqual(config["moveit_simple_controller_manager"]["gripper_trajectory_controller"]["joints"], ["gripper_finger_left_joint"])
        else:
            self.assertNotIn("debug_desk.gripper_position_scale", config)
        for value in limits.values():
            self.assertTrue(value["has_acceleration_limits"])
            self.assertEqual(value["max_velocity"], 2.0)
            self.assertEqual(value["max_acceleration"], 2.0)

    def test_shared_joint_limits_match_full_scaling_and_arm_bounds(self):
        config = yaml.safe_load((SOURCE.parents[1] / "config/joint_limits.yaml").read_text())
        self.assertEqual(config["default_velocity_scaling_factor"], 1.0)
        self.assertEqual(config["default_acceleration_scaling_factor"], 1.0)
        self.assertEqual(set(config["joint_limits"]), {f"joint{i}" for i in range(1, 7)})
        for value in config["joint_limits"].values():
            self.assertTrue(value["has_velocity_limits"])
            self.assertTrue(value["has_acceleration_limits"])
            self.assertEqual(value["max_velocity"], 2.0)
            self.assertEqual(value["max_acceleration"], 2.0)

    def test_real_description_and_bounded_execution_contract(self):
        self.check_config(False)

    def test_calibrated_gripper_controller_and_units(self):
        self.check_config(True)

if __name__ == "__main__": unittest.main()
