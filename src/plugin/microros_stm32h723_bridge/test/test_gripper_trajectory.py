"""Gripper ownership, units, and planning contract; never starts hardware."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
MOVEIT = WORKSPACE / 'src/rebotarm_moveit_config'


def test_gripper_controller_has_independent_timed_position_velocity_ownership():
    config = yaml.safe_load((PACKAGE / 'config/arm_controllers.yaml').read_text())
    gripper = config['gripper_trajectory_controller']['ros__parameters']
    assert gripper['joints'] == ['gripper_finger_left_joint']
    assert gripper['command_interfaces'] == ['position', 'velocity']
    assert gripper['state_interfaces'] == ['position', 'velocity']
    assert not gripper['allow_partial_joints_goal']
    assert not gripper['allow_nonzero_velocity_at_trajectory_end']
    assert gripper['constraints']['gripper_finger_left_joint']['goal'] == .001
    assert config['arm_trajectory_controller']['ros__parameters']['joints'] == [
        f'joint{i}' for i in range(1, 7)]


def test_joint_goal_groups_keep_gripper_out_of_six_axis_ik():
    robot = ET.parse(MOVEIT / 'config/rebotarm.srdf').getroot()
    groups = {g.get('name'): [j.get('name') for j in g.findall('joint')]
              for g in robot.findall('group')}
    assert groups['arm'] == [f'joint{i}' for i in range(1, 7)]
    assert groups['gripper'] == ['gripper_finger_left_joint']
    assert groups['arm_with_gripper'] == groups['arm'] + groups['gripper']
    solvers = yaml.safe_load((MOVEIT / 'config/kinematics.yaml').read_text())
    assert 'arm_with_gripper' not in solvers
    assert 'gripper' not in solvers
    home = robot.find("group_state[@name='home'][@group='arm']")
    assert [j.get('name') for j in home] == groups['arm']


@pytest.mark.parametrize('mode,enabled,state_only,expected', [
    ('teleop', 'true', 'false', ['gripper_teleop_controller']),
    ('trajectory', 'true', 'false', ['gripper_trajectory_controller']),
    ('trajectory', 'false', 'false', []),
    ('none', 'true', 'false', []),
    ('trajectory', 'true', 'true', []),
])
def test_launch_never_activates_two_gripper_owners(mode, enabled, state_only, expected):
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'gripper_arm_launch', PACKAGE / 'launch/stm32h723_arm.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_node = module.Node
    def node_with_selection(**kwargs):
        node = original_node(**kwargs)
        node.selected_controller = (kwargs.get('arguments') or [''])[0]
        return node
    module.Node = node_with_selection
    context = LaunchContext()
    context.launch_configurations.update({
        'position_controller': mode, 'enable_gripper': enabled,
        'state_only': state_only, 'enable_gravity_controller': 'false',
        'enable_joint4_calibration': 'false', 'use_sim_time': 'false',
    })
    owners = []
    for action in module.generate_launch_description().entities:
        if not isinstance(action, Node):
            continue
        condition = action.condition
        if condition is None or not condition.evaluate(context):
            continue
        command = action.selected_controller
        for name in ('gripper_teleop_controller', 'gripper_trajectory_controller'):
            if name in command:
                owners.append(name)
    assert owners == expected


@pytest.mark.parametrize('selected,enabled,controllers,accepted', [
    ('gripper_trajectory_controller', 'true', 'gripper_trajectory_controller x active', True),
    ('gripper_trajectory_controller', 'true', 'gripper_teleop_controller x active', False),
    ('gripper_trajectory_controller', 'true', 'gripper_trajectory_controller x active\ngripper_teleop_controller x active', False),
    ('gripper_teleop_controller', 'true', 'gripper_teleop_controller x active', True),
    ('gripper_teleop_controller', 'false', 'gripper_trajectory_controller x active', False),
    ('gripper_teleop_controller', 'false', 'gripper_trajectory_controller x inactive', True),
])
def test_startup_reuse_rejects_wrong_or_multiple_gripper_owners(selected, enabled, controllers, accepted):
    import os
    import subprocess
    source = (WORKSPACE / 'scripts/start_arm_teleop.sh').read_text()
    functions = 'controller_has_state() {' + source.split('controller_has_state() {', 1)[1].split('check_micro_ros_agent()', 1)[0]
    result = subprocess.run(['bash', '-c',
        'strip_ansi() { cat; }\n' + functions + '\ngripper_controller_matches "$CONTROLLERS"'],
        env=dict(os.environ, ENABLE_GRIPPER=enabled, GRIPPER_CONTROLLER=selected, CONTROLLERS=controllers),
        capture_output=True, text=True, timeout=3)
    assert (result.returncode == 0) is accepted, result.stderr


@pytest.mark.parametrize('raw,expected', [(.497253418, 0.), (-5.308041573, .05)])
def test_live_feedback_relay_converts_gripper_raw_units_before_tf(raw, expected, monkeypatch):
    from types import SimpleNamespace
    from sensor_msgs.msg import JointState
    import time
    monkeypatch.syspath_prepend(str(PACKAGE / 'scripts'))
    spec = importlib.util.spec_from_file_location(
        'feedback_relay', PACKAGE / 'scripts/stm32_state_republisher.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    outputs = []
    clock = module.FeedbackClock()
    clock.valid_since = time.monotonic_ns()-300000000
    relay = SimpleNamespace(
        _joint_names=list(module.ARM_JOINT_NAMES) + [module.GRIPPER_JOINT_NAME],
        _gripper_position_scale=-.008612826752,
        _gripper_position_offset=.004282757541,
        _missing_joints_logged=False, _first_state_logged=True,
        _publisher=SimpleNamespace(publish=outputs.append),
        _feedback_clock=clock, _tf_publisher=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100000000000)))
    message = JointState()
    message.header.stamp.sec = 100
    message.name = relay._joint_names
    message.position = [.1] * 6 + [raw]
    message.velocity = [.2] * 6 + [1.]
    module.Stm32StateRepublisher._state_callback(relay, message)
    assert len(outputs) == 1
    assert list(outputs[0].position[:6]) == [.1] * 6
    assert outputs[0].position[6] == pytest.approx(expected, abs=1e-10)
    assert outputs[0].velocity[6] == pytest.approx(-.008612826752)
