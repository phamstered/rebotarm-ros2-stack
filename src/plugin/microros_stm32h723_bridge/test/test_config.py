from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def test_all_gain_groups_have_seven_finite_nonnegative_values():
    config = yaml.safe_load((PACKAGE_ROOT / 'config' / 'mit_gains.yaml').read_text())
    for group_name in ('return', 'hold', 'track', 'idle'):
        for field in ('kp', 'kd'):
            values = config[group_name][field]
            assert len(values) == 7
            assert all(isinstance(value, (int, float)) and value >= 0 for value in values)
    assert len(config['effort_limits']) == 7
    assert all(value > 0 for value in config['effort_limits'])


def test_joint_command_uses_only_fixed_size_control_arrays():
    definition = (PACKAGE_ROOT / 'msg' / 'JointCommand.msg').read_text()
    for field in ('position', 'velocity', 'kp', 'kd', 'effort'):
        assert f'float32[7] {field}' in definition
    assert 'string[]' not in definition
    assert 'float64[]' not in definition


def test_hardware_status_exposes_active_mask_and_ack_sequence():
    definition = (PACKAGE_ROOT / 'msg' / 'HardwareStatus.msg').read_text()
    assert 'uint32 last_command_sequence' in definition
    assert 'uint32 active_joint_mask' in definition
    assert 'uint16 fault_code' in definition


def test_bringup_profile_exports_joints_1_to_6_without_gravity():
    xacro_path = (
        PACKAGE_ROOT / 'urdf' / 'rebotarm.single_joint.micro_ros.xacro')
    root = ET.parse(xacro_path).getroot()
    ros2_control = root.find('ros2_control')
    assert ros2_control is not None
    expected_joints = [f'joint{i}' for i in range(1, 7)]
    assert [joint.attrib['name'] for joint in ros2_control.findall('joint')] == (
        expected_joints)
    parameters = {
        item.attrib['name']: (item.text or '').strip()
        for item in ros2_control.findall('./hardware/param')
    }
    assert parameters['required_joint_mask'] == '0x3F'
    assert parameters['enable_gravity_compensation'] == 'false'
    assert parameters['model_urdf_path'] == '$(arg model_urdf_path)'
    assert parameters['joint4_gravity_scale'] == '0.85'
    assert int(parameters['state_timeout_ms']) >= 500


def test_bringup_controllers_claim_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'single_joint_controllers.yaml').read_text())
    expected_joints = [f'joint{i}' for i in range(1, 7)]
    assert config['joint1_trajectory_controller']['ros__parameters'][
        'joints'] == expected_joints
    assert config['joint1_trajectory_controller']['ros__parameters'][
        'allow_partial_joints_goal'] is True
    manager = config['controller_manager']['ros__parameters']
    assert manager['joint1_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    assert manager['joint1_teleop_controller']['type'] == (
        'position_controllers/JointGroupPositionController')
    assert config['joint1_teleop_controller']['ros__parameters']['joints'] == (
        expected_joints)
    broadcaster = config['joint_state_broadcaster']['ros__parameters']
    assert broadcaster['joints'] == expected_joints
    assert broadcaster['interfaces'] == ['position', 'velocity', 'effort']


def test_bringup_gains_match_configured_profiles_for_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'single_joint_mit_gains.yaml').read_text())
    expected_gains = {
        'return': ([30.0] * 3 + [15.0] * 3, [3.0] * 3 + [2.0] * 3),
        'hold': ([20.0] * 3 + [10.0] * 3, [2.0] * 3 + [1.5] * 3),
        'track': ([40.0] * 4 + [10.0] * 2, [3.0] * 3 + [1.5] * 3),
        'idle': ([0.0] * 6, [1.0] * 6),
    }
    for group_name, (expected_kp, expected_kd) in expected_gains.items():
        assert config[group_name]['kp'][:6] == expected_kp
        assert config[group_name]['kd'][:6] == expected_kd
    assert config['effort_limits'][:6] == [5.0] * 3 + [3.0] * 3
    for group_name in ('return', 'hold', 'track', 'idle'):
        assert config[group_name]['kp'][6] == 0.0
        assert config[group_name]['kd'][6] == 0.0
    assert config['effort_limits'][6] == 0.0
