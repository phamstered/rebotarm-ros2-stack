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


def test_state_only_launch_uses_read_only_republisher():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'stm32h723_arm.launch.py'
    ).read_text()
    assert "'state_only'" in launch_source
    assert "default_value='false'" in launch_source
    assert "executable='stm32_state_republisher.py'" in launch_source
    assert 'condition=IfCondition(state_only)' in launch_source
    assert launch_source.count('condition=UnlessCondition(state_only)') == 2
    assert "'position_controller'" in launch_source
    assert "'enable_gravity_controller'" in launch_source
    assert "choices=['teleop', 'trajectory', 'none']" in launch_source
    assert 'condition=teleop_active' in launch_source
    assert 'condition=trajectory_active' in launch_source
    assert 'condition=gravity_active' in launch_source
    assert 'gravity_compensation_controller' in launch_source

    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    assert 'scripts/stm32_state_republisher.py' in cmake_source


def test_bringup_profile_exports_separate_position_and_effort_interfaces():
    xacro_path = (
        PACKAGE_ROOT / 'urdf' / 'rebotarm.arm.micro_ros.xacro')
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
    assert 'enable_gravity_compensation' not in parameters
    assert 'model_urdf_path' not in parameters
    assert 'joint4_gravity_scale' not in parameters
    assert int(parameters['state_timeout_ms']) >= 500
    for joint in ros2_control.findall('joint'):
        assert [interface.attrib['name'] for interface in
                joint.findall('command_interface')] == ['position', 'effort']


def test_bringup_controllers_claim_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_controllers.yaml').read_text())
    expected_joints = [f'joint{i}' for i in range(1, 7)]
    assert config['arm_trajectory_controller']['ros__parameters'][
        'joints'] == expected_joints
    assert config['arm_trajectory_controller']['ros__parameters'][
        'allow_partial_joints_goal'] is True
    manager = config['controller_manager']['ros__parameters']
    assert manager['arm_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    assert manager['arm_teleop_controller']['type'] == (
        'position_controllers/JointGroupPositionController')
    assert manager['gravity_compensation_controller']['type'] == (
        'microros_stm32h723_bridge/GravityCompensationController')
    assert config['arm_teleop_controller']['ros__parameters']['joints'] == (
        expected_joints)
    broadcaster = config['joint_state_broadcaster']['ros__parameters']
    assert broadcaster['joints'] == expected_joints
    assert broadcaster['interfaces'] == ['position', 'velocity', 'effort']
    gravity = config['gravity_compensation_controller']['ros__parameters']
    assert gravity['joints'] == expected_joints
    assert gravity['gravity_scales'] == [0.1, 0.2, 0.2, 0.65, 1.0, 1.0]
    assert gravity['effort_limits'] == [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]


def test_gravity_is_computed_only_by_the_effort_controller():
    hardware_source = (PACKAGE_ROOT / 'src' / 'micro_ros_system.cpp').read_text()
    controller_source = (
        PACKAGE_ROOT / 'src' / 'gravity_compensation_controller.cpp').read_text()
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()

    assert 'enable_gravity_compensation' not in hardware_source
    assert 'joint4_gravity_scale' not in hardware_source
    assert 'gravity_.compute' not in hardware_source
    assert 'gravity_.compute' in controller_source
    assert 'HW_IF_EFFORT' in controller_source
    assert 'HW_IF_POSITION' in controller_source
    assert (
        'target_link_libraries(${HARDWARE_LIBRARY} yaml-cpp)'
        in cmake_source)


def test_bringup_gains_match_configured_profiles_for_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_mit_gains.yaml').read_text())
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


def test_gravity_mode_uses_zero_position_stiffness():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'gravity_mit_gains.yaml').read_text())
    assert config['return']['kp'] == [0.0] * 7
    assert config['hold']['kp'] == [0.0] * 7
    assert config['track']['kp'] == [0.0] * 7
    for group_name in ('return', 'hold', 'track', 'idle'):
        assert config[group_name]['kd'] == [1.0] * 6 + [0.0]
    assert config['idle']['kp'] == [0.0] * 7
    assert config['idle']['kd'] == [1.0] * 6 + [0.0]
    assert config['effort_limits'] == [5.0] * 3 + [3.0] * 3 + [0.0]
