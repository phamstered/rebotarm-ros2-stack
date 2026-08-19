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


def test_single_joint_profile_exports_only_joint1_without_gravity():
    xacro_path = (
        PACKAGE_ROOT / 'urdf' / 'rebotarm.single_joint.micro_ros.xacro')
    root = ET.parse(xacro_path).getroot()
    ros2_control = root.find('ros2_control')
    assert ros2_control is not None
    assert [joint.attrib['name'] for joint in ros2_control.findall('joint')] == [
        'joint1']
    parameters = {
        item.attrib['name']: (item.text or '').strip()
        for item in ros2_control.findall('./hardware/param')
    }
    assert parameters['required_joint_mask'] == '0x01'
    assert parameters['enable_gravity_compensation'] == 'false'


def test_single_joint_controller_claims_only_joint1():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'single_joint_controllers.yaml').read_text())
    assert config['joint1_controller']['ros__parameters']['joints'] == ['joint1']


def test_single_joint_gains_are_conservative_and_inactive_slots_are_zero():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'single_joint_mit_gains.yaml').read_text())
    assert config['hold']['kp'][0] <= 2.0
    assert config['track']['kp'][0] <= 5.0
    assert config['effort_limits'][0] <= 1.0
    for group_name in ('return', 'hold', 'track', 'idle'):
        assert config[group_name]['kp'][1:] == [0.0] * 6
        assert config[group_name]['kd'][1:] == [0.0] * 6
    assert config['effort_limits'][1:] == [0.0] * 6
