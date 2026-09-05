from pathlib import Path

import pytest
import yaml

from joint_state_relay.gripper_calibration import linear_map


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_arm_teleop_targets_ros2_control_controller():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' /
         'relay_microros_arm.yaml').read_text())

    assert config['mode'] == 'ros2_control'
    assert config['relative_mode'] is False
    assert config['leader_topic'] != config['follower_state_topic']
    assert config['follower_command_topic'] == (
        '/arm_teleop_controller/commands')
    assert config['gripper_command_topic'] == (
        '/gripper_teleop_controller/commands')
    assert config['joint_mapping'] == {
        'joint1': 'joint1',
        'joint2': 'joint2',
        'joint3': 'joint3',
        'joint4': 'joint4',
        'joint5': 'joint5',
        'joint6': 'joint6',
        'joint7_left': 'gripper_finger_left_joint',
    }
    assert list(config['joints']) == [
        *[f'joint{i}' for i in range(1, 7)],
        'gripper_finger_left_joint',
    ]
    gripper = config['joints']['gripper_finger_left_joint']
    assert gripper['sign'] == pytest.approx(0.053150073760)
    assert gripper['offset'] == pytest.approx(-0.000278293135)
    assert gripper['leader_limit'] == [0.005235988, 0.945968455]
    assert gripper['follower_limit'] == [0.0, 0.05]
    assert config['gripper_max_velocity'] == 0.02
    assert config['max_velocity'] == 1.0
    assert config['command_timeout'] <= 0.1
    assert config['follower_state_timeout'] <= 0.2


def test_gripper_linear_map_matches_both_endpoints():
    scale, offset = linear_map(0.1, 0.9, 0.5, -2.5)
    assert scale * 0.1 + offset == pytest.approx(0.5)
    assert scale * 0.9 + offset == pytest.approx(-2.5)


def test_gripper_linear_map_rejects_zero_input_range():
    with pytest.raises(ValueError):
        linear_map(0.2, 0.2, 0.0, 1.0)


def test_relay_supports_a_separate_rate_limited_gripper_topic():
    source = (
        PACKAGE_ROOT / 'joint_state_relay' / 'relay_node.py').read_text()
    assert "self._config.get('gripper_command_topic')" in source
    assert "'gripper_max_velocity'" in source
    assert 'self._publish_ros2_control_gripper()' in source
