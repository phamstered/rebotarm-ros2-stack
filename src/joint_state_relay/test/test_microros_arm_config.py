from pathlib import Path

import yaml


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
    assert config['joint_mapping'] == {
        'joint1': 'joint1',
        'joint2': 'joint2',
        'joint3': 'joint3',
        'joint4': 'joint4',
        'joint5': 'joint5',
        'joint6': 'joint6',
    }
    assert list(config['joints']) == [f'joint{i}' for i in range(1, 7)]
    assert config['max_velocity'] == 1.0
    assert config['command_timeout'] <= 0.1
    assert config['follower_state_timeout'] <= 0.2
