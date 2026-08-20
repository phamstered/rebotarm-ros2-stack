from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_single_joint_teleop_targets_ros2_control_controller():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' /
         'relay_microros_single_joint.yaml').read_text())

    assert config['mode'] == 'ros2_control'
    assert config['relative_mode'] is True
    assert config['leader_topic'] != config['follower_state_topic']
    assert config['follower_command_topic'] == (
        '/joint1_teleop_controller/commands')
    assert config['joint_mapping'] == {'joint1': 'joint1'}
    assert config['max_velocity'] <= 0.3
    assert config['command_timeout'] <= 0.1
    assert config['follower_state_timeout'] <= 0.2
