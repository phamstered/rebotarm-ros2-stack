from pathlib import Path

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
