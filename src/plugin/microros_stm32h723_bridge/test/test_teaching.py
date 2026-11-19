from pathlib import Path
import importlib.util
import os
import subprocess
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('teaching_core', ROOT / 'scripts/teaching_core.py')
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


def test_ros_entrypoint_is_executable_for_symlink_install():
    entrypoint = ROOT / 'scripts/teaching_gripper.py'
    assert os.access(entrypoint, os.X_OK)
    result = subprocess.run([str(entrypoint), '--help'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert '{move,cancel,check,check-manual}' in result.stdout


def test_manual_gains_leave_arm_compliant_and_gripper_unpowered_in_every_mode():
    gains = yaml.safe_load((ROOT / 'config/teaching_mit_gains.yaml').read_text())
    for mode in ('return', 'hold', 'track', 'idle'):
        assert gains[mode]['kp'][:6] == [0] * 6
        assert all(v > 0 for v in gains[mode]['kd'][:6])
    assert gains['return']['kp'] == [0] * 7
    for mode in ('return', 'hold', 'track', 'idle'):
        assert gains[mode]['kp'][6] == 0
        assert gains[mode]['kd'][6] == 0
    gravity = yaml.safe_load((ROOT / 'config/gravity_mit_gains.yaml').read_text())
    assert gains['effort_limits'][:6] == gravity['effort_limits'][:6]
    assert gains['effort_limits'][6] == 0


@pytest.mark.parametrize('operation', ['move', 'cancel'])
def test_legacy_electric_requests_fail_before_ros_initialization(operation):
    result = subprocess.run([str(ROOT / 'scripts/teaching_gripper.py'), operation,
                             '--command-id', '12345678-1234-1234-1234-123456789abc',
                             '--position', '0.02'], capture_output=True, text=True)
    assert result.returncode != 0
    assert '手动示教不支持电动夹爪操作' in result.stderr
    assert 'ModuleNotFoundError' not in result.stderr


def controller(name, claims=(), state='active'):
    return SimpleNamespace(name=name, claimed_interfaces=list(claims), state=state)


def test_manual_gripper_requires_gravity_and_no_motion_or_gripper_effort_owners():
    ready = [controller('gravity_compensation_controller')]
    assert core.controller_error(ready) is None
    assert core.controller_error([])
    for name in ('arm_trajectory_controller', 'arm_teleop_controller', 'vla_position_controller',
                 'joint4_calibration_controller', 'gripper_teleop_controller', 'gripper_trajectory_controller'):
        assert core.controller_error(ready + [controller(name)])
    for claim in ('joint2/position', 'joint4/velocity', core.JOINT + '/position',
                  core.JOINT + '/velocity', core.JOINT + '/effort'):
        assert core.controller_error(ready + [controller('unknown', [claim])])
    assert core.controller_error(ready + [controller('gripper_trajectory_controller', state='inactive')]) is None


def test_freshness_checks_source_clock_and_receipt_age():
    assert core.fresh(10, 10_000_000_000, 10.1, 10_100_000_000)
    assert not core.fresh(10, 10_000_000_000, 10.6, 10_100_000_000)
    assert not core.fresh(10, 0, 10.1, 10_100_000_000)
    assert not core.fresh(10, 11_000_000_000, 10.1, 10_100_000_000)


def test_teaching_preset_selects_manual_gains_and_rejects_incompatible_overrides():
    script = (ROOT.parents[2] / 'scripts/start_arm_teleop.sh').read_text()
    prefix = script.split('SELECTED_EFFORT_CONTROLLER=""')[0]
    command = prefix + '\nprintf "%s|%s|%s|%s" "$POSITION_CONTROLLER" "$ENABLE_GRAVITY_CONTROLLER" "$GRIPPER_CONTROLLER" "$GAINS_FILE"\n'
    result = subprocess.run(['bash', '-c', command, 'test', '--mode=teaching'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'teaching|true|none|' in result.stdout
    assert result.stdout.endswith('/config/teaching_mit_gains.yaml')
    for override in ('--gravity=false', '--gripper=false', '--joint4-calibration=true'):
        result = subprocess.run(['bash', '-c', command, 'test', '--mode=teaching', override], capture_output=True, text=True)
        assert result.returncode != 0
