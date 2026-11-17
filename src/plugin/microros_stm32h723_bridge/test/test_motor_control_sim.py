"""Protocol boundaries using an isolated MCU model, never a physical motor."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from motor_control_sim import (MotorControlSim, BUSY, FAULT, INVALID, LEASE,
                               LIMITS, MODE, NONFINITE, NOT_DISABLED,
                               READBACK, SEQUENCE, STALE_IDENTITY, UNSUPPORTED)


def request(model, action=1, op=1, source=7, drive=2, mask=1):
    return dict(action=action, operation_id=op, boot_id=model.boot_id,
                expected_control_epoch=model.control_epoch, source_id=source,
                active_joint_mask=mask, control_mode=drive, lease_id=model.lease_id,
                disable_sequence=model.last_command_sequence)


def disabled(model, sequence=1, now=0.0):
    assert model.command_v1(dict(mode=3, sequence=sequence), now) == 0
    model.tick(now + 0.001)


def configured():
    model = MotorControlSim()
    disabled(model)
    assert model.service(request(model), 0.01)['accepted']
    assert model.switching and model.control_modes[0] == 1
    model.tick(0.061)
    assert model.control_modes[0] == 2 and model.operation_state == 2
    return model


def acquired():
    model = configured()
    assert model.service(request(model, action=2, op=2), 0.07)['accepted']
    return model


def command(model, sequence=2, mode=1):
    return dict(boot_id=model.boot_id, control_epoch=model.control_epoch,
                source_id=model.owner_source_id, lease_id=model.lease_id,
                sequence=sequence, mode=mode, control_mode=2,
                active_joint_mask=1, position=[0.0 if mode == 0 else 0.5] + [0.0] * 6,
                velocity_limit=[0.2] + [0.0] * 6)


def test_switch_ack_does_not_confirm_mode_and_release_requires_all_mit():
    model = acquired()
    assert model.command_v2(command(model), 0.08) == 0
    model.tick(0.09)
    assert 0 < model.position[0] < 0.5
    assert model.enabled_joint_mask == 1
    assert model.position[1:] == [0.0] * 6
    disabled(model, sequence=3, now=0.1)
    assert model.service(request(model, action=3, op=3), 0.11)['error_code'] == MODE
    assert model.service(request(model, drive=1, op=4), 0.12)['accepted']
    assert model.control_modes[0] == 2
    model.tick(0.18)
    assert model.service(request(model, action=3, op=5), 0.19)['accepted']
    assert model.owner_source_id == 0 and model.lease_id == 0
    assert model.command_v1(dict(mode=1, sequence=4), 0.2) == 0


def test_configuration_requires_real_new_disabled_feedback_and_exact_sequence():
    model = MotorControlSim()
    model.tick(0.0)
    assert not model.disabled_verified
    assert model.service(request(model), 0.001)['error_code'] == NOT_DISABLED
    assert model.command_v1(dict(mode=3, sequence=1), 0.01) == 0
    assert not model.disabled_verified
    assert model.service(request(model, op=2), 0.01)['error_code'] == NOT_DISABLED
    model.tick(0.011)
    assert model.disabled_verified
    stale = request(model, op=3)
    stale['disable_sequence'] = 0
    assert model.service(stale, 0.02)['error_code'] == NOT_DISABLED
    assert model.service(request(model, op=4), 0.2)['error_code'] == NOT_DISABLED
    model.tick(0.21)
    assert model.service(request(model, op=5), 0.22)['accepted']


def test_sequence_wrap_is_valid_but_replay_is_not():
    model = MotorControlSim()
    model.last_command_sequence = 0xffffffff
    disabled(model, sequence=0)
    assert model.service(request(model), 0.01)['accepted']
    assert model.command_v1(dict(mode=3, sequence=0), 0.02) == SEQUENCE


def test_disabled_verified_expires_if_disabled_feedback_disappears():
    model = MotorControlSim()
    disabled(model)
    assert model.disabled_verified
    model.feedback_enabled = False
    model.tick(0.11)
    assert not model.disabled_verified
    assert model.service(request(model), 0.12)['error_code'] == NOT_DISABLED


def test_pending_switch_rejects_motion_and_disable_cancels_late_completion():
    model = MotorControlSim()
    disabled(model)
    assert model.service(request(model), 0.01)['accepted']
    assert model.command_v1(dict(mode=1, sequence=2), 0.02) == BUSY
    disabled(model, sequence=3, now=0.03)
    model.tick(1.0)
    assert model.operation_state == 4 and model.control_modes == [1] * 7


def test_cancel_after_motion_cannot_reuse_an_older_disable_proof():
    model = acquired()
    assert model.command_v2(command(model), 0.08) == 0
    assert not model.disabled_verified
    assert model.service(request(model, action=4, op=3), 0.09)['accepted']
    assert model.operation_id == 3 and model.operation_state == 4
    model.tick(0.1)
    assert model.service(request(model, op=4, drive=1), 0.11)['error_code'] == NOT_DISABLED
    disabled(model, sequence=3, now=0.12)
    assert model.service(request(model, op=5, drive=1), 0.13)['accepted']


def test_failed_readback_keeps_output_off_and_mode_unverified():
    model = MotorControlSim()
    disabled(model)
    model.readback_failure_mask = 1
    assert model.service(request(model), 0.01)['accepted']
    model.tick(0.07)
    assert model.mode == 3 and model.enabled_joint_mask == 0
    assert model.operation_state == 3 and model.error_code == READBACK
    assert model.verified_joint_mask == 0x7e
    assert model.service(request(model, action=2, op=2), 0.08)['error_code'] == MODE


def test_retries_are_idempotent_and_operation_collision_is_rejected():
    model = MotorControlSim()
    disabled(model)
    configure = request(model)
    reply = model.service(configure, 0.01)
    model.tick(0.07)
    assert model.service(configure, 0.08) == reply
    assert model.control_epoch == 2 and not model.switching
    configure['active_joint_mask'] = 2
    assert model.service(configure, 0.09)['error_code'] == INVALID


def test_recovered_source_may_configure_only_after_disabled_and_lease_revoked():
    model = acquired()
    assert model.service(request(model, op=3, source=8), 0.08)['error_code'] == BUSY
    disabled(model, sequence=2, now=0.09)
    assert model.service(request(model, op=4, source=8, drive=1), 0.1)['accepted']
    assert model.owner_source_id == 8


def test_v1_cannot_steal_v2_owner_and_disable_revokes_late_v2_frames():
    model = acquired()
    frame = command(model)
    assert model.command_v2(frame, 0.08) == 0
    target = model.target.copy()
    assert model.command_v1(dict(mode=2, sequence=3), 0.09) == BUSY
    assert model.target == target and model.mode == 1
    assert model.reject_code == BUSY and model.fault_code == 0
    disabled(model, sequence=4, now=0.1)
    frame['sequence'] = 5
    assert model.command_v2(frame, 0.11) == LEASE
    assert model.mode == 3 and model.enabled_joint_mask == 0


@pytest.mark.parametrize(('field', 'value', 'error'), [
    ('boot_id', 999, STALE_IDENTITY), ('control_epoch', 0, STALE_IDENTITY),
    ('source_id', 99, LEASE), ('lease_id', 999, LEASE),
    ('control_mode', 1, MODE), ('mode', 2, MODE), ('mode', 3, MODE),
    ('active_joint_mask', 0, INVALID), ('active_joint_mask', 0x81, INVALID),
    ('sequence', 1, SEQUENCE), ('position', [0.0], INVALID),
    ('velocity_limit', [0.0] * 8, INVALID),
])
def test_rejected_identity_state_and_shape_leave_outputs_unchanged(field, value, error):
    model = acquired()
    before = deepcopy((model.target, model.mode, model.enabled_joint_mask, model.control_modes))
    frame = command(model)
    frame[field] = value
    assert model.command_v2(frame, 0.08) == error
    assert (model.target, model.mode, model.enabled_joint_mask, model.control_modes) == before
    assert model.reject_code == error and model.rejected_sequence == frame['sequence']


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), 1e100])
def test_nonfinite_and_float32_overflow(value):
    model = acquired()
    frame = command(model)
    frame['position'][0] = value
    assert model.command_v2(frame, 0.08) == NONFINITE


@pytest.mark.parametrize('speed', [0.0, -0.1, 1.01])
def test_velocity_is_positive_and_respects_verified_limit(speed):
    model = acquired()
    frame = command(model)
    frame['velocity_limit'][0] = speed
    assert model.command_v2(frame, 0.08) == LIMITS


def test_position_limits_unselected_nonzero_and_unsupported_gripper():
    model = acquired()
    frame = command(model)
    frame['position'][0] = 3.1
    assert model.command_v2(frame, 0.08) == LIMITS
    frame['position'][0], frame['position'][1] = 0.5, 0.1
    assert model.command_v2(frame, 0.08) == INVALID
    model = MotorControlSim()
    disabled(model)
    assert model.service(request(model, mask=0x40), 0.01)['error_code'] == UNSUPPORTED


def test_actual_readback_has_precedence_over_claimed_mode():
    model = acquired()
    model.control_modes[0] = 1
    assert model.command_v2(command(model), 0.08) == MODE
    model.owner_source_id = model.lease_id = 0
    model.control_modes[0] = 2
    assert model.command_v1(dict(mode=1, sequence=2), 0.09) == MODE


def test_hold_latches_measured_position_and_watchdog_revokes_lease():
    model = acquired()
    assert model.command_v2(command(model), 0.08) == 0
    model.tick(0.09)
    measured = model.position.copy()
    assert model.command_v2(command(model, sequence=3, mode=0), 0.1) == 0
    assert model.target == measured
    model.tick(0.11)
    assert model.position == measured
    assert model.command_v2(command(model, sequence=4), 0.12) == 0
    model.tick(0.23)
    assert model.mode == 0 and model.target == model.position and model.lease_id == 0
    model.tick(0.63)
    assert model.fault_code == 11
    disabled(model, sequence=5, now=0.64)
    assert model.fault_code == 11 and model.mode == 3 and model.enabled_joint_mask == 0
    assert model.service(request(model, op=3), 0.65)['error_code'] == FAULT


def test_idle_lease_expires_without_enabling_outputs():
    model = acquired()
    model.tick(0.171)
    assert model.lease_id == 0 and model.enabled_joint_mask == 0 and model.mode == 3


def test_hold_stream_timeout_also_revokes_lease_without_reusing_old_goal():
    model = acquired()
    frame = command(model, mode=0)
    assert model.command_v2(frame, 0.08) == 0
    model.tick(0.181)
    assert model.lease_id == 0 and model.mode == 0
    assert model.target == model.position
    frame['sequence'] = 3
    assert model.command_v2(frame, 0.19) == LEASE


def test_hold_validates_measured_position_when_zero_is_outside_position_limits():
    model = acquired()
    model.position_min[0], model.position_max[0] = 1.0, 2.0
    model.position[0] = 1.5
    assert model.command_v2(command(model, mode=0), 0.08) == 0
    assert model.target[0] == 1.5
    track = command(model, sequence=3)
    track['position'][0] = 0.0
    assert model.command_v2(track, 0.09) == LIMITS


@pytest.mark.parametrize('position', [3.1, float('nan'), float('inf')])
def test_hold_rejects_out_of_bounds_or_invalid_measured_position(position):
    model = acquired()
    model.position[0] = position
    assert model.command_v2(command(model, mode=0), 0.08) == LIMITS


def test_hold_requires_zero_input_positions_and_fresh_measurements():
    model = acquired()
    frame = command(model, mode=0)
    frame['position'][0] = 0.1
    assert model.command_v2(frame, 0.08) == INVALID
    frame['position'][0] = 0.0
    assert model.command_v2(frame, 0.17) == FAULT


def test_reboot_rejects_prior_identity_even_if_sequence_and_epoch_repeat():
    old = acquired()
    frame = command(old)
    rebooted = acquired()
    rebooted.boot_id = old.boot_id + 1
    assert rebooted.command_v2(frame, 0.08) == STALE_IDENTITY


def test_accepted_command_clears_rejection_without_clearing_latched_fault():
    model = acquired()
    frame = command(model)
    frame['mode'] = 2
    assert model.command_v2(frame, 0.08) == MODE
    frame['mode'] = 1
    assert model.command_v2(frame, 0.09) == 0
    assert model.reject_code == 0
    model.fault_code = 11
    disabled(model, sequence=3, now=0.1)
    assert model.fault_code == 11
